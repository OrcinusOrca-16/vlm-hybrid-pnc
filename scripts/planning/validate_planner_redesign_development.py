"""Development-only regression; never builds or executes held-out scenarios.

Physical limits are checked without relaxing them. Initial-state tolerance is
validation-only (1e-5 absolute). Near-boundary flags are diagnostic, not tuning:
clearance/road margin <= 1 cm or curvature utilization >= 95 percent.
Checks apply at returned poses, not continuously between poses. Road checks
include the production Frenet-width contract and rotated footprint corners
projected onto the reference spline; the latter is diagnostic only.
"""

from dataclasses import asdict, fields
import json
import math
import statistics
import sys
from time import perf_counter

from scipy.optimize import minimize_scalar

from src.benchmark.optimization_adapter import run_optimization_benchmark_adapter
from src.benchmark.tuning_scenario_suite import build_tuning_scenarios
from src.benchmark.scenario_suite import build_stress_sweep_scenarios
from src.config.planning import DEFAULT_OPTIMIZATION_PLANNER_CONFIG
from src.planning.frenet import vehicle_state_to_frenet


INITIAL_ATOL = 1e-5
NEAR_MARGIN_M = 0.01
NEAR_CURVATURE_RATIO = 0.95
ROAD_LIMITATION = (
    "Current production road-boundary validation uses the Frenet lateral-width contract. "
    "Rotated-footprint road-boundary checking is diagnostic only and is not part "
    "of the production acceptance criterion."
)


def adapter_kwargs(scenario):
    """Pass scenario inputs verbatim; keep every production default intact."""
    names = (
        "reference_line", "vehicle_state", "vehicle_params", "road_min_l",
        "road_max_l", "max_curvature", "minimum_clearance_m",
        "planning_horizon_m", "resolution_m", "lateral_reference_m",
        "terminal_lateral_target_m", "terminal_first_derivative_target",
        "lateral_weight", "first_derivative_weight", "second_derivative_weight",
        "third_derivative_weight", "terminal_lateral_weight",
        "terminal_first_derivative_weight",
    )
    kwargs = {name: getattr(scenario, name) for name in names}
    kwargs["obstacles"] = list(scenario.obstacles)
    kwargs["max_second_derivative_abs"] = (
        DEFAULT_OPTIMIZATION_PLANNER_CONFIG.max_second_derivative_abs
    )
    return kwargs


def exact_clearance(point, vehicle, obstacle):
    """Independent oriented rectangle/circle distance, without planner helpers."""
    dx, dy = obstacle.x - point.x, obstacle.y - point.y
    c, s = math.cos(point.yaw), math.sin(point.yaw)
    local_x, local_y = c * dx + s * dy, -s * dx + c * dy
    outside_x = max(-vehicle.rear_overhang_m - local_x, 0.0,
                    local_x - vehicle.wheelbase_m - vehicle.front_overhang_m)
    outside_y = max(abs(local_y) - vehicle.width_m / 2.0, 0.0)
    return math.hypot(outside_x, outside_y) - obstacle.radius_m


def corner_road_margin(scenario, point, frenet_s):
    """Project rotated corners accurately in the locally unique road region.

    The bounded spline projection includes endpoints. Longitudinal extension
    past the reference line is not treated as a lateral road boundary.
    """
    vehicle, reference = scenario.vehicle_params, scenario.reference_line
    c, s = math.cos(point.yaw), math.sin(point.yaw)
    reach = (vehicle.wheelbase_m + vehicle.front_overhang_m
             + vehicle.rear_overhang_m + vehicle.width_m)
    lo, hi = max(0.0, frenet_s - reach), min(reference.length, frenet_s + reach)
    margins = []
    for body_x in (-vehicle.rear_overhang_m,
                   vehicle.wheelbase_m + vehicle.front_overhang_m):
        for body_y in (-vehicle.width_m / 2.0, vehicle.width_m / 2.0):
            x = point.x + c * body_x - s * body_y
            y = point.y + s * body_x + c * body_y

            def distance_squared(value):
                rx, ry = reference.position(value)
                return (x - rx) ** 2 + (y - ry) ** 2

            projection = minimize_scalar(distance_squared, bounds=(lo, hi),
                                         method="bounded", options={"xatol": 1e-10})
            if not projection.success:
                raise RuntimeError("Independent road projection did not converge")
            candidates = (lo, float(projection.x), hi)
            closest_s = min(candidates, key=distance_squared)
            rx, ry = reference.position(closest_s)
            yaw = reference.yaw(closest_s)
            lateral = -(x - rx) * math.sin(yaw) + (y - ry) * math.cos(yaw)
            margins.append(min(lateral - scenario.road_min_l,
                               scenario.road_max_l - lateral))
    return min(margins)


def inspect_result(scenario, result):
    errors, metrics = [], {}
    if not result.success:
        errors.append(result.failure_reason)
    if not math.isfinite(result.planning_time_ms):
        errors.append("numerical/non-finite planning time")
    if result.matched_objective_value is not None:
        metrics["canonical_objective"] = result.matched_objective_value
        if not math.isfinite(result.matched_objective_value):
            errors.append("numerical/non-finite canonical objective")
    if result.trajectory is None or result.path is None:
        if result.success:
            errors.append("missing trajectory/path")
        return errors, metrics
    points, path = result.trajectory.points, result.path
    metrics["trajectory_count"], metrics["path_count"] = len(points), len(path)
    if len(points) != len(path):
        errors.append("trajectory/path point count mismatch")
    if not points or not path:
        errors.append("empty trajectory/path")
        return errors, metrics
    if not all(math.isfinite(getattr(point, field.name))
               for point in [*points, *path] for field in fields(point)):
        errors.append("numerical/non-finite trajectory/path (NaN or inf)")
        return errors, metrics
    initial = vehicle_state_to_frenet(scenario.vehicle_state, scenario.reference_line)
    initial_errors = {name: getattr(points[0], name) - getattr(initial, name)
                      for name in ("s", "l", "dl_ds", "d2l_ds2")}
    metrics["initial_state_errors"] = initial_errors
    if any(abs(value) > INITIAL_ATOL for value in initial_errors.values()):
        errors.append("initial Frenet state not reproduced within 1e-5 absolute")
    metrics["terminal_lateral_error"] = points[-1].l - scenario.terminal_lateral_target_m
    metrics["terminal_dl_error"] = (
        points[-1].dl_ds - scenario.terminal_first_derivative_target
    )
    metrics["max_abs_curvature"] = max(abs(point.curvature) for point in path)
    metrics["curvature_margin"] = scenario.max_curvature - metrics["max_abs_curvature"]
    metrics["curvature_utilization"] = metrics["max_abs_curvature"] / scenario.max_curvature
    if metrics["curvature_margin"] < 0.0:
        errors.append("curvature violation")
    half_width = scenario.vehicle_params.width_m / 2.0
    metrics["frenet_road_margin"] = min(
        min(point.l - half_width - scenario.road_min_l,
            scenario.road_max_l - point.l - half_width) for point in points
    )
    if metrics["frenet_road_margin"] < 0.0:
        errors.append("road-boundary violation (production Frenet width contract)")
    if len(points) == len(path):
        road_margins = [corner_road_margin(scenario, point, frenet.s)
                        for point, frenet in zip(path, points)]
        worst_index = min(range(len(road_margins)), key=road_margins.__getitem__)
        metrics["rotated_corner_road_margin"] = road_margins[worst_index]
        metrics["worst_road_pose"] = {
            "index": worst_index, "frenet_s": points[worst_index].s,
            "x": path[worst_index].x, "y": path[worst_index].y,
            "yaw": path[worst_index].yaw,
        }
    if scenario.obstacles:
        metrics["minimum_exact_clearance"] = min(
            exact_clearance(point, scenario.vehicle_params, obstacle)
            for point in path for obstacle in scenario.obstacles
        )
        metrics["clearance_margin"] = (
            metrics["minimum_exact_clearance"] - scenario.minimum_clearance_m
        )
        if metrics["clearance_margin"] <= 0.0:
            errors.append("clearance violation (strict clearance > limit required)")
    return errors, metrics


def diagnose_failure(scenario):
    """Read-only trace of one identical adapter rerun; no monkeypatch or tuning.

    Diagnostics are separate from timed regression calls. QP infeasibility with
    heading rows is identified by stage, without claiming those rows alone are
    the cause. Infeasible solver iterates are never reported as physical paths.
    """
    events = []

    def trace(frame, event, arg):
        if not frame.f_code.co_filename.replace("\\", "/").endswith(
            "/src/planning/optimization_planner.py"
        ):
            return None
        name, local = frame.f_code.co_name, frame.f_locals
        if event == "exception" and name == "_solve_explicit_branch":
            error = arg[1]
            if isinstance(error, (RuntimeError, ValueError)):
                events.append({
                    "branch": local.get("branch_name"),
                    "stage": ("sequential refinement" if "initial_solution" in local
                              else "heading-aware nominal QP" if scenario.obstacles
                              else "nominal QP"),
                    "reason": str(error),
                    "iteration": local.get("iteration", 0),
                })
        if event == "exception" and name == "solve_qp" and "result" in local:
            info = local["result"].info
            events.append({"QP_status": info.status,
                           "primal_residual": float(info.prim_res),
                           "dual_residual": float(info.dual_res)})
        if event == "return" and name == "evaluate_exact_path" and arg is not None:
            events.append({"candidate_exact_ok": arg[1],
                           "curvature_ok": arg[2], "clearance_ok": arg[3],
                           "road_ok": arg[4],
                           "clearance_margin": (arg[5] - scenario.minimum_clearance_m
                                                if scenario.obstacles else None),
                           "curvature_margin": scenario.max_curvature - arg[6]})
        return trace

    previous_trace = sys.gettrace()
    try:
        sys.settrace(trace)
        rerun = run_optimization_benchmark_adapter(**adapter_kwargs(scenario))
    finally:
        sys.settrace(previous_trace)
    return {"rerun_success": rerun.success, "failure_reason": rerun.failure_reason,
            "events": events}


def scenario_parameters(scenario):
    data = {field.name: getattr(scenario, field.name) for field in fields(scenario)
            if field.name not in ("reference_line", "vehicle_state", "vehicle_params",
                                  "obstacles")}
    data["vehicle_state"] = asdict(scenario.vehicle_state)
    data["vehicle_params"] = asdict(scenario.vehicle_params)
    data["obstacles"] = [asdict(obstacle) for obstacle in scenario.obstacles]
    data["initial_frenet_state"] = asdict(vehicle_state_to_frenet(
        scenario.vehicle_state, scenario.reference_line
    ))
    return data


def format_value(value):
    return "N/A" if value is None else f"{value:.12g}"


def main():
    required = build_tuning_scenarios()
    stress = build_stress_sweep_scenarios()
    scenarios = [("required", scenario) for scenario in required]
    scenarios += [("stress", scenario) for scenario in stress]
    rows = []
    print("Development-only Optimization Planner regression", flush=True)
    print("Physical checks use unchanged limits at returned poses; no inter-pose guarantee.")
    print("Initial absolute tolerance=1e-5; near boundary: <=0.01 m or curvature >=95%.")
    print(ROAD_LIMITATION, flush=True)
    for group, scenario in scenarios:
        started = perf_counter()
        try:
            result = run_optimization_benchmark_adapter(**adapter_kwargs(scenario))
        except Exception as error:
            # A scenario exception must not prevent running the remaining development cases.
            row = {"name": scenario.name, "pass": False, "adapter_success": False,
                   "failure_reason": f"adapter exception: {type(error).__name__}: {error}",
                   "planning_time_ms": (perf_counter() - started) * 1000.0,
                   "metrics": {}}
        else:
            try:
                errors, metrics = inspect_result(scenario, result)
            except Exception as error:
                errors = [f"inspection exception: {type(error).__name__}: {error}"]
                metrics = {}
            row = {"name": scenario.name, "pass": not errors,
                   "adapter_success": result.success,
                   "failure_reason": "; ".join(errors) or None,
                   "planning_time_ms": result.planning_time_ms, "metrics": metrics}
        row["group"] = group
        metrics = row["metrics"]
        near = []
        for key in ("clearance_margin", "frenet_road_margin", "rotated_corner_road_margin"):
            if key in metrics and metrics[key] <= NEAR_MARGIN_M:
                near.append(f"{key}={metrics[key]:.12g} m")
        if metrics.get("curvature_utilization", 0.0) >= NEAR_CURVATURE_RATIO:
            near.append(f"curvature_utilization={metrics['curvature_utilization']:.12g}")
        row["near_boundary"] = near
        # Only explicit infeasibility is labelled INFEASIBLE; other failures
        # (including invalid returned paths) remain FAILURE.
        row["status"] = "SUCCESS" if row["pass"] else "FAILURE"
        if not row["adapter_success"] and "infeasible" in (row["failure_reason"] or "").lower():
            row["status"] = "INFEASIBLE"
        label = ("PASS" if row["pass"] else "FAIL") if group == "required" else row["status"]
        print(f"\n{label} {scenario.name}", flush=True)
        print(f"  reason={row['failure_reason'] or '-'}")
        print(f"  canonical_objective={format_value(metrics.get('canonical_objective'))} "
              f"planning_time_ms={row['planning_time_ms']:.6f}")
        print(f"  max_abs_cartesian_curvature={format_value(metrics.get('max_abs_curvature'))} "
              f"curvature_limit={scenario.max_curvature:.12g}")
        print(f"  minimum_exact_obstacle_clearance={format_value(metrics.get('minimum_exact_clearance'))} "
              f"clearance_limit={scenario.minimum_clearance_m:.12g} (strict >)")
        print(f"  terminal_lateral_error={format_value(metrics.get('terminal_lateral_error'))} "
              f"terminal_dl_ds_error={format_value(metrics.get('terminal_dl_error'))}")
        print("  independent_metrics=" + json.dumps(metrics, sort_keys=True))
        if near:
            print("  NEAR_BOUNDARY: " + "; ".join(near))
        if metrics.get("rotated_corner_road_margin", 0.0) < 0.0:
            row["additional_geometric_warning"] = {
                "name": scenario.name, "group": group,
                "minimum_rotated_corner_road_margin_m": metrics["rotated_corner_road_margin"],
                "worst_road_pose": metrics["worst_road_pose"],
            }
            print("  ADDITIONAL_GEOMETRIC_WARNING=" + json.dumps(
                row["additional_geometric_warning"], sort_keys=True))
        if not row["pass"]:
            print("  parameters=" + json.dumps(scenario_parameters(scenario), sort_keys=True))
            if not row["adapter_success"] and not row["failure_reason"].startswith("adapter exception"):
                try:
                    row["diagnostic"] = diagnose_failure(scenario)
                except Exception as error:
                    row["diagnostic"] = {"error": f"{type(error).__name__}: {error}"}
                print("  diagnostic=" + json.dumps(row["diagnostic"], sort_keys=True))
        rows.append(row)
    required_rows = [row for row in rows if row["group"] == "required"]
    stress_rows = [row for row in rows if row["group"] == "stress"]
    successes = [row for row in required_rows if row["pass"]]
    times = [row["planning_time_ms"] for row in successes]
    stress_successes = [row for row in stress_rows if row["pass"]]
    margins = [row["metrics"]["clearance_margin"] for row in stress_successes
               if "clearance_margin" in row["metrics"]]
    ratios = [row["metrics"]["curvature_utilization"] for row in stress_successes]
    required_summary = {
        "tuning_scenario_count": len(required_rows), "pass_count": len(successes),
        "fail_count": len(required_rows) - len(successes),
        "failed_tuning_scenario_names": [row["name"] for row in required_rows if not row["pass"]],
        "mean_successful_planning_time_ms": statistics.mean(times) if times else None,
        "maximum_successful_planning_time_ms": max(times) if times else None,
    }
    stress_summary = {
        "total_stress_scenario_count": len(stress_rows),
        "planner_success_count": sum(row["adapter_success"] for row in stress_rows),
        "production_validation_success_count": len(stress_successes),
        "non_success_count": len(stress_rows) - len(stress_successes),
        "infeasible_count": sum(row["status"] == "INFEASIBLE" for row in stress_rows),
        "failure_count": sum(row["status"] == "FAILURE" for row in stress_rows),
        "minimum_successful_obstacle_clearance_margin_m": min(margins) if margins else None,
        "maximum_successful_curvature_utilization_ratio": max(ratios) if ratios else None,
        "near_boundary_cases": [{"name": row["name"], "flags": row["near_boundary"]}
                                for row in stress_rows if row["near_boundary"]],
        "additional_rotated_footprint_road_warnings": [
            row["additional_geometric_warning"] for row in stress_rows
            if "additional_geometric_warning" in row
        ],
    }
    for key, prefix in (("obstacle_lateral_sweep_results", "obstacle_lateral_"),
                        ("obstacle_longitudinal_sweep_results", "obstacle_longitudinal_"),
                        ("road_width_sweep_results", "road_half_width_")):
        stress_summary[key] = [
            {"name": row["name"], "status": row["status"],
             "reason": row["failure_reason"], "metrics": row["metrics"]}
            for row in stress_rows if row["name"].startswith(prefix)
        ]
    print("\nREQUIRED_REGRESSION_SUMMARY=" + json.dumps(required_summary, sort_keys=True), flush=True)
    print("\nSTRESS_ENVELOPE_SUMMARY=" + json.dumps(stress_summary, sort_keys=True), flush=True)
    if len(successes) != len(required_rows):
        raise SystemExit(1)


if __name__ == "__main__":
    main()