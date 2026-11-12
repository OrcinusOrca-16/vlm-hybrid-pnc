"""Run the frozen final Planning & Control benchmark.

This script does not tune planners or controllers. It evaluates the frozen
9-scenario evaluation suite with the current production Sampling/Optimization
planners, the common curvature-aware MPC, and the current constant-safe-speed
layer.

Primary system comparison:
    each successful planner path is tracked at its own computed constant safe
    speed.

Auxiliary controlled comparison:
    when both planners succeed, both paths are also tracked at the lower of the
    two safe speeds.

Longitudinal speed-transition dynamics remain outside the current scope: each
closed-loop run starts directly at the requested constant speed.
"""

from __future__ import annotations

import csv
import json
import math
import subprocess
from dataclasses import asdict, dataclass, replace
from pathlib import Path
from statistics import mean

from src.benchmark.evaluation_scenario_suite import build_final_evaluation_scenarios
from src.benchmark.scenario_runner import run_benchmark_scenario
from src.common.types import ControlCommand
from src.control.curvature_aware_mpc import CurvatureAwareMPCController
from src.control.mpc_config import MPC_DT, MPC_MAX_STEERING_ANGLE, MPC_MAX_STEERING_RATE, MPC_WHEELBASE_M
from src.control.path_tracking import compute_constant_safe_speed, compute_path_tracking_error, curvature_to_steering, match_path_reference
from src.vehicle.kinematic_bicycle import update_vehicle_state


OUTPUT_DIR = Path("outputs/planning_control_benchmark")
CSV_PATH = OUTPUT_DIR / "final_planning_control_benchmark.csv"
SUMMARY_JSON_PATH = OUTPUT_DIR / "final_planning_control_benchmark_summary.json"

LATERAL_FINAL_TOLERANCE_M = 0.20
HEADING_FINAL_TOLERANCE_RAD = 0.10
STEERING_TOLERANCE_RAD = 1e-3
STEERING_RATE_TOLERANCE_RADPS = 0.02
END_PROGRESS_TOLERANCE_M = 0.25
EXTRA_SIMULATION_TIME_S = 5.0
MIN_REFERENCE_SPEED_MPS = 0.1


@dataclass(frozen=True)
class ClosedLoopMetrics:
    speed_mps: float
    final_lateral_error_m: float
    final_heading_error_rad: float
    max_abs_lateral_error_m: float
    rms_lateral_error_m: float
    max_abs_heading_error_rad: float
    rms_heading_error_rad: float
    max_abs_steering_rad: float
    max_abs_steering_rate_radps: float
    max_abs_feedback_rad: float
    rms_feedback_rad: float


def current_git_commit() -> str:
    result = subprocess.run(["git", "rev-parse", "HEAD"], check=True, capture_output=True, text=True)
    return result.stdout.strip()


def rms(values: list[float]) -> float:
    if not values:
        raise ValueError("RMS input must not be empty.")
    return math.sqrt(sum(value * value for value in values) / len(values))


def compute_safe_speed(path, target_speed_mps: float) -> float:
    return compute_constant_safe_speed(
        path=path,
        target_speed_mps=target_speed_mps,
        wheelbase_m=MPC_WHEELBASE_M,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )


def run_closed_loop(path, initial_vehicle_state, speed_mps: float) -> ClosedLoopMetrics:
    if speed_mps < MIN_REFERENCE_SPEED_MPS:
        raise ValueError(f"Reference speed is too small: {speed_mps:.6f} m/s")

    state = replace(initial_vehicle_state, speed=speed_mps)
    initial_reference = match_path_reference(path=path, vehicle_state=state, start_segment_index=0)
    controller = CurvatureAwareMPCController(wheelbase_m=MPC_WHEELBASE_M, dt=MPC_DT)
    controller.previous_steering_angle = curvature_to_steering(initial_reference.curvature, MPC_WHEELBASE_M)

    segment_index = initial_reference.segment_index
    previous_reference_s = initial_reference.s
    previous_steering = controller.previous_steering_angle
    lateral_errors: list[float] = []
    heading_errors: list[float] = []
    feedback_values: list[float] = []
    max_abs_steering = abs(previous_steering)
    max_abs_steering_rate = 0.0
    final_lateral_error = math.inf
    final_heading_error = math.inf
    final_reference_s = initial_reference.s
    reached_end = False

    path_length = path[-1].s - path[0].s
    max_steps = math.ceil((path_length / speed_mps + EXTRA_SIMULATION_TIME_S) / MPC_DT)

    for _ in range(max_steps):
        reference = match_path_reference(path=path, vehicle_state=state, start_segment_index=segment_index)
        if reference.s + 1e-9 < previous_reference_s:
            raise AssertionError("Reference progress moved backward.")

        segment_index = reference.segment_index
        previous_reference_s = reference.s
        lateral_error, heading_error = compute_path_tracking_error(vehicle_state=state, reference=reference)
        steering_result = controller.compute_steering_components(
            lateral_error=lateral_error,
            heading_error=heading_error,
            speed_mps=state.speed,
            reference_curvature=reference.curvature,
        )
        steering = steering_result.steering_angle
        steering_rate = (steering - previous_steering) / MPC_DT

        lateral_errors.append(lateral_error)
        heading_errors.append(heading_error)
        feedback_values.append(steering_result.feedback_angle)
        max_abs_steering = max(max_abs_steering, abs(steering))
        max_abs_steering_rate = max(max_abs_steering_rate, abs(steering_rate))

        state = update_vehicle_state(
            state=state,
            command=ControlCommand(acceleration=0.0, steering_angle=steering),
            wheelbase_m=MPC_WHEELBASE_M,
            dt=MPC_DT,
        )
        previous_steering = steering
        final_lateral_error = lateral_error
        final_heading_error = heading_error
        final_reference_s = reference.s

        if path[-1].s - reference.s <= END_PROGRESS_TOLERANCE_M:
            reached_end = True
            break

    if not reached_end:
        raise AssertionError(f"Vehicle did not reach path end: matched_s={final_reference_s:.3f}, path_end_s={path[-1].s:.3f}")
    if abs(final_lateral_error) > LATERAL_FINAL_TOLERANCE_M:
        raise AssertionError(f"Final lateral error too large: {final_lateral_error:.6f} m")
    if abs(final_heading_error) > HEADING_FINAL_TOLERANCE_RAD:
        raise AssertionError(f"Final heading error too large: {final_heading_error:.6f} rad")
    if max_abs_steering > MPC_MAX_STEERING_ANGLE + STEERING_TOLERANCE_RAD:
        raise AssertionError(f"Steering-angle constraint violated: {max_abs_steering:.6f} rad")
    if max_abs_steering_rate > MPC_MAX_STEERING_RATE + STEERING_RATE_TOLERANCE_RADPS:
        raise AssertionError(f"Steering-rate constraint violated: {max_abs_steering_rate:.6f} rad/s")
    if abs(state.speed - speed_mps) >= 1e-12:
        raise AssertionError("Vehicle speed changed during constant-speed validation.")

    return ClosedLoopMetrics(
        speed_mps=speed_mps,
        final_lateral_error_m=final_lateral_error,
        final_heading_error_rad=final_heading_error,
        max_abs_lateral_error_m=max(abs(value) for value in lateral_errors),
        rms_lateral_error_m=rms(lateral_errors),
        max_abs_heading_error_rad=max(abs(value) for value in heading_errors),
        rms_heading_error_rad=rms(heading_errors),
        max_abs_steering_rad=max_abs_steering,
        max_abs_steering_rate_radps=max_abs_steering_rate,
        max_abs_feedback_rad=max(abs(value) for value in feedback_values),
        rms_feedback_rad=rms(feedback_values),
    )


def planner_row(scenario, planner_name: str, result, metrics) -> dict:
    row = {
        "scenario": scenario.name,
        "planner": planner_name,
        "planner_success": result.success,
        "failure_reason": result.failure_reason,
        "planning_time_ms": metrics.planning_time_ms,
        "canonical_objective": metrics.canonical_matched_objective_value,
        "max_abs_curvature_per_m": metrics.max_abs_cartesian_curvature_per_m,
        "rms_curvature_per_m": metrics.rms_cartesian_curvature_per_m,
        "rms_curvature_rate_per_m2": metrics.rms_curvature_rate_per_m2,
        "safe_speed_mps": None,
        "safe_speed_ratio": None,
        "system_tracking_success": False,
        "system_tracking_failure_reason": None,
        "common_speed_mps": None,
        "same_speed_tracking_success": False,
        "same_speed_tracking_failure_reason": None,
    }

    if not result.success or result.path is None:
        return row

    safe_speed = compute_safe_speed(result.path, scenario.vehicle_state.speed)
    row["safe_speed_mps"] = safe_speed
    row["safe_speed_ratio"] = safe_speed / scenario.vehicle_state.speed if scenario.vehicle_state.speed > 0.0 else None

    try:
        system_metrics = run_closed_loop(result.path, scenario.vehicle_state, safe_speed)
    except Exception as error:
        row["system_tracking_failure_reason"] = f"{type(error).__name__}: {error}"
    else:
        row["system_tracking_success"] = True
        for key, value in asdict(system_metrics).items():
            row[f"system_{key}"] = value

    return row


def add_same_speed_result(row: dict, result, scenario, common_speed: float) -> None:
    row["common_speed_mps"] = common_speed
    if not result.success or result.path is None:
        return
    try:
        metrics = run_closed_loop(result.path, scenario.vehicle_state, common_speed)
    except Exception as error:
        row["same_speed_tracking_failure_reason"] = f"{type(error).__name__}: {error}"
    else:
        row["same_speed_tracking_success"] = True
        for key, value in asdict(metrics).items():
            row[f"same_speed_{key}"] = value


def aggregate(rows: list[dict], planner: str) -> dict:
    planner_rows = [row for row in rows if row["planner"] == planner]
    planner_successes = [row for row in planner_rows if row["planner_success"]]
    system_successes = [row for row in planner_rows if row["system_tracking_success"]]

    def average(key: str):
        values = [row.get(key) for row in system_successes if row.get(key) is not None]
        return mean(values) if values else None

    return {
        "scenario_count": len(planner_rows),
        "planner_success_count": len(planner_successes),
        "system_tracking_success_count": len(system_successes),
        "planner_failure_scenarios": [row["scenario"] for row in planner_rows if not row["planner_success"]],
        "system_tracking_failure_scenarios": [row["scenario"] for row in planner_rows if row["planner_success"] and not row["system_tracking_success"]],
        "mean_safe_speed_mps": average("safe_speed_mps"),
        "mean_safe_speed_ratio": average("safe_speed_ratio"),
        "mean_system_rms_lateral_error_m": average("system_rms_lateral_error_m"),
        "mean_system_rms_heading_error_rad": average("system_rms_heading_error_rad"),
        "mean_system_max_abs_feedback_rad": average("system_max_abs_feedback_rad"),
    }


def write_csv(rows: list[dict]) -> None:
    fieldnames = sorted({key for row in rows for key in row})
    with CSV_PATH.open("w", newline="", encoding="utf-8") as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    scenarios = build_final_evaluation_scenarios()
    rows: list[dict] = []

    print("Frozen final Planning & Control benchmark")
    print("No tuning is performed. Longitudinal speed-transition dynamics are outside scope.")

    for scenario in scenarios:
        print("\n" + "=" * 78)
        print(scenario.name)
        print("=" * 78)
        run = run_benchmark_scenario(scenario)

        sampling_row = planner_row(scenario, "sampling", run.sampling_result, run.sampling_metrics)
        optimization_row = planner_row(scenario, "optimization", run.optimization_result, run.optimization_metrics)

        if run.sampling_result.success and run.optimization_result.success:
            sampling_safe = sampling_row["safe_speed_mps"]
            optimization_safe = optimization_row["safe_speed_mps"]
            if sampling_safe is not None and optimization_safe is not None:
                common_speed = min(sampling_safe, optimization_safe)
                add_same_speed_result(sampling_row, run.sampling_result, scenario, common_speed)
                add_same_speed_result(optimization_row, run.optimization_result, scenario, common_speed)

        rows.extend((sampling_row, optimization_row))

        for row in (sampling_row, optimization_row):
            if not row["planner_success"]:
                print(f"{row['planner']:12s} PLANNER FAIL  {row['failure_reason']}")
                continue
            status = "PASS" if row["system_tracking_success"] else "TRACK FAIL"
            print(
                f"{row['planner']:12s} {status:10s} "
                f"plan={row['planning_time_ms']:.2f} ms  "
                f"v_safe={row['safe_speed_mps']:.3f} m/s  "
                f"rms_y={row.get('system_rms_lateral_error_m', math.nan):.4f} m  "
                f"rms_yaw={row.get('system_rms_heading_error_rad', math.nan):.4f} rad"
            )

    write_csv(rows)

    summary = {
        "git_commit": current_git_commit(),
        "methodology": {
            "scenario_suite": "frozen 9-scenario evaluation suite; previously inspected during diagnosis, not pristine unseen held-out",
            "primary_system_comparison": "each planner path tracked at its own constant safe speed",
            "auxiliary_comparison": "same-speed tracking on common-success scenarios",
            "longitudinal_speed_transition_dynamics": "outside current scope",
            "steering_rate_limit_note": "configured baseline MPC/vehicle parameter, not claimed as a measured actuator limit",
        },
        "sampling": aggregate(rows, "sampling"),
        "optimization": aggregate(rows, "optimization"),
        "common_success_scenarios": [
            scenario.name for scenario in scenarios
            if any(row["scenario"] == scenario.name and row["planner"] == "sampling" and row["planner_success"] for row in rows)
            and any(row["scenario"] == scenario.name and row["planner"] == "optimization" and row["planner_success"] for row in rows)
        ],
        "rows": rows,
    }

    SUMMARY_JSON_PATH.write_text(json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")

    print("\n" + "=" * 78)
    print("SUMMARY")
    print("=" * 78)
    for planner in ("sampling", "optimization"):
        data = summary[planner]
        print(
            f"{planner:12s} planner={data['planner_success_count']}/{data['scenario_count']}  "
            f"closed-loop={data['system_tracking_success_count']}/{data['scenario_count']}  "
            f"mean safe speed={data['mean_safe_speed_mps']:.3f} m/s"
            if data["mean_safe_speed_mps"] is not None
            else f"{planner:12s} no successful closed-loop runs"
        )

    print(f"\nCSV: {CSV_PATH}")
    print(f"Summary: {SUMMARY_JSON_PATH}")


if __name__ == "__main__":
    main()
