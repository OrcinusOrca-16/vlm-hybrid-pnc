"""Development-only experiment for Sampling bypass/return junction slope.

This script does not modify production planner logic and does not run held-out
or final-evaluation scenarios. It first reproduces the current production
winner for tuning_obstacle_longitudinal_18.00, then keeps that winner's bypass
lateral target and segment lengths fixed while sweeping only the shared
junction dl/ds. Junction d2l/ds2 stays fixed at zero.
"""

from dataclasses import dataclass
import math

from src.benchmark.sampling_adapter import run_sampling_benchmark_adapter
from src.benchmark.tuning_scenario_suite import build_tuning_obstacle_early_scenario
from src.common.types import FrenetTrajectory
from src.config.planning import DEFAULT_SAMPLING_PLANNER_CONFIG
from src.control.mpc_config import MPC_MAX_STEERING_ANGLE, MPC_MAX_STEERING_RATE, MPC_WHEELBASE_M
from src.control.path_tracking import compute_constant_safe_speed, find_safe_speed_limiting_segment
from src.planning.frenet import frenet_trajectory_to_path, vehicle_state_to_frenet
from src.planning.hard_constraints import (
    is_curvature_feasible,
    is_obstacle_clearance_feasible,
    is_road_boundary_feasible,
    obstacle_clearance,
)
from src.planning.quintic_polynomial import QuinticPolynomial
from src.planning.sampling_planner import generate_returning_frenet_candidate, sample_quintic_candidate
from src.planning.soft_cost import matched_total_cost

JUNCTION_DL_DS_VALUES = (-0.20, -0.15, -0.125, -0.10, -0.075, -0.05, -0.025, 0.00, 0.05, 0.10, 0.20)
TRACKABILITY_WEIGHTS = (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 10.0)


@dataclass(frozen=True)
class CandidateMetadata:
    bypass_l: float
    bypass_length: float
    return_length: float
    trajectory: FrenetTrajectory
    path: list
    selection_objective: float


@dataclass(frozen=True)
class CandidateEvaluation:
    junction_dl_ds: float
    feasible: bool
    geometric_cost: float
    safe_speed_mps: float | None
    speed_cost: float | None


def matched_selection_cost(trajectory, scenario) -> float:
    weights = DEFAULT_SAMPLING_PLANNER_CONFIG.matched_objective_weights
    return matched_total_cost(
        trajectory=trajectory,
        resolution_m=scenario.resolution_m,
        lateral_reference_m=scenario.lateral_reference_m,
        terminal_lateral_target_m=scenario.terminal_lateral_target_m,
        terminal_first_derivative_target=scenario.terminal_first_derivative_target,
        lateral_weight=weights.lateral_weight,
        first_derivative_weight=weights.first_derivative_weight,
        second_derivative_weight=weights.second_derivative_weight,
        third_derivative_weight=weights.third_derivative_weight,
        terminal_lateral_weight=weights.terminal_lateral_weight,
        terminal_first_derivative_weight=weights.terminal_first_derivative_weight,
    )


def physical_flags(trajectory, path, scenario) -> tuple[bool, bool, bool]:
    curvature_ok = is_curvature_feasible(path=path, max_curvature=scenario.max_curvature)
    clearance_ok = is_obstacle_clearance_feasible(
        path=path,
        vehicle_params=scenario.vehicle_params,
        obstacles=list(scenario.obstacles),
        minimum_clearance_m=scenario.minimum_clearance_m,
    )
    road_ok = is_road_boundary_feasible(
        trajectory=trajectory,
        vehicle_params=scenario.vehicle_params,
        min_l=scenario.road_min_l,
        max_l=scenario.road_max_l,
    )
    return curvature_ok, clearance_ok, road_ok


def minimum_clearance(path, scenario) -> float:
    if not scenario.obstacles:
        return math.inf
    return min(
        obstacle_clearance(path_point=point, vehicle_params=scenario.vehicle_params, obstacle=obstacle)
        for point in path
        for obstacle in scenario.obstacles
    )


def speed_trackability_cost(safe_speed_mps: float, target_speed_mps: float) -> float:
    if target_speed_mps <= 0.0:
        return 0.0
    normalized_deficit = max(0.0, target_speed_mps - safe_speed_mps) / target_speed_mps
    return normalized_deficit**2


def reproduce_production_winner(scenario) -> CandidateMetadata:
    initial = vehicle_state_to_frenet(vehicle_state=scenario.vehicle_state, reference_line=scenario.reference_line)
    config = DEFAULT_SAMPLING_PLANNER_CONFIG
    candidates = []

    for bypass_length in config.returning_bypass_lengths_m:
        return_length = scenario.planning_horizon_m - bypass_length
        if return_length <= 0.0:
            continue
        for bypass_l in config.returning_bypass_lateral_positions_m:
            trajectory = generate_returning_frenet_candidate(
                start_s=initial.s,
                start_l=initial.l,
                start_dl_ds=initial.dl_ds,
                start_d2l_ds2=initial.d2l_ds2,
                bypass_l=bypass_l,
                bypass_length=bypass_length,
                return_length=return_length,
                final_l=scenario.terminal_lateral_target_m,
                final_dl_ds=scenario.terminal_first_derivative_target,
                final_d2l_ds2=config.returning_final_d2l_ds2,
                resolution=scenario.resolution_m,
            )
            path = frenet_trajectory_to_path(trajectory=trajectory, reference_line=scenario.reference_line)
            if not all(physical_flags(trajectory, path, scenario)):
                continue
            candidates.append(
                CandidateMetadata(
                    bypass_l=bypass_l,
                    bypass_length=bypass_length,
                    return_length=return_length,
                    trajectory=trajectory,
                    path=path,
                    selection_objective=matched_selection_cost(trajectory, scenario),
                )
            )

    if not candidates:
        raise RuntimeError("No physically feasible production Sampling candidate found.")

    winner = min(candidates, key=lambda candidate: candidate.selection_objective)
    adapter_result = run_sampling_benchmark_adapter(
        junction_dl_ds_values=(0.0,),
        trackability_weight=0.0,
        reference_line=scenario.reference_line,
        vehicle_state=scenario.vehicle_state,
        vehicle_params=scenario.vehicle_params,
        obstacles=list(scenario.obstacles),
        road_min_l=scenario.road_min_l,
        road_max_l=scenario.road_max_l,
        max_curvature=scenario.max_curvature,
        minimum_clearance_m=scenario.minimum_clearance_m,
        lateral_reference_m=scenario.lateral_reference_m,
        terminal_lateral_target_m=scenario.terminal_lateral_target_m,
        terminal_first_derivative_target=scenario.terminal_first_derivative_target,
        lateral_weight=scenario.lateral_weight,
        first_derivative_weight=scenario.first_derivative_weight,
        second_derivative_weight=scenario.second_derivative_weight,
        third_derivative_weight=scenario.third_derivative_weight,
        terminal_lateral_weight=scenario.terminal_lateral_weight,
        terminal_first_derivative_weight=scenario.terminal_first_derivative_weight,
        total_length=scenario.planning_horizon_m,
        final_lateral_positions=(scenario.terminal_lateral_target_m,),
        final_dl_ds_values=(scenario.terminal_first_derivative_target,),
        resolution_m=scenario.resolution_m,
        planner_objective_weights=config.matched_objective_weights,
    )
    if not adapter_result.success or adapter_result.path is None:
        raise RuntimeError(f"Production Sampling adapter failed: {adapter_result.failure_reason}")
    if len(adapter_result.path) != len(winner.path):
        raise RuntimeError("Reproduced winner has different path length from production adapter.")

    max_path_difference = max(math.hypot(a.x - b.x, a.y - b.y) for a, b in zip(adapter_result.path, winner.path))
    if max_path_difference > 1e-9:
        raise RuntimeError(f"Failed to reproduce production winner; max path difference={max_path_difference:.3e} m")
    return winner


def build_fixed_geometry_candidate(scenario, winner: CandidateMetadata, junction_dl_ds: float):
    initial = vehicle_state_to_frenet(vehicle_state=scenario.vehicle_state, reference_line=scenario.reference_line)
    config = DEFAULT_SAMPLING_PLANNER_CONFIG

    bypass_curve = QuinticPolynomial(
        start_l=initial.l,
        start_dl_ds=initial.dl_ds,
        start_d2l_ds2=initial.d2l_ds2,
        end_l=winner.bypass_l,
        end_dl_ds=junction_dl_ds,
        end_d2l_ds2=0.0,
        length=winner.bypass_length,
    )
    bypass_trajectory = sample_quintic_candidate(
        curve=bypass_curve, start_s=initial.s, resolution=scenario.resolution_m
    )
    junction = bypass_trajectory.points[-1]

    return_curve = QuinticPolynomial(
        start_l=junction.l,
        start_dl_ds=junction.dl_ds,
        start_d2l_ds2=junction.d2l_ds2,
        end_l=scenario.terminal_lateral_target_m,
        end_dl_ds=scenario.terminal_first_derivative_target,
        end_d2l_ds2=config.returning_final_d2l_ds2,
        length=winner.return_length,
    )
    return_trajectory = sample_quintic_candidate(
        curve=return_curve, start_s=junction.s, resolution=scenario.resolution_m
    )
    trajectory = FrenetTrajectory(points=bypass_trajectory.points + return_trajectory.points[1:])
    path = frenet_trajectory_to_path(trajectory=trajectory, reference_line=scenario.reference_line)
    return trajectory, path


def evaluate_candidate(junction_dl_ds: float, trajectory, path, scenario) -> CandidateEvaluation:
    curvature_ok, clearance_ok, road_ok = physical_flags(trajectory, path, scenario)
    feasible = curvature_ok and clearance_ok and road_ok
    min_clearance = minimum_clearance(path, scenario)
    max_curvature = max(abs(point.curvature) for point in path)
    geometric_cost = matched_selection_cost(trajectory, scenario)

    if curvature_ok:
        limiting = find_safe_speed_limiting_segment(
            path=path,
            wheelbase_m=MPC_WHEELBASE_M,
            max_steering_angle=MPC_MAX_STEERING_ANGLE,
            max_steering_rate=MPC_MAX_STEERING_RATE,
        )
        safe_speed = compute_constant_safe_speed(
            path=path,
            target_speed_mps=scenario.vehicle_state.speed,
            wheelbase_m=MPC_WHEELBASE_M,
            max_steering_angle=MPC_MAX_STEERING_ANGLE,
            max_steering_rate=MPC_MAX_STEERING_RATE,
        )
        speed_cost = speed_trackability_cost(safe_speed, scenario.vehicle_state.speed)
        max_gradient_text = f"{0.0 if limiting is None else abs(limiting.dsteering_ds):.5f}"
        safe_speed_text = f"{safe_speed:6.3f}"
        speed_cost_text = f"{speed_cost:.6f}"
    else:
        safe_speed = None
        speed_cost = None
        max_gradient_text = "   n/a "
        safe_speed_text = "   n/a"
        speed_cost_text = "   n/a  "

    baseline = " <-- baseline" if abs(junction_dl_ds) < 1e-12 else ""
    print(
        f"junction_dl={junction_dl_ds:+.3f}  feasible={str(feasible):5s}  "
        f"curv={str(curvature_ok):5s} clear={str(clearance_ok):5s} road={str(road_ok):5s}  "
        f"min_clear={min_clearance:+.3f} m  max|kappa|={max_curvature:.5f}  "
        f"max|d_delta/ds|={max_gradient_text} rad/m  v_safe={safe_speed_text} m/s  "
        f"J_geo={geometric_cost:.6f}  J_speed={speed_cost_text}{baseline}"
    )
    return CandidateEvaluation(
        junction_dl_ds=junction_dl_ds,
        feasible=feasible,
        geometric_cost=geometric_cost,
        safe_speed_mps=safe_speed,
        speed_cost=speed_cost,
    )


def print_trackability_weight_sweep(evaluations: list[CandidateEvaluation]) -> None:
    feasible = [item for item in evaluations if item.feasible and item.speed_cost is not None]
    if not feasible:
        raise RuntimeError("No feasible candidate is available for trackability-cost ranking.")

    print("\n=== Trackability-weight winner sweep ===")
    for weight in TRACKABILITY_WEIGHTS:
        winner = min(feasible, key=lambda item: item.geometric_cost + weight * item.speed_cost)
        total_cost = winner.geometric_cost + weight * winner.speed_cost
        print(
            f"w_track={weight:>4.1f}  winner junction_dl={winner.junction_dl_ds:+.3f}  "
            f"v_safe={winner.safe_speed_mps:.3f} m/s  J_geo={winner.geometric_cost:.6f}  "
            f"J_speed={winner.speed_cost:.6f}  J_total={total_cost:.6f}"
        )


def main() -> None:
    scenario = build_tuning_obstacle_early_scenario()
    winner = reproduce_production_winner(scenario)

    print("=== Production winner reproduced ===")
    print(
        f"bypass_l={winner.bypass_l:+.3f} m, bypass_length={winner.bypass_length:.3f} m, "
        f"return_length={winner.return_length:.3f} m, selection_objective={winner.selection_objective:.6f}"
    )
    print("\n=== Fixed-geometry junction dl/ds sweep ===")

    evaluations = []
    for junction_dl_ds in JUNCTION_DL_DS_VALUES:
        trajectory, path = build_fixed_geometry_candidate(
            scenario=scenario, winner=winner, junction_dl_ds=junction_dl_ds
        )
        evaluations.append(evaluate_candidate(junction_dl_ds, trajectory, path, scenario))

    print_trackability_weight_sweep(evaluations)


if __name__ == "__main__":
    main()
