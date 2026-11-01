import math
from itertools import product

from scripts.mpc.validate_sampling_path_tracking import run_sampling_planner
from src.benchmark.tuning_scenario_suite import build_tuning_obstacle_early_scenario
from src.config.planning import DEFAULT_SAMPLING_PLANNER_CONFIG
from src.control.mpc_config import MPC_MAX_STEERING_ANGLE, MPC_MAX_STEERING_RATE, MPC_WHEELBASE_M
from src.control.path_tracking import compute_constant_safe_speed, find_safe_speed_limiting_segment
from src.planning.frenet import frenet_trajectory_to_path
from src.planning.hard_constraints import is_candidate_feasible
from src.planning.path_objective import evaluate_frenet_trajectory_objective
from src.planning.sampling_planner import generate_returning_frenet_candidates_from_vehicle_state


EPS = 1e-9


def selection_cost(trajectory, scenario) -> float:
    return evaluate_frenet_trajectory_objective(
        trajectory=trajectory,
        resolution_m=scenario.resolution_m,
        lateral_reference_m=scenario.lateral_reference_m,
        terminal_lateral_target_m=scenario.terminal_lateral_target_m,
        terminal_first_derivative_target=scenario.terminal_first_derivative_target,
        weights=DEFAULT_SAMPLING_PLANNER_CONFIG.matched_objective_weights,
    ).total_cost


def path_diagnostics(path, target_speed_mps: float):
    safe_speed = compute_constant_safe_speed(
        path=path,
        target_speed_mps=target_speed_mps,
        wheelbase_m=MPC_WHEELBASE_M,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )

    limiting = find_safe_speed_limiting_segment(
        path=path,
        wheelbase_m=MPC_WHEELBASE_M,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )

    max_curvature = max(abs(point.curvature) for point in path)
    max_dsteering_ds = 0.0 if limiting is None else abs(limiting.dsteering_ds)

    return safe_speed, max_curvature, max_dsteering_ds


def main():
    scenario = build_tuning_obstacle_early_scenario()
    cfg = DEFAULT_SAMPLING_PLANNER_CONFIG
    target_speed = scenario.vehicle_state.speed

    trajectories = generate_returning_frenet_candidates_from_vehicle_state(
        vehicle_state=scenario.vehicle_state,
        reference_line=scenario.reference_line,
        bypass_lateral_positions=cfg.returning_bypass_lateral_positions_m,
        bypass_lengths=cfg.returning_bypass_lengths_m,
        total_length=scenario.planning_horizon_m,
        final_lateral_positions=(scenario.terminal_lateral_target_m,),
        final_dl_ds_values=(scenario.terminal_first_derivative_target,),
        final_d2l_ds2=cfg.returning_final_d2l_ds2,
        resolution=scenario.resolution_m,
        junction_dl_ds_values=cfg.returning_junction_dl_ds_values,
    )

    metadata = list(product(
        cfg.returning_bypass_lengths_m,
        cfg.returning_bypass_lateral_positions_m,
        (scenario.terminal_lateral_target_m,),
        (scenario.terminal_first_derivative_target,),
        cfg.returning_junction_dl_ds_values,
    ))

    assert len(trajectories) == len(metadata)

    rows = []

    for trajectory, (bypass_length, bypass_l, _, _, junction_dl_ds) in zip(trajectories, metadata):
        path = frenet_trajectory_to_path(
            trajectory=trajectory,
            reference_line=scenario.reference_line,
        )

        physically_feasible = is_candidate_feasible(
            trajectory=trajectory,
            path=path,
            vehicle_params=scenario.vehicle_params,
            obstacles=list(scenario.obstacles),
            max_curvature=scenario.max_curvature,
            minimum_clearance_m=scenario.minimum_clearance_m,
            min_l=scenario.road_min_l,
            max_l=scenario.road_max_l,
        )

        if not physically_feasible:
            continue

        safe_speed, max_curvature, max_dsteering_ds = path_diagnostics(path, target_speed)

        rows.append({
            "bypass_length": bypass_length,
            "bypass_l": bypass_l,
            "junction_dl_ds": junction_dl_ds,
            "cost": selection_cost(trajectory, scenario),
            "safe_speed": safe_speed,
            "trackable_at_target": safe_speed >= target_speed - EPS,
            "max_curvature": max_curvature,
            "max_dsteering_ds": max_dsteering_ds,
        })

    selected = run_sampling_planner(scenario)
    assert selected.success and selected.path is not None and selected.trajectory is not None

    selected_safe_speed, selected_max_curvature, selected_gradient = path_diagnostics(
        selected.path,
        target_speed,
    )

    target_trackable = [row for row in rows if row["trackable_at_target"]]

    print(f"Scenario: {scenario.name}")
    print(f"Target speed: {target_speed:.3f} m/s")
    print(f"Total candidates: {len(trajectories)}")
    print(f"Physically feasible candidates: {len(rows)}")
    print(f"{target_speed:.1f} m/s steering-rate-feasible candidates: {len(target_trackable)}")

    print("\nCurrent selected Sampling path")
    print(
        f"  selection cost={selection_cost(selected.trajectory, scenario):.6f}, "
        f"safe speed={selected_safe_speed:.3f} m/s, "
        f"max|kappa|={selected_max_curvature:.6f} 1/m, "
        f"max|d(delta_ff)/ds|={selected_gradient:.6f} rad/m"
    )

    if target_trackable:
        best_trackable = min(target_trackable, key=lambda row: row["cost"])

        print(f"\nBEST {target_speed:.1f} m/s-trackable candidate by current Sampling cost")
        print(
            f"  bypass_l={best_trackable['bypass_l']:+.2f} m, "
            f"bypass_length={best_trackable['bypass_length']:.1f} m, "
            f"junction_dl_ds={best_trackable['junction_dl_ds']:+.3f}"
        )
        print(
            f"  selection cost={best_trackable['cost']:.6f}, "
            f"safe speed={best_trackable['safe_speed']:.3f} m/s, "
            f"max|kappa|={best_trackable['max_curvature']:.6f} 1/m, "
            f"max|d(delta_ff)/ds|={best_trackable['max_dsteering_ds']:.6f} rad/m"
        )
        print(
            f"  cost increase vs selected="
            f"{(best_trackable['cost'] / selection_cost(selected.trajectory, scenario) - 1.0) * 100.0:+.1f}%"
        )
    else:
        print(f"\nNO existing Sampling candidate is steering-rate-feasible at {target_speed:.1f} m/s.")

    print("\nTop physically feasible candidates by safe speed")
    for index, row in enumerate(sorted(rows, key=lambda item: (-item["safe_speed"], item["cost"]))[:10], start=1):
        print(
            f"  {index:2d}. v_safe={row['safe_speed']:.3f} m/s  "
            f"cost={row['cost']:.6f}  "
            f"bypass_l={row['bypass_l']:+.2f}  "
            f"bypass_len={row['bypass_length']:.1f}  "
            f"max|kappa|={row['max_curvature']:.5f}  "
            f"max|dδ/ds|={row['max_dsteering_ds']:.5f}"
        )

    if target_trackable:
        print("\nInterpretation: candidate generation already contains a target-speed-trackable solution.")
        print("The current geometric objective simply does not select it.")
    else:
        print("\nInterpretation: the current candidate family contains no target-speed-trackable solution.")
        print("Fixing this would require changing candidate generation and/or adding planner-level trackability design.")


if __name__ == "__main__":
    main()