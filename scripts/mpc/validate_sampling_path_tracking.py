"""Development-only closed-loop validation for Sampling path tracking.

This script uses tuning/development scenarios only. It does not build or run the
frozen final evaluation suite.

Longitudinal speed-transition dynamics are intentionally outside the current
scope: each case starts directly at the constant safe reference speed computed
from the selected path and then runs with zero acceleration.
"""

import math
from dataclasses import replace

from src.benchmark.sampling_adapter import run_sampling_benchmark_adapter
from src.benchmark.tuning_scenario_suite import build_tuning_curved_obstacle_scenario, build_tuning_obstacle_early_scenario
from src.common.types import ControlCommand
from src.config.planning import DEFAULT_SAMPLING_PLANNER_CONFIG
from src.control.curvature_aware_mpc import CurvatureAwareMPCController
from src.control.mpc_config import MPC_DT, MPC_MAX_STEERING_ANGLE, MPC_MAX_STEERING_RATE, MPC_WHEELBASE_M
from src.control.path_tracking import (
    compute_constant_safe_speed,
    compute_path_tracking_error,
    curvature_to_steering,
    find_safe_speed_limiting_segment,
    match_path_reference,
)
from src.vehicle.kinematic_bicycle import update_vehicle_state


LATERAL_FINAL_TOLERANCE_M = 0.20
HEADING_FINAL_TOLERANCE_RAD = 0.10
STEERING_TOLERANCE_RAD = 1e-3
STEERING_RATE_TOLERANCE_RADPS = 0.02
END_PROGRESS_TOLERANCE_M = 0.25
EXTRA_SIMULATION_TIME_S = 5.0
MIN_REFERENCE_SPEED_MPS = 0.1


def run_sampling_planner(scenario):
    """Run the frozen Sampling Planner configuration on one dev scenario."""

    return run_sampling_benchmark_adapter(
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
        planner_objective_weights=DEFAULT_SAMPLING_PLANNER_CONFIG.matched_objective_weights,
    )


def print_speed_limit_diagnostic(path) -> None:
    diagnostic = find_safe_speed_limiting_segment(
        path=path,
        wheelbase_m=MPC_WHEELBASE_M,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )

    if diagnostic is None:
        print("  speed limit: no path segment")
        return

    print(
        "  speed-limiting segment: "
        f"index={diagnostic.segment_index}, "
        f"s={diagnostic.start_s:.3f}->{diagnostic.end_s:.3f} m, "
        f"kappa={diagnostic.start_curvature:+.5f}->{diagnostic.end_curvature:+.5f} 1/m, "
        f"delta_ff={diagnostic.start_steering:+.5f}->{diagnostic.end_steering:+.5f} rad, "
        f"d(delta_ff)/ds={diagnostic.dsteering_ds:+.5f} rad/m, "
        f"v_limit={diagnostic.speed_limit_mps:.3f} m/s"
    )

    start = max(0, diagnostic.segment_index - 2)
    stop = min(len(path), diagnostic.segment_index + 4)
    print("  local path samples:")
    for index in range(start, stop):
        steering = curvature_to_steering(path[index].curvature, MPC_WHEELBASE_M)
        marker = " <-- limiting start" if index == diagnostic.segment_index else ""
        print(
            f"    i={index:3d} s={path[index].s:7.3f} "
            f"kappa={path[index].curvature:+.6f} "
            f"delta_ff={steering:+.6f}{marker}"
        )


def run_closed_loop_case(scenario) -> None:
    """Plan once with Sampling, then track the selected spatial path."""

    result = run_sampling_planner(scenario)
    assert result.success, f"{scenario.name}: Sampling Planner failed: {result.failure_reason}"
    assert result.path is not None
    path = result.path

    reference_speed = compute_constant_safe_speed(
        path=path,
        target_speed_mps=scenario.vehicle_state.speed,
        wheelbase_m=MPC_WHEELBASE_M,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )
    assert reference_speed >= MIN_REFERENCE_SPEED_MPS, (
        f"{scenario.name}: computed reference speed is too small: {reference_speed:.6f} m/s"
    )

    state = replace(scenario.vehicle_state, speed=reference_speed)
    initial_reference = match_path_reference(path=path, vehicle_state=state, start_segment_index=0)
    controller = CurvatureAwareMPCController(wheelbase_m=MPC_WHEELBASE_M, dt=MPC_DT)
    controller.previous_steering_angle = curvature_to_steering(initial_reference.curvature, MPC_WHEELBASE_M)

    segment_index = initial_reference.segment_index
    previous_steering = controller.previous_steering_angle
    previous_reference_s = initial_reference.s

    max_abs_lateral_error = 0.0
    max_abs_heading_error = 0.0
    max_abs_steering = abs(previous_steering)
    max_abs_steering_rate = 0.0
    max_abs_feedforward = 0.0
    max_abs_feedback = 0.0
    sum_abs_feedback = 0.0
    sum_squared_feedback = 0.0
    feedback_sample_count = 0
    peak_feedback_snapshot = None

    final_lateral_error = math.inf
    final_heading_error = math.inf
    final_reference_s = initial_reference.s

    path_length = path[-1].s - path[0].s
    max_steps = math.ceil((path_length / reference_speed + EXTRA_SIMULATION_TIME_S) / MPC_DT)
    reached_end = False

    for step in range(max_steps):
        reference = match_path_reference(path=path, vehicle_state=state, start_segment_index=segment_index)
        assert reference.s + 1e-9 >= previous_reference_s, f"{scenario.name}: reference progress moved backward."

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

        max_abs_lateral_error = max(max_abs_lateral_error, abs(lateral_error))
        max_abs_heading_error = max(max_abs_heading_error, abs(heading_error))
        max_abs_steering = max(max_abs_steering, abs(steering))
        max_abs_steering_rate = max(max_abs_steering_rate, abs(steering_rate))
        max_abs_feedforward = max(max_abs_feedforward, abs(steering_result.feedforward_angle))

        abs_feedback = abs(steering_result.feedback_angle)
        sum_abs_feedback += abs_feedback
        sum_squared_feedback += steering_result.feedback_angle**2
        feedback_sample_count += 1

        if abs_feedback > max_abs_feedback:
            max_abs_feedback = abs_feedback
            peak_feedback_snapshot = {
                "step": step,
                "s": reference.s,
                "kappa": reference.curvature,
                "lateral_error": lateral_error,
                "heading_error": heading_error,
                "feedforward": steering_result.feedforward_angle,
                "feedback": steering_result.feedback_angle,
                "total": steering,
                "steering_rate": steering_rate,
            }

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

    assert reached_end, (
        f"{scenario.name}: vehicle did not reach the end of the selected path; "
        f"matched_s={final_reference_s:.3f}, path_end_s={path[-1].s:.3f}."
    )
    assert abs(final_lateral_error) <= LATERAL_FINAL_TOLERANCE_M, (
        f"{scenario.name}: final lateral error too large: {final_lateral_error:.6f} m"
    )
    assert abs(final_heading_error) <= HEADING_FINAL_TOLERANCE_RAD, (
        f"{scenario.name}: final heading error too large: {final_heading_error:.6f} rad"
    )
    assert max_abs_steering <= MPC_MAX_STEERING_ANGLE + STEERING_TOLERANCE_RAD, (
        f"{scenario.name}: steering-angle constraint violated: {max_abs_steering:.6f} rad"
    )
    assert max_abs_steering_rate <= MPC_MAX_STEERING_RATE + STEERING_RATE_TOLERANCE_RADPS, (
        f"{scenario.name}: steering-rate constraint violated: {max_abs_steering_rate:.6f} rad/s"
    )
    assert abs(state.speed - reference_speed) < 1e-12

    mean_abs_feedback = sum_abs_feedback / feedback_sample_count
    rms_feedback = math.sqrt(sum_squared_feedback / feedback_sample_count)

    print(
        f"{scenario.name:32s} PASS  "
        f"v_ref={reference_speed:.3f} m/s  "
        f"final_y={final_lateral_error:+.4f} m  "
        f"final_yaw={final_heading_error:+.4f} rad  "
        f"max|e_y|={max_abs_lateral_error:.4f} m  "
        f"max|e_yaw|={max_abs_heading_error:.4f} rad  "
        f"max|delta|={max_abs_steering:.4f} rad  "
        f"max|delta_dot|={max_abs_steering_rate:.4f} rad/s"
    )
    print(
        "  steering components: "
        f"max|ff|={max_abs_feedforward:.5f} rad, "
        f"max|fb|={max_abs_feedback:.5f} rad, "
        f"mean|fb|={mean_abs_feedback:.5f} rad, "
        f"rms(fb)={rms_feedback:.5f} rad"
    )

    if peak_feedback_snapshot is not None:
        peak = peak_feedback_snapshot
        print(
            "  peak feedback: "
            f"step={peak['step']}, s={peak['s']:.3f} m, "
            f"kappa={peak['kappa']:+.5f} 1/m, "
            f"e_y={peak['lateral_error']:+.5f} m, "
            f"e_yaw={peak['heading_error']:+.5f} rad, "
            f"ff={peak['feedforward']:+.5f} rad, "
            f"fb={peak['feedback']:+.5f} rad, "
            f"total={peak['total']:+.5f} rad, "
            f"delta_dot={peak['steering_rate']:+.5f} rad/s"
        )

    print_speed_limit_diagnostic(path)


def main() -> None:
    scenarios = (
        build_tuning_obstacle_early_scenario(),
        build_tuning_curved_obstacle_scenario(),
    )

    for scenario in scenarios:
        run_closed_loop_case(scenario)

    print("\nSampling path -> curvature-aware MPC development validation passed.")


if __name__ == "__main__":
    main()
