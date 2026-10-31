"""Development-only closed-loop validation for Sampling path tracking.

This script uses tuning/development scenarios only. It does not build or run the
frozen final evaluation suite.

Longitudinal speed-transition dynamics are intentionally outside the current
scope: each case starts directly at the constant safe reference speed computed
from the selected path and then runs with zero acceleration.
"""

import math
from dataclasses import replace

from src.benchmark.sampling_adapter import (
    run_sampling_benchmark_adapter,
)
from src.benchmark.tuning_scenario_suite import (
    build_tuning_curved_obstacle_scenario,
    build_tuning_obstacle_early_scenario,
)
from src.common.types import (
    ControlCommand,
    VehicleState,
)
from src.config.planning import (
    DEFAULT_SAMPLING_PLANNER_CONFIG,
)
from src.control.curvature_aware_mpc import (
    CurvatureAwareMPCController,
)
from src.control.mpc_config import (
    MPC_DT,
    MPC_MAX_STEERING_ANGLE,
    MPC_MAX_STEERING_RATE,
    MPC_WHEELBASE_M,
)
from src.control.path_tracking import (
    compute_constant_safe_speed,
    compute_path_tracking_error,
    curvature_to_steering,
    match_path_reference,
)
from src.vehicle.kinematic_bicycle import (
    update_vehicle_state,
)


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
        terminal_first_derivative_target=(
            scenario.terminal_first_derivative_target
        ),
        lateral_weight=scenario.lateral_weight,
        first_derivative_weight=scenario.first_derivative_weight,
        second_derivative_weight=scenario.second_derivative_weight,
        third_derivative_weight=scenario.third_derivative_weight,
        terminal_lateral_weight=scenario.terminal_lateral_weight,
        terminal_first_derivative_weight=(
            scenario.terminal_first_derivative_weight
        ),
        total_length=scenario.planning_horizon_m,
        final_lateral_positions=(
            scenario.terminal_lateral_target_m,
        ),
        final_dl_ds_values=(
            scenario.terminal_first_derivative_target,
        ),
        resolution_m=scenario.resolution_m,
        planner_objective_weights=(
            DEFAULT_SAMPLING_PLANNER_CONFIG
            .matched_objective_weights
        ),
    )


def run_closed_loop_case(scenario) -> None:
    """Plan once with Sampling, then track the selected spatial path."""

    result = run_sampling_planner(scenario)

    assert result.success, (
        f"{scenario.name}: Sampling Planner failed: "
        f"{result.failure_reason}"
    )
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
        f"{scenario.name}: computed reference speed is too small: "
        f"{reference_speed:.6f} m/s"
    )

    # Chapter 16 scope decision:
    # start directly at the constant safe path speed and keep acceleration zero.
    state = replace(
        scenario.vehicle_state,
        speed=reference_speed,
    )

    initial_reference = match_path_reference(
        path=path,
        vehicle_state=state,
        start_segment_index=0,
    )

    controller = CurvatureAwareMPCController(
        wheelbase_m=MPC_WHEELBASE_M,
        dt=MPC_DT,
    )

    controller.previous_steering_angle = curvature_to_steering(
        curvature_per_m=initial_reference.curvature,
        wheelbase_m=MPC_WHEELBASE_M,
    )

    segment_index = initial_reference.segment_index
    previous_steering = controller.previous_steering_angle

    max_abs_lateral_error = 0.0
    max_abs_heading_error = 0.0
    max_abs_steering = abs(previous_steering)
    max_abs_steering_rate = 0.0
    previous_reference_s = initial_reference.s

    final_lateral_error = math.inf
    final_heading_error = math.inf
    final_reference_s = initial_reference.s

    path_length = path[-1].s - path[0].s
    max_steps = math.ceil(
        (
            path_length / reference_speed
            + EXTRA_SIMULATION_TIME_S
        ) / MPC_DT
    )

    reached_end = False

    for _ in range(max_steps):
        reference = match_path_reference(
            path=path,
            vehicle_state=state,
            start_segment_index=segment_index,
        )

        assert reference.s + 1e-9 >= previous_reference_s, (
            f"{scenario.name}: reference progress moved backward."
        )

        segment_index = reference.segment_index
        previous_reference_s = reference.s

        lateral_error, heading_error = compute_path_tracking_error(
            vehicle_state=state,
            reference=reference,
        )

        steering = controller.compute_steering(
            lateral_error=lateral_error,
            heading_error=heading_error,
            speed_mps=state.speed,
            reference_curvature=reference.curvature,
        )

        steering_rate = (
            steering - previous_steering
        ) / MPC_DT

        max_abs_lateral_error = max(
            max_abs_lateral_error,
            abs(lateral_error),
        )
        max_abs_heading_error = max(
            max_abs_heading_error,
            abs(heading_error),
        )
        max_abs_steering = max(
            max_abs_steering,
            abs(steering),
        )
        max_abs_steering_rate = max(
            max_abs_steering_rate,
            abs(steering_rate),
        )

        state = update_vehicle_state(
            state=state,
            command=ControlCommand(
                acceleration=0.0,
                steering_angle=steering,
            ),
            wheelbase_m=MPC_WHEELBASE_M,
            dt=MPC_DT,
        )

        previous_steering = steering
        final_lateral_error = lateral_error
        final_heading_error = heading_error
        final_reference_s = reference.s

        if (
            path[-1].s - reference.s
            <= END_PROGRESS_TOLERANCE_M
        ):
            reached_end = True
            break

    assert reached_end, (
        f"{scenario.name}: vehicle did not reach the end of the selected path; "
        f"matched_s={final_reference_s:.3f}, path_end_s={path[-1].s:.3f}."
    )

    assert abs(final_lateral_error) <= LATERAL_FINAL_TOLERANCE_M, (
        f"{scenario.name}: final lateral error too large: "
        f"{final_lateral_error:.6f} m"
    )

    assert abs(final_heading_error) <= HEADING_FINAL_TOLERANCE_RAD, (
        f"{scenario.name}: final heading error too large: "
        f"{final_heading_error:.6f} rad"
    )

    assert max_abs_steering <= (
        MPC_MAX_STEERING_ANGLE
        + STEERING_TOLERANCE_RAD
    ), (
        f"{scenario.name}: steering-angle constraint violated: "
        f"{max_abs_steering:.6f} rad"
    )

    assert max_abs_steering_rate <= (
        MPC_MAX_STEERING_RATE
        + STEERING_RATE_TOLERANCE_RADPS
    ), (
        f"{scenario.name}: steering-rate constraint violated: "
        f"{max_abs_steering_rate:.6f} rad/s"
    )

    assert abs(state.speed - reference_speed) < 1e-12

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


def main() -> None:
    scenarios = (
        build_tuning_obstacle_early_scenario(),
        build_tuning_curved_obstacle_scenario(),
    )

    for scenario in scenarios:
        run_closed_loop_case(scenario)

    print(
        "\nSampling path -> curvature-aware MPC development validation passed."
    )


if __name__ == "__main__":
    main()
