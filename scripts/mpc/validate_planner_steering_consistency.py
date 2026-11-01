"""Development-only planner-to-steering consistency validation.

Checks the full geometric/control interface for Sampling and Optimization paths:

    curvature -> feedforward steering -> spatial steering gradient
              -> constant safe speed -> executed total steering

Only tuning/development scenarios are used. The frozen evaluation suite is not
built or run. This script does not modify planner or MPC parameters.
"""

import math

from scripts.mpc.compare_planner_path_tracking import run_closed_loop
from scripts.mpc.validate_optimization_path_tracking import run_optimization_planner
from scripts.mpc.validate_sampling_path_tracking import run_sampling_planner
from src.benchmark.tuning_scenario_suite import build_tuning_curved_obstacle_scenario, build_tuning_obstacle_early_scenario
from src.control.mpc_config import MPC_MAX_STEERING_ANGLE, MPC_MAX_STEERING_RATE, MPC_WHEELBASE_M
from src.control.path_tracking import compute_constant_safe_speed, curvature_to_steering, find_safe_speed_limiting_segment


ANGLE_TOLERANCE_RAD = 1e-9
RATE_EXECUTION_TOLERANCE_RADPS = 0.02
CURVATURE_TOLERANCE_PER_M = 1e-9


def validate_path(planner_name: str, result, scenario) -> None:
    if not result.success:
        raise AssertionError(f"{planner_name}: planner failed: {result.failure_reason}")
    if result.path is None:
        raise AssertionError(f"{planner_name}: successful planner result is missing a path.")

    path = result.path
    target_speed = scenario.vehicle_state.speed
    curvature_limit = math.tan(MPC_MAX_STEERING_ANGLE) / MPC_WHEELBASE_M
    curvatures = [point.curvature for point in path]
    feedforward = [curvature_to_steering(value, MPC_WHEELBASE_M) for value in curvatures]
    max_abs_curvature = max(abs(value) for value in curvatures)
    max_abs_feedforward = max(abs(value) for value in feedforward)

    assert max_abs_curvature <= curvature_limit + CURVATURE_TOLERANCE_PER_M, (
        f"{planner_name}: path curvature exceeds vehicle-derived steering-angle limit: "
        f"{max_abs_curvature:.9f} > {curvature_limit:.9f} 1/m"
    )
    assert max_abs_feedforward <= MPC_MAX_STEERING_ANGLE + ANGLE_TOLERANCE_RAD, (
        f"{planner_name}: feedforward steering exceeds configured steering-angle limit: "
        f"{max_abs_feedforward:.9f} > {MPC_MAX_STEERING_ANGLE:.9f} rad"
    )

    diagnostic = find_safe_speed_limiting_segment(
        path=path,
        wheelbase_m=MPC_WHEELBASE_M,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )
    safe_speed = compute_constant_safe_speed(
        path=path,
        target_speed_mps=target_speed,
        wheelbase_m=MPC_WHEELBASE_M,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )

    if diagnostic is None:
        max_abs_dsteering_ds = 0.0
        target_ff_rate = 0.0
        safe_ff_rate = 0.0
    else:
        max_abs_dsteering_ds = abs(diagnostic.dsteering_ds)
        target_ff_rate = target_speed * max_abs_dsteering_ds
        safe_ff_rate = safe_speed * max_abs_dsteering_ds
        assert safe_ff_rate <= MPC_MAX_STEERING_RATE + 1e-9, (
            f"{planner_name}: computed safe speed does not satisfy feedforward steering-rate limit: "
            f"{safe_ff_rate:.9f} > {MPC_MAX_STEERING_RATE:.9f} rad/s"
        )

    closed_loop = run_closed_loop(path, scenario.vehicle_state, safe_speed)
    assert closed_loop.max_abs_steering_rad <= MPC_MAX_STEERING_ANGLE + 1e-3, (
        f"{planner_name}: executed steering exceeds configured angle limit: "
        f"{closed_loop.max_abs_steering_rad:.9f} rad"
    )
    assert closed_loop.max_abs_steering_rate_radps <= MPC_MAX_STEERING_RATE + RATE_EXECUTION_TOLERANCE_RADPS, (
        f"{planner_name}: executed steering rate exceeds validation tolerance: "
        f"{closed_loop.max_abs_steering_rate_radps:.9f} rad/s"
    )

    target_rate_status = "PASS" if target_ff_rate <= MPC_MAX_STEERING_RATE + 1e-9 else "LIMITED"
    execution_excess = max(0.0, closed_loop.max_abs_steering_rate_radps - MPC_MAX_STEERING_RATE)

    print(f"  {planner_name:12s} PASS")
    print(
        f"    curvature: max={max_abs_curvature:.6f} 1/m, "
        f"vehicle limit={curvature_limit:.6f} 1/m"
    )
    print(
        f"    feedforward angle: max={max_abs_feedforward:.6f} rad, "
        f"limit={MPC_MAX_STEERING_ANGLE:.6f} rad"
    )
    print(
        f"    steering gradient: max={max_abs_dsteering_ds:.6f} rad/m, "
        f"target-speed FF rate={target_ff_rate:.6f} rad/s [{target_rate_status}]"
    )
    print(
        f"    safe speed: target={target_speed:.3f} m/s -> safe={safe_speed:.3f} m/s, "
        f"FF rate at safe speed={safe_ff_rate:.6f} rad/s"
    )
    print(
        f"    closed loop: max|delta_cmd|={closed_loop.max_abs_steering_rad:.6f} rad, "
        f"max|delta_dot_cmd|={closed_loop.max_abs_steering_rate_radps:.6f} rad/s, "
        f"rate excess={execution_excess:.6f} rad/s"
    )


def validate_scenario(scenario) -> None:
    print("\n" + "=" * 78)
    print(scenario.name)
    print("=" * 78)
    validate_path("Sampling", run_sampling_planner(scenario), scenario)
    validate_path("Optimization", run_optimization_planner(scenario), scenario)


def main() -> None:
    print("Development-only planner -> steering consistency validation")
    print("Frozen evaluation scenarios are NOT used here.")
    print(
        f"Limits: |delta| <= {MPC_MAX_STEERING_ANGLE:.3f} rad, "
        f"|delta_dot| <= {MPC_MAX_STEERING_RATE:.3f} rad/s"
    )

    for scenario in (build_tuning_obstacle_early_scenario(), build_tuning_curved_obstacle_scenario()):
        validate_scenario(scenario)

    print("\nPlanner curvature / steering-angle / steering-rate consistency passed.")


if __name__ == "__main__":
    main()
