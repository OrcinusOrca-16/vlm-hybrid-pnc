"""Development-only Sampling vs Optimization closed-loop comparison.

The primary comparison runs each planner path at its own steering-rate-derived
constant safe speed. A secondary same-speed diagnostic then runs both paths at
the lower of the two safe speeds to isolate path/controller differences.

Only development/tuning scenarios are used. The frozen evaluation suite is not
built or run here. Longitudinal speed-transition dynamics remain out of scope:
each simulation starts directly at the requested constant speed.
"""

from dataclasses import dataclass, replace
import math

from scripts.mpc.validate_optimization_path_tracking import run_optimization_planner
from scripts.mpc.validate_sampling_path_tracking import run_sampling_planner
from src.benchmark.metrics import PlannerBenchmarkMetrics, evaluate_planner_benchmark_metrics
from src.benchmark.tuning_scenario_suite import build_tuning_curved_obstacle_scenario, build_tuning_obstacle_early_scenario
from src.common.types import ControlCommand
from src.control.curvature_aware_mpc import CurvatureAwareMPCController
from src.control.mpc_config import MPC_DT, MPC_MAX_STEERING_ANGLE, MPC_MAX_STEERING_RATE, MPC_WHEELBASE_M
from src.control.path_tracking import compute_constant_safe_speed, compute_path_tracking_error, curvature_to_steering, match_path_reference
from src.vehicle.kinematic_bicycle import update_vehicle_state


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


def rms(values: list[float]) -> float:
    if not values:
        raise ValueError("RMS input must not be empty.")
    return math.sqrt(sum(value * value for value in values) / len(values))


def safe_speed(path, target_speed_mps: float) -> float:
    return compute_constant_safe_speed(
        path=path,
        target_speed_mps=target_speed_mps,
        wheelbase_m=MPC_WHEELBASE_M,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )


def evaluate_planner_metrics(result, scenario) -> PlannerBenchmarkMetrics:
    return evaluate_planner_benchmark_metrics(
        result=result,
        vehicle_params=scenario.vehicle_params,
        obstacles=list(scenario.obstacles),
        lateral_reference_m=scenario.lateral_reference_m,
        terminal_lateral_target_m=scenario.terminal_lateral_target_m,
        terminal_first_derivative_target=scenario.terminal_first_derivative_target,
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


def print_planner_table(sampling: PlannerBenchmarkMetrics, optimization: PlannerBenchmarkMetrics) -> None:
    rows = (
        ("planning time [ms]", sampling.planning_time_ms, optimization.planning_time_ms, ".3f"),
        ("canonical objective", sampling.canonical_matched_objective_value, optimization.canonical_matched_objective_value, ".6f"),
        ("max |curvature| [1/m]", sampling.max_abs_cartesian_curvature_per_m, optimization.max_abs_cartesian_curvature_per_m, ".6f"),
        ("RMS curvature [1/m]", sampling.rms_cartesian_curvature_per_m, optimization.rms_cartesian_curvature_per_m, ".6f"),
        ("RMS curvature rate [1/m^2]", sampling.rms_curvature_rate_per_m2, optimization.rms_curvature_rate_per_m2, ".6f"),
    )
    print(f"{'Planner metric':34s} {'Sampling':>14s} {'Optimization':>14s}")
    print("-" * 66)
    for name, sampling_value, optimization_value, spec in rows:
        s = "n/a" if sampling_value is None else format(sampling_value, spec)
        o = "n/a" if optimization_value is None else format(optimization_value, spec)
        print(f"{name:34s} {s:>14s} {o:>14s}")


def print_closed_loop_table(title: str, sampling: ClosedLoopMetrics, optimization: ClosedLoopMetrics) -> None:
    rows = (
        ("speed [m/s]", sampling.speed_mps, optimization.speed_mps, ".3f"),
        ("max |e_y| [m]", sampling.max_abs_lateral_error_m, optimization.max_abs_lateral_error_m, ".4f"),
        ("RMS e_y [m]", sampling.rms_lateral_error_m, optimization.rms_lateral_error_m, ".4f"),
        ("max |e_yaw| [rad]", sampling.max_abs_heading_error_rad, optimization.max_abs_heading_error_rad, ".4f"),
        ("RMS e_yaw [rad]", sampling.rms_heading_error_rad, optimization.rms_heading_error_rad, ".4f"),
        ("final e_y [m]", sampling.final_lateral_error_m, optimization.final_lateral_error_m, "+.4f"),
        ("final e_yaw [rad]", sampling.final_heading_error_rad, optimization.final_heading_error_rad, "+.4f"),
        ("max |steering| [rad]", sampling.max_abs_steering_rad, optimization.max_abs_steering_rad, ".4f"),
        ("max |steering rate| [rad/s]", sampling.max_abs_steering_rate_radps, optimization.max_abs_steering_rate_radps, ".4f"),
        ("max |feedback| [rad]", sampling.max_abs_feedback_rad, optimization.max_abs_feedback_rad, ".4f"),
        ("RMS feedback [rad]", sampling.rms_feedback_rad, optimization.rms_feedback_rad, ".4f"),
    )
    print(f"\n{title}")
    print(f"{'Closed-loop metric':34s} {'Sampling':>14s} {'Optimization':>14s}")
    print("-" * 66)
    for name, sampling_value, optimization_value, spec in rows:
        print(f"{name:34s} {format(sampling_value, spec):>14s} {format(optimization_value, spec):>14s}")


def compare_scenario(scenario) -> None:
    print("\n" + "=" * 78)
    print(scenario.name)
    print("=" * 78)

    sampling_result = run_sampling_planner(scenario)
    optimization_result = run_optimization_planner(scenario)
    if not sampling_result.success:
        raise AssertionError(f"Sampling Planner failed: {sampling_result.failure_reason}")
    if not optimization_result.success:
        raise AssertionError(f"Optimization Planner failed: {optimization_result.failure_reason}")
    if sampling_result.path is None or optimization_result.path is None:
        raise AssertionError("Successful planner result is missing a path.")

    sampling_planner_metrics = evaluate_planner_metrics(sampling_result, scenario)
    optimization_planner_metrics = evaluate_planner_metrics(optimization_result, scenario)
    print_planner_table(sampling_planner_metrics, optimization_planner_metrics)

    target_speed = scenario.vehicle_state.speed
    sampling_safe_speed = safe_speed(sampling_result.path, target_speed)
    optimization_safe_speed = safe_speed(optimization_result.path, target_speed)

    sampling_system = run_closed_loop(sampling_result.path, scenario.vehicle_state, sampling_safe_speed)
    optimization_system = run_closed_loop(optimization_result.path, scenario.vehicle_state, optimization_safe_speed)
    print_closed_loop_table("B. System-level comparison: each path uses its own safe speed", sampling_system, optimization_system)

    common_speed = min(sampling_safe_speed, optimization_safe_speed)
    sampling_common = run_closed_loop(sampling_result.path, scenario.vehicle_state, common_speed)
    optimization_common = run_closed_loop(optimization_result.path, scenario.vehicle_state, common_speed)
    print_closed_loop_table(f"A. Same-speed diagnostic: both paths at {common_speed:.3f} m/s", sampling_common, optimization_common)

    print(
        "\nSafe-speed diagnostic only (not a planner objective): "
        f"Sampling={sampling_safe_speed:.3f} m/s, Optimization={optimization_safe_speed:.3f} m/s, "
        f"target={target_speed:.3f} m/s"
    )


def main() -> None:
    print("Development-only Sampling vs Optimization Planning-Control comparison")
    print("Frozen evaluation scenarios are NOT used here.")
    print("Primary result = each planner path at its own safe speed; same-speed run = auxiliary diagnostic.")

    for scenario in (build_tuning_obstacle_early_scenario(), build_tuning_curved_obstacle_scenario()):
        compare_scenario(scenario)

    print("\nPlanner path tracking comparison passed.")


if __name__ == "__main__":
    main()
