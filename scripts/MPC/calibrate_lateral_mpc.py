"""Evaluate baseline lateral MPC performance across multiple test cases."""

import math

from src.common.types import VehicleState, ControlCommand
from src.control.lateral_mpc import MPCController
from src.control.mpc_config import (
    MPC_HORIZON,
    MPC_MAX_STEERING_ANGLE,
    MPC_MAX_STEERING_RATE,
    MPC_Q_HEADING,
    MPC_Q_LATERAL,
    MPC_R_STEERING,
)
from src.vehicle.kinematic_bicycle import update_vehicle_state


DT = 0.1
WHEELBASE_M = 2.7
SIMULATION_STEPS = 50

# Allow a small numerical tolerance from the QP solver.
RATE_TOLERANCE = 0.02


def compute_rms(values: list[float]) -> float:
    """Compute root-mean-square value."""

    return math.sqrt(
        sum(value ** 2 for value in values)
        / len(values)
    )


def run_case(
    name: str,
    speed_mps: float,
    initial_y: float,
    initial_yaw: float,
) -> dict[str, float]:
    """Run one closed-loop MPC calibration case."""

    if speed_mps <= 0.0:
        raise ValueError("speed_mps must be positive.")

    controller = MPCController(
        horizon=MPC_HORIZON,
        wheelbase_m=WHEELBASE_M,
        dt=DT,
        q_lateral=MPC_Q_LATERAL,
        q_heading=MPC_Q_HEADING,
        r_steering=MPC_R_STEERING,
        min_steering_angle=-MPC_MAX_STEERING_ANGLE,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )

    state = VehicleState(
        x=0.0,
        y=initial_y,
        yaw=initial_yaw,
        speed=speed_mps,
    )

    previous_steering = 0.0

    lateral_errors = []
    heading_errors = []
    steering_angles = []
    steering_rates = []

    for _ in range(SIMULATION_STEPS):
        steering = controller.compute_steering(
            lateral_error=state.y,
            heading_error=state.yaw,
            speed_mps=state.speed,
        )

        steering_rate = (
            steering - previous_steering
        ) / DT

        # Log all quantities at the same simulation instant,
        # before updating the vehicle state.
        lateral_errors.append(state.y)
        heading_errors.append(state.yaw)
        steering_angles.append(steering)
        steering_rates.append(steering_rate)

        command = ControlCommand(
            acceleration=0.0,
            steering_angle=steering,
        )

        state = update_vehicle_state(
            state=state,
            command=command,
            wheelbase_m=WHEELBASE_M,
            dt=DT,
        )

        previous_steering = steering

    metrics = {
        "speed": speed_mps,
        "lateral_rmse": compute_rms(lateral_errors),
        "heading_rmse": compute_rms(heading_errors),
        "steering_rms": compute_rms(steering_angles),
        "max_steering": max(
            abs(value) for value in steering_angles
        ),
        "max_steering_rate": max(
            abs(value) for value in steering_rates
        ),
        "final_y": state.y,
        "final_yaw": state.yaw,
    }

    # Hard constraints should never be meaningfully violated.
    assert metrics["max_steering"] <= (
        MPC_MAX_STEERING_ANGLE + 1e-6
    ), f"{name}: steering-angle constraint violated."

    assert metrics["max_steering_rate"] <= (
        MPC_MAX_STEERING_RATE + RATE_TOLERANCE
    ), f"{name}: steering-rate constraint violated."

    return metrics


def print_result(
    name: str,
    metrics: dict[str, float],
) -> None:
    """Print one calibration result."""

    print(
        f"{name:12s} "
        f"{metrics['speed']:5.1f} "
        f"{metrics['lateral_rmse']:9.4f} "
        f"{metrics['heading_rmse']:10.4f} "
        f"{metrics['steering_rms']:10.4f} "
        f"{metrics['max_steering']:10.4f} "
        f"{metrics['max_steering_rate']:9.4f} "
        f"{metrics['final_y']:9.4f} "
        f"{metrics['final_yaw']:10.4f}"
    )


def main() -> None:
    print("Baseline MPC configuration:")
    print(f"  horizon       = {MPC_HORIZON}")
    print(f"  Q lateral     = {MPC_Q_LATERAL}")
    print(f"  Q heading     = {MPC_Q_HEADING}")
    print(f"  R steering    = {MPC_R_STEERING}")
    print(f"  steering lim  = ±{MPC_MAX_STEERING_ANGLE} rad")
    print(f"  rate lim      = ±{MPC_MAX_STEERING_RATE} rad/s")
    print()

    print(
        f"{'case':12s} "
        f"{'speed':>5s} "
        f"{'y_rmse':>9s} "
        f"{'yaw_rmse':>10s} "
        f"{'steer_rms':>10s} "
        f"{'max_steer':>10s} "
        f"{'max_rate':>9s} "
        f"{'final_y':>9s} "
        f"{'final_yaw':>10s}"
    )

    print("-" * 96)

    test_cases = [
        ("lateral", 1.0, 0.0),
        ("heading", 0.0, 0.1),
        ("combined", 1.0, 0.1),
    ]

    speeds = [
        5.0,
        10.0,
        15.0,
    ]

    for name, initial_y, initial_yaw in test_cases:
        for speed in speeds:
            metrics = run_case(
                name=name,
                speed_mps=speed,
                initial_y=initial_y,
                initial_yaw=initial_yaw,
            )

            print_result(
                name=name,
                metrics=metrics,
            )

    print("\nBaseline MPC calibration run completed.")


if __name__ == "__main__":
    main()
