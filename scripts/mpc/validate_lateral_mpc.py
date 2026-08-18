"""Validate the lateral MPC controller in closed-loop simulation."""

from src.common.types import VehicleState, ControlCommand
from src.control.lateral_mpc import MPCController
from src.control.mpc_config import (
    MPC_MAX_STEERING_ANGLE,
    MPC_MAX_STEERING_RATE,
)
from src.vehicle.kinematic_bicycle import update_vehicle_state


DT = 0.1
WHEELBASE_M = 2.7
SIMULATION_STEPS = 30

# Small tolerance for numerical QP solver error.
RATE_TOLERANCE = 0.002


def run_case(
    name: str,
    initial_y: float,
    initial_yaw: float,
) -> None:
    """Run one lateral MPC closed-loop test case."""

    controller = MPCController(
        wheelbase_m=WHEELBASE_M,
        dt=DT,
    )

    state = VehicleState(
        x=0.0,
        y=initial_y,
        yaw=initial_yaw,
        speed=10.0,
    )

    previous_steering = 0.0
    max_steering_change = 0.0

    for _ in range(SIMULATION_STEPS):
        steering = controller.compute_steering(
            lateral_error=state.y,
            heading_error=state.yaw,
            speed_mps=state.speed,
        )

        steering_change = steering - previous_steering

        max_steering_change = max(
            max_steering_change,
            abs(steering_change),
        )

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

    expected_max_change = MPC_MAX_STEERING_RATE * DT

    assert abs(state.y) < 0.01, (
        f"{name}: lateral error did not converge."
    )

    assert abs(state.yaw) < 0.01, (
        f"{name}: heading error did not converge."
    )

    assert max_steering_change <= (
        expected_max_change + RATE_TOLERANCE
    ), (
        f"{name}: steering-rate constraint violated."
    )

    print(
        f"{name:24s} "
        f"final_y={state.y: .5f}, "
        f"final_yaw={state.yaw: .5f}, "
        f"max_delta={max_steering_change: .5f}"
    )


def main() -> None:
    run_case(
        name="zero error",
        initial_y=0.0,
        initial_yaw=0.0,
    )

    run_case(
        name="lateral error",
        initial_y=1.0,
        initial_yaw=0.0,
    )

    run_case(
        name="heading error",
        initial_y=0.0,
        initial_yaw=0.1,
    )

    run_case(
        name="combined error",
        initial_y=1.0,
        initial_yaw=0.1,
    )

    print("\nAll lateral MPC validation cases passed.")


if __name__ == "__main__":
    main()
