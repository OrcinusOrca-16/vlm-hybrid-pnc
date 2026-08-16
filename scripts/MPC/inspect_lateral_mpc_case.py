"""Inspect one lateral MPC closed-loop case in detail."""

import matplotlib.pyplot as plt

from src.common.types import VehicleState, ControlCommand
from src.control.lateral_mpc import MPCController
from src.vehicle.kinematic_bicycle import update_vehicle_state


DT = 0.1
WHEELBASE_M = 2.7
SIMULATION_STEPS = 50

MAX_STEERING_ANGLE = 0.5
MAX_STEERING_RATE = 0.5

MPC_HORIZON = 15
Q_LATERAL = 10.0
Q_HEADING = 1.0
R_STEERING = 1.0


def main() -> None:
    controller = MPCController(
        horizon=MPC_HORIZON,
        wheelbase_m=WHEELBASE_M,
        dt=DT,
        q_lateral=Q_LATERAL,
        q_heading=Q_HEADING,
        r_steering=R_STEERING,
        min_steering_angle=-MAX_STEERING_ANGLE,
        max_steering_angle=MAX_STEERING_ANGLE,
        max_steering_rate=MAX_STEERING_RATE,
    )

    # Problematic calibration case:
    # low speed + lateral error + heading error.
    state = VehicleState(
        x=0.0,
        y=1.0,
        yaw=0.1,
        speed=5.0,
    )

    previous_steering = 0.0

    times = []
    lateral_errors = []
    heading_errors = []
    steering_angles = []
    steering_rates = []

    for step in range(SIMULATION_STEPS):
        time = step * DT

        steering = controller.compute_steering(
            lateral_error=state.y,
            heading_error=state.yaw,
            speed_mps=state.speed,
        )

        steering_rate = (
            steering - previous_steering
        ) / DT

        times.append(time)
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

    print("Final state:")
    print(f"  y        = {state.y:.4f} m")
    print(f"  yaw      = {state.yaw:.4f} rad")
    print(
        f"  max steer = "
        f"{max(abs(v) for v in steering_angles):.4f} rad"
    )
    print(
        f"  max rate  = "
        f"{max(abs(v) for v in steering_rates):.4f} rad/s"
    )

    plt.figure(figsize=(9, 8))

    plt.suptitle(
        "Lateral MPC Inspection\n"
        f"Horizon={MPC_HORIZON}, "
        f"Qy={Q_LATERAL}, "
        f"Qyaw={Q_HEADING}, "
        f"R={R_STEERING}, "
        f"RateLimit={MAX_STEERING_RATE} rad/s, "
        f"Speed={state.speed:.1f} m/s",
    )

    plt.subplot(3, 1, 1)
    plt.plot(times, lateral_errors)
    plt.axhline(0.0, linestyle="--")
    plt.ylabel("Lateral error [m]")
    plt.grid(True)

    plt.subplot(3, 1, 2)
    plt.plot(times, heading_errors)
    plt.axhline(0.0, linestyle="--")
    plt.ylabel("Heading error [rad]")
    plt.grid(True)

    plt.subplot(3, 1, 3)
    plt.plot(times, steering_angles, label="steering")
    plt.axhline(
        MAX_STEERING_ANGLE,
        linestyle="--",
        label="angle limit",
    )
    plt.axhline(
        -MAX_STEERING_ANGLE,
        linestyle="--",
    )
    plt.xlabel("Time [s]")
    plt.ylabel("Steering [rad]")
    plt.grid(True)
    plt.legend()

    plt.tight_layout(rect=[0.0, 0.0, 1.0, 0.94])
    plt.savefig(
        "results/mpc_combined_5ms_inspection.png",
        dpi=150,
    )
    print(
        "Saved plot to "
        "results/mpc_combined_5ms_inspection.png"
    )

if __name__ == "__main__":
    main()