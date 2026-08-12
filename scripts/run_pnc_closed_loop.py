"""Run and evaluate the complete Planning & Control closed-loop simulation."""

from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from src.behavior.rule_based import decide_behavior
from src.common.types import ControlCommand
from src.control.lateral_mpc import MPCController
from src.control.pid import PIDController
from src.control.longitudinal_controller import LongitudinalController
from src.planning.trajectory_generator import generate_straight_trajectory
from src.scenario.basic_scenarios import (
    create_lead_vehicle_slowdown_scenario,
    create_normal_driving_scenario,
    create_pedestrian_crossing_scenario,
)
from src.vehicle.kinematic_bicycle import update_vehicle_state


DT = 0.1
WHEELBASE_M = 2.7

MAX_ACCELERATION = 2.0
MAX_DECELERATION = 3.0

RESULTS_DIR = Path("results")

# Options:
#   "normal"
#   "slowdown"
#   "stop"
SCENARIO_NAME = "stop"


def create_scenario(name: str):
    """Create one of the predefined Phase 1 scenarios."""

    if name == "normal":
        return create_normal_driving_scenario()

    if name == "slowdown":
        return create_lead_vehicle_slowdown_scenario()

    if name == "stop":
        return create_pedestrian_crossing_scenario()

    raise ValueError(f"Unsupported scenario: {name}")


def run_simulation():
    """Run the complete Planning & Control closed loop."""

    ego_state, scenario = create_scenario(SCENARIO_NAME)

    behavior_command = decide_behavior(
        ego_state=ego_state,
        scenario=scenario,
    )

    trajectory = generate_straight_trajectory(
        ego_state=ego_state,
        command=behavior_command,
        dt=DT,
        max_accel_mps2=MAX_ACCELERATION,
        max_decel_mps2=MAX_DECELERATION,
    )

    pid = PIDController(
        kp=1.0,
        ki=0.0,
        kd=0.0,
        min_output=-MAX_DECELERATION,
        max_output=MAX_ACCELERATION,
    )

    longitudinal_controller = LongitudinalController(
        pid=pid,
        dt=DT,
        max_acceleration=MAX_ACCELERATION,
        max_deceleration=MAX_DECELERATION,
    )

    mpc = MPCController(
        horizon=10,
        wheelbase_m=WHEELBASE_M,
        dt=DT,
        q_lateral=10.0,
        q_heading=1.0,
        r_steering=1.0,
        min_steering_angle=-0.5,
        max_steering_angle=0.5,
        max_steering_rate=0.5,
    )

    state = ego_state

    logs = {
        "time": [],
        "reference_speed": [],
        "actual_speed": [],
        "speed_error": [],
        "lateral_error": [],
        "heading_error": [],
        "acceleration": [],
        "steering": [],
    }

    for step, reference_point in enumerate(trajectory):
        # Tracking errors at the current time step.
        speed_error = reference_point.speed - state.speed
        lateral_error = state.y - reference_point.y
        heading_error = state.yaw - reference_point.yaw

        if step < len(trajectory) - 1:
            next_reference_speed = trajectory[step + 1].speed
        else:
            next_reference_speed = None

        acceleration = longitudinal_controller.compute_acceleration(
            current_speed=state.speed,
            reference_speed=reference_point.speed,
            next_reference_speed=next_reference_speed,
        )

        steering = mpc.compute_steering(
            lateral_error=lateral_error,
            heading_error=heading_error,
            speed_mps=state.speed,
        )

        command = ControlCommand(
            acceleration=acceleration,
            steering_angle=steering,
        )

        # Log values before state update so that all values
        # correspond to the same time step k.
        logs["time"].append(reference_point.time)
        logs["reference_speed"].append(reference_point.speed)
        logs["actual_speed"].append(state.speed)
        logs["speed_error"].append(speed_error)
        logs["lateral_error"].append(lateral_error)
        logs["heading_error"].append(heading_error)
        logs["acceleration"].append(acceleration)
        logs["steering"].append(steering)

        # Vehicle:
        #   x[k+1] = f(x[k], u[k])
        state = update_vehicle_state(
            state=state,
            command=command,
            wheelbase_m=WHEELBASE_M,
            dt=DT,
        )

    print("Scenario:", SCENARIO_NAME)
    print("Behavior:", behavior_command.behavior.value)
    print(
        "Target speed:",
        f"{behavior_command.target_speed_mps:.3f} m/s",
    )
    print(
        "Final speed:",
        f"{state.speed:.3f} m/s",
    )
    print(
        "Final position:",
        f"x={state.x:.3f} m, y={state.y:.3f} m",
    )

    return logs


def calculate_metrics(logs: dict) -> dict:
    """Calculate basic Phase 1 tracking and smoothness metrics."""

    speed_error = np.asarray(logs["speed_error"])
    lateral_error = np.asarray(logs["lateral_error"])
    heading_error = np.asarray(logs["heading_error"])
    steering = np.asarray(logs["steering"])

    # Speed tracking.
    speed_mae = np.mean(np.abs(speed_error))

    speed_rmse = np.sqrt(
        np.mean(speed_error ** 2)
    )

    speed_max = np.max(
        np.abs(speed_error)
    )

    # Lateral tracking.
    lateral_mae = np.mean(
        np.abs(lateral_error)
    )

    lateral_rmse = np.sqrt(
        np.mean(lateral_error ** 2)
    )

    lateral_max = np.max(
        np.abs(lateral_error)
    )

    # Heading tracking.
    heading_rmse = np.sqrt(
        np.mean(heading_error ** 2)
    )

    heading_max = np.max(
        np.abs(heading_error)
    )

    # Steering smoothness:
    #
    #   steering_rate[k]
    #       = (delta[k+1] - delta[k]) / dt
    if len(steering) > 1:
        steering_rate = np.diff(steering) / DT

        mean_abs_steering_rate = np.mean(
            np.abs(steering_rate)
        )

        max_abs_steering_rate = np.max(
            np.abs(steering_rate)
        )
    else:
        mean_abs_steering_rate = 0.0
        max_abs_steering_rate = 0.0

    return {
        "speed_mae": speed_mae,
        "speed_rmse": speed_rmse,
        "speed_max": speed_max,
        "lateral_mae": lateral_mae,
        "lateral_rmse": lateral_rmse,
        "lateral_max": lateral_max,
        "heading_rmse": heading_rmse,
        "heading_max": heading_max,
        "mean_abs_steering_rate": mean_abs_steering_rate,
        "max_abs_steering_rate": max_abs_steering_rate,
    }


def print_metrics(metrics: dict) -> None:
    """Print the main evaluation metrics."""

    print("\n=== Evaluation Metrics ===")

    print("\nSpeed tracking:")
    print(
        f"MAE:  {metrics['speed_mae']:.4f} m/s"
    )
    print(
        f"RMSE: {metrics['speed_rmse']:.4f} m/s"
    )
    print(
        f"MAX:  {metrics['speed_max']:.4f} m/s"
    )

    print("\nLateral tracking:")
    print(
        f"MAE:  {metrics['lateral_mae']:.4f} m"
    )
    print(
        f"RMSE: {metrics['lateral_rmse']:.4f} m"
    )
    print(
        f"MAX:  {metrics['lateral_max']:.4f} m"
    )

    print("\nHeading tracking:")
    print(
        f"RMSE: {metrics['heading_rmse']:.4f} rad"
    )
    print(
        f"MAX:  {metrics['heading_max']:.4f} rad"
    )

    print("\nSteering smoothness:")
    print(
        "Mean |steering rate|: "
        f"{metrics['mean_abs_steering_rate']:.4f} rad/s"
    )
    print(
        "Max |steering rate|:  "
        f"{metrics['max_abs_steering_rate']:.4f} rad/s"
    )


def save_summary_plot(logs: dict) -> None:
    """Save one summary figure for the current scenario."""

    RESULTS_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    time = logs["time"]

    figure, axes = plt.subplots(
        3,
        2,
        figsize=(12, 12),
    )

    # Speed tracking.
    axes[0, 0].plot(
        time,
        logs["reference_speed"],
        label="Reference",
    )

    axes[0, 0].plot(
        time,
        logs["actual_speed"],
        label="Actual",
    )

    axes[0, 0].set_title(
        "Speed Tracking"
    )
    axes[0, 0].set_ylabel(
        "Speed [m/s]"
    )
    axes[0, 0].legend()
    axes[0, 0].grid(True)

    # Speed error.
    axes[0, 1].plot(
        time,
        logs["speed_error"],
    )

    axes[0, 1].set_title(
        "Speed Tracking Error"
    )
    axes[0, 1].set_ylabel(
        "Error [m/s]"
    )
    axes[0, 1].grid(True)

    # Lateral error.
    axes[1, 0].plot(
        time,
        logs["lateral_error"],
    )

    axes[1, 0].set_title(
        "Lateral Tracking Error"
    )
    axes[1, 0].set_ylabel(
        "Error [m]"
    )
    axes[1, 0].grid(True)

    # Heading error.
    axes[1, 1].plot(
        time,
        logs["heading_error"],
    )

    axes[1, 1].set_title(
        "Heading Tracking Error"
    )
    axes[1, 1].set_ylabel(
        "Error [rad]"
    )
    axes[1, 1].grid(True)

    # Acceleration command.
    axes[2, 0].plot(
        time,
        logs["acceleration"],
    )

    axes[2, 0].set_title(
        "Acceleration Command"
    )
    axes[2, 0].set_xlabel(
        "Time [s]"
    )
    axes[2, 0].set_ylabel(
        "Acceleration [m/s^2]"
    )
    axes[2, 0].grid(True)

    # Steering command.
    axes[2, 1].plot(
        time,
        logs["steering"],
    )

    axes[2, 1].set_title(
        "Steering Command"
    )
    axes[2, 1].set_xlabel(
        "Time [s]"
    )
    axes[2, 1].set_ylabel(
        "Steering [rad]"
    )
    axes[2, 1].grid(True)

    figure.suptitle(
        f"P&C Evaluation - {SCENARIO_NAME}",
        fontsize=16,
    )

    figure.tight_layout()

    output_path = (
        RESULTS_DIR
        / f"{SCENARIO_NAME}_summary.png"
    )

    figure.savefig(
        output_path,
        dpi=150,
        bbox_inches="tight",
    )

    plt.close(figure)

    print(
        f"\nSaved summary figure: {output_path}"
    )


def main() -> None:
    logs = run_simulation()

    metrics = calculate_metrics(
        logs
    )

    print_metrics(
        metrics
    )

    save_summary_plot(
        logs
    )


if __name__ == "__main__":
    main()