"""Run the complete Planning & Control closed-loop simulation."""

# from src.scenario.basic_scenarios import create_normal_driving_scenario
# from src.scenario.basic_scenarios import create_lead_vehicle_slowdown_scenario
from src.scenario.basic_scenarios import create_pedestrian_crossing_scenario

from src.behavior.rule_based import decide_behavior
from src.planning.trajectory_generator import generate_straight_trajectory
from src.common.types import ControlCommand
from src.control.pid import PIDController
from src.vehicle.kinematic_bicycle import update_vehicle_state
from src.control.lateral_mpc import MPCController

DT = 0.1
MAX_ACCELERATION = 2.0
MAX_DECELERATION = 3.0


def main() -> None:
    # ego_state, scenario = create_normal_driving_scenario()
    # ego_state, scenario = create_lead_vehicle_slowdown_scenario()
    ego_state, scenario = create_pedestrian_crossing_scenario()

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
    min_output=-3.0,
    max_output=2.0,
    )

    mpc = MPCController(
        horizon=10,
        wheelbase_m=2.7,
        dt=DT,
        q_lateral=10.0,
        q_heading=1.0,
        r_steering=1.0,
        min_steering_angle=-0.5,
        max_steering_angle=0.5,
        max_steering_rate=0.5,
    )

    state = ego_state

    print("behavior:", behavior_command.behavior.value)
    print("target speed:", behavior_command.target_speed_mps)

    print("\nFirst 5 trajectory points:")

    for point in trajectory[:5]:
        print(
            f"t={point.time:.1f}, "
            f"x={point.x:.3f}, "
            f"y={point.y:.3f}, "
            f"yaw={point.yaw:.3f}, "
            f"speed={point.speed:.3f}"
        )

    print("\nP&C closed-loop:")

    for step, reference_point in enumerate(trajectory):
        speed_error = reference_point.speed - state.speed
        lateral_error = state.y - reference_point.y
        heading_error = state.yaw - reference_point.yaw

        if step < len(trajectory) - 1:
            next_reference_point = trajectory[step + 1]

            reference_acceleration = (
                next_reference_point.speed - reference_point.speed
            ) / DT
        else:
            reference_acceleration = 0.0

        feedback_acceleration  = pid.update(
            error=speed_error,
            dt=DT,
        )

        raw_acceleration = (
            reference_acceleration
            + feedback_acceleration
        )

        acceleration = max(
            -MAX_DECELERATION,
            min(raw_acceleration, MAX_ACCELERATION),
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

        print(
            f"t={reference_point.time: .1f}, "
            f"x={state.x: .3f}, "
            f"v_ref={reference_point.speed: .3f}, "
            f"v={state.speed: .3f}, "
            f"error={speed_error: .3f}, "
            f"a_ref={reference_acceleration: .3f}, "
            f"a_fb={feedback_acceleration: .3f}, "
            f"a_cmd={acceleration: .3f}, "
            f"steering_cmd={steering: .4f}, "
            f"e_y={lateral_error: .4f}, "
            f"e_yaw={heading_error: .4f}"
        )

        state = update_vehicle_state(
            state=state,
            command=command,
            wheelbase_m=2.7,
            dt=DT,
        )


if __name__ == "__main__":
    main()