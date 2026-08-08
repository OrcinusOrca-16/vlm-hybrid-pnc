import math

from src.common.types import (
    BehaviorCommand,
    TrajectoryPoint,
    VehicleState,
)

def calculate_stopping_distance(
    speed_mps: float,
    decel_mps2: float,
) -> float:
    """Calculate braking distance under constant deceleration."""

    if speed_mps < 0.0:
        raise ValueError("speed_mps must not be negative.")

    if decel_mps2 <= 0.0:
        raise ValueError("decel_mps2 must be positive.")

    return speed_mps**2 / (2.0 * decel_mps2)

def generate_straight_trajectory(
    ego_state: VehicleState,
    command: BehaviorCommand,
    dt: float = 0.1,
    horizon_s: float = 3.0,
    max_accel_mps2: float = 2.0,
    max_decel_mps2: float = 3.0,
) -> list[TrajectoryPoint]:
    """Generate a straight reference trajectory with acceleration limits."""

    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    if horizon_s <= 0.0:
        raise ValueError("horizon_s must be positive.")

    if max_accel_mps2 <= 0.0:
        raise ValueError("max_accel_mps2 must be positive.")

    if max_decel_mps2 <= 0.0:
        raise ValueError("max_decel_mps2 must be positive.")

    if command.target_speed_mps < 0.0:
        raise ValueError("target speed must not be negative.")

    num_steps = int(horizon_s / dt) + 1

    trajectory: list[TrajectoryPoint] = []

    x = ego_state.x
    y = ego_state.y
    speed = ego_state.speed

    for i in range(num_steps):
        time = i * dt

        trajectory.append(
            TrajectoryPoint(
                x=x,
                y=y,
                yaw=ego_state.yaw,
                speed=speed,
                time=time,
            )
        )

        old_speed = speed
        if speed < command.target_speed_mps:
            speed = min(
                speed + max_accel_mps2 * dt,
                command.target_speed_mps,
            )

        elif speed > command.target_speed_mps:
            speed = max(
                speed - max_decel_mps2 * dt,
                command.target_speed_mps,
            )

        average_speed = (old_speed + speed) / 2.0

        x += average_speed * math.cos(ego_state.yaw) * dt
        y += average_speed * math.sin(ego_state.yaw) * dt

    return trajectory