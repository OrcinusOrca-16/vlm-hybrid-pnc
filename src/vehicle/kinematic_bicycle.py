import math

from src.common.types import ControlCommand, VehicleState


# Kinematic bicycle model (rear-axle reference):
#
# Continuous model:
#   x_dot   = v * cos(yaw)
#   y_dot   = v * sin(yaw)
#   yaw_dot = v / L * tan(delta)
#   v_dot   = a
#
# Geometry:
#   tan(delta) = L / R
#   yaw_dot = v / R
#           = v / L * tan(delta)
#
# Forward Euler:
#   state_next = state + state_dot * dt


def update_vehicle_state(
    state: VehicleState,
    command: ControlCommand,
    wheelbase_m: float = 2.7,
    dt: float = 0.1,
) -> VehicleState:
    """Update the vehicle state by one time step."""

    if wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive.")

    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    if state.speed < 0.0:
        raise ValueError("vehicle speed must not be negative.")

    x_dot = state.speed * math.cos(state.yaw)
    y_dot = state.speed * math.sin(state.yaw)
    yaw_dot = (
        state.speed
        / wheelbase_m
        * math.tan(command.steering_angle)
    )
    speed_dot = command.acceleration

    next_x = state.x + x_dot * dt
    next_y = state.y + y_dot * dt
    next_yaw = state.yaw + yaw_dot * dt
    next_speed = max(
        0.0,
        state.speed + speed_dot * dt,
    )

    return VehicleState(
        x=next_x,
        y=next_y,
        yaw=next_yaw,
        speed=next_speed,
    )