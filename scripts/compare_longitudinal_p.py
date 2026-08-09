from src.common.types import ControlCommand, VehicleState
from src.control.pid import PIDController
from src.vehicle.kinematic_bicycle import update_vehicle_state


def run_simulation(
    kp: float,
    initial_speed: float = 0.0,
    target_speed: float = 10.0,
    dt: float = 0.1,
    simulation_time: float = 15.0,
) -> None:
    """Run one longitudinal P-control simulation."""

    if kp <= 0.0:
        raise ValueError("kp must be positive.")
    if initial_speed < 0.0:
        raise ValueError("initial_speed must not be negative.")
    if target_speed < 0.0:
        raise ValueError("target_speed must not be negative.")
    if dt <= 0.0:
        raise ValueError("dt must be positive.")
    if simulation_time <= 0.0:
        raise ValueError("simulation_time must be positive.")

    state = VehicleState(
        x=0.0,
        y=0.0,
        yaw=0.0,
        speed=initial_speed,
    )

    controller = PIDController(
        kp=kp,
        ki=0.0,
        kd=0.0,
        min_output=-3.0,
        max_output=2.0,
    )

    steps = int(simulation_time / dt)

    initial_error = abs(target_speed - initial_speed)
    tolerance = 0.02 * initial_error
    settling_time = None

    for i in range(steps):
        error = target_speed - state.speed

        acceleration = controller.update(
            error=error,
            dt=dt,
        )

        command = ControlCommand(
            acceleration=acceleration,
            steering_angle=0.0,
        )

        state = update_vehicle_state(
            state=state,
            command=command,
            wheelbase_m=2.7,
            dt=dt,
        )

        current_error = abs(target_speed - state.speed)

        if settling_time is None and current_error <= tolerance:
            settling_time = (i + 1) * dt

    final_error = target_speed - state.speed

    print(
        "kp:", kp,
        "settling_time:", round(settling_time, 2)
        if settling_time is not None
        else None,
        "final_speed:", round(state.speed, 3),
        "final_error:", round(final_error, 3),
    )


for kp in [0.5, 1.0, 2.0]:
    run_simulation(kp)