"""Development validation for curvature-aware lateral MPC."""

import math

from src.common.types import (
    ControlCommand,
    PathPoint,
    VehicleState,
)
from src.control.curvature_aware_mpc import (
    CurvatureAwareMPCController,
)
from src.control.lateral_mpc import MPCController
from src.control.mpc_config import (
    MPC_MAX_STEERING_ANGLE,
    MPC_MAX_STEERING_RATE,
    MPC_WHEELBASE_M,
)
from src.control.path_tracking import (
    compute_constant_safe_speed,
    curvature_to_steering,
)
from src.vehicle.kinematic_bicycle import (
    update_vehicle_state,
)


DT = 0.1
SPEED_MPS = 5.0
SIMULATION_STEPS = 80

STEERING_TOLERANCE = 1e-3
# OSQP numerical tolerance, consistent with the existing MPC calibration check.
RATE_TOLERANCE = 0.02


def normalize_angle(
    angle: float,
) -> float:
    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def build_path_from_steering(
    steering_angles: list[float],
    spacing_m: float = 1.0,
) -> list[PathPoint]:
    """Build a synthetic path from a steering sequence."""

    points = []

    for index, steering in enumerate(
        steering_angles
    ):
        curvature = (
            math.tan(steering)
            / MPC_WHEELBASE_M
        )

        points.append(
            PathPoint(
                x=index * spacing_m,
                y=0.0,
                yaw=0.0,
                curvature=curvature,
                s=index * spacing_m,
            )
        )

    return points


def validate_speed_rule() -> None:
    """Validate constant steering-rate-limited speed selection."""

    straight_path = build_path_from_steering(
        [0.0, 0.0, 0.0]
    )

    speed = compute_constant_safe_speed(
        path=straight_path,
        target_speed_mps=8.0,
        wheelbase_m=MPC_WHEELBASE_M,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )

    assert abs(speed - 8.0) < 1e-9

    # Steering changes by 0.1 rad per meter.
    # With 0.5 rad/s steering-rate limit:
    #   v_max = 0.5 / 0.1 = 5.0 m/s
    steering_ramp_path = build_path_from_steering(
        [0.0, 0.1, 0.2]
    )

    speed = compute_constant_safe_speed(
        path=steering_ramp_path,
        target_speed_mps=8.0,
        wheelbase_m=MPC_WHEELBASE_M,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )

    assert abs(speed - 5.0) < 1e-9

    speed = compute_constant_safe_speed(
        path=steering_ramp_path,
        target_speed_mps=4.0,
        wheelbase_m=MPC_WHEELBASE_M,
        max_steering_angle=MPC_MAX_STEERING_ANGLE,
        max_steering_rate=MPC_MAX_STEERING_RATE,
    )

    assert abs(speed - 4.0) < 1e-9

    print("constant safe-speed rule: PASS")


def validate_straight_equivalence() -> None:
    """Zero reference curvature must reproduce the frozen baseline MPC."""

    cases = [
        (0.0, 0.0, 5.0),
        (0.5, 0.0, 5.0),
        (0.0, 0.05, 10.0),
        (0.5, 0.05, 10.0),
    ]

    for lateral_error, heading_error, speed_mps in cases:
        baseline = MPCController()
        curvature_aware = CurvatureAwareMPCController()

        baseline_steering = baseline.compute_steering(
            lateral_error=lateral_error,
            heading_error=heading_error,
            speed_mps=speed_mps,
        )

        curvature_aware_steering = curvature_aware.compute_steering(
            lateral_error=lateral_error,
            heading_error=heading_error,
            speed_mps=speed_mps,
            reference_curvature=0.0,
        )

        assert abs(
            curvature_aware_steering - baseline_steering
        ) < 1e-9

    print("zero-curvature baseline equivalence: PASS")


def validate_curved_equilibrium(
    reference_curvature: float,
) -> None:
    """At zero tracking error, steering should equal feedforward."""

    controller = CurvatureAwareMPCController()

    feedforward = curvature_to_steering(
        curvature_per_m=reference_curvature,
        wheelbase_m=MPC_WHEELBASE_M,
    )

    # Start this isolated equilibrium check at the matching steady steering.
    controller.previous_steering_angle = feedforward

    steering = controller.compute_steering(
        lateral_error=0.0,
        heading_error=0.0,
        speed_mps=SPEED_MPS,
        reference_curvature=reference_curvature,
    )

    assert abs(
        steering - feedforward
    ) < STEERING_TOLERANCE

    print(
        "curved equilibrium "
        f"kappa={reference_curvature:+.3f}: "
        "PASS "
        f"delta_ff={feedforward:+.4f}"
    )


def project_to_circle(
    x: float,
    y: float,
    curvature: float,
) -> tuple[float, float, float]:
    """Project a point onto a constant-curvature reference circle."""

    if abs(curvature) < 1e-12:
        return x, 0.0, 0.0

    signed_radius = 1.0 / curvature
    center_x = 0.0
    center_y = signed_radius

    dx = x - center_x
    dy = y - center_y
    distance = math.hypot(dx, dy)

    if distance < 1e-12:
        raise ValueError("Vehicle is at the circle center.")

    radius = abs(signed_radius)
    reference_x = center_x + radius * dx / distance
    reference_y = center_y + radius * dy / distance

    radial_angle = math.atan2(
        reference_y - center_y,
        reference_x - center_x,
    )

    turn_sign = 1.0 if curvature > 0.0 else -1.0
    reference_yaw = normalize_angle(
        radial_angle + turn_sign * math.pi / 2.0
    )

    return reference_x, reference_y, reference_yaw


def compute_circle_tracking_error(
    state: VehicleState,
    reference_curvature: float,
) -> tuple[float, float]:
    """Compute signed lateral and heading errors to the reference circle."""

    (
        reference_x,
        reference_y,
        reference_yaw,
    ) = project_to_circle(
        x=state.x,
        y=state.y,
        curvature=reference_curvature,
    )

    error_x = state.x - reference_x
    error_y = state.y - reference_y

    normal_x = -math.sin(reference_yaw)
    normal_y = math.cos(reference_yaw)

    lateral_error = (
        error_x * normal_x
        + error_y * normal_y
    )

    heading_error = normalize_angle(
        state.yaw - reference_yaw
    )

    return lateral_error, heading_error


def run_curved_recovery_case(
    name: str,
    reference_curvature: float,
    initial_lateral_error: float,
    initial_heading_error: float,
) -> None:
    """Run one nonlinear closed-loop curved-path recovery case."""

    controller = CurvatureAwareMPCController(
        wheelbase_m=MPC_WHEELBASE_M,
        dt=DT,
    )

    feedforward = curvature_to_steering(
        curvature_per_m=reference_curvature,
        wheelbase_m=MPC_WHEELBASE_M,
    )

    controller.previous_steering_angle = feedforward

    # The reference starts at x=0, y=0, yaw=0. Its left normal is +y.
    state = VehicleState(
        x=0.0,
        y=initial_lateral_error,
        yaw=initial_heading_error,
        speed=SPEED_MPS,
        curvature=reference_curvature,
    )

    previous_steering = feedforward
    max_abs_steering = 0.0
    max_abs_steering_rate = 0.0

    for _ in range(SIMULATION_STEPS):
        lateral_error, heading_error = compute_circle_tracking_error(
            state=state,
            reference_curvature=reference_curvature,
        )

        steering = controller.compute_steering(
            lateral_error=lateral_error,
            heading_error=heading_error,
            speed_mps=state.speed,
            reference_curvature=reference_curvature,
        )

        steering_rate = (
            steering - previous_steering
        ) / DT

        max_abs_steering = max(
            max_abs_steering,
            abs(steering),
        )
        max_abs_steering_rate = max(
            max_abs_steering_rate,
            abs(steering_rate),
        )

        state = update_vehicle_state(
            state=state,
            command=ControlCommand(
                acceleration=0.0,
                steering_angle=steering,
            ),
            wheelbase_m=MPC_WHEELBASE_M,
            dt=DT,
        )

        previous_steering = steering

    final_lateral_error, final_heading_error = (
        compute_circle_tracking_error(
            state=state,
            reference_curvature=reference_curvature,
        )
    )

    assert max_abs_steering <= (
        MPC_MAX_STEERING_ANGLE
        + STEERING_TOLERANCE
    ), f"{name}: steering-angle constraint violated."

    assert max_abs_steering_rate <= (
        MPC_MAX_STEERING_RATE
        + RATE_TOLERANCE
    ), f"{name}: steering-rate constraint violated."

    assert abs(final_lateral_error) < 0.05, (
        f"{name}: lateral error did not converge."
    )

    assert abs(final_heading_error) < 0.02, (
        f"{name}: heading error did not converge."
    )

    print(
        f"{name:28s} "
        f"final_y={final_lateral_error:+.5f} "
        f"final_yaw={final_heading_error:+.5f} "
        f"max_delta={max_abs_steering:.5f} "
        f"max_rate={max_abs_steering_rate:.5f}"
    )


def main() -> None:
    validate_speed_rule()
    validate_straight_equivalence()

    validate_curved_equilibrium(
        reference_curvature=0.05
    )
    validate_curved_equilibrium(
        reference_curvature=-0.05
    )

    run_curved_recovery_case(
        name="left curved recovery",
        reference_curvature=0.05,
        initial_lateral_error=0.5,
        initial_heading_error=0.05,
    )

    run_curved_recovery_case(
        name="right curved recovery",
        reference_curvature=-0.05,
        initial_lateral_error=-0.5,
        initial_heading_error=-0.05,
    )

    print(
        "\nCurvature-aware MPC development validation passed."
    )


if __name__ == "__main__":
    main()
