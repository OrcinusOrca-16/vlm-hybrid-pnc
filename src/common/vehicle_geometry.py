import math


def curvature_from_steering_angle(
    steering_angle_rad: float,
    wheelbase_m: float,
) -> float:
    """Convert front-wheel steering angle to kinematic curvature."""

    if wheelbase_m <= 0.0:
        raise ValueError(
            "wheelbase_m must be positive."
        )

    return (
        math.tan(steering_angle_rad)
        / wheelbase_m
    )


def steering_angle_from_curvature(
    curvature_per_m: float,
    wheelbase_m: float,
) -> float:
    """Convert kinematic curvature to front-wheel steering angle."""

    if wheelbase_m <= 0.0:
        raise ValueError(
            "wheelbase_m must be positive."
        )

    return math.atan(
        wheelbase_m * curvature_per_m
    )
