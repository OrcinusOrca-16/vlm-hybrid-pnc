import math

from src.common.types import PathPoint


def curvature_to_steering(
    curvature_per_m: float,
    wheelbase_m: float,
) -> float:
    """Convert path curvature to kinematic-bicycle steering angle."""

    if wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive.")
    if not math.isfinite(curvature_per_m):
        raise ValueError("curvature_per_m must be finite.")

    return math.atan(wheelbase_m * curvature_per_m)


def compute_constant_safe_speed(
    path: list[PathPoint],
    target_speed_mps: float,
    wheelbase_m: float,
    max_steering_angle: float,
    max_steering_rate: float,
    numerical_tolerance: float = 1e-9,
) -> float:
    """Return one steering-rate-limited constant speed for the whole path.

    Planning already constrains path curvature from the vehicle steering-angle
    limit. The angle check here is therefore a consistency assertion. Speed
    reduction addresses steering-rate demand through

        delta_dot ~= v * d(delta)/ds,

    with ``delta = atan(L * kappa)``. The most restrictive path segment sets
    the constant speed for the complete selected path.
    """

    if not path:
        raise ValueError("path must contain at least one point.")
    if target_speed_mps < 0.0:
        raise ValueError("target_speed_mps must not be negative.")
    if wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive.")
    if max_steering_angle <= 0.0:
        raise ValueError("max_steering_angle must be positive.")
    if max_steering_rate <= 0.0:
        raise ValueError("max_steering_rate must be positive.")
    if numerical_tolerance <= 0.0:
        raise ValueError("numerical_tolerance must be positive.")

    steering_angles: list[float] = []

    for point in path:
        steering = curvature_to_steering(
            curvature_per_m=point.curvature,
            wheelbase_m=wheelbase_m,
        )

        if abs(steering) > max_steering_angle + numerical_tolerance:
            raise ValueError(
                "Path requires steering outside the configured "
                "steering-angle limit."
            )

        steering_angles.append(steering)

    if len(path) == 1 or target_speed_mps == 0.0:
        return target_speed_mps

    max_abs_dsteering_ds = 0.0

    for index in range(len(path) - 1):
        delta_s = path[index + 1].s - path[index].s

        if delta_s <= 0.0:
            raise ValueError("PathPoint.s must be strictly increasing.")

        dsteering_ds = (
            steering_angles[index + 1] - steering_angles[index]
        ) / delta_s

        max_abs_dsteering_ds = max(
            max_abs_dsteering_ds,
            abs(dsteering_ds),
        )

    if max_abs_dsteering_ds <= numerical_tolerance:
        return target_speed_mps

    steering_rate_speed_limit = (
        max_steering_rate / max_abs_dsteering_ds
    )

    return min(target_speed_mps, steering_rate_speed_limit)
