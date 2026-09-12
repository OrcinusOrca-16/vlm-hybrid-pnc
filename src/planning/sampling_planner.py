from src.common.types import (
    FrenetTrajectory,
    FrenetTrajectoryPoint,
)
from src.planning.quintic_polynomial import QuinticPolynomial

_SAMPLING_EPSILON = 1e-9

def sample_quintic_candidate(
    curve: QuinticPolynomial,
    start_s: float,
    resolution: float,
) -> FrenetTrajectory:
    """Sample one quintic lateral curve into a Frenet trajectory."""

    if resolution <= 0.0:
        raise ValueError(
            "Sampling resolution must be positive."
        )

    points = []
    delta_s = 0.0

    while delta_s < curve.length - _SAMPLING_EPSILON:
        points.append(
            FrenetTrajectoryPoint(
                s=start_s + delta_s,
                l=curve.position(delta_s),
                dl_ds=curve.first_derivative(delta_s),
                d2l_ds2=curve.second_derivative(delta_s),
            )
        )

        delta_s += resolution

    delta_s = curve.length

    points.append(
        FrenetTrajectoryPoint(
            s=start_s + delta_s,
            l=curve.position(delta_s),
            dl_ds=curve.first_derivative(delta_s),
            d2l_ds2=curve.second_derivative(delta_s),
        )
    )

    return FrenetTrajectory(
        points=points
    )