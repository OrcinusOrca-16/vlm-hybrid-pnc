from src.common.types import (
    FrenetTrajectory,
    FrenetTrajectoryPoint,
    VehicleState,
)
from src.planning.quintic_polynomial import QuinticPolynomial
from src.planning.frenet import vehicle_state_to_frenet
from src.planning.reference_line import ReferenceLine

_SAMPLING_EPSILON = 1e-9
DEFAULT_TARGET_LATERAL_POSITIONS = (
    -1.5,
    -0.75,
    0.0,
    0.75,
    1.5,
)

DEFAULT_TARGET_LENGTHS = (
    10.0,
    20.0,
)

DEFAULT_SAMPLING_RESOLUTION = 0.5


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


def generate_frenet_candidates(
    start_s: float,
    start_l: float,
    start_dl_ds: float,
    start_d2l_ds2: float,
    target_lateral_positions: tuple[float, ...] = DEFAULT_TARGET_LATERAL_POSITIONS,
    target_lengths: tuple[float, ...] = DEFAULT_TARGET_LENGTHS,
    resolution: float = DEFAULT_SAMPLING_RESOLUTION,
) -> list[FrenetTrajectory]:
    """Generate multiple Frenet candidate paths."""

    candidates = []

    for target_length in target_lengths:
        for target_l in target_lateral_positions:
            curve = QuinticPolynomial(
                start_l=start_l,
                start_dl_ds=start_dl_ds,
                start_d2l_ds2=start_d2l_ds2,
                end_l=target_l,
                end_dl_ds=0.0,
                end_d2l_ds2=0.0,
                length=target_length,
            )

            candidate = sample_quintic_candidate(
                curve=curve,
                start_s=start_s,
                resolution=resolution,
            )

            candidates.append(candidate)

    return candidates


def generate_frenet_candidates_from_vehicle_state(
    vehicle_state: VehicleState,
    reference_line: ReferenceLine,
    target_lateral_positions: tuple[float, ...] = DEFAULT_TARGET_LATERAL_POSITIONS,
    target_lengths: tuple[float, ...] = DEFAULT_TARGET_LENGTHS,
    resolution: float = DEFAULT_SAMPLING_RESOLUTION,
) -> list[FrenetTrajectory]:
    """Generate Frenet candidate paths from the ego vehicle state."""

    initial_state = vehicle_state_to_frenet(
        vehicle_state=vehicle_state,
        reference_line=reference_line,
    )

    return generate_frenet_candidates(
        start_s=initial_state.s,
        start_l=initial_state.l,
        start_dl_ds=initial_state.dl_ds,
        start_d2l_ds2=initial_state.d2l_ds2,
        target_lateral_positions=target_lateral_positions,
        target_lengths=target_lengths,
        resolution=resolution,
    )
