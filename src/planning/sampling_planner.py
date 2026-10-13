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


def generate_returning_frenet_candidate(
    start_s: float,
    start_l: float,
    start_dl_ds: float,
    start_d2l_ds2: float,
    bypass_l: float,
    bypass_length: float,
    return_length: float,
    final_l: float = 0.0,
    resolution: float = DEFAULT_SAMPLING_RESOLUTION,
) -> FrenetTrajectory:
    """Generate a two-stage candidate that bypasses and returns laterally."""

    if bypass_length <= 0.0:
        raise ValueError(
            "bypass_length must be positive."
        )

    if return_length <= 0.0:
        raise ValueError(
            "return_length must be positive."
        )

    bypass_curve = QuinticPolynomial(
        start_l=start_l,
        start_dl_ds=start_dl_ds,
        start_d2l_ds2=start_d2l_ds2,
        end_l=bypass_l,
        end_dl_ds=0.0,
        end_d2l_ds2=0.0,
        length=bypass_length,
    )

    bypass_trajectory = sample_quintic_candidate(
        curve=bypass_curve,
        start_s=start_s,
        resolution=resolution,
    )

    junction_point = (
        bypass_trajectory.points[-1]
    )

    return_curve = QuinticPolynomial(
        start_l=junction_point.l,
        start_dl_ds=junction_point.dl_ds,
        start_d2l_ds2=junction_point.d2l_ds2,
        end_l=final_l,
        end_dl_ds=0.0,
        end_d2l_ds2=0.0,
        length=return_length,
    )

    return_trajectory = sample_quintic_candidate(
        curve=return_curve,
        start_s=junction_point.s,
        resolution=resolution,
    )

    points = (
        bypass_trajectory.points
        + return_trajectory.points[1:]
    )

    return FrenetTrajectory(
        points=points
    )


def generate_returning_frenet_candidates(
    start_s: float,
    start_l: float,
    start_dl_ds: float,
    start_d2l_ds2: float,
    bypass_lateral_positions: tuple[float, ...],
    bypass_lengths: tuple[float, ...],
    total_length: float,
    final_l: float = 0.0,
    resolution: float = DEFAULT_SAMPLING_RESOLUTION,
) -> list[FrenetTrajectory]:
    """Generate multiple two-stage bypass-and-return candidates."""

    if total_length <= 0.0:
        raise ValueError(
            "total_length must be positive."
        )

    candidates = []

    for bypass_length in bypass_lengths:
        return_length = (
            total_length
            - bypass_length
        )

        if return_length <= 0.0:
            raise ValueError(
                "Each bypass_length must be smaller than total_length."
            )

        for bypass_l in bypass_lateral_positions:
            candidate = generate_returning_frenet_candidate(
                start_s=start_s,
                start_l=start_l,
                start_dl_ds=start_dl_ds,
                start_d2l_ds2=start_d2l_ds2,
                bypass_l=bypass_l,
                bypass_length=bypass_length,
                return_length=return_length,
                final_l=final_l,
                resolution=resolution,
            )

            candidates.append(
                candidate
            )

    return candidates


def generate_returning_frenet_candidates_from_vehicle_state(
    vehicle_state: VehicleState,
    reference_line: ReferenceLine,
    bypass_lateral_positions: tuple[float, ...],
    bypass_lengths: tuple[float, ...],
    total_length: float,
    final_l: float = 0.0,
    resolution: float = DEFAULT_SAMPLING_RESOLUTION,
) -> list[FrenetTrajectory]:
    """Generate bypass-and-return candidates from the ego vehicle state."""

    initial_state = vehicle_state_to_frenet(
        vehicle_state=vehicle_state,
        reference_line=reference_line,
    )

    return generate_returning_frenet_candidates(
        start_s=initial_state.s,
        start_l=initial_state.l,
        start_dl_ds=initial_state.dl_ds,
        start_d2l_ds2=initial_state.d2l_ds2,
        bypass_lateral_positions=bypass_lateral_positions,
        bypass_lengths=bypass_lengths,
        total_length=total_length,
        final_l=final_l,
        resolution=resolution,
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
