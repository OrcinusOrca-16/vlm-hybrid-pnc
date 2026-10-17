import numpy as np

from src.common.types import (
    FrenetTrajectory,
    PathPoint,
)
from src.planning.path_objective import (
    evaluate_path_objective,
)


def lateral_deviation_cost(
    trajectory: FrenetTrajectory,
) -> float:
    """Mean squared lateral deviation from the reference line."""

    if not trajectory.points:
        raise ValueError(
            "trajectory must not be empty."
        )

    lateral_squared_sum = sum(
        point.l ** 2
        for point in trajectory.points
    )

    return (
        lateral_squared_sum
        / len(trajectory.points)
    )


def curvature_cost(
    path: list[PathPoint],
) -> float:
    """Mean absolute curvature along the candidate path."""

    if not path:
        raise ValueError(
            "path must not be empty."
        )

    curvature_sum = sum(
        abs(point.curvature)
        for point in path
    )

    return curvature_sum / len(path)


def path_length_cost(
    path: list[PathPoint],
) -> float:
    """Cartesian path length used as a positive progress measure."""

    if not path:
        raise ValueError(
            "path must not be empty."
        )

    return path[-1].s


def total_cost(
    trajectory: FrenetTrajectory,
    path: list[PathPoint],
    lateral_weight: float,
    curvature_weight: float,
    length_weight: float,
) -> float:
    """Compute native Sampling Planner cost."""

    lateral = lateral_deviation_cost(
        trajectory
    )

    curvature = curvature_cost(
        path
    )

    length = path_length_cost(
        path
    )

    return (
        lateral_weight * lateral
        + curvature_weight * curvature
        - length_weight * length
    )


def select_best_candidate(
    feasible_candidates: list[
        tuple[
            FrenetTrajectory,
            list[PathPoint],
        ]
    ],
    lateral_weight: float,
    curvature_weight: float,
    length_weight: float,
) -> tuple[
    FrenetTrajectory,
    list[PathPoint],
] | None:
    """Select the minimum-cost candidate using the native Sampling cost."""

    if not feasible_candidates:
        return None

    return min(
        feasible_candidates,
        key=lambda candidate: total_cost(
            trajectory=candidate[0],
            path=candidate[1],
            lateral_weight=lateral_weight,
            curvature_weight=curvature_weight,
            length_weight=length_weight,
        ),
    )


def matched_total_cost(
    trajectory: FrenetTrajectory,
    resolution_m: float,
    lateral_reference_m: float,
    terminal_lateral_target_m: float,
    terminal_first_derivative_target: float,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
) -> float:
    """Compute the canonical matched objective for one Sampling candidate."""

    if not trajectory.points:
        raise ValueError(
            "trajectory must not be empty."
        )

    lateral_values = np.array(
        [
            point.l
            for point in trajectory.points
        ],
        dtype=float,
    )

    evaluation = evaluate_path_objective(
        lateral_values=lateral_values,
        resolution_m=resolution_m,
        lateral_reference_m=lateral_reference_m,
        terminal_lateral_target_m=terminal_lateral_target_m,
        terminal_first_derivative_target=(
            terminal_first_derivative_target
        ),
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=(
            terminal_first_derivative_weight
        ),
    )

    return evaluation.total_cost


def select_best_matched_candidate(
    feasible_candidates: list[
        tuple[
            FrenetTrajectory,
            list[PathPoint],
        ]
    ],
    resolution_m: float,
    lateral_reference_m: float,
    terminal_lateral_target_m: float,
    terminal_first_derivative_target: float,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
) -> tuple[
    FrenetTrajectory,
    list[PathPoint],
] | None:
    """Select the minimum-cost candidate using the matched objective."""

    if not feasible_candidates:
        return None

    return min(
        feasible_candidates,
        key=lambda candidate: matched_total_cost(
            trajectory=candidate[0],
            resolution_m=resolution_m,
            lateral_reference_m=lateral_reference_m,
            terminal_lateral_target_m=terminal_lateral_target_m,
            terminal_first_derivative_target=(
                terminal_first_derivative_target
            ),
            lateral_weight=lateral_weight,
            first_derivative_weight=first_derivative_weight,
            second_derivative_weight=second_derivative_weight,
            third_derivative_weight=third_derivative_weight,
            terminal_lateral_weight=terminal_lateral_weight,
            terminal_first_derivative_weight=(
                terminal_first_derivative_weight
            ),
        ),
    )