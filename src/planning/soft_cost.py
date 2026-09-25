from src.common.types import (
    FrenetTrajectory,
    PathPoint,
)


def lateral_deviation_cost(
    trajectory: FrenetTrajectory,
) -> float:
    """Cost for terminal lateral deviation from the reference line."""

    if not trajectory.points:
        raise ValueError(
            "trajectory must not be empty."
        )

    return abs(
        trajectory.points[-1].l
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
    """Compute weighted soft cost for one feasible candidate."""

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
    """Select the minimum-cost feasible candidate."""

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

