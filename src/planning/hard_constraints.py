import math

from src.common.types import (
    FrenetTrajectory,
    PathPoint,
    StaticObstacle,
    VehicleParams,
)


def compute_vehicle_footprint(
    path_point: PathPoint,
    vehicle_params: VehicleParams,
) -> list[tuple[float, float]]:
    """Compute ego vehicle footprint corners in the global frame."""

    front_x = (
        vehicle_params.wheelbase_m
        + vehicle_params.front_overhang_m
    )
    rear_x = -vehicle_params.rear_overhang_m
    half_width = vehicle_params.width_m / 2.0

    local_corners = [
        (front_x, half_width),    # front-left
        (front_x, -half_width),   # front-right
        (rear_x, -half_width),    # rear-right
        (rear_x, half_width),     # rear-left
    ]

    cos_yaw = math.cos(path_point.yaw)
    sin_yaw = math.sin(path_point.yaw)

    global_corners = []

    for local_x, local_y in local_corners:
        global_x = (
            path_point.x
            + cos_yaw * local_x
            - sin_yaw * local_y
        )

        global_y = (
            path_point.y
            + sin_yaw * local_x
            + cos_yaw * local_y
        )

        global_corners.append(
            (global_x, global_y)
        )

    return global_corners


def _clamp(
    value: float,
    min_value: float,
    max_value: float,
) -> float:
    return min(
        max(value, min_value),
        max_value,
    )


def obstacle_clearance(
    path_point: PathPoint,
    vehicle_params: VehicleParams,
    obstacle: StaticObstacle,
) -> float:
    """Compute clearance between ego footprint and a circular obstacle."""

    dx = obstacle.x - path_point.x
    dy = obstacle.y - path_point.y

    cos_yaw = math.cos(path_point.yaw)
    sin_yaw = math.sin(path_point.yaw)

    obstacle_local_x = (
        cos_yaw * dx
        + sin_yaw * dy
    )

    obstacle_local_y = (
        -sin_yaw * dx
        + cos_yaw * dy
    )

    min_x = -vehicle_params.rear_overhang_m
    max_x = (
        vehicle_params.wheelbase_m
        + vehicle_params.front_overhang_m
    )

    half_width = vehicle_params.width_m / 2.0
    min_y = -half_width
    max_y = half_width

    closest_x = _clamp(
        obstacle_local_x,
        min_x,
        max_x,
    )

    closest_y = _clamp(
        obstacle_local_y,
        min_y,
        max_y,
    )

    distance_to_footprint = math.hypot(
        obstacle_local_x - closest_x,
        obstacle_local_y - closest_y,
    )

    return (
        distance_to_footprint
        - obstacle.radius_m
    )


def has_collision(
    path_point: PathPoint,
    vehicle_params: VehicleParams,
    obstacle: StaticObstacle,
) -> bool:
    return obstacle_clearance(
        path_point=path_point,
        vehicle_params=vehicle_params,
        obstacle=obstacle,
    ) <= 0.0


def has_safe_clearance(
    path_point: PathPoint,
    vehicle_params: VehicleParams,
    obstacle: StaticObstacle,
    minimum_clearance_m: float,
) -> bool:
    return obstacle_clearance(
        path_point=path_point,
        vehicle_params=vehicle_params,
        obstacle=obstacle,
    ) > minimum_clearance_m


def is_curvature_feasible(
    path: list[PathPoint],
    max_curvature: float,
) -> bool:
    """Check whether all path curvatures are within the vehicle limit."""

    if max_curvature <= 0.0:
        raise ValueError(
            "max_curvature must be positive."
        )

    for point in path:
        if abs(point.curvature) > max_curvature:
            return False

    return True


def is_road_boundary_feasible(
    trajectory: FrenetTrajectory,
    vehicle_params: VehicleParams,
    min_l: float,
    max_l: float,
) -> bool:
    """Check whether the vehicle stays inside simple Frenet road bounds."""

    if min_l >= max_l:
        raise ValueError(
            "min_l must be smaller than max_l."
        )

    half_width = vehicle_params.width_m / 2.0

    allowed_min_l = min_l + half_width
    allowed_max_l = max_l - half_width

    if allowed_min_l > allowed_max_l:
        raise ValueError(
            "Road width is smaller than vehicle width."
        )

    for point in trajectory.points:
        if (
            point.l < allowed_min_l
            or point.l > allowed_max_l
        ):
            return False

    return True


def is_obstacle_clearance_feasible(
    path: list[PathPoint],
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
    minimum_clearance_m: float,
) -> bool:
    """Check obstacle clearance along the whole candidate path."""

    if minimum_clearance_m < 0.0:
        raise ValueError(
            "minimum_clearance_m must not be negative."
        )

    for point in path:
        for obstacle in obstacles:
            clearance = obstacle_clearance(
                path_point=point,
                vehicle_params=vehicle_params,
                obstacle=obstacle,
            )

            if clearance <= minimum_clearance_m:
                return False

    return True


def is_candidate_feasible(
    trajectory: FrenetTrajectory,
    path: list[PathPoint],
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
    max_curvature: float,
    minimum_clearance_m: float,
    min_l: float,
    max_l: float,
) -> bool:
    """Check all hard constraints for one candidate."""

    if not is_curvature_feasible(
        path=path,
        max_curvature=max_curvature,
    ):
        return False

    if not is_obstacle_clearance_feasible(
        path=path,
        vehicle_params=vehicle_params,
        obstacles=obstacles,
        minimum_clearance_m=minimum_clearance_m,
    ):
        return False

    if not is_road_boundary_feasible(
        trajectory=trajectory,
        vehicle_params=vehicle_params,
        min_l=min_l,
        max_l=max_l,
    ):
        return False

    return True


def filter_feasible_candidates(
    trajectories: list[FrenetTrajectory],
    paths: list[list[PathPoint]],
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
    max_curvature: float,
    minimum_clearance_m: float,
    min_l: float,
    max_l: float,
) -> list[tuple[FrenetTrajectory, list[PathPoint]]]:
    """Keep only candidates that satisfy all hard constraints."""

    if len(trajectories) != len(paths):
        raise ValueError(
            "trajectories and paths must have the same length."
        )

    feasible_candidates = []

    for trajectory, path in zip(
        trajectories,
        paths,
    ):
        if is_candidate_feasible(
            trajectory=trajectory,
            path=path,
            vehicle_params=vehicle_params,
            obstacles=obstacles,
            max_curvature=max_curvature,
            minimum_clearance_m=minimum_clearance_m,
            min_l=min_l,
            max_l=max_l,
        ):
            feasible_candidates.append(
                (trajectory, path)
            )

    return feasible_candidates
