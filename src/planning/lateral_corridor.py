from dataclasses import dataclass

import numpy as np

from src.common.types import VehicleParams
from src.planning.reference_line import ReferenceLine
from src.common.types import (
    StaticObstacle,
    VehicleParams,
)
from src.planning.frenet import xy_to_sl


_CORRIDOR_EPSILON = 1e-2

@dataclass
class LateralCorridor:
    s_values: np.ndarray
    lower_bounds: np.ndarray
    upper_bounds: np.ndarray

@dataclass
class FrenetObstacle:
    s: float
    l: float
    radius_m: float

@dataclass
class InflatedFrenetObstacle:
    min_s: float
    max_s: float
    min_l: float
    max_l: float


def discretize_planning_horizon(
    start_s: float,
    reference_line: ReferenceLine,
    planning_horizon_m: float,
    resolution_m: float,
) -> np.ndarray:
    if not 0.0 <= start_s <= reference_line.length:
        raise ValueError(
            "start_s must lie on the reference line."
        )

    if planning_horizon_m <= 0.0:
        raise ValueError(
            "planning_horizon_m must be positive."
        )

    if resolution_m <= 0.0:
        raise ValueError(
            "resolution_m must be positive."
        )

    end_s = min(
        start_s + planning_horizon_m,
        reference_line.length,
    )

    num_intervals = int(
        np.floor(
            (end_s - start_s)
            / resolution_m
            + 1e-9
        )
    )

    return (
        start_s
        + np.arange(num_intervals + 1)
        * resolution_m
    )


def build_base_lateral_corridor(
    start_s: float,
    reference_line: ReferenceLine,
    vehicle_params: VehicleParams,
    road_min_l: float,
    road_max_l: float,
    planning_horizon_m: float,
    resolution_m: float,
) -> LateralCorridor:
    if road_min_l >= road_max_l:
        raise ValueError(
            "road_min_l must be smaller than road_max_l."
        )

    half_width = (
        vehicle_params.width_m / 2.0
    )

    allowed_min_l = (
        road_min_l + half_width
    )

    allowed_max_l = (
        road_max_l - half_width
    )

    if allowed_min_l > allowed_max_l:
        raise ValueError(
            "Road width is smaller than vehicle width."
        )

    s_values = discretize_planning_horizon(
        start_s=start_s,
        reference_line=reference_line,
        planning_horizon_m=planning_horizon_m,
        resolution_m=resolution_m,
    )

    lower_bounds = np.full(
        s_values.size,
        allowed_min_l,
    )

    upper_bounds = np.full(
        s_values.size,
        allowed_max_l,
    )

    return LateralCorridor(
        s_values=s_values,
        lower_bounds=lower_bounds,
        upper_bounds=upper_bounds,
    )


def project_obstacle_to_frenet(
    obstacle: StaticObstacle,
    reference_line: ReferenceLine,
) -> FrenetObstacle:
    frenet_point = xy_to_sl(
        x=obstacle.x,
        y=obstacle.y,
        reference_line=reference_line,
    )

    return FrenetObstacle(
        s=frenet_point.s,
        l=frenet_point.l,
        radius_m=obstacle.radius_m,
    )


def inflate_frenet_obstacle(
    obstacle: FrenetObstacle,
    vehicle_params: VehicleParams,
    minimum_clearance_m: float,
    safety_margin_m: float = 1e-2,
) -> InflatedFrenetObstacle:
    if minimum_clearance_m < 0.0:
        raise ValueError(
            "minimum_clearance_m must not be negative."
        )

    if safety_margin_m < 0.0:
        raise ValueError(
            "safety_margin_m must not be negative."
        )

    effective_radius = (
        obstacle.radius_m
        + minimum_clearance_m
        + safety_margin_m
    )

    front_extent = (
        vehicle_params.wheelbase_m
        + vehicle_params.front_overhang_m
    )

    rear_extent = (
        vehicle_params.rear_overhang_m
    )

    half_width = (
        vehicle_params.width_m / 2.0
    )

    return InflatedFrenetObstacle(
        min_s=(
            obstacle.s
            - effective_radius
            - front_extent
        ),
        max_s=(
            obstacle.s
            + effective_radius
            + rear_extent
        ),
        min_l=(
            obstacle.l
            - effective_radius
            - half_width
        ),
        max_l=(
            obstacle.l
            + effective_radius
            + half_width
        ),
    )


def is_corridor_feasible(
    corridor: LateralCorridor,
) -> bool:
    return bool(
        np.all(
            corridor.lower_bounds
            <= corridor.upper_bounds
        )
    )


def build_obstacle_branch_corridors(
    base_corridor: LateralCorridor,
    obstacle: InflatedFrenetObstacle,
) -> tuple[
    LateralCorridor,
    LateralCorridor,
]:
    affected = (
    (
        base_corridor.s_values
        >= obstacle.min_s - _CORRIDOR_EPSILON
    )
    & (
        base_corridor.s_values
        <= obstacle.max_s + _CORRIDOR_EPSILON
    )
)

    negative_lower_bounds = (
        base_corridor.lower_bounds.copy()
    )
    negative_upper_bounds = (
        base_corridor.upper_bounds.copy()
    )

    negative_upper_bounds[affected] = np.minimum(
        negative_upper_bounds[affected],
        obstacle.min_l,
    )

    positive_lower_bounds = (
        base_corridor.lower_bounds.copy()
    )
    positive_upper_bounds = (
        base_corridor.upper_bounds.copy()
    )

    positive_lower_bounds[affected] = np.maximum(
        positive_lower_bounds[affected],
        obstacle.max_l,
    )

    negative_l_corridor = LateralCorridor(
        s_values=base_corridor.s_values.copy(),
        lower_bounds=negative_lower_bounds,
        upper_bounds=negative_upper_bounds,
    )

    positive_l_corridor = LateralCorridor(
        s_values=base_corridor.s_values.copy(),
        lower_bounds=positive_lower_bounds,
        upper_bounds=positive_upper_bounds,
    )

    return (
        negative_l_corridor,
        positive_l_corridor,
    )