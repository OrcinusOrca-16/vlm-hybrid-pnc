from dataclasses import dataclass

from src.common.types import (
    StaticObstacle,
    VehicleParams,
    VehicleState,
)
from src.planning.reference_line import (
    ReferenceLine,
)


@dataclass(frozen=True)
class BenchmarkScenario:
    """Shared planner-independent inputs for one benchmark scenario."""

    name: str

    reference_line: ReferenceLine
    vehicle_state: VehicleState
    vehicle_params: VehicleParams
    obstacles: tuple[StaticObstacle, ...]

    road_min_l: float
    road_max_l: float

    max_curvature: float
    minimum_clearance_m: float

    planning_horizon_m: float
    resolution_m: float

    lateral_reference_m: float
    terminal_lateral_target_m: float
    terminal_first_derivative_target: float

    lateral_weight: float
    first_derivative_weight: float
    second_derivative_weight: float
    third_derivative_weight: float
    terminal_lateral_weight: float
    terminal_first_derivative_weight: float

    def __post_init__(self) -> None:
        if not self.name:
            raise ValueError(
                "Benchmark scenario name must not be empty."
            )

        if self.road_min_l >= self.road_max_l:
            raise ValueError(
                "road_min_l must be smaller than road_max_l."
            )

        if self.max_curvature <= 0.0:
            raise ValueError(
                "max_curvature must be positive."
            )

        if self.minimum_clearance_m < 0.0:
            raise ValueError(
                "minimum_clearance_m must not be negative."
            )

        if self.planning_horizon_m <= 0.0:
            raise ValueError(
                "planning_horizon_m must be positive."
            )

        if self.resolution_m <= 0.0:
            raise ValueError(
                "resolution_m must be positive."
            )