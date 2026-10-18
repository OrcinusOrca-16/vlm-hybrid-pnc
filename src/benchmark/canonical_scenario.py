from src.benchmark.scenario import (
    BenchmarkScenario,
)
from src.common.types import (
    StaticObstacle,
    VehicleParams,
    VehicleState,
)
from src.config.vehicle import (
    DEFAULT_VEHICLE_CONFIG,
)
from src.planning.reference_line import (
    ReferenceLine,
)


class CanonicalBenchmarkScenario:
    """Shared inputs for the canonical planner benchmark."""

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


def build_canonical_static_obstacle_scenario(
) -> BenchmarkScenario:
    """Build the fixed static-obstacle benchmark scenario."""

    reference_line = ReferenceLine([
        (0.0, 0.0),
        (20.0, 0.0),
        (40.0, 0.0),
        (50.0, 0.0),
    ])

    vehicle_state = VehicleState(
        x=12.0,
        y=0.0,
        yaw=0.0,
        speed=5.0,
        curvature=0.0,
    )

    vehicle_params = (
        DEFAULT_VEHICLE_CONFIG
        .to_vehicle_params()
    )

    obstacles = (
        StaticObstacle(
            x=24.0,
            y=1.0,
            radius_m=0.5,
        ),
    )

    return BenchmarkScenario(
        name="static_obstacle",
        reference_line=reference_line,
        vehicle_state=vehicle_state,
        vehicle_params=vehicle_params,
        obstacles=obstacles,
        road_min_l=-3.0,
        road_max_l=3.0,
        max_curvature=1.0,
        minimum_clearance_m=0.5,
        planning_horizon_m=28.0,
        resolution_m=0.5,
        lateral_reference_m=0.0,
        terminal_lateral_target_m=0.0,
        terminal_first_derivative_target=0.0,
        lateral_weight=1.0,
        first_derivative_weight=1.0,
        second_derivative_weight=5.0,
        third_derivative_weight=20.0,
        terminal_lateral_weight=5.0,
        terminal_first_derivative_weight=5.0,
    )