from src.benchmark.scenario import (
    BenchmarkScenario,
)
from src.common.types import (
    StaticObstacle,
    VehicleParams,
    VehicleState,
)
from src.config.benchmark import (
    DEFAULT_CANONICAL_BENCHMARK_CONFIG,
)
from src.config.planning import (
    DEFAULT_PLANNING_CONSTRAINT_CONFIG,
)
from src.config.scenario import (
    DEFAULT_ROAD_CONFIG,
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

    benchmark_config = (
        DEFAULT_CANONICAL_BENCHMARK_CONFIG
    )

    physical_config = (
        DEFAULT_PLANNING_CONSTRAINT_CONFIG
    )

    road_config = (
        DEFAULT_ROAD_CONFIG
    )

    objective_weights = (
        benchmark_config
        .matched_objective_weights
    )

    return BenchmarkScenario(
        name="static_obstacle",
        reference_line=reference_line,
        vehicle_state=vehicle_state,
        vehicle_params=vehicle_params,
        obstacles=obstacles,
        road_min_l=road_config.min_l_m,
        road_max_l=road_config.max_l_m,
        max_curvature=physical_config.max_curvature,
        minimum_clearance_m=(
            physical_config.minimum_clearance_m
        ),
        planning_horizon_m=(
            benchmark_config.planning_horizon_m
        ),
        resolution_m=(
            benchmark_config.resolution_m
        ),
        lateral_reference_m=(
            benchmark_config.lateral_reference_m
        ),
        terminal_lateral_target_m=(
            benchmark_config
            .terminal_lateral_target_m
        ),
        terminal_first_derivative_target=(
            benchmark_config
            .terminal_first_derivative_target
        ),
        lateral_weight=(
            objective_weights.lateral_weight
        ),
        first_derivative_weight=(
            objective_weights
            .first_derivative_weight
        ),
        second_derivative_weight=(
            objective_weights
            .second_derivative_weight
        ),
        third_derivative_weight=(
            objective_weights
            .third_derivative_weight
        ),
        terminal_lateral_weight=(
            objective_weights
            .terminal_lateral_weight
        ),
        terminal_first_derivative_weight=(
            objective_weights
            .terminal_first_derivative_weight
        ),
    )