from time import perf_counter

from src.benchmark.planner_benchmark import (
    PlannerBenchmarkResult,
)
from src.common.types import (
    StaticObstacle,
    VehicleParams,
    VehicleState,
)
from src.config.planning import (
    DEFAULT_OPTIMIZATION_PLANNER_CONFIG,
)
from src.planning.frenet import (
    vehicle_state_to_frenet,
)
from src.planning.lateral_corridor import (
    build_base_lateral_corridor,
    build_obstacle_branch_corridors,
    inflate_frenet_obstacle,
    project_obstacle_to_frenet,
)
from src.planning.optimization_planner import (
    plan_matched_multi_corridor_path,
)
from src.planning.path_objective import (
    PathObjectiveWeights,
    evaluate_frenet_trajectory_objective,
)
from src.planning.reference_line import (
    ReferenceLine,
)


def run_optimization_benchmark_adapter(
    reference_line: ReferenceLine,
    vehicle_state: VehicleState,
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
    road_min_l: float,
    road_max_l: float,
    max_curvature: float,
    minimum_clearance_m: float,
    planning_horizon_m: float,
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
    max_second_derivative_abs: float,
    obstacle_inflation_safety_margin_m: float = (
        DEFAULT_OPTIMIZATION_PLANNER_CONFIG
        .obstacle_inflation_safety_margin_m
    ),
    corridor_affected_epsilon_m: float = (
        DEFAULT_OPTIMIZATION_PLANNER_CONFIG
        .corridor_affected_epsilon_m
    ),
    osqp_eps_abs: float = (
        DEFAULT_OPTIMIZATION_PLANNER_CONFIG
        .osqp_eps_abs
    ),
    osqp_eps_rel: float = (
        DEFAULT_OPTIMIZATION_PLANNER_CONFIG
        .osqp_eps_rel
    ),
    planner_objective_weights: PathObjectiveWeights = (
        DEFAULT_OPTIMIZATION_PLANNER_CONFIG
        .matched_objective_weights
    ),
) -> PlannerBenchmarkResult:
    """Run the matched Optimization Planner through the benchmark interface."""

    if len(obstacles) > 1:
        raise ValueError(
            "Optimization benchmark adapter currently supports "
            "at most one static obstacle."
        )

    start_time = perf_counter()

    initial_state = vehicle_state_to_frenet(
        vehicle_state=vehicle_state,
        reference_line=reference_line,
    )

    base_corridor = build_base_lateral_corridor(
        start_s=initial_state.s,
        reference_line=reference_line,
        vehicle_params=vehicle_params,
        road_min_l=road_min_l,
        road_max_l=road_max_l,
        planning_horizon_m=planning_horizon_m,
        resolution_m=resolution_m,
    )

    corridors = [
        base_corridor,
    ]

    if obstacles:
        frenet_obstacle = project_obstacle_to_frenet(
            obstacle=obstacles[0],
            reference_line=reference_line,
        )

        inflated_obstacle = inflate_frenet_obstacle(
            obstacle=frenet_obstacle,
            vehicle_params=vehicle_params,
            minimum_clearance_m=minimum_clearance_m,
            safety_margin_m=obstacle_inflation_safety_margin_m,
        )

        (
            negative_l_corridor,
            positive_l_corridor,
        ) = build_obstacle_branch_corridors(
            base_corridor=base_corridor,
            obstacle=inflated_obstacle,
            corridor_epsilon_m=corridor_affected_epsilon_m,
        )

        corridors = [
            negative_l_corridor,
            positive_l_corridor,
        ]

    try:
        result = plan_matched_multi_corridor_path(
            reference_line=reference_line,
            corridors=corridors,
            resolution_m=resolution_m,
            initial_l=initial_state.l,
            initial_dl_ds=initial_state.dl_ds,
            initial_d2l_ds2=initial_state.d2l_ds2,
            lateral_reference_m=lateral_reference_m,
            terminal_lateral_target_m=(
                terminal_lateral_target_m
            ),
            terminal_first_derivative_target=(
                terminal_first_derivative_target
            ),
            lateral_weight=(
                planner_objective_weights
                .lateral_weight
            ),
            first_derivative_weight=(
                planner_objective_weights
                .first_derivative_weight
            ),
            second_derivative_weight=(
                planner_objective_weights
                .second_derivative_weight
            ),
            third_derivative_weight=(
                planner_objective_weights
                .third_derivative_weight
            ),
            terminal_lateral_weight=(
                planner_objective_weights
                .terminal_lateral_weight
            ),
            terminal_first_derivative_weight=(
                planner_objective_weights
                .terminal_first_derivative_weight
            ),
            max_second_derivative_abs=(
                max_second_derivative_abs
            ),
            vehicle_params=vehicle_params,
            obstacles=obstacles,
            max_curvature=max_curvature,
            minimum_clearance_m=minimum_clearance_m,
            road_min_l=road_min_l,
            road_max_l=road_max_l,
            osqp_eps_abs=osqp_eps_abs,
            osqp_eps_rel=osqp_eps_rel,
        )
    except RuntimeError as error:
        planning_time_ms = (
            perf_counter()
            - start_time
        ) * 1000.0

        return PlannerBenchmarkResult.failed(
            planner_name="optimization",
            planning_time_ms=planning_time_ms,
            failure_reason=str(error),
        )

    canonical_weights = PathObjectiveWeights(
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=(
            terminal_first_derivative_weight
        ),
    )

    canonical_evaluation = (
        evaluate_frenet_trajectory_objective(
            trajectory=result.trajectory,
            resolution_m=resolution_m,
            lateral_reference_m=lateral_reference_m,
            terminal_lateral_target_m=(
                terminal_lateral_target_m
            ),
            terminal_first_derivative_target=(
                terminal_first_derivative_target
            ),
            weights=canonical_weights,
        )
    )

    planning_time_ms = (
        perf_counter()
        - start_time
    ) * 1000.0

    return PlannerBenchmarkResult.succeeded(
        planner_name="optimization",
        trajectory=result.trajectory,
        path=result.path,
        planning_time_ms=planning_time_ms,
        matched_objective_value=(
            canonical_evaluation.total_cost
        ),
    )