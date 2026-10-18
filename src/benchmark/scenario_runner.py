from dataclasses import dataclass

from src.benchmark.metrics import (
    PlannerBenchmarkMetrics,
    evaluate_planner_benchmark_metrics,
)
from src.benchmark.optimization_adapter import (
    run_optimization_benchmark_adapter,
)
from src.benchmark.planner_benchmark import (
    PlannerBenchmarkResult,
)
from src.benchmark.sampling_adapter import (
    run_sampling_benchmark_adapter,
)
from src.benchmark.scenario import (
    BenchmarkScenario,
)
from src.config.planning import (
    DEFAULT_OPTIMIZATION_PLANNER_CONFIG,
)


@dataclass(frozen=True)
class ScenarioBenchmarkRun:
    """Outputs of both planners for one benchmark scenario."""

    scenario: BenchmarkScenario

    sampling_result: PlannerBenchmarkResult
    optimization_result: PlannerBenchmarkResult

    sampling_metrics: PlannerBenchmarkMetrics
    optimization_metrics: PlannerBenchmarkMetrics


def run_benchmark_scenario(
    scenario: BenchmarkScenario,
) -> ScenarioBenchmarkRun:
    """Run both planners on one shared benchmark scenario."""

    obstacles = list(
        scenario.obstacles
    )

    sampling_result = (
        run_sampling_benchmark_adapter(
            reference_line=scenario.reference_line,
            vehicle_state=scenario.vehicle_state,
            vehicle_params=scenario.vehicle_params,
            obstacles=obstacles,
            road_min_l=scenario.road_min_l,
            road_max_l=scenario.road_max_l,
            max_curvature=scenario.max_curvature,
            minimum_clearance_m=(
                scenario.minimum_clearance_m
            ),
            lateral_reference_m=(
                scenario.lateral_reference_m
            ),
            terminal_lateral_target_m=(
                scenario.terminal_lateral_target_m
            ),
            terminal_first_derivative_target=(
                scenario
                .terminal_first_derivative_target
            ),
            lateral_weight=scenario.lateral_weight,
            first_derivative_weight=(
                scenario.first_derivative_weight
            ),
            second_derivative_weight=(
                scenario.second_derivative_weight
            ),
            third_derivative_weight=(
                scenario.third_derivative_weight
            ),
            terminal_lateral_weight=(
                scenario.terminal_lateral_weight
            ),
            terminal_first_derivative_weight=(
                scenario
                .terminal_first_derivative_weight
            ),
            total_length=(
                scenario.planning_horizon_m
            ),
            final_lateral_positions=(
                scenario.terminal_lateral_target_m,
            ),
            final_dl_ds_values=(
                scenario
                .terminal_first_derivative_target,
            ),
            resolution_m=scenario.resolution_m,
        )
    )

    optimization_result = (
        run_optimization_benchmark_adapter(
            reference_line=scenario.reference_line,
            vehicle_state=scenario.vehicle_state,
            vehicle_params=scenario.vehicle_params,
            obstacles=obstacles,
            road_min_l=scenario.road_min_l,
            road_max_l=scenario.road_max_l,
            max_curvature=scenario.max_curvature,
            minimum_clearance_m=(
                scenario.minimum_clearance_m
            ),
            planning_horizon_m=(
                scenario.planning_horizon_m
            ),
            resolution_m=scenario.resolution_m,
            lateral_reference_m=(
                scenario.lateral_reference_m
            ),
            terminal_lateral_target_m=(
                scenario.terminal_lateral_target_m
            ),
            terminal_first_derivative_target=(
                scenario
                .terminal_first_derivative_target
            ),
            lateral_weight=scenario.lateral_weight,
            first_derivative_weight=(
                scenario.first_derivative_weight
            ),
            second_derivative_weight=(
                scenario.second_derivative_weight
            ),
            third_derivative_weight=(
                scenario.third_derivative_weight
            ),
            terminal_lateral_weight=(
                scenario.terminal_lateral_weight
            ),
            terminal_first_derivative_weight=(
                scenario
                .terminal_first_derivative_weight
            ),
            max_second_derivative_abs=(
                DEFAULT_OPTIMIZATION_PLANNER_CONFIG
                .max_second_derivative_abs
            ),
        )
    )

    sampling_metrics = (
        evaluate_planner_benchmark_metrics(
            result=sampling_result,
            vehicle_params=scenario.vehicle_params,
            obstacles=obstacles,
            lateral_reference_m=(
                scenario.lateral_reference_m
            ),
            terminal_lateral_target_m=(
                scenario.terminal_lateral_target_m
            ),
            terminal_first_derivative_target=(
                scenario
                .terminal_first_derivative_target
            ),
        )
    )

    optimization_metrics = (
        evaluate_planner_benchmark_metrics(
            result=optimization_result,
            vehicle_params=scenario.vehicle_params,
            obstacles=obstacles,
            lateral_reference_m=(
                scenario.lateral_reference_m
            ),
            terminal_lateral_target_m=(
                scenario.terminal_lateral_target_m
            ),
            terminal_first_derivative_target=(
                scenario
                .terminal_first_derivative_target
            ),
        )
    )

    return ScenarioBenchmarkRun(
        scenario=scenario,
        sampling_result=sampling_result,
        optimization_result=optimization_result,
        sampling_metrics=sampling_metrics,
        optimization_metrics=optimization_metrics,
    )