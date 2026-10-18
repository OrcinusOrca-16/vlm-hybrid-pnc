from src.benchmark.canonical_scenario import (
    build_canonical_static_obstacle_scenario,
)
from src.benchmark.optimization_adapter import (
    run_optimization_benchmark_adapter,
)
from src.benchmark.runtime import (
    benchmark_planner_runtime,
)
from src.benchmark.sampling_adapter import (
    run_sampling_benchmark_adapter,
)
from src.config.benchmark import (
    DEFAULT_BENCHMARK_RUNTIME_CONFIG,
)
from src.config.planning import (
    DEFAULT_OPTIMIZATION_PLANNER_CONFIG,
)


def main() -> None:
    scenario = (
        build_canonical_static_obstacle_scenario()
    )

    obstacles = list(
        scenario.obstacles
    )

    def run_sampling():
        return run_sampling_benchmark_adapter(
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

    def run_optimization():
        return run_optimization_benchmark_adapter(
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

    runtime_config = (
        DEFAULT_BENCHMARK_RUNTIME_CONFIG
    )

    print(
        "Canonical runtime benchmark"
    )
    print(
        "  warmup_runs="
        f"{runtime_config.warmup_runs}"
    )
    print(
        "  repeated_runs="
        f"{runtime_config.repeated_runs}"
    )
    print()

    sampling_statistics = (
        benchmark_planner_runtime(
            run_once=run_sampling,
            warmup_runs=(
                runtime_config.warmup_runs
            ),
            repeated_runs=(
                runtime_config.repeated_runs
            ),
        )
    )

    optimization_statistics = (
        benchmark_planner_runtime(
            run_once=run_optimization,
            warmup_runs=(
                runtime_config.warmup_runs
            ),
            repeated_runs=(
                runtime_config.repeated_runs
            ),
        )
    )

    print("Sampling:")
    print(
        "  mean_ms="
        f"{sampling_statistics.mean_ms:.4f}"
    )
    print(
        "  median_ms="
        f"{sampling_statistics.median_ms:.4f}"
    )
    print(
        "  p95_ms="
        f"{sampling_statistics.p95_ms:.4f}"
    )
    print(
        "  min_ms="
        f"{sampling_statistics.min_ms:.4f}"
    )
    print(
        "  max_ms="
        f"{sampling_statistics.max_ms:.4f}"
    )

    print()

    print("Optimization:")
    print(
        "  mean_ms="
        f"{optimization_statistics.mean_ms:.4f}"
    )
    print(
        "  median_ms="
        f"{optimization_statistics.median_ms:.4f}"
    )
    print(
        "  p95_ms="
        f"{optimization_statistics.p95_ms:.4f}"
    )
    print(
        "  min_ms="
        f"{optimization_statistics.min_ms:.4f}"
    )
    print(
        "  max_ms="
        f"{optimization_statistics.max_ms:.4f}"
    )


if __name__ == "__main__":
    main()