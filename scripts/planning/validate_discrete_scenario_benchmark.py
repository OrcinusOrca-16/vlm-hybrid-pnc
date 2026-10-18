import math

from src.benchmark.scenario_runner import (
    run_benchmark_scenario,
)
from src.benchmark.scenario_suite import (
    build_discrete_benchmark_scenarios,
)


EXPECTED_SCENARIO_NAMES = (
    "nominal_straight",
    "lateral_recovery",
    "static_obstacle",
    "mirrored_obstacle",
    "curved_reference",
    "curved_reference_obstacle",
    "narrower_road",
)

ZERO_TOLERANCE = 1e-6
PATH_LENGTH_TOLERANCE = 1e-4


def format_optional(
    value,
    digits=6,
) -> str:
    if value is None:
        return "N/A"

    return f"{value:.{digits}f}"


def print_planner_result(
    planner_name,
    result,
    metrics,
) -> None:
    print(
        f"  {planner_name}:"
    )

    print(
        "    success="
        f"{result.success}"
    )

    if not result.success:
        print(
            "    reason="
            f"{result.failure_reason}"
        )
        print(
            "    planning_time_ms="
            f"{result.planning_time_ms:.4f}"
        )
        return

    print(
        "    objective="
        + format_optional(
            metrics
            .canonical_matched_objective_value
        )
    )

    print(
        "    terminal_l_error="
        + format_optional(
            metrics.terminal_lateral_error_m
        )
    )

    print(
        "    terminal_slope_error="
        + format_optional(
            metrics.terminal_slope_error
        )
    )

    print(
        "    rms_lateral="
        + format_optional(
            metrics.rms_lateral_deviation_m
        )
    )

    print(
        "    path_length="
        + format_optional(
            metrics.cartesian_path_length_m
        )
    )

    print(
        "    min_clearance="
        + format_optional(
            metrics
            .minimum_exact_obstacle_clearance_m
        )
    )

    print(
        "    max_curvature="
        + format_optional(
            metrics
            .max_abs_cartesian_curvature_per_m
        )
    )

    print(
        "    rms_curvature="
        + format_optional(
            metrics
            .rms_cartesian_curvature_per_m
        )
    )

    print(
        "    planning_time_ms="
        f"{metrics.planning_time_ms:.4f}"
    )


def validate_nominal_planner(
    planner_name,
    metrics,
) -> None:
    assert metrics.planning_success

    assert (
        metrics.canonical_matched_objective_value
        is not None
    )
    assert abs(
        metrics.canonical_matched_objective_value
    ) < ZERO_TOLERANCE

    assert (
        metrics.terminal_lateral_error_m
        is not None
    )
    assert (
        metrics.terminal_lateral_error_m
        < ZERO_TOLERANCE
    )

    assert (
        metrics.terminal_slope_error
        is not None
    )
    assert (
        metrics.terminal_slope_error
        < ZERO_TOLERANCE
    )

    assert (
        metrics.rms_lateral_deviation_m
        is not None
    )
    assert (
        metrics.rms_lateral_deviation_m
        < ZERO_TOLERANCE
    )

    assert (
        metrics.max_abs_cartesian_curvature_per_m
        is not None
    )
    assert (
        metrics
        .max_abs_cartesian_curvature_per_m
        < ZERO_TOLERANCE
    )

    assert (
        metrics.rms_cartesian_curvature_per_m
        is not None
    )
    assert (
        metrics
        .rms_cartesian_curvature_per_m
        < ZERO_TOLERANCE
    )

    assert (
        metrics.minimum_exact_obstacle_clearance_m
        is None
    )

    assert (
        metrics.cartesian_path_length_m
        is not None
    )
    assert math.isclose(
        metrics.cartesian_path_length_m,
        28.0,
        abs_tol=PATH_LENGTH_TOLERANCE,
    )

    print(
        f"{planner_name} nominal validation passed."
    )


def validate_nominal_straight(
    benchmark,
) -> None:
    scenario = benchmark.scenario

    assert scenario.name == "nominal_straight"
    assert not scenario.obstacles

    validate_nominal_planner(
        planner_name="Sampling",
        metrics=benchmark.sampling_metrics,
    )

    validate_nominal_planner(
        planner_name="Optimization",
        metrics=benchmark.optimization_metrics,
    )


def validate_lateral_recovery(
    benchmark,
) -> None:
    scenario = benchmark.scenario

    assert scenario.name == "lateral_recovery"
    assert not scenario.obstacles

    assert benchmark.sampling_result.success
    assert benchmark.optimization_result.success

    sampling_trajectory = (
        benchmark.sampling_result.trajectory
    )
    optimization_trajectory = (
        benchmark.optimization_result.trajectory
    )

    assert sampling_trajectory is not None
    assert optimization_trajectory is not None

    sampling_initial_l = (
        sampling_trajectory.points[0].l
    )
    sampling_final_l = (
        sampling_trajectory.points[-1].l
    )

    optimization_initial_l = (
        optimization_trajectory.points[0].l
    )
    optimization_final_l = (
        optimization_trajectory.points[-1].l
    )

    assert math.isclose(
        sampling_initial_l,
        1.0,
        abs_tol=1e-4,
    )
    assert math.isclose(
        optimization_initial_l,
        1.0,
        abs_tol=1e-4,
    )

    assert abs(
        sampling_final_l
    ) < abs(
        sampling_initial_l
    )

    assert abs(
        optimization_final_l
    ) < abs(
        optimization_initial_l
    )

    assert (
        benchmark
        .sampling_metrics
        .terminal_lateral_error_m
        is not None
    )

    assert (
        benchmark
        .optimization_metrics
        .terminal_lateral_error_m
        is not None
    )

    assert (
        benchmark
        .sampling_metrics
        .terminal_lateral_error_m
        < 1e-4
    )

    assert (
        benchmark
        .optimization_metrics
        .terminal_lateral_error_m
        < 1e-4
    )

    print(
        "Lateral recovery validation passed."
    )
    print(
        "  Sampling:"
        f" {sampling_initial_l:.6f}"
        f" -> {sampling_final_l:.6f}"
    )
    print(
        "  Optimization:"
        f" {optimization_initial_l:.6f}"
        f" -> {optimization_final_l:.6f}"
    )


def validate_static_and_mirrored(
    static_benchmark,
    mirrored_benchmark,
) -> None:
    static = static_benchmark.scenario
    mirrored = mirrored_benchmark.scenario

    assert len(static.obstacles) == 1
    assert len(mirrored.obstacles) == 1

    static_obstacle = (
        static.obstacles[0]
    )
    mirrored_obstacle = (
        mirrored.obstacles[0]
    )

    assert math.isclose(
        static_obstacle.x,
        mirrored_obstacle.x,
        abs_tol=ZERO_TOLERANCE,
    )

    assert math.isclose(
        static_obstacle.y,
        -mirrored_obstacle.y,
        abs_tol=ZERO_TOLERANCE,
    )

    assert math.isclose(
        static_obstacle.radius_m,
        mirrored_obstacle.radius_m,
        abs_tol=ZERO_TOLERANCE,
    )

    print(
        "Static / mirrored obstacle "
        "scenario validation passed."
    )


def validate_curved_scenarios(
    curved_benchmark,
    curved_obstacle_benchmark,
) -> None:
    curved = curved_benchmark.scenario
    curved_obstacle = (
        curved_obstacle_benchmark.scenario
    )

    assert not curved.obstacles

    assert len(
        curved_obstacle.obstacles
    ) == 1

    assert (
        curved.reference_line.length
        > curved.planning_horizon_m
    )

    print(
        "Curved scenario definitions passed."
    )


def validate_narrower_road(
    static_benchmark,
    narrower_benchmark,
) -> None:
    static = static_benchmark.scenario
    narrower = narrower_benchmark.scenario

    static_width = (
        static.road_max_l
        - static.road_min_l
    )

    narrower_width = (
        narrower.road_max_l
        - narrower.road_min_l
    )

    assert narrower_width < static_width

    assert math.isclose(
        narrower.road_min_l,
        -2.5,
        abs_tol=ZERO_TOLERANCE,
    )

    assert math.isclose(
        narrower.road_max_l,
        2.5,
        abs_tol=ZERO_TOLERANCE,
    )

    print(
        "Narrower-road scenario validation passed."
    )


def main() -> None:
    scenarios = (
        build_discrete_benchmark_scenarios()
    )

    scenario_names = tuple(
        scenario.name
        for scenario in scenarios
    )

    assert (
        scenario_names
        == EXPECTED_SCENARIO_NAMES
    )

    benchmark_runs = {}
    failures = []

    for scenario in scenarios:
        benchmark = (
            run_benchmark_scenario(
                scenario
            )
        )

        benchmark_runs[
            scenario.name
        ] = benchmark

        print(
            f"[{scenario.name}]"
        )

        print_planner_result(
            planner_name="Sampling",
            result=benchmark.sampling_result,
            metrics=benchmark.sampling_metrics,
        )

        print_planner_result(
            planner_name="Optimization",
            result=benchmark.optimization_result,
            metrics=benchmark.optimization_metrics,
        )

        if not benchmark.sampling_result.success:
            failures.append(
                (
                    scenario.name,
                    "Sampling",
                    benchmark
                    .sampling_result
                    .failure_reason,
                )
            )

        if not benchmark.optimization_result.success:
            failures.append(
                (
                    scenario.name,
                    "Optimization",
                    benchmark
                    .optimization_result
                    .failure_reason,
                )
            )

        print()

    validate_nominal_straight(
        benchmark_runs[
            "nominal_straight"
        ]
    )

    validate_lateral_recovery(
        benchmark_runs[
            "lateral_recovery"
        ]
    )

    validate_static_and_mirrored(
        static_benchmark=(
            benchmark_runs[
                "static_obstacle"
            ]
        ),
        mirrored_benchmark=(
            benchmark_runs[
                "mirrored_obstacle"
            ]
        ),
    )

    validate_curved_scenarios(
        curved_benchmark=(
            benchmark_runs[
                "curved_reference"
            ]
        ),
        curved_obstacle_benchmark=(
            benchmark_runs[
                "curved_reference_obstacle"
            ]
        ),
    )

    validate_narrower_road(
        static_benchmark=(
            benchmark_runs[
                "static_obstacle"
            ]
        ),
        narrower_benchmark=(
            benchmark_runs[
                "narrower_road"
            ]
        ),
    )

    print()

    if failures:
        print(
            "Discrete scenario failures:"
        )

        for (
            scenario_name,
            planner_name,
            reason,
        ) in failures:
            print(
                "  "
                f"{scenario_name} / "
                f"{planner_name}: "
                f"{reason}"
            )

        raise AssertionError(
            "One or more discrete benchmark "
            "cases failed."
        )

    print(
        "All discrete benchmark scenarios passed."
    )


if __name__ == "__main__":
    main()