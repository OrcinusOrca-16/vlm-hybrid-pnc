from src.benchmark.scenario_runner import (
    run_benchmark_scenario,
)
from src.benchmark.scenario_suite import (
    build_obstacle_lateral_sweep_scenarios,
    build_obstacle_longitudinal_sweep_scenarios,
    build_road_width_sweep_scenarios,
)
from src.benchmark.stress import (
    summarize_planner_stress,
)


def status_text(
    success: bool,
) -> str:
    return (
        "PASS"
        if success
        else "FAIL"
    )


def run_sweep(
    title,
    scenarios,
    sampling_results,
    optimization_results,
) -> None:
    print(
        title
    )

    print(
        "Scenario"
        " | Sampling"
        " | Optimization"
        " | Sampling ms"
        " | Optimization ms"
    )

    print(
        "-" * 92
    )

    for scenario in scenarios:
        benchmark = (
            run_benchmark_scenario(
                scenario
            )
        )

        sampling_results.append(
            (
                scenario.name,
                benchmark.sampling_result,
            )
        )

        optimization_results.append(
            (
                scenario.name,
                benchmark.optimization_result,
            )
        )

        print(
            f"{scenario.name:<32}"
            " | "
            f"{status_text(benchmark.sampling_result.success):<8}"
            " | "
            f"{status_text(benchmark.optimization_result.success):<12}"
            " | "
            f"{benchmark.sampling_result.planning_time_ms:11.4f}"
            " | "
            f"{benchmark.optimization_result.planning_time_ms:15.4f}"
        )

    print()


def print_summary(
    summary,
) -> None:
    print(
        f"{summary.planner_name}:"
    )

    print(
        "  successful="
        f"{summary.successful_cases}"
        "/"
        f"{summary.total_cases}"
    )

    print(
        "  failed="
        f"{summary.failed_cases}"
    )

    print(
        "  success_rate="
        f"{100.0 * summary.success_rate:.2f}%"
    )

    if not summary.failure_cases:
        print(
            "  failure_cases=None"
        )
        return

    print(
        "  failure_cases:"
    )

    for (
        scenario_name,
        reason,
    ) in summary.failure_cases:
        print(
            "    "
            f"{scenario_name}: "
            f"{reason}"
        )


def validate_anchor_cases(
    sampling_results,
    optimization_results,
) -> None:
    sampling_map = {
        name: result
        for name, result in sampling_results
    }

    optimization_map = {
        name: result
        for name, result in optimization_results
    }

    required_success_cases = (
        "obstacle_lateral_-1.00",
        "obstacle_lateral_+1.00",
        "obstacle_longitudinal_24.00",
        "road_half_width_3.00",
        "road_half_width_2.50",
    )

    for scenario_name in (
        required_success_cases
    ):
        assert (
            sampling_map[
                scenario_name
            ].success
        )

        assert (
            optimization_map[
                scenario_name
            ].success
        )

    print(
        "Stress benchmark anchor cases passed."
    )


def main() -> None:
    sampling_results = []
    optimization_results = []

    lateral_scenarios = (
        build_obstacle_lateral_sweep_scenarios()
    )

    longitudinal_scenarios = (
        build_obstacle_longitudinal_sweep_scenarios()
    )

    road_width_scenarios = (
        build_road_width_sweep_scenarios()
    )

    assert len(
        lateral_scenarios
    ) == 7

    assert len(
        longitudinal_scenarios
    ) == 6

    assert len(
        road_width_scenarios
    ) == 6

    run_sweep(
        title="Obstacle lateral-position sweep",
        scenarios=lateral_scenarios,
        sampling_results=sampling_results,
        optimization_results=optimization_results,
    )

    run_sweep(
        title="Obstacle longitudinal-position sweep",
        scenarios=longitudinal_scenarios,
        sampling_results=sampling_results,
        optimization_results=optimization_results,
    )

    run_sweep(
        title="Road-width sweep",
        scenarios=road_width_scenarios,
        sampling_results=sampling_results,
        optimization_results=optimization_results,
    )

    validate_anchor_cases(
        sampling_results=sampling_results,
        optimization_results=optimization_results,
    )

    sampling_summary = (
        summarize_planner_stress(
            planner_name="Sampling",
            results=sampling_results,
        )
    )

    optimization_summary = (
        summarize_planner_stress(
            planner_name="Optimization",
            results=optimization_results,
        )
    )

    assert (
        sampling_summary.total_cases
        == 19
    )

    assert (
        optimization_summary.total_cases
        == 19
    )

    assert (
        sampling_summary.successful_cases
        + sampling_summary.failed_cases
        == sampling_summary.total_cases
    )

    assert (
        optimization_summary.successful_cases
        + optimization_summary.failed_cases
        == optimization_summary.total_cases
    )

    assert (
        len(
            sampling_summary.failure_cases
        )
        == sampling_summary.failed_cases
    )

    assert (
        len(
            optimization_summary.failure_cases
        )
        == optimization_summary.failed_cases
    )

    print()
    print(
        "Stress benchmark summary"
    )
    print()

    print_summary(
        sampling_summary
    )

    print()

    print_summary(
        optimization_summary
    )

    print()
    print(
        "Stress benchmark validation passed."
    )


if __name__ == "__main__":
    main()