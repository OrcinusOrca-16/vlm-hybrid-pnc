from src.benchmark.canonical_scenario import (
    build_canonical_static_obstacle_scenario,
)
from src.benchmark.scenario_runner import (
    ScenarioBenchmarkRun,
    run_benchmark_scenario,
)


CanonicalBenchmarkRun = ScenarioBenchmarkRun


def run_canonical_benchmark(
) -> CanonicalBenchmarkRun:
    """Run the fixed canonical static-obstacle benchmark."""

    scenario = (
        build_canonical_static_obstacle_scenario()
    )

    return run_benchmark_scenario(
        scenario
    )