import math

from src.benchmark.planner_benchmark import (
    PlannerBenchmarkResult,
)
from src.benchmark.runtime import (
    benchmark_planner_runtime,
    summarize_planning_times,
)
from src.common.types import (
    FrenetTrajectory,
    FrenetTrajectoryPoint,
    PathPoint,
)


def build_test_trajectory() -> FrenetTrajectory:
    return FrenetTrajectory(
        points=[
            FrenetTrajectoryPoint(
                s=0.0,
                l=0.0,
                dl_ds=0.0,
                d2l_ds2=0.0,
            ),
            FrenetTrajectoryPoint(
                s=1.0,
                l=0.0,
                dl_ds=0.0,
                d2l_ds2=0.0,
            ),
        ]
    )


def build_test_path() -> list[PathPoint]:
    return [
        PathPoint(
            x=0.0,
            y=0.0,
            yaw=0.0,
            curvature=0.0,
            s=0.0,
        ),
        PathPoint(
            x=1.0,
            y=0.0,
            yaw=0.0,
            curvature=0.0,
            s=1.0,
        ),
    ]


def validate_summary() -> None:
    statistics = summarize_planning_times(
        planner_name="test_planner",
        samples_ms=[
            1.0,
            2.0,
            3.0,
            4.0,
            5.0,
        ],
        warmup_runs=2,
    )

    assert statistics.planner_name == "test_planner"
    assert statistics.warmup_runs == 2
    assert statistics.repeated_runs == 5

    assert math.isclose(
        statistics.mean_ms,
        3.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        statistics.median_ms,
        3.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        statistics.p95_ms,
        4.8,
        abs_tol=1e-12,
    )

    assert math.isclose(
        statistics.min_ms,
        1.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        statistics.max_ms,
        5.0,
        abs_tol=1e-12,
    )

    print(
        "Runtime summary statistics passed."
    )


def validate_warmup_exclusion() -> None:
    runtimes = iter([
        100.0,
        200.0,
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
    ])

    trajectory = build_test_trajectory()
    path = build_test_path()

    def run_once() -> PlannerBenchmarkResult:
        return PlannerBenchmarkResult.succeeded(
            planner_name="test_planner",
            trajectory=trajectory,
            path=path,
            planning_time_ms=next(runtimes),
            matched_objective_value=0.0,
        )

    statistics = benchmark_planner_runtime(
        run_once=run_once,
        warmup_runs=2,
        repeated_runs=5,
    )

    assert statistics.warmup_runs == 2
    assert statistics.repeated_runs == 5

    assert statistics.samples_ms == (
        1.0,
        2.0,
        3.0,
        4.0,
        5.0,
    )

    assert math.isclose(
        statistics.mean_ms,
        3.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        statistics.median_ms,
        3.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        statistics.p95_ms,
        4.8,
        abs_tol=1e-12,
    )

    print(
        "Runtime warm-up exclusion passed."
    )


def main() -> None:
    validate_summary()
    validate_warmup_exclusion()

    print()
    print(
        "All runtime benchmark utility cases passed."
    )


if __name__ == "__main__":
    main()