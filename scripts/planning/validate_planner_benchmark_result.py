from src.benchmark.planner_benchmark import (
    PlannerBenchmarkResult,
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


def validate_success_result() -> None:
    trajectory = build_test_trajectory()
    path = build_test_path()

    result = PlannerBenchmarkResult.succeeded(
        planner_name="sampling",
        trajectory=trajectory,
        path=path,
        planning_time_ms=1.25,
        matched_objective_value=3.50,
    )

    assert result.planner_name == "sampling"
    assert result.success
    assert result.trajectory is trajectory
    assert result.path is path
    assert result.planning_time_ms == 1.25
    assert result.failure_reason is None
    assert result.matched_objective_value == 3.50

    print(
        "PlannerBenchmarkResult success case passed."
    )


def validate_failure_result() -> None:
    result = PlannerBenchmarkResult.failed(
        planner_name="optimization",
        planning_time_ms=2.50,
        failure_reason="No physically feasible path.",
    )

    assert result.planner_name == "optimization"
    assert not result.success
    assert result.trajectory is None
    assert result.path is None
    assert result.planning_time_ms == 2.50
    assert (
        result.failure_reason
        == "No physically feasible path."
    )
    assert result.matched_objective_value is None

    print(
        "PlannerBenchmarkResult failure case passed."
    )


def validate_failed_result_can_keep_output() -> None:
    trajectory = build_test_trajectory()
    path = build_test_path()

    result = PlannerBenchmarkResult.failed(
        planner_name="optimization",
        planning_time_ms=3.00,
        failure_reason=(
            "Exact physical validation failed."
        ),
        trajectory=trajectory,
        path=path,
        matched_objective_value=2.75,
    )

    assert not result.success
    assert result.trajectory is trajectory
    assert result.path is path
    assert result.matched_objective_value == 2.75

    print(
        "Failed-result debug output case passed."
    )


def validate_invalid_success_rejected() -> None:
    try:
        PlannerBenchmarkResult(
            planner_name="sampling",
            success=True,
            trajectory=None,
            path=None,
            planning_time_ms=1.0,
            failure_reason=None,
            matched_objective_value=1.0,
        )
    except ValueError:
        print(
            "Invalid success result rejection passed."
        )
        return

    raise AssertionError(
        "Invalid successful result was not rejected."
    )


def validate_success_without_objective_rejected() -> None:
    trajectory = build_test_trajectory()
    path = build_test_path()

    try:
        PlannerBenchmarkResult(
            planner_name="sampling",
            success=True,
            trajectory=trajectory,
            path=path,
            planning_time_ms=1.0,
            failure_reason=None,
            matched_objective_value=None,
        )
    except ValueError:
        print(
            "Missing matched-objective rejection passed."
        )
        return

    raise AssertionError(
        "Successful result without matched objective was not rejected."
    )


def validate_invalid_failure_rejected() -> None:
    try:
        PlannerBenchmarkResult(
            planner_name="optimization",
            success=False,
            trajectory=None,
            path=None,
            planning_time_ms=1.0,
            failure_reason=None,
            matched_objective_value=None,
        )
    except ValueError:
        print(
            "Invalid failure result rejection passed."
        )
        return

    raise AssertionError(
        "Invalid failed result was not rejected."
    )


def validate_negative_time_rejected() -> None:
    try:
        PlannerBenchmarkResult.failed(
            planner_name="sampling",
            planning_time_ms=-1.0,
            failure_reason="test failure",
        )
    except ValueError:
        print(
            "Negative planning-time rejection passed."
        )
        return

    raise AssertionError(
        "Negative planning time was not rejected."
    )


def main() -> None:
    validate_success_result()
    validate_failure_result()
    validate_failed_result_can_keep_output()
    validate_invalid_success_rejected()
    validate_success_without_objective_rejected()
    validate_invalid_failure_rejected()
    validate_negative_time_rejected()

    print()
    print(
        "All PlannerBenchmarkResult cases passed."
    )


if __name__ == "__main__":
    main()