import math

from src.benchmark.metrics import (
    evaluate_planner_benchmark_metrics,
)
from src.benchmark.planner_benchmark import (
    PlannerBenchmarkResult,
)
from src.common.types import (
    FrenetTrajectory,
    FrenetTrajectoryPoint,
    PathPoint,
    StaticObstacle,
    VehicleParams,
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
                l=1.0,
                dl_ds=0.0,
                d2l_ds2=0.0,
            ),
            FrenetTrajectoryPoint(
                s=2.0,
                l=0.0,
                dl_ds=123.0,
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
            curvature=0.2,
            s=1.0,
        ),
        PathPoint(
            x=3.0,
            y=0.0,
            yaw=0.0,
            curvature=-0.1,
            s=3.0,
        ),
    ]


def build_vehicle_params() -> VehicleParams:
    return VehicleParams(
        wheelbase_m=1.0,
        front_overhang_m=0.5,
        rear_overhang_m=0.5,
        width_m=1.0,
    )


def validate_success_metrics() -> None:
    result = PlannerBenchmarkResult.succeeded(
        planner_name="test_planner",
        trajectory=build_test_trajectory(),
        path=build_test_path(),
        planning_time_ms=4.5,
        matched_objective_value=7.5,
    )

    obstacle = StaticObstacle(
        x=0.0,
        y=3.0,
        radius_m=0.5,
    )

    metrics = evaluate_planner_benchmark_metrics(
        result=result,
        vehicle_params=build_vehicle_params(),
        obstacles=[
            obstacle,
        ],
        lateral_reference_m=0.0,
        terminal_lateral_target_m=0.5,
        terminal_first_derivative_target=0.0,
    )

    assert metrics.planner_name == "test_planner"
    assert metrics.planning_success

    assert metrics.canonical_matched_objective_value == 7.5
    assert metrics.planning_time_ms == 4.5

    assert math.isclose(
        metrics.terminal_lateral_error_m,
        0.5,
        abs_tol=1e-12,
    )

    # Common terminal slope:
    # (0.0 - 1.0) / (2.0 - 1.0) = -1.0.
    #
    # The stored final dl_ds=123.0 is intentionally ignored.
    assert math.isclose(
        metrics.terminal_slope_error,
        1.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        metrics.rms_lateral_deviation_m,
        math.sqrt(1.0 / 3.0),
        abs_tol=1e-12,
    )

    assert math.isclose(
        metrics.cartesian_path_length_m,
        3.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        metrics.minimum_exact_obstacle_clearance_m,
        2.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        metrics.max_abs_cartesian_curvature_per_m,
        0.2,
        abs_tol=1e-12,
    )

    assert math.isclose(
        metrics.rms_cartesian_curvature_per_m,
        math.sqrt(
            0.05 / 3.0
        ),
        abs_tol=1e-12,
    )

    expected_rms_curvature_rate = math.sqrt(
        (
            0.2 ** 2
            + (-0.15) ** 2
        )
        / 2.0
    )

    assert math.isclose(
        metrics.rms_curvature_rate_per_m2,
        expected_rms_curvature_rate,
        abs_tol=1e-12,
    )

    print(
        "Successful benchmark metrics passed."
    )
    print(
        "  terminal_lateral_error="
        f"{metrics.terminal_lateral_error_m:.6f}"
    )
    print(
        "  terminal_slope_error="
        f"{metrics.terminal_slope_error:.6f}"
    )
    print(
        "  rms_lateral_deviation="
        f"{metrics.rms_lateral_deviation_m:.6f}"
    )
    print(
        "  min_clearance="
        f"{metrics.minimum_exact_obstacle_clearance_m:.6f}"
    )
    print(
        "  max_abs_curvature="
        f"{metrics.max_abs_cartesian_curvature_per_m:.6f}"
    )
    print(
        "  rms_curvature_rate="
        f"{metrics.rms_curvature_rate_per_m2:.6f}"
    )


def validate_failure_metrics() -> None:
    result = PlannerBenchmarkResult.failed(
        planner_name="test_planner",
        planning_time_ms=2.0,
        failure_reason="Physical validation failed.",
        trajectory=build_test_trajectory(),
        path=build_test_path(),
        matched_objective_value=1.0,
    )

    metrics = evaluate_planner_benchmark_metrics(
        result=result,
        vehicle_params=build_vehicle_params(),
        obstacles=[],
        lateral_reference_m=0.0,
        terminal_lateral_target_m=0.0,
        terminal_first_derivative_target=0.0,
    )

    assert not metrics.planning_success
    assert metrics.planning_time_ms == 2.0

    assert metrics.canonical_matched_objective_value is None
    assert metrics.terminal_lateral_error_m is None
    assert metrics.terminal_slope_error is None
    assert metrics.rms_lateral_deviation_m is None
    assert metrics.cartesian_path_length_m is None
    assert metrics.minimum_exact_obstacle_clearance_m is None
    assert metrics.max_abs_cartesian_curvature_per_m is None
    assert metrics.rms_cartesian_curvature_per_m is None
    assert metrics.rms_curvature_rate_per_m2 is None

    print(
        "Failed benchmark metrics suppression passed."
    )


def validate_no_obstacle_clearance() -> None:
    result = PlannerBenchmarkResult.succeeded(
        planner_name="test_planner",
        trajectory=build_test_trajectory(),
        path=build_test_path(),
        planning_time_ms=1.0,
        matched_objective_value=1.0,
    )

    metrics = evaluate_planner_benchmark_metrics(
        result=result,
        vehicle_params=build_vehicle_params(),
        obstacles=[],
        lateral_reference_m=0.0,
        terminal_lateral_target_m=0.0,
        terminal_first_derivative_target=0.0,
    )

    assert (
        metrics.minimum_exact_obstacle_clearance_m
        is None
    )

    print(
        "No-obstacle clearance case passed."
    )


def main() -> None:
    validate_success_metrics()
    validate_failure_metrics()
    validate_no_obstacle_clearance()

    print()
    print(
        "All common benchmark metric cases passed."
    )


if __name__ == "__main__":
    main()