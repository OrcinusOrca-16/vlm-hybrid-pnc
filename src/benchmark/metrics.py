from dataclasses import dataclass
import math

from src.benchmark.planner_benchmark import (
    PlannerBenchmarkResult,
)
from src.common.types import (
    PathPoint,
    StaticObstacle,
    VehicleParams,
)
from src.planning.hard_constraints import (
    obstacle_clearance,
)


@dataclass(frozen=True)
class PlannerBenchmarkMetrics:
    """Planner-independent metrics for one planner result."""

    planner_name: str
    planning_success: bool

    canonical_matched_objective_value: float | None

    terminal_lateral_error_m: float | None
    terminal_slope_error: float | None
    rms_lateral_deviation_m: float | None

    cartesian_path_length_m: float | None
    minimum_exact_obstacle_clearance_m: float | None

    max_abs_cartesian_curvature_per_m: float | None
    rms_cartesian_curvature_per_m: float | None
    rms_curvature_rate_per_m2: float | None

    planning_time_ms: float


def _rms(
    values: list[float],
) -> float:
    if not values:
        raise ValueError(
            "RMS input must not be empty."
        )

    return math.sqrt(
        sum(
            value ** 2
            for value in values
        )
        / len(values)
    )


def _terminal_slope(
    trajectory_s: list[float],
    lateral_values: list[float],
) -> float:
    if len(trajectory_s) < 2:
        raise ValueError(
            "At least two trajectory points are required."
        )

    delta_s = (
        trajectory_s[-1]
        - trajectory_s[-2]
    )

    if delta_s <= 0.0:
        raise ValueError(
            "Trajectory s must be strictly increasing."
        )

    return (
        lateral_values[-1]
        - lateral_values[-2]
    ) / delta_s


def _minimum_exact_obstacle_clearance(
    path: list[PathPoint],
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
) -> float | None:
    if not obstacles:
        return None

    return min(
        obstacle_clearance(
            path_point=path_point,
            vehicle_params=vehicle_params,
            obstacle=obstacle,
        )
        for path_point in path
        for obstacle in obstacles
    )


def _rms_curvature_rate(
    path: list[PathPoint],
) -> float:
    if len(path) < 2:
        raise ValueError(
            "At least two path points are required."
        )

    curvature_rates = []

    for previous, current in zip(
        path[:-1],
        path[1:],
    ):
        delta_s = (
            current.s
            - previous.s
        )

        if delta_s <= 0.0:
            raise ValueError(
                "Cartesian path s must be strictly increasing."
            )

        curvature_rates.append(
            (
                current.curvature
                - previous.curvature
            )
            / delta_s
        )

    return _rms(
        curvature_rates
    )


def evaluate_planner_benchmark_metrics(
    result: PlannerBenchmarkResult,
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
    lateral_reference_m: float,
    terminal_lateral_target_m: float,
    terminal_first_derivative_target: float,
) -> PlannerBenchmarkMetrics:
    """Evaluate common metrics from one planner output."""

    if not result.success:
        return PlannerBenchmarkMetrics(
            planner_name=result.planner_name,
            planning_success=False,
            canonical_matched_objective_value=None,
            terminal_lateral_error_m=None,
            terminal_slope_error=None,
            rms_lateral_deviation_m=None,
            cartesian_path_length_m=None,
            minimum_exact_obstacle_clearance_m=None,
            max_abs_cartesian_curvature_per_m=None,
            rms_cartesian_curvature_per_m=None,
            rms_curvature_rate_per_m2=None,
            planning_time_ms=result.planning_time_ms,
        )

    if result.trajectory is None or result.path is None:
        raise RuntimeError(
            "Successful planner result is missing its output."
        )

    trajectory = result.trajectory
    path = result.path

    if len(trajectory.points) < 2:
        raise ValueError(
            "Successful trajectory must contain at least two points."
        )

    if len(path) < 2:
        raise ValueError(
            "Successful path must contain at least two points."
        )

    if len(trajectory.points) != len(path):
        raise ValueError(
            "Frenet trajectory and Cartesian path must "
            "contain the same number of points."
        )

    trajectory_s = [
        point.s
        for point in trajectory.points
    ]

    lateral_values = [
        point.l
        for point in trajectory.points
    ]

    terminal_lateral_error_m = abs(
        lateral_values[-1]
        - terminal_lateral_target_m
    )

    terminal_slope = _terminal_slope(
        trajectory_s=trajectory_s,
        lateral_values=lateral_values,
    )

    terminal_slope_error = abs(
        terminal_slope
        - terminal_first_derivative_target
    )

    lateral_errors = [
        lateral
        - lateral_reference_m
        for lateral in lateral_values
    ]

    rms_lateral_deviation_m = _rms(
        lateral_errors
    )

    cartesian_path_length_m = (
        path[-1].s
    )

    minimum_exact_obstacle_clearance_m = (
        _minimum_exact_obstacle_clearance(
            path=path,
            vehicle_params=vehicle_params,
            obstacles=obstacles,
        )
    )

    curvatures = [
        point.curvature
        for point in path
    ]

    max_abs_cartesian_curvature_per_m = max(
        abs(curvature)
        for curvature in curvatures
    )

    rms_cartesian_curvature_per_m = _rms(
        curvatures
    )

    rms_curvature_rate_per_m2 = (
        _rms_curvature_rate(
            path
        )
    )

    return PlannerBenchmarkMetrics(
        planner_name=result.planner_name,
        planning_success=True,
        canonical_matched_objective_value=(
            result.matched_objective_value
        ),
        terminal_lateral_error_m=(
            terminal_lateral_error_m
        ),
        terminal_slope_error=(
            terminal_slope_error
        ),
        rms_lateral_deviation_m=(
            rms_lateral_deviation_m
        ),
        cartesian_path_length_m=(
            cartesian_path_length_m
        ),
        minimum_exact_obstacle_clearance_m=(
            minimum_exact_obstacle_clearance_m
        ),
        max_abs_cartesian_curvature_per_m=(
            max_abs_cartesian_curvature_per_m
        ),
        rms_cartesian_curvature_per_m=(
            rms_cartesian_curvature_per_m
        ),
        rms_curvature_rate_per_m2=(
            rms_curvature_rate_per_m2
        ),
        planning_time_ms=result.planning_time_ms,
    )