from src.benchmark.metrics import (
    PlannerBenchmarkMetrics,
)


def _format_float(
    value: float | None,
    precision: int = 6,
) -> str:
    if value is None:
        return "-"

    return f"{value:.{precision}f}"


def format_benchmark_comparison_table(
    sampling: PlannerBenchmarkMetrics,
    optimization: PlannerBenchmarkMetrics,
) -> str:
    """Format two planner metric sets into one terminal table."""

    rows = [
        (
            "planning success",
            str(sampling.planning_success),
            str(optimization.planning_success),
        ),
        (
            "canonical objective",
            _format_float(
                sampling
                .canonical_matched_objective_value
            ),
            _format_float(
                optimization
                .canonical_matched_objective_value
            ),
        ),
        (
            "terminal lateral error [m]",
            _format_float(
                sampling.terminal_lateral_error_m
            ),
            _format_float(
                optimization.terminal_lateral_error_m
            ),
        ),
        (
            "terminal slope error",
            _format_float(
                sampling.terminal_slope_error
            ),
            _format_float(
                optimization.terminal_slope_error
            ),
        ),
        (
            "RMS lateral deviation [m]",
            _format_float(
                sampling.rms_lateral_deviation_m
            ),
            _format_float(
                optimization.rms_lateral_deviation_m
            ),
        ),
        (
            "Cartesian path length [m]",
            _format_float(
                sampling.cartesian_path_length_m
            ),
            _format_float(
                optimization.cartesian_path_length_m
            ),
        ),
        (
            "minimum exact clearance [m]",
            _format_float(
                sampling
                .minimum_exact_obstacle_clearance_m
            ),
            _format_float(
                optimization
                .minimum_exact_obstacle_clearance_m
            ),
        ),
        (
            "max abs curvature [1/m]",
            _format_float(
                sampling
                .max_abs_cartesian_curvature_per_m
            ),
            _format_float(
                optimization
                .max_abs_cartesian_curvature_per_m
            ),
        ),
        (
            "RMS curvature [1/m]",
            _format_float(
                sampling
                .rms_cartesian_curvature_per_m
            ),
            _format_float(
                optimization
                .rms_cartesian_curvature_per_m
            ),
        ),
        (
            "RMS curvature rate [1/m^2]",
            _format_float(
                sampling
                .rms_curvature_rate_per_m2
            ),
            _format_float(
                optimization
                .rms_curvature_rate_per_m2
            ),
        ),
        (
            "planning time [ms]",
            _format_float(
                sampling.planning_time_ms,
                precision=4,
            ),
            _format_float(
                optimization.planning_time_ms,
                precision=4,
            ),
        ),
    ]

    headers = (
        "Metric",
        sampling.planner_name,
        optimization.planner_name,
    )

    metric_width = max(
        len(headers[0]),
        max(
            len(row[0])
            for row in rows
        ),
    )

    sampling_width = max(
        len(headers[1]),
        max(
            len(row[1])
            for row in rows
        ),
    )

    optimization_width = max(
        len(headers[2]),
        max(
            len(row[2])
            for row in rows
        ),
    )

    lines = [
        (
            f"{headers[0]:<{metric_width}} | "
            f"{headers[1]:>{sampling_width}} | "
            f"{headers[2]:>{optimization_width}}"
        ),
        (
            f"{'-' * metric_width}-+-"
            f"{'-' * sampling_width}-+-"
            f"{'-' * optimization_width}"
        ),
    ]

    for metric, sampling_value, optimization_value in rows:
        lines.append(
            f"{metric:<{metric_width}} | "
            f"{sampling_value:>{sampling_width}} | "
            f"{optimization_value:>{optimization_width}}"
        )

    return "\n".join(lines)