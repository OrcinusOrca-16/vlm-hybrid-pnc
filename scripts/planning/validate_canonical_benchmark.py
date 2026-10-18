from src.benchmark.canonical_runner import (
    run_canonical_benchmark,
)


def main() -> None:
    benchmark = run_canonical_benchmark()

    sampling_result = benchmark.sampling_result
    optimization_result = (
        benchmark.optimization_result
    )

    sampling_metrics = benchmark.sampling_metrics
    optimization_metrics = (
        benchmark.optimization_metrics
    )

    print(
        "Sampling result: "
        f"success={sampling_result.success}, "
        f"reason={sampling_result.failure_reason}"
    )

    print(
        "Optimization result: "
        f"success={optimization_result.success}, "
        f"reason={optimization_result.failure_reason}"
    )

    assert sampling_result.success
    assert optimization_result.success

    assert sampling_metrics.planning_success
    assert optimization_metrics.planning_success

    assert (
        sampling_metrics
        .canonical_matched_objective_value
        is not None
    )
    assert (
        optimization_metrics
        .canonical_matched_objective_value
        is not None
    )

    assert (
        sampling_metrics
        .minimum_exact_obstacle_clearance_m
        is not None
    )
    assert (
        optimization_metrics
        .minimum_exact_obstacle_clearance_m
        is not None
    )

    assert (
        sampling_metrics
        .minimum_exact_obstacle_clearance_m
        > benchmark.scenario.minimum_clearance_m
    )

    assert (
        optimization_metrics
        .minimum_exact_obstacle_clearance_m
        > benchmark.scenario.minimum_clearance_m
    )

    assert (
        sampling_metrics
        .max_abs_cartesian_curvature_per_m
        is not None
    )
    assert (
        optimization_metrics
        .max_abs_cartesian_curvature_per_m
        is not None
    )

    assert (
        sampling_metrics
        .max_abs_cartesian_curvature_per_m
        <= benchmark.scenario.max_curvature
    )

    assert (
        optimization_metrics
        .max_abs_cartesian_curvature_per_m
        <= benchmark.scenario.max_curvature
    )

    print(
        "Canonical planner benchmark passed."
    )

    print()
    print("Sampling:")
    print(
        "  objective="
        f"{sampling_metrics.canonical_matched_objective_value:.6f}"
    )
    print(
        "  terminal_l_error="
        f"{sampling_metrics.terminal_lateral_error_m:.6f}"
    )
    print(
        "  terminal_slope_error="
        f"{sampling_metrics.terminal_slope_error:.6f}"
    )
    print(
        "  rms_lateral="
        f"{sampling_metrics.rms_lateral_deviation_m:.6f}"
    )
    print(
        "  path_length="
        f"{sampling_metrics.cartesian_path_length_m:.6f}"
    )
    print(
        "  min_clearance="
        f"{sampling_metrics.minimum_exact_obstacle_clearance_m:.6f}"
    )
    print(
        "  max_curvature="
        f"{sampling_metrics.max_abs_cartesian_curvature_per_m:.6f}"
    )
    print(
        "  rms_curvature="
        f"{sampling_metrics.rms_cartesian_curvature_per_m:.6f}"
    )
    print(
        "  rms_curvature_rate="
        f"{sampling_metrics.rms_curvature_rate_per_m2:.6f}"
    )
    print(
        "  planning_time_ms="
        f"{sampling_metrics.planning_time_ms:.4f}"
    )

    print()
    print("Optimization:")
    print(
        "  objective="
        f"{optimization_metrics.canonical_matched_objective_value:.6f}"
    )
    print(
        "  terminal_l_error="
        f"{optimization_metrics.terminal_lateral_error_m:.6f}"
    )
    print(
        "  terminal_slope_error="
        f"{optimization_metrics.terminal_slope_error:.6f}"
    )
    print(
        "  rms_lateral="
        f"{optimization_metrics.rms_lateral_deviation_m:.6f}"
    )
    print(
        "  path_length="
        f"{optimization_metrics.cartesian_path_length_m:.6f}"
    )
    print(
        "  min_clearance="
        f"{optimization_metrics.minimum_exact_obstacle_clearance_m:.6f}"
    )
    print(
        "  max_curvature="
        f"{optimization_metrics.max_abs_cartesian_curvature_per_m:.6f}"
    )
    print(
        "  rms_curvature="
        f"{optimization_metrics.rms_cartesian_curvature_per_m:.6f}"
    )
    print(
        "  rms_curvature_rate="
        f"{optimization_metrics.rms_curvature_rate_per_m2:.6f}"
    )
    print(
        "  planning_time_ms="
        f"{optimization_metrics.planning_time_ms:.4f}"
    )

    print()
    print(
        "Both planners passed the canonical benchmark."
    )


if __name__ == "__main__":
    main()