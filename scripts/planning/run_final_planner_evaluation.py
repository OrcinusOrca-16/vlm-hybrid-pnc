from __future__ import annotations

import csv
import json
import subprocess

from dataclasses import (
    asdict,
)
from pathlib import Path
from statistics import (
    mean,
    median,
)

import matplotlib.pyplot as plt

from matplotlib.patches import Circle

from src.benchmark.evaluation_scenario_suite import (
    build_final_evaluation_scenarios,
)
from src.benchmark.metrics import (
    PlannerBenchmarkMetrics,
)
from src.benchmark.scenario_runner import (
    ScenarioBenchmarkRun,
    run_benchmark_scenario,
)
from src.config.planning import (
    DEFAULT_OPTIMIZATION_PLANNER_CONFIG,
    DEFAULT_SAMPLING_PLANNER_CONFIG,
)
from src.planning.frenet import (
    sl_to_xy,
    vehicle_state_to_frenet,
)


OUTPUT_DIR = Path(
    "outputs/final_evaluation"
)

FIGURE_DIR = (
    OUTPUT_DIR
    / "figures"
)

CSV_PATH = (
    OUTPUT_DIR
    / "final_planner_evaluation.csv"
)

SUMMARY_JSON_PATH = (
    OUTPUT_DIR
    / "final_planner_evaluation_summary.json"
)


REFERENCE_PLOT_RESOLUTION_M = 0.25


def current_git_commit() -> str:
    result = subprocess.run(
        [
            "git",
            "rev-parse",
            "HEAD",
        ],
        check=True,
        capture_output=True,
        text=True,
    )

    return result.stdout.strip()


def sanitize_filename(
    name: str,
) -> str:
    return "".join(
        character
        if (
            character.isalnum()
            or character in (
                "_",
                "-",
            )
        )
        else "_"
        for character in name
    )


def metric_row(
    scenario_name: str,
    metrics: PlannerBenchmarkMetrics,
    failure_reason: str | None,
) -> dict:
    return {
        "scenario": scenario_name,
        "planner": (
            metrics.planner_name
        ),
        "success": (
            metrics.planning_success
        ),
        "failure_reason": (
            failure_reason
        ),
        "canonical_objective": (
            metrics
            .canonical_matched_objective_value
        ),
        "terminal_lateral_error_m": (
            metrics
            .terminal_lateral_error_m
        ),
        "terminal_slope_error": (
            metrics
            .terminal_slope_error
        ),
        "rms_lateral_deviation_m": (
            metrics
            .rms_lateral_deviation_m
        ),
        "cartesian_path_length_m": (
            metrics
            .cartesian_path_length_m
        ),
        "minimum_exact_obstacle_clearance_m": (
            metrics
            .minimum_exact_obstacle_clearance_m
        ),
        "max_abs_cartesian_curvature_per_m": (
            metrics
            .max_abs_cartesian_curvature_per_m
        ),
        "rms_cartesian_curvature_per_m": (
            metrics
            .rms_cartesian_curvature_per_m
        ),
        "rms_curvature_rate_per_m2": (
            metrics
            .rms_curvature_rate_per_m2
        ),
        "planning_time_ms": (
            metrics.planning_time_ms
        ),
    }


def optional_mean(
    values: list[
        float | None
    ],
) -> float | None:
    valid_values = [
        value
        for value in values
        if value is not None
    ]

    if not valid_values:
        return None

    return mean(
        valid_values
    )


def optional_min(
    values: list[
        float | None
    ],
) -> float | None:
    valid_values = [
        value
        for value in values
        if value is not None
    ]

    if not valid_values:
        return None

    return min(
        valid_values
    )


def aggregate_common_success_metrics(
    runs: list[
        ScenarioBenchmarkRun
    ],
    planner: str,
) -> dict:
    if planner not in (
        "sampling",
        "optimization",
    ):
        raise ValueError(
            "Unknown planner."
        )

    if planner == "sampling":
        metrics_list = [
            run.sampling_metrics
            for run in runs
        ]
    else:
        metrics_list = [
            run.optimization_metrics
            for run in runs
        ]

    return {
        "mean_canonical_objective": (
            optional_mean([
                metrics
                .canonical_matched_objective_value
                for metrics in metrics_list
            ])
        ),
        "mean_terminal_lateral_error_m": (
            optional_mean([
                metrics
                .terminal_lateral_error_m
                for metrics in metrics_list
            ])
        ),
        "mean_terminal_slope_error": (
            optional_mean([
                metrics
                .terminal_slope_error
                for metrics in metrics_list
            ])
        ),
        "mean_rms_lateral_deviation_m": (
            optional_mean([
                metrics
                .rms_lateral_deviation_m
                for metrics in metrics_list
            ])
        ),
        "mean_cartesian_path_length_m": (
            optional_mean([
                metrics
                .cartesian_path_length_m
                for metrics in metrics_list
            ])
        ),
        "mean_minimum_obstacle_clearance_m": (
            optional_mean([
                metrics
                .minimum_exact_obstacle_clearance_m
                for metrics in metrics_list
            ])
        ),
        "worst_minimum_obstacle_clearance_m": (
            optional_min([
                metrics
                .minimum_exact_obstacle_clearance_m
                for metrics in metrics_list
            ])
        ),
        "mean_max_abs_curvature_per_m": (
            optional_mean([
                metrics
                .max_abs_cartesian_curvature_per_m
                for metrics in metrics_list
            ])
        ),
        "mean_rms_curvature_per_m": (
            optional_mean([
                metrics
                .rms_cartesian_curvature_per_m
                for metrics in metrics_list
            ])
        ),
        "mean_rms_curvature_rate_per_m2": (
            optional_mean([
                metrics
                .rms_curvature_rate_per_m2
                for metrics in metrics_list
            ])
        ),
    }


def runtime_summary(
    values: list[float],
) -> dict:
    return {
        "mean_ms": mean(
            values
        ),
        "median_ms": median(
            values
        ),
        "min_ms": min(
            values
        ),
        "max_ms": max(
            values
        ),
    }


def plot_scenario(
    run: ScenarioBenchmarkRun,
) -> None:
    scenario = run.scenario

    initial_frenet = (
        vehicle_state_to_frenet(
            vehicle_state=(
                scenario.vehicle_state
            ),
            reference_line=(
                scenario.reference_line
            ),
        )
    )

    start_s = (
        initial_frenet.s
    )

    end_s = min(
        (
            start_s
            + scenario.planning_horizon_m
        ),
        scenario.reference_line.length,
    )

    plot_point_count = max(
        2,
        int(
            (
                end_s
                - start_s
            )
            / REFERENCE_PLOT_RESOLUTION_M
        )
        + 1,
    )

    s_values = [
        (
            start_s
            + (
                end_s
                - start_s
            )
            * index
            / (
                plot_point_count
                - 1
            )
        )
        for index in range(
            plot_point_count
        )
    ]

    centerline = [
        sl_to_xy(
            s=s,
            l=0.0,
            reference_line=(
                scenario.reference_line
            ),
        )
        for s in s_values
    ]

    left_boundary = [
        sl_to_xy(
            s=s,
            l=scenario.road_max_l,
            reference_line=(
                scenario.reference_line
            ),
        )
        for s in s_values
    ]

    right_boundary = [
        sl_to_xy(
            s=s,
            l=scenario.road_min_l,
            reference_line=(
                scenario.reference_line
            ),
        )
        for s in s_values
    ]

    figure, axis = plt.subplots(
        figsize=(
            10,
            6,
        )
    )

    axis.plot(
        [
            point[0]
            for point in centerline
        ],
        [
            point[1]
            for point in centerline
        ],
        linestyle="--",
        label="reference",
    )

    axis.plot(
        [
            point[0]
            for point in left_boundary
        ],
        [
            point[1]
            for point in left_boundary
        ],
        linestyle=":",
        label="road boundary",
    )

    axis.plot(
        [
            point[0]
            for point in right_boundary
        ],
        [
            point[1]
            for point in right_boundary
        ],
        linestyle=":",
    )

    axis.scatter(
        [
            scenario.vehicle_state.x
        ],
        [
            scenario.vehicle_state.y
        ],
        marker="x",
        label="ego start",
    )

    for (
        obstacle_index,
        obstacle,
    ) in enumerate(
        scenario.obstacles
    ):
        obstacle_patch = Circle(
            (
                obstacle.x,
                obstacle.y,
            ),
            radius=(
                obstacle.radius_m
            ),
            fill=False,
            linestyle="--",
            label=(
                "obstacle"
                if obstacle_index == 0
                else None
            ),
        )

        axis.add_patch(
            obstacle_patch
        )

    if (
        run.sampling_result.success
        and run.sampling_result.path
        is not None
    ):
        axis.plot(
            [
                point.x
                for point
                in run.sampling_result.path
            ],
            [
                point.y
                for point
                in run.sampling_result.path
            ],
            label="Sampling",
        )

    if (
        run.optimization_result.success
        and run.optimization_result.path
        is not None
    ):
        axis.plot(
            [
                point.x
                for point
                in run.optimization_result.path
            ],
            [
                point.y
                for point
                in run.optimization_result.path
            ],
            label="Optimization",
        )

    axis.set_title(
        scenario.name
    )

    axis.set_xlabel(
        "x [m]"
    )

    axis.set_ylabel(
        "y [m]"
    )

    axis.axis(
        "equal"
    )

    axis.grid(
        True
    )

    axis.legend()

    figure.tight_layout()

    FIGURE_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    figure.savefig(
        FIGURE_DIR
        / (
            sanitize_filename(
                scenario.name
            )
            + ".png"
        ),
        dpi=160,
    )

    plt.close(
        figure
    )


def write_csv(
    rows: list[dict],
) -> None:
    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    with CSV_PATH.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=list(
                rows[0].keys()
            ),
        )

        writer.writeheader()
        writer.writerows(
            rows
        )


def main() -> None:
    scenarios = list(
        build_final_evaluation_scenarios()
    )

    assert len(
        scenarios
    ) == 9

    for scenario in scenarios:
        assert (
            scenario.resolution_m
            == DEFAULT_SAMPLING_PLANNER_CONFIG
            .resolution_m
        )

        assert (
            scenario.resolution_m
            == DEFAULT_OPTIMIZATION_PLANNER_CONFIG
            .resolution_m
        )

    frozen_commit = (
        current_git_commit()
    )

    print(
        "Final Planner Evaluation"
    )

    print(
        f"  frozen_commit={frozen_commit}"
    )

    print(
        f"  held_out_scenarios={len(scenarios)}"
    )

    print()

    runs = []
    csv_rows = []

    for (
        index,
        scenario,
    ) in enumerate(
        scenarios,
        start=1,
    ):
        print(
            f"[{index}/{len(scenarios)}] "
            f"{scenario.name}"
        )

        run = (
            run_benchmark_scenario(
                scenario
            )
        )

        runs.append(
            run
        )

        csv_rows.append(
            metric_row(
                scenario_name=(
                    scenario.name
                ),
                metrics=(
                    run.sampling_metrics
                ),
                failure_reason=(
                    run.sampling_result
                    .failure_reason
                ),
            )
        )

        csv_rows.append(
            metric_row(
                scenario_name=(
                    scenario.name
                ),
                metrics=(
                    run.optimization_metrics
                ),
                failure_reason=(
                    run.optimization_result
                    .failure_reason
                ),
            )
        )

        print(
            "  Sampling: "
            f"{'PASS' if run.sampling_result.success else 'FAIL'}"
            ", "
            f"{run.sampling_result.planning_time_ms:.4f} ms"
        )

        print(
            "  Optimization: "
            f"{'PASS' if run.optimization_result.success else 'FAIL'}"
            ", "
            f"{run.optimization_result.planning_time_ms:.4f} ms"
        )

        plot_scenario(
            run
        )

    sampling_success_runs = [
        run
        for run in runs
        if run.sampling_result.success
    ]

    optimization_success_runs = [
        run
        for run in runs
        if run.optimization_result.success
    ]

    common_success_runs = [
        run
        for run in runs
        if (
            run.sampling_result.success
            and run.optimization_result.success
        )
    ]

    sampling_failure_names = [
        run.scenario.name
        for run in runs
        if not run.sampling_result.success
    ]

    optimization_failure_names = [
        run.scenario.name
        for run in runs
        if not run.optimization_result.success
    ]

    common_success_names = [
        run.scenario.name
        for run in common_success_runs
    ]

    sampling_runtimes = [
        run.sampling_metrics
        .planning_time_ms
        for run in runs
    ]

    optimization_runtimes = [
        run.optimization_metrics
        .planning_time_ms
        for run in runs
    ]

    summary = {
        "metadata": {
            "frozen_git_commit": (
                frozen_commit
            ),
            "evaluation_scenario_count": (
                len(scenarios)
            ),
            "comparison_protocol": {
                "success_rate": (
                    "all held-out scenarios"
                ),
                "path_quality": (
                    "common-success scenarios only"
                ),
                "smoothness": (
                    "common-success scenarios only"
                ),
                "clearance": (
                    "common-success obstacle scenarios only"
                ),
                "runtime": (
                    "all held-out scenarios"
                ),
            },
            "sampling_config": asdict(
                DEFAULT_SAMPLING_PLANNER_CONFIG
            ),
            "optimization_config": asdict(
                DEFAULT_OPTIMIZATION_PLANNER_CONFIG
            ),
        },
        "scenario_names": [
            scenario.name
            for scenario in scenarios
        ],
        "success": {
            "sampling": {
                "count": len(
                    sampling_success_runs
                ),
                "total": len(
                    scenarios
                ),
                "rate": (
                    len(
                        sampling_success_runs
                    )
                    / len(scenarios)
                ),
                "failure_scenarios": (
                    sampling_failure_names
                ),
            },
            "optimization": {
                "count": len(
                    optimization_success_runs
                ),
                "total": len(
                    scenarios
                ),
                "rate": (
                    len(
                        optimization_success_runs
                    )
                    / len(scenarios)
                ),
                "failure_scenarios": (
                    optimization_failure_names
                ),
            },
        },
        "common_success": {
            "count": len(
                common_success_runs
            ),
            "scenario_names": (
                common_success_names
            ),
            "sampling": (
                aggregate_common_success_metrics(
                    runs=(
                        common_success_runs
                    ),
                    planner="sampling",
                )
            ),
            "optimization": (
                aggregate_common_success_metrics(
                    runs=(
                        common_success_runs
                    ),
                    planner="optimization",
                )
            ),
        },
        "runtime_all_scenarios": {
            "sampling": (
                runtime_summary(
                    sampling_runtimes
                )
            ),
            "optimization": (
                runtime_summary(
                    optimization_runtimes
                )
            ),
        },
        "scenario_results": [
            {
                "scenario": (
                    run.scenario.name
                ),
                "sampling": {
                    "success": (
                        run.sampling_result.success
                    ),
                    "failure_reason": (
                        run.sampling_result
                        .failure_reason
                    ),
                },
                "optimization": {
                    "success": (
                        run.optimization_result
                        .success
                    ),
                    "failure_reason": (
                        run.optimization_result
                        .failure_reason
                    ),
                },
            }
            for run in runs
        ],
    }

    write_csv(
        csv_rows
    )

    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    with SUMMARY_JSON_PATH.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            summary,
            file,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        )

    print()
    print(
        "Final evaluation summary"
    )

    print(
        "  Sampling success="
        f"{len(sampling_success_runs)}"
        f"/{len(scenarios)}"
    )

    print(
        "  Optimization success="
        f"{len(optimization_success_runs)}"
        f"/{len(scenarios)}"
    )

    print(
        "  Common success="
        f"{len(common_success_runs)}"
        f"/{len(scenarios)}"
    )

    print()

    print(
        "Runtime over all held-out scenarios"
    )

    print(
        "  Sampling mean="
        f"{mean(sampling_runtimes):.4f} ms"
    )

    print(
        "  Optimization mean="
        f"{mean(optimization_runtimes):.4f} ms"
    )

    print()

    print(
        f"CSV: {CSV_PATH}"
    )

    print(
        f"Summary: {SUMMARY_JSON_PATH}"
    )

    print(
        f"Figures: {FIGURE_DIR}"
    )


if __name__ == "__main__":
    main()