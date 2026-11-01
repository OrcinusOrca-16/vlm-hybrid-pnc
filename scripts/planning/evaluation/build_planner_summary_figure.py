"""Build a planning-only summary figure from the frozen planner CSV.

This script does not rerun planners. It reads the frozen evaluation artifact and
visualizes planning success, runtime, canonical objective, and path geometry.

Color convention:
    Sampling     = red
    Optimization = purple
"""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from scripts.plotting_style import OPTIMIZATION_COLOR, SAMPLING_COLOR

INPUT_CSV = Path("outputs/final_evaluation/final_planner_evaluation.csv")
OUTPUT_PATH = Path("outputs/final_evaluation/figures/planning_only_summary.png")


def as_bool(value: str) -> bool:
    return value.strip().lower() == "true"


def as_float(value: str) -> float | None:
    return None if value == "" else float(value)


def load_rows() -> list[dict]:
    with INPUT_CSV.open("r", encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))

    float_keys = (
        "canonical_objective",
        "rms_lateral_deviation_m",
        "max_abs_cartesian_curvature_per_m",
        "rms_curvature_rate_per_m2",
        "planning_time_ms",
    )
    for row in rows:
        row["success"] = as_bool(row["success"])
        for key in float_keys:
            row[key] = as_float(row[key])
    return rows


def common_success_scenarios(rows: list[dict]) -> list[str]:
    scenarios = sorted({row["scenario"] for row in rows})
    return [
        scenario
        for scenario in scenarios
        if all(
            any(row["scenario"] == scenario and row["planner"] == planner and row["success"] for row in rows)
            for planner in ("sampling", "optimization")
        )
    ]


def planner_rows(rows: list[dict], planner: str) -> list[dict]:
    return [row for row in rows if row["planner"] == planner]


def common_rows(rows: list[dict], scenarios: list[str], planner: str) -> list[dict]:
    return [row for row in rows if row["planner"] == planner and row["scenario"] in scenarios]


def mean_value(rows: list[dict], key: str) -> float:
    values = [row[key] for row in rows if row[key] is not None]
    if not values:
        raise ValueError(f"No valid values for {key}")
    return float(np.mean(values))


def grouped_bar(
    ax, labels: tuple[str, ...], sampling: tuple[float, ...], optimization: tuple[float, ...], ylabel: str, title: str
) -> None:
    x = np.arange(len(labels))
    width = 0.36
    ax.bar(x - width / 2, sampling, width, color=SAMPLING_COLOR, label="Sampling")
    ax.bar(x + width / 2, optimization, width, color=OPTIMIZATION_COLOR, label="Optimization")
    ax.set_xticks(x, labels)
    ax.set_ylabel(ylabel)
    ax.set_title(title)


def main() -> None:
    rows = load_rows()
    scenarios = common_success_scenarios(rows)
    sampling_all = planner_rows(rows, "sampling")
    optimization_all = planner_rows(rows, "optimization")
    sampling_common = common_rows(rows, scenarios, "sampling")
    optimization_common = common_rows(rows, scenarios, "optimization")

    sampling_success = sum(row["success"] for row in sampling_all)
    optimization_success = sum(row["success"] for row in optimization_all)

    sampling_runtime = mean_value(sampling_all, "planning_time_ms")
    optimization_runtime = mean_value(optimization_all, "planning_time_ms")
    sampling_objective = mean_value(sampling_common, "canonical_objective")
    optimization_objective = mean_value(optimization_common, "canonical_objective")
    sampling_rms_l = mean_value(sampling_common, "rms_lateral_deviation_m")
    optimization_rms_l = mean_value(optimization_common, "rms_lateral_deviation_m")
    sampling_peak_kappa = mean_value(sampling_common, "max_abs_cartesian_curvature_per_m")
    optimization_peak_kappa = mean_value(optimization_common, "max_abs_cartesian_curvature_per_m")

    figure, axes = plt.subplots(2, 2, figsize=(10.5, 7.5))

    grouped_bar(
        axes[0, 0],
        ("Success",),
        (sampling_success,),
        (optimization_success,),
        "Successful scenarios / 9",
        "Planning Coverage",
    )
    axes[0, 0].set_ylim(0, 10)

    grouped_bar(
        axes[0, 1],
        ("Mean runtime",),
        (sampling_runtime,),
        (optimization_runtime,),
        "Planning time [ms]",
        "Planner Runtime (all 9 scenarios)",
    )

    grouped_bar(
        axes[1, 0],
        ("Canonical objective", "RMS lateral deviation"),
        (sampling_objective, sampling_rms_l),
        (optimization_objective, optimization_rms_l),
        "Mean value",
        "Path Objective / Lateral Deviation (common 8)",
    )

    grouped_bar(
        axes[1, 1],
        ("Mean max |curvature|",),
        (sampling_peak_kappa,),
        (optimization_peak_kappa,),
        "Curvature [1/m]",
        "Peak Curvature Trade-off (common 8)",
    )

    handles, labels = axes[0, 0].get_legend_handles_labels()
    figure.legend(handles, labels, loc="upper center", ncol=2)
    figure.suptitle("Frozen Planning-Only Benchmark Summary", y=0.98)
    figure.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))

    OUTPUT_PATH.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(OUTPUT_PATH, dpi=180)
    plt.close(figure)

    print(f"Saved: {OUTPUT_PATH}")
    print(f"Sampling success: {sampling_success}/9")
    print(f"Optimization success: {optimization_success}/9")
    print(f"Mean runtime: Sampling={sampling_runtime:.3f} ms, Optimization={optimization_runtime:.3f} ms")
    print(
        f"Mean canonical objective on common-success cases: Sampling={sampling_objective:.4f}, Optimization={optimization_objective:.4f}"
    )


if __name__ == "__main__":
    main()
