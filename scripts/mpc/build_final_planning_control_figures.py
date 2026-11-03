"""Build portfolio figures from the frozen final Planning & Control benchmark CSV.

Run after:
    python -m scripts.mpc.run_final_planning_control_benchmark

This script does not rerun or tune any planner/controller logic.
"""

from __future__ import annotations

import csv
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

from scripts.plotting_style import OPTIMIZATION_COLOR, SAMPLING_COLOR


INPUT_CSV = Path("outputs/final_planning_control/final_planning_control_benchmark.csv")
OUTPUT_DIR = Path("outputs/final_planning_control/figures")


def as_bool(value: str) -> bool:
    return value.strip().lower() == "true"


def as_float(value: str) -> float | None:
    return None if value == "" else float(value)


def load_rows() -> list[dict]:
    with INPUT_CSV.open("r", encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))

    float_keys = (
        "safe_speed_mps",
        "system_rms_lateral_error_m",
        "same_speed_rms_lateral_error_m",
        "same_speed_rms_heading_error_rad",
        "same_speed_max_abs_feedback_rad",
        "planning_time_ms",
    )
    for row in rows:
        row["planner_success"] = as_bool(row["planner_success"])
        row["system_tracking_success"] = as_bool(row["system_tracking_success"])
        for key in float_keys:
            row[key] = as_float(row.get(key, ""))
    return rows


def common_success_scenarios(rows: list[dict]) -> list[str]:
    scenarios = sorted({row["scenario"] for row in rows})
    return [
        scenario
        for scenario in scenarios
        if all(
            any(row["scenario"] == scenario and row["planner"] == planner and row["planner_success"] for row in rows)
            for planner in ("sampling", "optimization")
        )
    ]


def row_map(rows: list[dict], scenarios: list[str], planner: str) -> dict[str, dict]:
    return {
        scenario: next(row for row in rows if row["scenario"] == scenario and row["planner"] == planner)
        for scenario in scenarios
    }


def short_name(name: str) -> str:
    replacements = {
        "evaluation_": "",
        "obstacle_longitudinal_": "obs_s_",
        "obstacle_lateral_": "obs_l_",
        "road_half_width_": "road_",
        "lateral_recovery": "lat_recovery",
        "curved_obstacle": "curved_obs",
        "curved_nominal": "curved_nom",
    }
    for source, target in replacements.items():
        name = name.replace(source, target)
    return name


def mean_metric(rows: list[dict], scenarios: list[str], planner: str, key: str) -> float:
    mapping = row_map(rows, scenarios, planner)
    values = [mapping[scenario][key] for scenario in scenarios if mapping[scenario][key] is not None]
    if not values:
        raise ValueError(f"No values available for {planner=} {key=}")
    return float(np.mean(values))


def save_success_counts(rows: list[dict]) -> None:
    metrics = ("Planner success", "Closed-loop success")
    sampling = (
        sum(row["planner"] == "sampling" and row["planner_success"] for row in rows),
        sum(row["planner"] == "sampling" and row["system_tracking_success"] for row in rows),
    )
    optimization = (
        sum(row["planner"] == "optimization" and row["planner_success"] for row in rows),
        sum(row["planner"] == "optimization" and row["system_tracking_success"] for row in rows),
    )

    x = np.arange(len(metrics))
    width = 0.34
    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    ax.bar(x - width / 2, sampling, width, label="Sampling", color=SAMPLING_COLOR)
    ax.bar(x + width / 2, optimization, width, label="Optimization", color=OPTIMIZATION_COLOR)
    ax.set_xticks(x, metrics)
    ax.set_ylim(0, 10)
    ax.set_ylabel("Successful scenarios / 9")
    ax.set_title("Frozen Planning & Control Benchmark Success")
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / "success_counts.png", dpi=180)
    plt.close(fig)


def save_grouped_scenario_metric(rows: list[dict], scenarios: list[str], key: str, ylabel: str, title: str, filename: str) -> None:
    sampling = row_map(rows, scenarios, "sampling")
    optimization = row_map(rows, scenarios, "optimization")
    labels = [short_name(scenario) for scenario in scenarios]
    sampling_values = [sampling[scenario][key] for scenario in scenarios]
    optimization_values = [optimization[scenario][key] for scenario in scenarios]

    x = np.arange(len(scenarios))
    width = 0.38
    fig, ax = plt.subplots(figsize=(11.5, 5.0))
    ax.bar(x - width / 2, sampling_values, width, label="Sampling", color=SAMPLING_COLOR)
    ax.bar(x + width / 2, optimization_values, width, label="Optimization", color=OPTIMIZATION_COLOR)
    ax.set_xticks(x, labels, rotation=30, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / filename, dpi=180)
    plt.close(fig)


def save_control_summary_figure(rows: list[dict], scenarios: list[str]) -> None:
    metrics = (
        ("safe_speed_mps", "Safe Speed", "Mean constant safe speed [m/s]", "higher is better", ".3f"),
        (
            "system_rms_lateral_error_m",
            "System Lateral Error",
            "Mean system RMS lateral error [m]",
            "lower is better",
            ".5f",
        ),
        (
            "same_speed_rms_lateral_error_m",
            "Same-Speed Lateral Error",
            "Mean same-speed RMS lateral error [m]",
            "lower is better",
            ".5f",
        ),
        (
            "same_speed_max_abs_feedback_rad",
            "Same-Speed Feedback Effort",
            "Mean same-speed max |feedback| [rad]",
            "lower is better",
            ".4f",
        ),
    )

    fig, axes = plt.subplots(2, 2, figsize=(12.0, 8.5))
    x = np.array((0.0, 1.0))
    width = 0.42

    for ax, (key, panel_title, metric_label, direction, value_format) in zip(axes.flat, metrics):
        values = (
            mean_metric(rows, scenarios, "sampling", key),
            mean_metric(rows, scenarios, "optimization", key),
        )
        bars = ax.bar(x, values, width=width, color=(SAMPLING_COLOR, OPTIMIZATION_COLOR))
        ax.set_xticks(x, ("Sampling", "Optimization"))
        ax.set_xlim(-0.72, 1.72)
        ax.set_xlabel(metric_label, labelpad=8)
        ax.set_title(panel_title, fontsize=12, fontweight="bold", pad=24)
        ax.text(0.5, 1.035, direction, transform=ax.transAxes, ha="center", va="bottom", fontsize=9)
        ax.grid(axis="y", alpha=0.25)
        ax.set_axisbelow(True)

        ymax = max(values)
        ax.set_ylim(0.0, ymax * 1.14)
        for bar, value in zip(bars, values):
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                bar.get_height() + 0.025 * ymax,
                format(value, value_format),
                ha="center",
                va="bottom",
                fontsize=9,
            )

    handles = (
        Patch(facecolor=SAMPLING_COLOR, label="Sampling"),
        Patch(facecolor=OPTIMIZATION_COLOR, label="Optimization"),
    )
    fig.suptitle("Closed-Loop Control Summary\nCommon-success scenarios only", y=0.965, fontsize=16)
    fig.legend(handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.885), ncol=2, frameon=False)
    fig.subplots_adjust(left=0.105, right=0.895, bottom=0.135, top=0.755, wspace=0.40, hspace=0.55)
    fig.savefig(OUTPUT_DIR / "closed_loop_control_summary.png", dpi=180)
    fig.savefig(OUTPUT_DIR / "closed_loop_control_summary.svg")
    plt.close(fig)


def main() -> None:
    rows = load_rows()
    scenarios = common_success_scenarios(rows)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    save_success_counts(rows)
    save_grouped_scenario_metric(
        rows,
        scenarios,
        key="safe_speed_mps",
        ylabel="Constant safe speed [m/s]",
        title="Planner-Controller Compatibility on Common-Success Scenarios",
        filename="safe_speed_by_scenario.png",
    )
    save_grouped_scenario_metric(
        rows,
        scenarios,
        key="same_speed_rms_lateral_error_m",
        ylabel="RMS lateral tracking error [m]",
        title="Same-Speed Closed-Loop Lateral Tracking",
        filename="same_speed_lateral_rms_by_scenario.png",
    )
    save_grouped_scenario_metric(
        rows,
        scenarios,
        key="planning_time_ms",
        ylabel="Single-run planning time [ms]",
        title="Planning Computation Time on Common-Success Scenarios",
        filename="planning_time_by_scenario.png",
    )
    save_control_summary_figure(rows, scenarios)

    print(f"Saved figures to: {OUTPUT_DIR}")
    for path in sorted(OUTPUT_DIR.glob("*.*")):
        print(f"  {path}")


if __name__ == "__main__":
    main()
