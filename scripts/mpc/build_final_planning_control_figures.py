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


INPUT_CSV = Path("outputs/final_planning_control/final_planning_control_benchmark.csv")
OUTPUT_DIR = Path("outputs/final_planning_control/figures")


def as_bool(value: str) -> bool:
    return value.strip().lower() == "true"


def as_float(value: str) -> float | None:
    return None if value == "" else float(value)


def load_rows() -> list[dict]:
    with INPUT_CSV.open("r", encoding="utf-8", newline="") as file:
        rows = list(csv.DictReader(file))

    for row in rows:
        row["planner_success"] = as_bool(row["planner_success"])
        row["system_tracking_success"] = as_bool(row["system_tracking_success"])
        for key in (
            "safe_speed_mps",
            "same_speed_rms_lateral_error_m",
            "same_speed_rms_heading_error_rad",
            "planning_time_ms",
        ):
            row[key] = as_float(row[key])
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


def save_success_counts(rows: list[dict]) -> None:
    planners = ("Sampling", "Optimization")
    keys = ("sampling", "optimization")
    planner_counts = [sum(row["planner"] == key and row["planner_success"] for row in rows) for key in keys]
    closed_loop_counts = [sum(row["planner"] == key and row["system_tracking_success"] for row in rows) for key in keys]

    x = np.arange(len(planners))
    width = 0.34
    fig, ax = plt.subplots(figsize=(7.0, 4.5))
    ax.bar(x - width / 2, planner_counts, width, label="Planner success")
    ax.bar(x + width / 2, closed_loop_counts, width, label="Closed-loop success")
    ax.set_xticks(x, planners)
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
    ax.bar(x - width / 2, sampling_values, width, label="Sampling")
    ax.bar(x + width / 2, optimization_values, width, label="Optimization")
    ax.set_xticks(x, labels, rotation=30, ha="right")
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    fig.tight_layout()
    fig.savefig(OUTPUT_DIR / filename, dpi=180)
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

    print(f"Saved figures to: {OUTPUT_DIR}")
    for path in sorted(OUTPUT_DIR.glob("*.png")):
        print(f"  {path}")


if __name__ == "__main__":
    main()
