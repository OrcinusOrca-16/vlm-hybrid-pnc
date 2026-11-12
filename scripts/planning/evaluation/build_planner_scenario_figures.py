"""Rebuild final planner figures with stable planner colors.

This script uses the frozen 9-scenario suite and current production planners only
for figure reconstruction. It does not write benchmark CSV/JSON or tune any
planner parameter.

Color convention:
    Sampling     = red
    Optimization = purple
"""

from __future__ import annotations

from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.patches import Circle

from scripts.plotting_style import OPTIMIZATION_COLOR, SAMPLING_COLOR
from src.benchmark.evaluation_scenario_suite import build_final_evaluation_scenarios
from src.benchmark.scenario_runner import run_benchmark_scenario
from src.planning.frenet import sl_to_xy, vehicle_state_to_frenet

FIGURE_DIR = Path("outputs/planning_benchmark/figures")
REFERENCE_PLOT_RESOLUTION_M = 0.25


def sanitize_filename(name: str) -> str:
    return "".join(character if character.isalnum() or character in ("_", "-") else "_" for character in name)


def plot_scenario(run) -> None:
    scenario = run.scenario
    initial_frenet = vehicle_state_to_frenet(
        vehicle_state=scenario.vehicle_state, reference_line=scenario.reference_line
    )
    start_s = initial_frenet.s
    end_s = min(start_s + scenario.planning_horizon_m, scenario.reference_line.length)
    point_count = max(2, int((end_s - start_s) / REFERENCE_PLOT_RESOLUTION_M) + 1)
    s_values = [start_s + (end_s - start_s) * index / (point_count - 1) for index in range(point_count)]

    centerline = [sl_to_xy(s=s, l=0.0, reference_line=scenario.reference_line) for s in s_values]
    left_boundary = [sl_to_xy(s=s, l=scenario.road_max_l, reference_line=scenario.reference_line) for s in s_values]
    right_boundary = [sl_to_xy(s=s, l=scenario.road_min_l, reference_line=scenario.reference_line) for s in s_values]

    figure, axis = plt.subplots(figsize=(10, 6))
    axis.plot([p[0] for p in centerline], [p[1] for p in centerline], linestyle="--", color="0.45", label="Reference")
    axis.plot(
        [p[0] for p in left_boundary], [p[1] for p in left_boundary], linestyle=":", color="0.5", label="Road boundary"
    )
    axis.plot([p[0] for p in right_boundary], [p[1] for p in right_boundary], linestyle=":", color="0.5")
    axis.scatter([scenario.vehicle_state.x], [scenario.vehicle_state.y], marker="x", color="black", label="Ego start")

    for obstacle_index, obstacle in enumerate(scenario.obstacles):
        axis.add_patch(
            Circle(
                (obstacle.x, obstacle.y),
                radius=obstacle.radius_m,
                fill=False,
                linestyle="--",
                color="0.25",
                label="Obstacle" if obstacle_index == 0 else None,
            )
        )

    if run.sampling_result.success and run.sampling_result.path is not None:
        axis.plot(
            [point.x for point in run.sampling_result.path],
            [point.y for point in run.sampling_result.path],
            color=SAMPLING_COLOR,
            linewidth=2.0,
            label="Sampling",
        )

    if run.optimization_result.success and run.optimization_result.path is not None:
        axis.plot(
            [point.x for point in run.optimization_result.path],
            [point.y for point in run.optimization_result.path],
            color=OPTIMIZATION_COLOR,
            linewidth=2.0,
            label="Optimization",
        )

    axis.set_title(scenario.name)
    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")
    axis.axis("equal")
    axis.grid(True)
    axis.legend()
    figure.tight_layout()

    FIGURE_DIR.mkdir(parents=True, exist_ok=True)
    figure.savefig(FIGURE_DIR / f"{sanitize_filename(scenario.name)}.png", dpi=180)
    plt.close(figure)


def main() -> None:
    scenarios = list(build_final_evaluation_scenarios())
    assert len(scenarios) == 9

    print("Rebuilding final planner figures only")
    print("Sampling=red, Optimization=purple")

    for index, scenario in enumerate(scenarios, start=1):
        print(f"[{index}/{len(scenarios)}] {scenario.name}")
        plot_scenario(run_benchmark_scenario(scenario))

    print(f"Saved figures to: {FIGURE_DIR}")


if __name__ == "__main__":
    main()
