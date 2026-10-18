from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Circle

from src.benchmark.canonical_runner import (
    run_canonical_benchmark,
)
from src.planning.frenet import (
    sl_to_xy,
    vehicle_state_to_frenet,
)


RESULT_DIR = Path(
    "results/benchmark"
)


def plot_cartesian(
    benchmark,
) -> None:
    scenario = benchmark.scenario

    sampling_path = (
        benchmark.sampling_result.path
    )
    optimization_path = (
        benchmark.optimization_result.path
    )

    assert sampling_path is not None
    assert optimization_path is not None

    initial_frenet = vehicle_state_to_frenet(
        vehicle_state=scenario.vehicle_state,
        reference_line=scenario.reference_line,
    )

    start_s = initial_frenet.s

    end_s = min(
        start_s
        + scenario.planning_horizon_m,
        scenario.reference_line.length,
    )

    reference_s = np.linspace(
        start_s,
        end_s,
        300,
    )

    reference_xy = [
        scenario.reference_line.position(
            float(s)
        )
        for s in reference_s
    ]

    lower_boundary_xy = [
        sl_to_xy(
            s=float(s),
            l=scenario.road_min_l,
            reference_line=scenario.reference_line,
        )
        for s in reference_s
    ]

    upper_boundary_xy = [
        sl_to_xy(
            s=float(s),
            l=scenario.road_max_l,
            reference_line=scenario.reference_line,
        )
        for s in reference_s
    ]

    reference_x = [
        point[0]
        for point in reference_xy
    ]
    reference_y = [
        point[1]
        for point in reference_xy
    ]

    lower_x = [
        point[0]
        for point in lower_boundary_xy
    ]
    lower_y = [
        point[1]
        for point in lower_boundary_xy
    ]

    upper_x = [
        point[0]
        for point in upper_boundary_xy
    ]
    upper_y = [
        point[1]
        for point in upper_boundary_xy
    ]

    sampling_x = [
        point.x
        for point in sampling_path
    ]
    sampling_y = [
        point.y
        for point in sampling_path
    ]

    optimization_x = [
        point.x
        for point in optimization_path
    ]
    optimization_y = [
        point.y
        for point in optimization_path
    ]

    fig, ax = plt.subplots(
        figsize=(11, 5)
    )

    ax.plot(
        reference_x,
        reference_y,
        linestyle=":",
        linewidth=2.0,
        label="Reference Line",
    )

    ax.plot(
        lower_x,
        lower_y,
        linestyle="--",
        linewidth=1.5,
        label="Road Boundary",
    )

    ax.plot(
        upper_x,
        upper_y,
        linestyle="--",
        linewidth=1.5,
    )

    for index, obstacle in enumerate(
        scenario.obstacles
    ):
        obstacle_patch = Circle(
            (
                obstacle.x,
                obstacle.y,
            ),
            obstacle.radius_m,
            fill=False,
            linewidth=2.0,
            label=(
                "Original Obstacle"
                if index == 0
                else None
            ),
        )

        ax.add_patch(
            obstacle_patch
        )

    ax.plot(
        sampling_x,
        sampling_y,
        linewidth=2.5,
        label="Sampling",
    )

    ax.plot(
        optimization_x,
        optimization_y,
        linewidth=2.5,
        label="Optimization",
    )

    ax.scatter(
        [
            scenario.vehicle_state.x
        ],
        [
            scenario.vehicle_state.y
        ],
        marker="x",
        s=80,
        label="Initial Vehicle State",
    )

    ax.set_title(
        "Canonical Benchmark - Cartesian Overlay"
    )
    ax.set_xlabel(
        "x [m]"
    )
    ax.set_ylabel(
        "y [m]"
    )

    ax.grid(
        True
    )
    ax.legend()

    ax.set_aspect(
        "equal",
        adjustable="box",
    )

    fig.tight_layout()

    output_path = (
        RESULT_DIR
        / "canonical_benchmark_cartesian.png"
    )

    fig.savefig(
        output_path,
        dpi=150,
    )


def plot_frenet(
    benchmark,
) -> None:
    scenario = benchmark.scenario

    sampling_trajectory = (
        benchmark.sampling_result.trajectory
    )
    optimization_trajectory = (
        benchmark.optimization_result.trajectory
    )

    assert sampling_trajectory is not None
    assert optimization_trajectory is not None

    sampling_s = [
        point.s
        for point in sampling_trajectory.points
    ]

    sampling_l = [
        point.l
        for point in sampling_trajectory.points
    ]

    optimization_s = [
        point.s
        for point in optimization_trajectory.points
    ]

    optimization_l = [
        point.l
        for point in optimization_trajectory.points
    ]

    start_s = min(
        sampling_s[0],
        optimization_s[0],
    )

    end_s = max(
        sampling_s[-1],
        optimization_s[-1],
    )

    fig, ax = plt.subplots(
        figsize=(11, 5)
    )

    ax.axhline(
        scenario.lateral_reference_m,
        linestyle=":",
        linewidth=2.0,
        label="Reference Line",
    )

    ax.axhline(
        scenario.road_min_l,
        linestyle="--",
        linewidth=1.5,
        label="Road Boundary",
    )

    ax.axhline(
        scenario.road_max_l,
        linestyle="--",
        linewidth=1.5,
    )

    ax.plot(
        sampling_s,
        sampling_l,
        linewidth=2.5,
        label="Sampling",
    )

    ax.plot(
        optimization_s,
        optimization_l,
        linewidth=2.5,
        label="Optimization",
    )

    for index, obstacle in enumerate(
        scenario.obstacles
    ):
        obstacle_s = (
            scenario.reference_line.nearest_s(
                x=obstacle.x,
                y=obstacle.y,
            )
        )

        reference_point = (
            scenario.reference_line.query(
                obstacle_s
            )
        )

        normal_x = -np.sin(
            reference_point.yaw
        )
        normal_y = np.cos(
            reference_point.yaw
        )

        obstacle_l = (
            (
                obstacle.x
                - reference_point.x
            )
            * normal_x
            + (
                obstacle.y
                - reference_point.y
            )
            * normal_y
        )

        ax.scatter(
            [obstacle_s],
            [obstacle_l],
            s=80,
            marker="o",
            label=(
                "Original Obstacle Center"
                if index == 0
                else None
            ),
        )

    ax.set_xlim(
        start_s - 1.0,
        end_s + 1.0,
    )

    ax.set_ylim(
        scenario.road_min_l - 0.5,
        scenario.road_max_l + 0.5,
    )

    ax.set_title(
        "Canonical Benchmark - Frenet s-l Overlay"
    )
    ax.set_xlabel(
        "Reference-line s [m]"
    )
    ax.set_ylabel(
        "Lateral offset l [m]"
    )

    ax.grid(
        True
    )
    ax.legend()

    ax.set_aspect(
        "equal",
        adjustable="box",
    )

    fig.tight_layout()

    output_path = (
        RESULT_DIR
        / "canonical_benchmark_frenet.png"
    )

    fig.savefig(
        output_path,
        dpi=150,
    )


def main() -> None:
    RESULT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    benchmark = (
        run_canonical_benchmark()
    )

    assert benchmark.sampling_result.success
    assert benchmark.optimization_result.success

    plot_cartesian(
        benchmark
    )

    plot_frenet(
        benchmark
    )

    print(
        "Canonical benchmark visualization passed."
    )
    print(
        "  saved="
        "results/benchmark/"
        "canonical_benchmark_cartesian.png"
    )
    print(
        "  saved="
        "results/benchmark/"
        "canonical_benchmark_frenet.png"
    )

    plt.show()


if __name__ == "__main__":
    main()