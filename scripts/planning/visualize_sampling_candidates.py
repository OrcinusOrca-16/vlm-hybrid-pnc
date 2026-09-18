from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

from src.common.types import VehicleState
from src.planning.frenet import frenet_trajectories_to_paths
from src.planning.reference_line import ReferenceLine
from src.planning.sampling_planner import (
    generate_frenet_candidates_from_vehicle_state,
)


RESULTS_DIR = Path("results")


def main() -> None:
    reference_line = ReferenceLine([
        (0.0, 0.0),
        (20.0, 0.0),
        (40.0, 0.0),
    ])

    ego_state = VehicleState(
        x=12.0,
        y=0.0,
        yaw=0.0,
        speed=5.0,
        curvature=0.0,
    )

    candidates = generate_frenet_candidates_from_vehicle_state(
        vehicle_state=ego_state,
        reference_line=reference_line,
    )

    paths = frenet_trajectories_to_paths(
        trajectories=candidates,
        reference_line=reference_line,
    )

    # Reference Line
    reference_s = np.linspace(
        0.0,
        reference_line.length,
        200,
    )

    reference_xy = [
        reference_line.position(s)
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

    plt.figure(
        figsize=(10, 6)
    )

    plt.plot(
        reference_x,
        reference_y,
        "--",
        label="Reference Line",
    )

    # Candidate Paths
    for candidate, path in zip(
        candidates,
        paths,
    ):
        path_x = [
            point.x
            for point in path
        ]

        path_y = [
            point.y
            for point in path
        ]

        target_l = candidate.points[-1].l

        target_length = (
            candidate.points[-1].s
            - candidate.points[0].s
        )

        plt.plot(
            path_x,
            path_y,
            label=(
                f"L={target_length:.0f} m, "
                f"l={target_l:.2f} m"
            ),
        )

    # Ego
    plt.scatter(
        ego_state.x,
        ego_state.y,
        marker="o",
        s=60,
        label="Ego",
    )

    plt.xlabel("x [m]")
    plt.ylabel("y [m]")
    plt.title(
        "Frenet Sampling Candidate Paths"
    )

    plt.axis("equal")
    plt.grid(True)
    plt.legend(
        loc="best",
        fontsize=8,
    )

    RESULTS_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path = (
        RESULTS_DIR
        / "sampling_candidates.png"
    )

    plt.savefig(
        output_path,
        dpi=150,
        bbox_inches="tight",
    )

    plt.close()

    print(
        f"Saved figure: {output_path}"
    )


if __name__ == "__main__":
    main()