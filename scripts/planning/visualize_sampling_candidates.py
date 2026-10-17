from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import (
    Circle,
    Polygon,
)

from src.common.types import (
    StaticObstacle,
    VehicleParams,
    VehicleState,
)
from src.planning.frenet import (
    frenet_trajectories_to_paths,
    sl_to_xy,
)
from src.planning.hard_constraints import (
    compute_vehicle_footprint,
    is_candidate_feasible,
    obstacle_clearance,
)
from src.planning.reference_line import ReferenceLine
from src.planning.sampling_planner import (
    generate_frenet_candidates_from_vehicle_state,
    generate_returning_frenet_candidates_from_vehicle_state,
)
from src.planning.soft_cost import (
    select_best_candidate,
)


RESULTS_DIR = Path("results")

LATERAL_WEIGHT = 1.0
CURVATURE_WEIGHT = 10.0
LENGTH_WEIGHT = 0.05

MAX_CURVATURE = 1.0
MINIMUM_CLEARANCE_M = 0.5


def build_vehicle_params() -> VehicleParams:
    return VehicleParams(
        wheelbase_m=2.7,
        front_overhang_m=0.9,
        rear_overhang_m=1.0,
        width_m=1.8,
    )


def visualize_obstacle_case() -> None:
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

    vehicle_params = build_vehicle_params()

    obstacles = [
        StaticObstacle(
            x=24.0,
            y=1.0,
            radius_m=0.5,
        )
    ]

    min_l = -3.0
    max_l = 3.0

    candidates = generate_frenet_candidates_from_vehicle_state(
        vehicle_state=ego_state,
        reference_line=reference_line,
        target_lateral_positions=(
            -2.0,
            -1.0,
            0.0,
            1.0,
            2.0,
        ),
    )

    paths = frenet_trajectories_to_paths(
        trajectories=candidates,
        reference_line=reference_line,
    )

    feasible_candidates = []
    feasibility = []

    for trajectory, path in zip(
        candidates,
        paths,
    ):
        feasible = is_candidate_feasible(
            trajectory=trajectory,
            path=path,
            vehicle_params=vehicle_params,
            obstacles=obstacles,
            max_curvature=MAX_CURVATURE,
            minimum_clearance_m=MINIMUM_CLEARANCE_M,
            min_l=min_l,
            max_l=max_l,
        )

        feasibility.append(
            feasible
        )

        if feasible:
            feasible_candidates.append(
                (trajectory, path)
            )

    best_candidate = select_best_candidate(
        feasible_candidates=feasible_candidates,
        lateral_weight=LATERAL_WEIGHT,
        curvature_weight=CURVATURE_WEIGHT,
        length_weight=LENGTH_WEIGHT,
    )

    if best_candidate is None:
        raise RuntimeError(
            "No feasible candidate found."
        )

    best_trajectory, best_path = best_candidate

    reference_s = np.linspace(
        0.0,
        reference_line.length,
        200,
    )

    reference_xy = [
        reference_line.position(s)
        for s in reference_s
    ]

    left_boundary_xy = [
        sl_to_xy(
            s=s,
            l=max_l,
            reference_line=reference_line,
        )
        for s in reference_s
    ]

    right_boundary_xy = [
        sl_to_xy(
            s=s,
            l=min_l,
            reference_line=reference_line,
        )
        for s in reference_s
    ]

    figure, axis = plt.subplots(
        figsize=(11, 6)
    )

    axis.plot(
        [point[0] for point in reference_xy],
        [point[1] for point in reference_xy],
        "--",
        linewidth=1.5,
        label="Reference Line",
    )

    axis.plot(
        [point[0] for point in left_boundary_xy],
        [point[1] for point in left_boundary_xy],
        ":",
        linewidth=1.5,
        label="Road Boundary",
    )

    axis.plot(
        [point[0] for point in right_boundary_xy],
        [point[1] for point in right_boundary_xy],
        ":",
        linewidth=1.5,
    )

    feasible_label_used = False
    infeasible_label_used = False

    for path, feasible in zip(
        paths,
        feasibility,
    ):
        path_x = [
            point.x
            for point in path
        ]

        path_y = [
            point.y
            for point in path
        ]

        if feasible:
            label = (
                "Feasible Candidates"
                if not feasible_label_used
                else None
            )

            axis.plot(
                path_x,
                path_y,
                linewidth=1.5,
                alpha=0.8,
                label=label,
            )

            feasible_label_used = True

        else:
            label = (
                "Infeasible Candidates"
                if not infeasible_label_used
                else None
            )

            axis.plot(
                path_x,
                path_y,
                "--",
                linewidth=1.0,
                alpha=0.3,
                label=label,
            )

            infeasible_label_used = True

    axis.plot(
        [point.x for point in best_path],
        [point.y for point in best_path],
        linewidth=4.0,
        label=(
            "Best Path "
            f"(l={best_trajectory.points[-1].l:.1f} m)"
        ),
    )

    for index, obstacle in enumerate(
        obstacles
    ):
        obstacle_circle = Circle(
            (obstacle.x, obstacle.y),
            obstacle.radius_m,
            alpha=0.6,
            label=(
                "Obstacle"
                if index == 0
                else None
            ),
        )

        axis.add_patch(
            obstacle_circle
        )

    axis.scatter(
        ego_state.x,
        ego_state.y,
        marker="o",
        s=80,
        label="Ego",
        zorder=5,
    )

    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")

    axis.set_title(
        "Sampling Planner - Static Obstacle Avoidance"
    )

    axis.set_xlim(
        10.0,
        34.0,
    )

    axis.set_ylim(
        -4.0,
        4.0,
    )

    axis.grid(True)

    axis.legend(
        loc="best",
        fontsize=9,
    )

    RESULTS_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path = (
        RESULTS_DIR
        / "sampling_planner_obstacle.png"
    )

    figure.savefig(
        output_path,
        dpi=150,
        bbox_inches="tight",
    )

    print(
        "Obstacle visualization:"
    )

    print(
        f"  total={len(candidates)}, "
        f"feasible={len(feasible_candidates)}, "
        f"best_l={best_trajectory.points[-1].l:.2f}, "
        f"length={best_path[-1].s:.2f}"
    )

    print(
        f"  saved={output_path}"
    )


def visualize_returning_obstacle_case() -> None:
    reference_line = ReferenceLine([
        (0.0, 0.0),
        (20.0, 0.0),
        (40.0, 0.0),
        (50.0, 0.0),
    ])

    ego_state = VehicleState(
        x=12.0,
        y=0.0,
        yaw=0.0,
        speed=5.0,
        curvature=0.0,
    )

    vehicle_params = build_vehicle_params()

    obstacle = StaticObstacle(
        x=24.0,
        y=1.0,
        radius_m=0.5,
    )

    min_l = -3.0
    max_l = 3.0

    candidates = (
        generate_returning_frenet_candidates_from_vehicle_state(
            vehicle_state=ego_state,
            reference_line=reference_line,
            bypass_lateral_positions=(
                -1.5,
                -1.0,
                -0.5,
                0.0,
                0.5,
                1.0,
                1.5,
            ),
            bypass_lengths=(
                10.0,
                14.0,
                18.0,
            ),
            total_length=28.0,
            final_lateral_positions=(0.0,),
            resolution=0.5,
        )
    )

    paths = frenet_trajectories_to_paths(
        trajectories=candidates,
        reference_line=reference_line,
    )

    feasible_candidates = []
    feasibility = []

    for trajectory, path in zip(
        candidates,
        paths,
    ):
        feasible = is_candidate_feasible(
            trajectory=trajectory,
            path=path,
            vehicle_params=vehicle_params,
            obstacles=[obstacle],
            max_curvature=MAX_CURVATURE,
            minimum_clearance_m=MINIMUM_CLEARANCE_M,
            min_l=min_l,
            max_l=max_l,
        )

        feasibility.append(
            feasible
        )

        if feasible:
            feasible_candidates.append(
                (trajectory, path)
            )

    best_candidate = select_best_candidate(
        feasible_candidates=feasible_candidates,
        lateral_weight=LATERAL_WEIGHT,
        curvature_weight=CURVATURE_WEIGHT,
        length_weight=LENGTH_WEIGHT,
    )

    if best_candidate is None:
        raise RuntimeError(
            "No feasible returning candidate found."
        )

    best_trajectory, best_path = best_candidate

    best_l_values = np.array([
        point.l
        for point in best_trajectory.points
    ])

    clearances = [
        obstacle_clearance(
            path_point=point,
            vehicle_params=vehicle_params,
            obstacle=obstacle,
        )
        for point in best_path
    ]

    minimum_clearance = min(
        clearances
    )

    minimum_clearance_index = int(
        np.argmin(clearances)
    )

    minimum_clearance_point = (
        best_path[minimum_clearance_index]
    )

    minimum_clearance_footprint = (
        compute_vehicle_footprint(
            path_point=minimum_clearance_point,
            vehicle_params=vehicle_params,
        )
    )

    max_abs_curvature = max(
        abs(point.curvature)
        for point in best_path
    )

    reference_s = np.linspace(
        0.0,
        reference_line.length,
        200,
    )

    reference_xy = [
        reference_line.position(s)
        for s in reference_s
    ]

    left_boundary_xy = [
        sl_to_xy(
            s=s,
            l=max_l,
            reference_line=reference_line,
        )
        for s in reference_s
    ]

    right_boundary_xy = [
        sl_to_xy(
            s=s,
            l=min_l,
            reference_line=reference_line,
        )
        for s in reference_s
    ]

    figure, axis = plt.subplots(
        figsize=(12, 6)
    )

    axis.plot(
        [point[0] for point in reference_xy],
        [point[1] for point in reference_xy],
        "--",
        linewidth=1.5,
        label="Reference Line",
    )

    axis.plot(
        [point[0] for point in left_boundary_xy],
        [point[1] for point in left_boundary_xy],
        ":",
        linewidth=1.5,
        label="Road Boundary",
    )

    axis.plot(
        [point[0] for point in right_boundary_xy],
        [point[1] for point in right_boundary_xy],
        ":",
        linewidth=1.5,
    )

    safety_circle = Circle(
        (
            obstacle.x,
            obstacle.y,
        ),
        (
            obstacle.radius_m
            + MINIMUM_CLEARANCE_M
        ),
        alpha=0.18,
        label="Obstacle + Minimum Clearance",
        zorder=0,
    )

    axis.add_patch(
        safety_circle
    )

    feasible_label_used = False
    infeasible_label_used = False

    for path, feasible in zip(
        paths,
        feasibility,
    ):
        path_x = [
            point.x
            for point in path
        ]

        path_y = [
            point.y
            for point in path
        ]

        if feasible:
            label = (
                "Feasible Candidates"
                if not feasible_label_used
                else None
            )

            axis.plot(
                path_x,
                path_y,
                linewidth=1.5,
                alpha=0.8,
                label=label,
            )

            feasible_label_used = True

        else:
            label = (
                "Infeasible Candidates"
                if not infeasible_label_used
                else None
            )

            axis.plot(
                path_x,
                path_y,
                "--",
                linewidth=1.0,
                alpha=0.25,
                label=label,
            )

            infeasible_label_used = True

    axis.plot(
        [point.x for point in best_path],
        [point.y for point in best_path],
        linewidth=4.0,
        label=(
            "Best Returning Path "
            f"(min l={np.min(best_l_values):.1f} m)"
        ),
        zorder=4,
    )

    footprint_patch = Polygon(
        minimum_clearance_footprint,
        closed=True,
        alpha=0.25,
        label="Vehicle at Minimum Clearance",
        zorder=3,
    )

    axis.add_patch(
        footprint_patch
    )

    obstacle_circle = Circle(
        (
            obstacle.x,
            obstacle.y,
        ),
        obstacle.radius_m,
        alpha=0.65,
        label="Original Obstacle",
        zorder=5,
    )

    axis.add_patch(
        obstacle_circle
    )

    axis.scatter(
        ego_state.x,
        ego_state.y,
        marker="o",
        s=80,
        label="Ego Start",
        zorder=6,
    )

    axis.scatter(
        minimum_clearance_point.x,
        minimum_clearance_point.y,
        marker="x",
        s=80,
        label="Rear Axle at Minimum Clearance",
        zorder=7,
    )

    axis.set_xlabel(
        "x [m]"
    )

    axis.set_ylabel(
        "y [m]"
    )

    axis.set_title(
        "Sampling Planner - Bypass and Return"
    )

    axis.set_xlim(
        10.0,
        41.0,
    )

    axis.set_ylim(
        -4.0,
        4.0,
    )

    axis.grid(
        True
    )

    axis.legend(
        loc="best",
        fontsize=9,
    )

    RESULTS_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path = (
        RESULTS_DIR
        / "sampling_planner_returning_obstacle.png"
    )

    figure.savefig(
        output_path,
        dpi=150,
        bbox_inches="tight",
    )

    print(
        "Returning obstacle visualization:"
    )

    print(
        f"  total={len(candidates)}, "
        f"feasible={len(feasible_candidates)}"
    )

    print(
        f"  start_l="
        f"{best_trajectory.points[0].l:.2f}, "
        f"min_l="
        f"{np.min(best_l_values):.2f}, "
        f"end_l="
        f"{best_trajectory.points[-1].l:.2f}"
    )

    print(
        f"  minimum_clearance="
        f"{minimum_clearance:.4f}, "
        f"clearance_limit="
        f"{MINIMUM_CLEARANCE_M:.4f}"
    )

    print(
        f"  min_clearance_point="
        f"({minimum_clearance_point.x:.2f}, "
        f"{minimum_clearance_point.y:.2f}), "
        f"yaw="
        f"{minimum_clearance_point.yaw:.4f}"
    )

    print(
        f"  max_curvature="
        f"{max_abs_curvature:.4f}, "
        f"path_length="
        f"{best_path[-1].s:.2f}"
    )

    print(
        f"  saved={output_path}"
    )


def visualize_lateral_maneuver_case() -> None:
    reference_line = ReferenceLine([
        (0.0, 0.0),
        (20.0, 0.0),
        (40.0, 0.0),
    ])

    ego_state = VehicleState(
        x=12.0,
        y=1.0,
        yaw=-0.05,
        speed=5.0,
        curvature=0.0,
    )

    vehicle_params = build_vehicle_params()

    candidates = generate_frenet_candidates_from_vehicle_state(
        vehicle_state=ego_state,
        reference_line=reference_line,
    )

    paths = frenet_trajectories_to_paths(
        trajectories=candidates,
        reference_line=reference_line,
    )

    feasible_candidates = []

    for trajectory, path in zip(
        candidates,
        paths,
    ):
        if is_candidate_feasible(
            trajectory=trajectory,
            path=path,
            vehicle_params=vehicle_params,
            obstacles=[],
            max_curvature=MAX_CURVATURE,
            minimum_clearance_m=MINIMUM_CLEARANCE_M,
            min_l=-3.0,
            max_l=3.0,
        ):
            feasible_candidates.append(
                (trajectory, path)
            )

    best_candidate = select_best_candidate(
        feasible_candidates=feasible_candidates,
        lateral_weight=LATERAL_WEIGHT,
        curvature_weight=CURVATURE_WEIGHT,
        length_weight=LENGTH_WEIGHT,
    )

    if best_candidate is None:
        raise RuntimeError(
            "No feasible candidate found."
        )

    best_trajectory, best_path = best_candidate

    reference_s = np.linspace(
        0.0,
        reference_line.length,
        200,
    )

    reference_xy = [
        reference_line.position(s)
        for s in reference_s
    ]

    figure, axis = plt.subplots(
        figsize=(10, 5)
    )

    axis.plot(
        [point[0] for point in reference_xy],
        [point[1] for point in reference_xy],
        "--",
        linewidth=1.5,
        label="Reference Line",
    )

    first_candidate = True

    for path in paths:
        axis.plot(
            [point.x for point in path],
            [point.y for point in path],
            linewidth=1.0,
            alpha=0.3,
            label=(
                "Candidates"
                if first_candidate
                else None
            ),
        )

        first_candidate = False

    axis.plot(
        [point.x for point in best_path],
        [point.y for point in best_path],
        linewidth=4.0,
        label=(
            "Best Path "
            f"(l={best_trajectory.points[-1].l:.1f} m)"
        ),
    )

    axis.scatter(
        ego_state.x,
        ego_state.y,
        marker="o",
        s=80,
        label="Ego",
        zorder=5,
    )

    axis.set_xlabel("x [m]")
    axis.set_ylabel("y [m]")

    axis.set_title(
        "Sampling Planner - Lateral Maneuver"
    )

    axis.set_xlim(
        10.0,
        34.0,
    )

    axis.set_ylim(
        -2.0,
        2.5,
    )

    axis.grid(True)

    axis.legend(
        loc="best",
    )

    RESULTS_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    output_path = (
        RESULTS_DIR
        / "sampling_planner_lateral_maneuver.png"
    )

    figure.savefig(
        output_path,
        dpi=150,
        bbox_inches="tight",
    )

    print(
        "Lateral maneuver visualization:"
    )

    print(
        f"  total={len(candidates)}, "
        f"feasible={len(feasible_candidates)}, "
        f"start_l={best_trajectory.points[0].l:.2f}, "
        f"best_l={best_trajectory.points[-1].l:.2f}, "
        f"length={best_path[-1].s:.2f}"
    )

    print(
        f"  saved={output_path}"
    )


def main() -> None:
    visualize_obstacle_case()
    visualize_returning_obstacle_case()
    visualize_lateral_maneuver_case()

    plt.show()


if __name__ == "__main__":
    main()