from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import (
    Circle,
    Rectangle,
)

from src.common.types import (
    StaticObstacle,
    VehicleParams,
)
from src.planning.frenet import (
    sl_to_xy,
)
from src.planning.hard_constraints import (
    is_obstacle_clearance_feasible,
)
from src.planning.lateral_corridor import (
    build_base_lateral_corridor,
    build_obstacle_branch_corridors,
    inflate_frenet_obstacle,
    is_corridor_feasible,
    project_obstacle_to_frenet,
)
from src.planning.optimization_planner import (
    solve_multi_corridor_path,
)
from src.planning.reference_line import (
    ReferenceLine,
)


def lateral_bound_to_xy(
    s_values: np.ndarray,
    lateral_values: np.ndarray,
    reference_line: ReferenceLine,
) -> tuple[np.ndarray, np.ndarray]:
    xy_points = [
        sl_to_xy(
            s=float(s),
            l=float(l),
            reference_line=reference_line,
        )
        for s, l in zip(
            s_values,
            lateral_values,
        )
    ]

    x_values = np.array([
        point[0]
        for point in xy_points
    ])

    y_values = np.array([
        point[1]
        for point in xy_points
    ])

    return (
        x_values,
        y_values,
    )


def main() -> None:
    reference_line = ReferenceLine([
        (0.0, 0.0),
        (10.0, 0.0),
        (20.0, 0.0),
        (30.0, 0.0),
        (40.0, 0.0),
    ])

    vehicle_params = VehicleParams(
        wheelbase_m=2.7,
        front_overhang_m=0.9,
        rear_overhang_m=1.0,
        width_m=1.8,
    )

    obstacle = StaticObstacle(
        x=24.0,
        y=1.0,
        radius_m=0.5,
    )

    corridor = build_base_lateral_corridor(
        start_s=12.0,
        reference_line=reference_line,
        vehicle_params=vehicle_params,
        road_min_l=-3.0,
        road_max_l=3.0,
        planning_horizon_m=28.0,
        resolution_m=0.5,
    )

    frenet_obstacle = project_obstacle_to_frenet(
        obstacle=obstacle,
        reference_line=reference_line,
    )

    inflated_obstacle = inflate_frenet_obstacle(
        obstacle=frenet_obstacle,
        vehicle_params=vehicle_params,
        minimum_clearance_m=0.5,
        safety_margin_m=0.01,
    )

    (
        negative_l_corridor,
        positive_l_corridor,
    ) = build_obstacle_branch_corridors(
        base_corridor=corridor,
        obstacle=inflated_obstacle,
    )

    assert is_corridor_feasible(
        negative_l_corridor
    )

    assert not is_corridor_feasible(
        positive_l_corridor
    )

    result = solve_multi_corridor_path(
        reference_line=reference_line,
        corridors=[
            negative_l_corridor,
            positive_l_corridor,
        ],
        resolution_m=0.5,
        initial_l=1.0,
        initial_dl_ds=-0.05,
        initial_d2l_ds2=0.0,
        lateral_reference_m=0.0,
        lateral_weight=0.2,
        first_derivative_weight=1.0,
        second_derivative_weight=5.0,
        third_derivative_weight=20.0,
    )

    assert is_obstacle_clearance_feasible(
        path=result.path,
        vehicle_params=vehicle_params,
        obstacles=[obstacle],
        minimum_clearance_m=0.5,
    )

    output_directory = Path(
        "results"
    )

    output_directory.mkdir(
        parents=True,
        exist_ok=True,
    )

    frenet_output_path = (
        output_directory
        / "optimization_corridor_frenet.png"
    )

    cartesian_output_path = (
        output_directory
        / "optimization_corridor_cartesian.png"
    )

    # ---------------------------------------------------------
    # Frenet / SL visualization
    # ---------------------------------------------------------

    plt.figure(
        figsize=(11, 6)
    )

    plt.plot(
        corridor.s_values,
        corridor.lower_bounds,
        linestyle="--",
        label="Base corridor lower",
    )

    plt.plot(
        corridor.s_values,
        corridor.upper_bounds,
        linestyle="--",
        label="Base corridor upper",
    )

    plt.plot(
        negative_l_corridor.s_values,
        negative_l_corridor.lower_bounds,
        linestyle="-.",
        label="Negative branch lower",
    )

    plt.plot(
        negative_l_corridor.s_values,
        negative_l_corridor.upper_bounds,
        linestyle="-.",
        label="Negative branch upper",
    )

    plt.plot(
        positive_l_corridor.s_values,
        positive_l_corridor.lower_bounds,
        linestyle=":",
        label="Positive branch lower",
    )

    plt.plot(
        positive_l_corridor.s_values,
        positive_l_corridor.upper_bounds,
        linestyle=":",
        label="Positive branch upper",
    )

    plt.plot(
        corridor.s_values,
        result.lateral_solution,
        linewidth=2.0,
        label="Optimized path",
    )

    obstacle_rectangle = Rectangle(
        (
            inflated_obstacle.min_s,
            inflated_obstacle.min_l,
        ),
        inflated_obstacle.max_s
        - inflated_obstacle.min_s,
        inflated_obstacle.max_l
        - inflated_obstacle.min_l,
        alpha=0.2,
        label="Inflated obstacle",
    )

    plt.gca().add_patch(
        obstacle_rectangle
    )

    plt.xlabel(
        "Reference s [m]"
    )

    plt.ylabel(
        "Lateral offset l [m]"
    )

    plt.title(
        "Optimization Planner - Frenet Corridor"
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        frenet_output_path,
        dpi=150,
    )

    # ---------------------------------------------------------
    # Cartesian visualization
    # ---------------------------------------------------------

    (
        base_lower_x,
        base_lower_y,
    ) = lateral_bound_to_xy(
        s_values=corridor.s_values,
        lateral_values=corridor.lower_bounds,
        reference_line=reference_line,
    )

    (
        base_upper_x,
        base_upper_y,
    ) = lateral_bound_to_xy(
        s_values=corridor.s_values,
        lateral_values=corridor.upper_bounds,
        reference_line=reference_line,
    )

    (
        selected_lower_x,
        selected_lower_y,
    ) = lateral_bound_to_xy(
        s_values=negative_l_corridor.s_values,
        lateral_values=negative_l_corridor.lower_bounds,
        reference_line=reference_line,
    )

    (
        selected_upper_x,
        selected_upper_y,
    ) = lateral_bound_to_xy(
        s_values=negative_l_corridor.s_values,
        lateral_values=negative_l_corridor.upper_bounds,
        reference_line=reference_line,
    )

    reference_s_values = np.linspace(
        corridor.s_values[0],
        corridor.s_values[-1],
        200,
    )

    reference_xy = np.array([
        reference_line.position(
            float(s)
        )
        for s in reference_s_values
    ])

    path_x = np.array([
        point.x
        for point in result.path
    ])

    path_y = np.array([
        point.y
        for point in result.path
    ])

    plt.figure(
        figsize=(11, 6)
    )

    plt.plot(
        reference_xy[:, 0],
        reference_xy[:, 1],
        linestyle="--",
        label="Reference line",
    )

    plt.plot(
        base_lower_x,
        base_lower_y,
        linestyle=":",
        label="Road lower bound",
    )

    plt.plot(
        base_upper_x,
        base_upper_y,
        linestyle=":",
        label="Road upper bound",
    )

    plt.plot(
        selected_lower_x,
        selected_lower_y,
        linestyle="-.",
        label="Selected corridor lower",
    )

    plt.plot(
        selected_upper_x,
        selected_upper_y,
        linestyle="-.",
        label="Selected corridor upper",
    )

    plt.plot(
        path_x,
        path_y,
        linewidth=2.0,
        label="Optimized path",
    )

    obstacle_circle = Circle(
        (
            obstacle.x,
            obstacle.y,
        ),
        obstacle.radius_m,
        fill=False,
        linewidth=2.0,
        label="Obstacle",
    )

    plt.gca().add_patch(
        obstacle_circle
    )

    plt.xlabel(
        "x [m]"
    )

    plt.ylabel(
        "y [m]"
    )

    plt.title(
        "Optimization Planner - Cartesian Path"
    )

    plt.axis(
        "equal"
    )

    plt.grid(
        True
    )

    plt.legend()

    plt.tight_layout()

    plt.savefig(
        cartesian_output_path,
        dpi=150,
    )

    print(
        "Optimization corridor visualization passed."
    )
    print(
        f"  objective_value="
        f"{result.objective_value:.4f}"
    )
    print(
        f"  Frenet figure: "
        f"{frenet_output_path}"
    )
    print(
        f"  Cartesian figure: "
        f"{cartesian_output_path}"
    )

    plt.show()


if __name__ == "__main__":
    main()