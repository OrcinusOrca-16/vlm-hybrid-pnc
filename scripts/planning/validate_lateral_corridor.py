import numpy as np

from src.common.types import (
    StaticObstacle,
    VehicleParams,
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
    solve_optimized_path,
)
from src.planning.reference_line import (
    ReferenceLine,
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

    corridor = build_base_lateral_corridor(
        start_s=12.0,
        reference_line=reference_line,
        vehicle_params=vehicle_params,
        road_min_l=-3.0,
        road_max_l=3.0,
        planning_horizon_m=20.0,
        resolution_m=0.5,
    )

    assert corridor.s_values.size == 41

    assert np.isclose(
        corridor.s_values[0],
        12.0,
    )

    assert np.isclose(
        corridor.s_values[-1],
        32.0,
    )

    assert np.allclose(
        np.diff(corridor.s_values),
        0.5,
    )

    assert np.allclose(
        corridor.lower_bounds,
        -2.1,
    )

    assert np.allclose(
        corridor.upper_bounds,
        2.1,
    )

    print("Base road corridor passed.")
    print(
        f"  points={corridor.s_values.size}, "
        f"start_s={corridor.s_values[0]:.2f}, "
        f"end_s={corridor.s_values[-1]:.2f}, "
        f"lower={corridor.lower_bounds[0]:.2f}, "
        f"upper={corridor.upper_bounds[0]:.2f}"
    )

    clamped_corridor = build_base_lateral_corridor(
        start_s=35.0,
        reference_line=reference_line,
        vehicle_params=vehicle_params,
        road_min_l=-3.0,
        road_max_l=3.0,
        planning_horizon_m=20.0,
        resolution_m=0.5,
    )

    assert clamped_corridor.s_values.size == 11

    assert np.isclose(
        clamped_corridor.s_values[0],
        35.0,
    )

    assert np.isclose(
        clamped_corridor.s_values[-1],
        40.0,
    )

    print("Reference-line end clamp passed.")
    print(
        f"  points={clamped_corridor.s_values.size}, "
        f"start_s={clamped_corridor.s_values[0]:.2f}, "
        f"end_s={clamped_corridor.s_values[-1]:.2f}"
    )

    obstacle = StaticObstacle(
        x=24.0,
        y=1.0,
        radius_m=0.5,
    )

    frenet_obstacle = project_obstacle_to_frenet(
        obstacle=obstacle,
        reference_line=reference_line,
    )

    assert np.isclose(
        frenet_obstacle.s,
        24.0,
        atol=1e-2,
    )

    assert np.isclose(
        frenet_obstacle.l,
        1.0,
        atol=1e-2,
    )

    assert np.isclose(
        frenet_obstacle.radius_m,
        0.5,
    )

    print("Obstacle Frenet projection passed.")
    print(
        f"  s={frenet_obstacle.s:.2f}, "
        f"l={frenet_obstacle.l:.2f}, "
        f"radius={frenet_obstacle.radius_m:.2f}"
    )

    inflated_obstacle = inflate_frenet_obstacle(
        obstacle=frenet_obstacle,
        vehicle_params=vehicle_params,
        minimum_clearance_m=0.5,
        safety_margin_m=0.01,
    )

    assert np.isclose(
        inflated_obstacle.min_s,
        19.39,
        atol=1e-2,
    )

    assert np.isclose(
        inflated_obstacle.max_s,
        26.01,
        atol=1e-2,
    )

    assert np.isclose(
        inflated_obstacle.min_l,
        -0.91,
        atol=1e-2,
    )

    assert np.isclose(
        inflated_obstacle.max_l,
        2.91,
        atol=1e-2,
    )

    print("Obstacle inflation passed.")
    print(
        f"  s=[{inflated_obstacle.min_s:.2f}, "
        f"{inflated_obstacle.max_s:.2f}], "
        f"l=[{inflated_obstacle.min_l:.2f}, "
        f"{inflated_obstacle.max_l:.2f}]"
    )

    (
        negative_l_corridor,
        positive_l_corridor,
    ) = build_obstacle_branch_corridors(
        base_corridor=corridor,
        obstacle=inflated_obstacle,
    )

    affected = (
        negative_l_corridor.upper_bounds
        < corridor.upper_bounds
    )

    assert np.any(
        affected
    )

    assert np.allclose(
        negative_l_corridor.lower_bounds[affected],
        -2.1,
    )

    assert np.allclose(
        negative_l_corridor.upper_bounds[affected],
        -0.91,
        atol=1e-2,
    )

    assert np.allclose(
        positive_l_corridor.lower_bounds[affected],
        2.91,
        atol=1e-2,
    )

    assert np.allclose(
        positive_l_corridor.upper_bounds[affected],
        2.1,
    )

    assert is_corridor_feasible(
        negative_l_corridor
    )

    assert not is_corridor_feasible(
        positive_l_corridor
    )

    assert np.any(
        np.isclose(
            negative_l_corridor.s_values[affected],
            26.0,
        )
    )

    print("Obstacle branch corridors passed.")
    print(
        f"  affected_points="
        f"{np.count_nonzero(affected)}"
    )
    print(
        "  negative-l branch: "
        f"[{negative_l_corridor.lower_bounds[affected][0]:.2f}, "
        f"{negative_l_corridor.upper_bounds[affected][0]:.2f}] "
        "feasible"
    )
    print(
        "  positive-l branch: "
        f"[{positive_l_corridor.lower_bounds[affected][0]:.2f}, "
        f"{positive_l_corridor.upper_bounds[affected][0]:.2f}] "
        "infeasible"
    )

    best_result = solve_multi_corridor_path(
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

    optimized_path = best_result.path

    affected_indices = np.where(
        affected
    )[0]

    max_affected_y = max(
        optimized_path[index].y
        for index in affected_indices
    )

    assert max_affected_y <= (
        inflated_obstacle.min_l + 1e-5
    )

    print("Multi-corridor obstacle solve passed.")
    print(
        f"  objective_value="
        f"{best_result.objective_value:.4f}"
    )
    print(
        f"  path_points={len(optimized_path)}, "
        f"max_y_in_obstacle_region="
        f"{max_affected_y:.4f}"
    )

    exact_clearance_feasible = (
        is_obstacle_clearance_feasible(
            path=optimized_path,
            vehicle_params=vehicle_params,
            obstacles=[obstacle],
            minimum_clearance_m=0.5,
        )
    )

    assert exact_clearance_feasible

    print("Exact obstacle clearance passed.")

    centered_obstacle = StaticObstacle(
        x=24.0,
        y=0.0,
        radius_m=0.2,
    )

    centered_frenet_obstacle = (
        project_obstacle_to_frenet(
            obstacle=centered_obstacle,
            reference_line=reference_line,
        )
    )

    centered_inflated_obstacle = (
        inflate_frenet_obstacle(
            obstacle=centered_frenet_obstacle,
            vehicle_params=vehicle_params,
            minimum_clearance_m=0.5,
            safety_margin_m=0.01,
        )
    )

    (
        centered_negative_corridor,
        centered_positive_corridor,
    ) = build_obstacle_branch_corridors(
        base_corridor=corridor,
        obstacle=centered_inflated_obstacle,
    )

    assert is_corridor_feasible(
        centered_negative_corridor
    )

    assert is_corridor_feasible(
        centered_positive_corridor
    )

    negative_result = solve_optimized_path(
        reference_line=reference_line,
        s_values=centered_negative_corridor.s_values,
        corridor_lower_bounds=(
            centered_negative_corridor.lower_bounds
        ),
        corridor_upper_bounds=(
            centered_negative_corridor.upper_bounds
        ),
        resolution_m=0.5,
        initial_l=0.8,
        initial_dl_ds=0.0,
        initial_d2l_ds2=0.0,
    )

    positive_result = solve_optimized_path(
        reference_line=reference_line,
        s_values=centered_positive_corridor.s_values,
        corridor_lower_bounds=(
            centered_positive_corridor.lower_bounds
        ),
        corridor_upper_bounds=(
            centered_positive_corridor.upper_bounds
        ),
        resolution_m=0.5,
        initial_l=0.8,
        initial_dl_ds=0.0,
        initial_d2l_ds2=0.0,
    )

    multi_result = solve_multi_corridor_path(
        reference_line=reference_line,
        corridors=[
            centered_negative_corridor,
            centered_positive_corridor,
        ],
        resolution_m=0.5,
        initial_l=0.8,
        initial_dl_ds=0.0,
        initial_d2l_ds2=0.0,
    )

    expected_result = min(
        [
            negative_result,
            positive_result,
        ],
        key=lambda result: result.objective_value,
    )

    assert np.isclose(
        multi_result.objective_value,
        expected_result.objective_value,
        atol=1e-5,
    )

    assert np.allclose(
        multi_result.lateral_solution,
        expected_result.lateral_solution,
        atol=1e-5,
    )

    print("Multi-corridor objective selection passed.")
    print(
        f"  negative_objective="
        f"{negative_result.objective_value:.4f}"
    )
    print(
        f"  positive_objective="
        f"{positive_result.objective_value:.4f}"
    )
    print(
        f"  selected_objective="
        f"{multi_result.objective_value:.4f}"
    )

    print()
    print("All lateral corridor cases passed.")


if __name__ == "__main__":
    main()