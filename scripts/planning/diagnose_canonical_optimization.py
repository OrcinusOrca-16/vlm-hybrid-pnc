import numpy as np

from src.benchmark.canonical_scenario import (
    build_canonical_static_obstacle_scenario,
)
from src.config.planning import (
    DEFAULT_OPTIMIZATION_PLANNER_CONFIG,
)
from src.planning.frenet import (
    vehicle_state_to_frenet,
)
from src.planning.hard_constraints import (
    is_curvature_feasible,
    is_obstacle_clearance_feasible,
    is_road_boundary_feasible,
    obstacle_clearance,
)
from src.planning.lateral_corridor import (
    build_base_lateral_corridor,
    build_obstacle_branch_corridors,
    inflate_frenet_obstacle,
    is_corridor_feasible,
    project_obstacle_to_frenet,
)
from src.planning.optimization_planner import (
    solve_matched_optimized_path,
)


def diagnose_branch(
    branch_name,
    corridor,
    scenario,
    initial_state,
) -> None:
    print()
    print(f"{branch_name} branch:")

    print(
        "  corridor_feasible="
        f"{is_corridor_feasible(corridor)}"
    )

    print(
        "  min_lower="
        f"{np.min(corridor.lower_bounds):.6f}"
    )
    print(
        "  max_lower="
        f"{np.max(corridor.lower_bounds):.6f}"
    )
    print(
        "  min_upper="
        f"{np.min(corridor.upper_bounds):.6f}"
    )
    print(
        "  max_upper="
        f"{np.max(corridor.upper_bounds):.6f}"
    )

    if not is_corridor_feasible(corridor):
        return

    try:
        result = solve_matched_optimized_path(
            reference_line=scenario.reference_line,
            s_values=corridor.s_values,
            corridor_lower_bounds=(
                corridor.lower_bounds
            ),
            corridor_upper_bounds=(
                corridor.upper_bounds
            ),
            resolution_m=scenario.resolution_m,
            initial_l=initial_state.l,
            initial_dl_ds=initial_state.dl_ds,
            initial_d2l_ds2=(
                initial_state.d2l_ds2
            ),
            lateral_reference_m=(
                scenario.lateral_reference_m
            ),
            terminal_lateral_target_m=(
                scenario.terminal_lateral_target_m
            ),
            terminal_first_derivative_target=(
                scenario
                .terminal_first_derivative_target
            ),
            lateral_weight=scenario.lateral_weight,
            first_derivative_weight=(
                scenario.first_derivative_weight
            ),
            second_derivative_weight=(
                scenario.second_derivative_weight
            ),
            third_derivative_weight=(
                scenario.third_derivative_weight
            ),
            terminal_lateral_weight=(
                scenario.terminal_lateral_weight
            ),
            terminal_first_derivative_weight=(
                scenario
                .terminal_first_derivative_weight
            ),
            max_second_derivative_abs=(
                DEFAULT_OPTIMIZATION_PLANNER_CONFIG
                .max_second_derivative_abs
            ),
        )
    except RuntimeError as error:
        print(
            "  qp_solve_success=False"
        )
        print(
            "  qp_failure_reason="
            f"{error}"
        )
        return

    print(
        "  qp_solve_success=True"
    )

    obstacle = scenario.obstacles[0]

    clearances = [
        obstacle_clearance(
            path_point=path_point,
            vehicle_params=scenario.vehicle_params,
            obstacle=obstacle,
        )
        for path_point in result.path
    ]

    min_clearance = min(clearances)

    max_abs_curvature = max(
        abs(path_point.curvature)
        for path_point in result.path
    )

    road_ok = is_road_boundary_feasible(
        trajectory=result.trajectory,
        vehicle_params=scenario.vehicle_params,
        min_l=scenario.road_min_l,
        max_l=scenario.road_max_l,
    )

    clearance_ok = (
        is_obstacle_clearance_feasible(
            path=result.path,
            vehicle_params=scenario.vehicle_params,
            obstacles=list(
                scenario.obstacles
            ),
            minimum_clearance_m=(
                scenario.minimum_clearance_m
            ),
        )
    )

    curvature_ok = is_curvature_feasible(
        path=result.path,
        max_curvature=scenario.max_curvature,
    )

    min_clearance_index = int(
        np.argmin(clearances)
    )

    min_clearance_point = (
        result.path[min_clearance_index]
    )

    min_clearance_frenet = (
        result.trajectory.points[
            min_clearance_index
        ]
    )

    print(
        "  objective="
        f"{result.objective_value:.6f}"
    )

    print(
        "  road_ok="
        f"{road_ok}"
    )

    print(
        "  clearance_ok="
        f"{clearance_ok}"
    )

    print(
        "  curvature_ok="
        f"{curvature_ok}"
    )

    print(
        "  min_clearance="
        f"{min_clearance:.6f}"
    )

    print(
        "  min_clearance_s="
        f"{min_clearance_frenet.s:.6f}"
    )

    print(
        "  min_clearance_l="
        f"{min_clearance_frenet.l:.6f}"
    )

    print(
        "  min_clearance_yaw="
        f"{min_clearance_point.yaw:.6f}"
    )

    print(
        "  max_abs_curvature="
        f"{max_abs_curvature:.6f}"
    )


def main() -> None:
    scenario = (
        build_canonical_static_obstacle_scenario()
    )

    initial_state = vehicle_state_to_frenet(
        vehicle_state=scenario.vehicle_state,
        reference_line=scenario.reference_line,
    )

    base_corridor = build_base_lateral_corridor(
        start_s=initial_state.s,
        reference_line=scenario.reference_line,
        vehicle_params=scenario.vehicle_params,
        road_min_l=scenario.road_min_l,
        road_max_l=scenario.road_max_l,
        planning_horizon_m=(
            scenario.planning_horizon_m
        ),
        resolution_m=scenario.resolution_m,
    )

    obstacle = project_obstacle_to_frenet(
        obstacle=scenario.obstacles[0],
        reference_line=scenario.reference_line,
    )

    inflated_obstacle = (
        inflate_frenet_obstacle(
            obstacle=obstacle,
            vehicle_params=scenario.vehicle_params,
            minimum_clearance_m=(
                scenario.minimum_clearance_m
            ),
        )
    )

    (
        negative_corridor,
        positive_corridor,
    ) = build_obstacle_branch_corridors(
        base_corridor=base_corridor,
        obstacle=inflated_obstacle,
    )

    print(
        "Canonical Optimization diagnosis"
    )
    print(
        "  initial_s="
        f"{initial_state.s:.6f}"
    )
    print(
        "  obstacle_s="
        f"{obstacle.s:.6f}"
    )
    print(
        "  obstacle_l="
        f"{obstacle.l:.6f}"
    )
    print(
        "  inflated_min_s="
        f"{inflated_obstacle.min_s:.6f}"
    )
    print(
        "  inflated_max_s="
        f"{inflated_obstacle.max_s:.6f}"
    )
    print(
        "  inflated_min_l="
        f"{inflated_obstacle.min_l:.6f}"
    )
    print(
        "  inflated_max_l="
        f"{inflated_obstacle.max_l:.6f}"
    )

    diagnose_branch(
        branch_name="negative",
        corridor=negative_corridor,
        scenario=scenario,
        initial_state=initial_state,
    )

    diagnose_branch(
        branch_name="positive",
        corridor=positive_corridor,
        scenario=scenario,
        initial_state=initial_state,
    )


if __name__ == "__main__":
    main()