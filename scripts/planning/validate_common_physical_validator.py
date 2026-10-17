import numpy as np

from src.common.types import (
    FrenetTrajectory,
    FrenetTrajectoryPoint,
    PathPoint,
    StaticObstacle,
)
from src.config.vehicle import (
    DEFAULT_VEHICLE_CONFIG,
)
from src.planning.hard_constraints import (
    is_candidate_feasible,
    is_physical_path_feasible,
)
from src.planning.optimization_planner import (
    solve_matched_optimized_path,
)
from src.planning.reference_line import (
    ReferenceLine,
)


def build_straight_test_path(
    lateral_m: float = 0.0,
    curvature: float = 0.0,
) -> tuple[
    FrenetTrajectory,
    list[PathPoint],
]:
    s_values = (
        0.0,
        5.0,
        10.0,
    )

    trajectory = FrenetTrajectory(
        points=[
            FrenetTrajectoryPoint(
                s=s,
                l=lateral_m,
                dl_ds=0.0,
                d2l_ds2=0.0,
            )
            for s in s_values
        ]
    )

    path = [
        PathPoint(
            x=s,
            y=lateral_m,
            yaw=0.0,
            curvature=curvature,
            s=s,
        )
        for s in s_values
    ]

    return (
        trajectory,
        path,
    )


def validate_exact_constraint_cases() -> None:
    vehicle_params = (
        DEFAULT_VEHICLE_CONFIG
        .to_vehicle_params()
    )

    safe_trajectory, safe_path = (
        build_straight_test_path()
    )

    safe_obstacle = StaticObstacle(
        x=5.0,
        y=3.0,
        radius_m=0.5,
    )

    safe = is_physical_path_feasible(
        trajectory=safe_trajectory,
        path=safe_path,
        vehicle_params=vehicle_params,
        obstacles=[safe_obstacle],
        max_curvature=0.1,
        minimum_clearance_m=0.5,
        min_l=-2.0,
        max_l=2.0,
    )

    assert safe

    curvature_trajectory, curvature_path = (
        build_straight_test_path(
            curvature=0.2,
        )
    )

    curvature_feasible = (
        is_physical_path_feasible(
            trajectory=curvature_trajectory,
            path=curvature_path,
            vehicle_params=vehicle_params,
            obstacles=[safe_obstacle],
            max_curvature=0.1,
            minimum_clearance_m=0.5,
            min_l=-2.0,
            max_l=2.0,
        )
    )

    assert not curvature_feasible

    clearance_obstacle = StaticObstacle(
        x=5.0,
        y=1.5,
        radius_m=0.5,
    )

    clearance_feasible = (
        is_physical_path_feasible(
            trajectory=safe_trajectory,
            path=safe_path,
            vehicle_params=vehicle_params,
            obstacles=[clearance_obstacle],
            max_curvature=0.1,
            minimum_clearance_m=0.5,
            min_l=-2.0,
            max_l=2.0,
        )
    )

    assert not clearance_feasible

    road_trajectory, road_path = (
        build_straight_test_path(
            lateral_m=1.2,
        )
    )

    road_feasible = (
        is_physical_path_feasible(
            trajectory=road_trajectory,
            path=road_path,
            vehicle_params=vehicle_params,
            obstacles=[],
            max_curvature=0.1,
            minimum_clearance_m=0.5,
            min_l=-2.0,
            max_l=2.0,
        )
    )

    assert not road_feasible

    print(
        "Common exact physical constraint cases passed."
    )
    print(
        "  safe_path=True"
    )
    print(
        "  curvature_violation=False"
    )
    print(
        "  clearance_violation=False"
    )
    print(
        "  road_boundary_violation=False"
    )


def validate_sampling_uses_common_validator() -> None:
    vehicle_params = (
        DEFAULT_VEHICLE_CONFIG
        .to_vehicle_params()
    )

    trajectory, path = (
        build_straight_test_path()
    )

    obstacle = StaticObstacle(
        x=5.0,
        y=3.0,
        radius_m=0.5,
    )

    common_result = (
        is_physical_path_feasible(
            trajectory=trajectory,
            path=path,
            vehicle_params=vehicle_params,
            obstacles=[obstacle],
            max_curvature=0.1,
            minimum_clearance_m=0.5,
            min_l=-2.0,
            max_l=2.0,
        )
    )

    sampling_result = (
        is_candidate_feasible(
            trajectory=trajectory,
            path=path,
            vehicle_params=vehicle_params,
            obstacles=[obstacle],
            max_curvature=0.1,
            minimum_clearance_m=0.5,
            min_l=-2.0,
            max_l=2.0,
        )
    )

    assert (
        sampling_result
        == common_result
    )

    print(
        "Sampling/common validator equivalence passed."
    )


def validate_optimization_output_compatible() -> None:
    reference_line = ReferenceLine([
        (0.0, 0.0),
        (20.0, 0.0),
    ])

    resolution_m = 0.5

    s_values = np.arange(
        0.0,
        10.0 + resolution_m,
        resolution_m,
    )

    lower_bounds = np.full(
        len(s_values),
        -2.0,
    )

    upper_bounds = np.full(
        len(s_values),
        2.0,
    )

    result = solve_matched_optimized_path(
        reference_line=reference_line,
        s_values=s_values,
        corridor_lower_bounds=lower_bounds,
        corridor_upper_bounds=upper_bounds,
        resolution_m=resolution_m,
        initial_l=0.0,
        initial_dl_ds=0.0,
        initial_d2l_ds2=0.0,
        lateral_reference_m=0.0,
        terminal_lateral_target_m=0.0,
        terminal_first_derivative_target=0.0,
        lateral_weight=1.0,
        first_derivative_weight=1.0,
        second_derivative_weight=5.0,
        third_derivative_weight=20.0,
        terminal_lateral_weight=5.0,
        terminal_first_derivative_weight=5.0,
        max_second_derivative_abs=1.0,
    )

    vehicle_params = (
        DEFAULT_VEHICLE_CONFIG
        .to_vehicle_params()
    )

    feasible = is_physical_path_feasible(
        trajectory=result.trajectory,
        path=result.path,
        vehicle_params=vehicle_params,
        obstacles=[],
        max_curvature=0.1,
        minimum_clearance_m=0.5,
        min_l=-2.0,
        max_l=2.0,
    )

    assert feasible

    print(
        "Optimization/common validator compatibility passed."
    )

    print(
        "  max_abs_curvature="
        f"{max(abs(point.curvature) for point in result.path):.6f}"
    )


def main() -> None:
    validate_exact_constraint_cases()
    validate_sampling_uses_common_validator()
    validate_optimization_output_compatible()

    print()
    print(
        "All common physical validator cases passed."
    )


if __name__ == "__main__":
    main()