import numpy as np

from src.common.types import (
    VehicleParams,
)
from src.planning.frenet import (
    sl_to_xy,
)
from src.planning.lateral_corridor import (
    build_base_lateral_corridor,
)
from src.planning.optimization_planner import (
    solve_optimized_path,
)
from src.planning.reference_line import (
    ReferenceLine,
)


def main() -> None:
    reference_line = ReferenceLine([
        (0.0, 0.0),
        (10.0, 0.0),
        (20.0, 2.0),
        (30.0, 6.0),
        (40.0, 12.0),
        (50.0, 20.0),
    ])

    vehicle_params = VehicleParams(
        wheelbase_m=2.7,
        front_overhang_m=0.9,
        rear_overhang_m=1.0,
        width_m=1.8,
    )

    resolution_m = 0.5

    corridor = build_base_lateral_corridor(
        start_s=10.0,
        reference_line=reference_line,
        vehicle_params=vehicle_params,
        road_min_l=-3.0,
        road_max_l=3.0,
        planning_horizon_m=20.0,
        resolution_m=resolution_m,
    )

    reference_curvatures = np.array([
        reference_line.curvature(float(s))
        for s in corridor.s_values
    ])

    assert np.max(
        np.abs(reference_curvatures)
    ) > 1e-3

    result = solve_optimized_path(
        reference_line=reference_line,
        s_values=corridor.s_values,
        corridor_lower_bounds=corridor.lower_bounds,
        corridor_upper_bounds=corridor.upper_bounds,
        resolution_m=resolution_m,
        initial_l=0.8,
        initial_dl_ds=-0.03,
        initial_d2l_ds2=0.0,
        lateral_reference_m=0.0,
        lateral_weight=0.2,
        first_derivative_weight=1.0,
        second_derivative_weight=5.0,
        third_derivative_weight=20.0,
    )

    solution = result.lateral_solution
    path = result.path
    trajectory = result.trajectory

    assert len(
        solution
    ) == corridor.s_values.size

    assert len(
        trajectory.points
    ) == corridor.s_values.size

    assert len(
        path
    ) == corridor.s_values.size

    assert np.isclose(
        solution[0],
        0.8,
        atol=1e-5,
    )

    assert np.isclose(
        trajectory.points[0].dl_ds,
        -0.03,
        atol=1e-5,
    )

    assert np.isclose(
        trajectory.points[0].d2l_ds2,
        0.0,
        atol=1e-5,
    )

    assert np.all(
        solution
        >= corridor.lower_bounds - 1e-5
    )

    assert np.all(
        solution
        <= corridor.upper_bounds + 1e-5
    )

    assert abs(
        solution[-1]
    ) < 0.1

    expected_start_x, expected_start_y = sl_to_xy(
        s=float(corridor.s_values[0]),
        l=0.8,
        reference_line=reference_line,
    )

    assert np.isclose(
        path[0].x,
        expected_start_x,
        atol=1e-5,
    )

    assert np.isclose(
        path[0].y,
        expected_start_y,
        atol=1e-5,
    )

    path_x = np.array([
        point.x
        for point in path
    ])

    path_y = np.array([
        point.y
        for point in path
    ])

    path_yaw = np.array([
        point.yaw
        for point in path
    ])

    path_curvature = np.array([
        point.curvature
        for point in path
    ])

    assert np.all(
        np.isfinite(path_x)
    )

    assert np.all(
        np.isfinite(path_y)
    )

    assert np.all(
        np.isfinite(path_yaw)
    )

    assert np.all(
        np.isfinite(path_curvature)
    )

    assert np.max(
        np.abs(path_curvature)
    ) > 1e-3

    assert path[-1].s > 19.0

    print("Curved Reference Line optimization passed.")
    print(
        f"  reference_length="
        f"{reference_line.length:.2f}"
    )
    print(
        f"  planning_s="
        f"[{corridor.s_values[0]:.2f}, "
        f"{corridor.s_values[-1]:.2f}]"
    )
    print(
        f"  start_l={solution[0]:.4f}, "
        f"end_l={solution[-1]:.4f}"
    )
    print(
        f"  max_abs_reference_curvature="
        f"{np.max(np.abs(reference_curvatures)):.4f}"
    )
    print(
        f"  max_abs_path_curvature="
        f"{np.max(np.abs(path_curvature)):.4f}"
    )
    print(
        f"  path_length={path[-1].s:.2f}"
    )
    print(
        f"  objective_value="
        f"{result.objective_value:.4f}"
    )

    print()
    print(
        "All curved Reference Line cases passed."
    )


if __name__ == "__main__":
    main()