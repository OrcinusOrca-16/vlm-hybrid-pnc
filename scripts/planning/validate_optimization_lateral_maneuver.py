import numpy as np

from src.common.types import (
    VehicleParams,
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
        (20.0, 0.0),
        (30.0, 0.0),
        (40.0, 0.0),
        (50.0, 0.0),
        (60.0, 0.0),
    ])

    vehicle_params = VehicleParams(
        wheelbase_m=2.7,
        front_overhang_m=0.9,
        rear_overhang_m=1.0,
        width_m=1.8,
    )

    resolution_m = 0.5
    target_l = -1.0

    corridor = build_base_lateral_corridor(
        start_s=10.0,
        reference_line=reference_line,
        vehicle_params=vehicle_params,
        road_min_l=-3.0,
        road_max_l=3.0,
        planning_horizon_m=30.0,
        resolution_m=resolution_m,
    )

    result = solve_optimized_path(
        reference_line=reference_line,
        s_values=corridor.s_values,
        corridor_lower_bounds=corridor.lower_bounds,
        corridor_upper_bounds=corridor.upper_bounds,
        resolution_m=resolution_m,
        initial_l=1.0,
        initial_dl_ds=0.0,
        initial_d2l_ds2=0.0,
        lateral_reference_m=target_l,
        lateral_weight=0.2,
        first_derivative_weight=1.0,
        second_derivative_weight=5.0,
        third_derivative_weight=20.0,
    )

    solution = result.lateral_solution
    trajectory = result.trajectory
    path = result.path

    assert len(
        solution
    ) == corridor.s_values.size

    assert np.isclose(
        solution[0],
        1.0,
        atol=1e-5,
    )

    assert np.isclose(
        trajectory.points[0].dl_ds,
        0.0,
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

    initial_target_error = abs(
        solution[0] - target_l
    )

    final_target_error = abs(
        solution[-1] - target_l
    )

    assert (
        final_target_error
        < initial_target_error
    )

    assert np.any(
        solution < 0.0
    )

    assert np.isclose(
        solution[-1],
        target_l,
        atol=0.1,
    )

    assert np.isclose(
        path[0].x,
        10.0,
        atol=1e-5,
    )

    assert np.isclose(
        path[0].y,
        1.0,
        atol=1e-5,
    )

    assert np.isclose(
        path[-1].y,
        solution[-1],
        atol=1e-5,
    )

    path_curvatures = np.array([
        point.curvature
        for point in path
    ])

    assert np.all(
        np.isfinite(path_curvatures)
    )

    crossing_indices = np.where(
        solution <= 0.0
    )[0]

    assert crossing_indices.size > 0

    first_crossing_index = int(
        crossing_indices[0]
    )

    first_crossing_s = (
        corridor.s_values[
            first_crossing_index
        ]
    )

    print("Lateral maneuver optimization passed.")
    print(
        f"  start_l={solution[0]:.4f}"
    )
    print(
        f"  target_l={target_l:.4f}"
    )
    print(
        f"  end_l={solution[-1]:.4f}"
    )
    print(
        f"  first_center_crossing_s="
        f"{first_crossing_s:.2f}"
    )
    print(
        f"  max_abs_curvature="
        f"{np.max(np.abs(path_curvatures)):.4f}"
    )
    print(
        f"  path_length="
        f"{path[-1].s:.2f}"
    )
    print(
        f"  objective_value="
        f"{result.objective_value:.4f}"
    )

    print()
    print(
        "All lateral maneuver cases passed."
    )


if __name__ == "__main__":
    main()