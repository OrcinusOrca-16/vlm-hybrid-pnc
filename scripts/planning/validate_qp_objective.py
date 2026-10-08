import numpy as np

from src.planning.optimization_planner import (
    build_difference_matrices,
    build_frenet_trajectory,
    build_qp_constraints,
    build_qp_objective,
    plan_optimized_path,
    solve_qp,
)
from src.planning.frenet import (
    frenet_trajectory_to_path,
)
from src.planning.reference_line import (
    ReferenceLine,
)


def run_difference_matrix_case() -> None:
    d1, d2, d3 = build_difference_matrices(
        num_points=4,
        resolution_m=1.0,
    )

    expected_d1 = np.array([
        [-1.0, 1.0, 0.0, 0.0],
        [0.0, -1.0, 1.0, 0.0],
        [0.0, 0.0, -1.0, 1.0],
    ])

    expected_d2 = np.array([
        [1.0, -2.0, 1.0, 0.0],
        [0.0, 1.0, -2.0, 1.0],
    ])

    expected_d3 = np.array([
        [-1.0, 3.0, -3.0, 1.0],
    ])

    assert np.allclose(
        d1,
        expected_d1,
    )

    assert np.allclose(
        d2,
        expected_d2,
    )

    assert np.allclose(
        d3,
        expected_d3,
    )

    print("Difference matrices passed.")


def run_qp_objective_case() -> None:
    p_matrix, q_vector = build_qp_objective(
        num_points=4,
        resolution_m=1.0,
        lateral_reference_m=0.0,
        lateral_weight=1.0,
        first_derivative_weight=1.0,
        second_derivative_weight=1.0,
        third_derivative_weight=1.0,
    )

    expected_p = np.array([
        [8.0, -12.0, 8.0, -2.0],
        [-12.0, 34.0, -28.0, 8.0],
        [8.0, -28.0, 34.0, -12.0],
        [-2.0, 8.0, -12.0, 8.0],
    ])

    expected_q = np.zeros(
        4
    )

    assert np.allclose(
        p_matrix,
        expected_p,
    )

    assert np.allclose(
        q_vector,
        expected_q,
    )

    assert np.allclose(
        p_matrix,
        p_matrix.T,
    )

    eigenvalues = np.linalg.eigvalsh(
        p_matrix
    )

    assert np.all(
        eigenvalues > 0.0
    )

    print("QP objective passed.")
    print(
        f"  eigenvalues={eigenvalues}"
    )


def run_nonzero_reference_case() -> None:
    _, q_vector = build_qp_objective(
        num_points=4,
        resolution_m=1.0,
        lateral_reference_m=-1.0,
        lateral_weight=1.0,
        first_derivative_weight=1.0,
        second_derivative_weight=1.0,
        third_derivative_weight=1.0,
    )

    expected_q = np.array([
        2.0,
        2.0,
        2.0,
        2.0,
    ])

    assert np.allclose(
        q_vector,
        expected_q,
    )

    print("Nonzero reference q passed.")


def run_qp_constraints_case() -> None:
    corridor_lower_bounds = np.full(
        4,
        -2.1,
    )

    corridor_upper_bounds = np.full(
        4,
        2.1,
    )

    (
        a_matrix,
        lower_bounds,
        upper_bounds,
    ) = build_qp_constraints(
        corridor_lower_bounds=corridor_lower_bounds,
        corridor_upper_bounds=corridor_upper_bounds,
        resolution_m=0.5,
        initial_l=1.0,
        initial_dl_ds=-0.05,
        initial_d2l_ds2=0.0,
    )

    expected_a = np.array([
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0, 0.0],
        [-2.0, 2.0, 0.0, 0.0],
        [4.0, -8.0, 4.0, 0.0],
    ])

    expected_lower_bounds = np.array([
        -2.1,
        -2.1,
        -2.1,
        -2.1,
        1.0,
        -0.05,
        0.0,
    ])

    expected_upper_bounds = np.array([
        2.1,
        2.1,
        2.1,
        2.1,
        1.0,
        -0.05,
        0.0,
    ])

    assert np.allclose(
        a_matrix,
        expected_a,
    )

    assert np.allclose(
        lower_bounds,
        expected_lower_bounds,
    )

    assert np.allclose(
        upper_bounds,
        expected_upper_bounds,
    )

    print("QP constraints passed.")


def run_qp_solver_case() -> None:
    num_points = 41
    resolution_m = 0.5

    p_matrix, q_vector = build_qp_objective(
        num_points=num_points,
        resolution_m=resolution_m,
        lateral_reference_m=0.0,
        lateral_weight=1.0,
        first_derivative_weight=1.0,
        second_derivative_weight=10.0,
        third_derivative_weight=100.0,
    )

    corridor_lower_bounds = np.full(
        num_points,
        -2.1,
    )

    corridor_upper_bounds = np.full(
        num_points,
        2.1,
    )

    (
        a_matrix,
        lower_bounds,
        upper_bounds,
    ) = build_qp_constraints(
        corridor_lower_bounds=corridor_lower_bounds,
        corridor_upper_bounds=corridor_upper_bounds,
        resolution_m=resolution_m,
        initial_l=1.0,
        initial_dl_ds=-0.05,
        initial_d2l_ds2=0.0,
    )

    solution = solve_qp(
        p_matrix=p_matrix,
        q_vector=q_vector,
        a_matrix=a_matrix,
        lower_bounds=lower_bounds,
        upper_bounds=upper_bounds,
    )

    initial_dl_ds = (
        solution[1] - solution[0]
    ) / resolution_m

    initial_d2l_ds2 = (
        solution[0]
        - 2.0 * solution[1]
        + solution[2]
    ) / (resolution_m ** 2)

    s_values = (
    12.0
    + np.arange(num_points)
    * resolution_m
    )

    trajectory = build_frenet_trajectory(
        s_values=s_values,
        lateral_solution=solution,
        resolution_m=resolution_m,
    )

    reference_line = ReferenceLine([
    (0.0, 0.0),
    (10.0, 0.0),
    (20.0, 0.0),
    (30.0, 0.0),
    (40.0, 0.0),
    ])

    path = frenet_trajectory_to_path(
        trajectory=trajectory,
        reference_line=reference_line,
    )


    assert np.isclose(
        solution[0],
        1.0,
        atol=1e-5,
    )

    assert np.isclose(
        initial_dl_ds,
        -0.05,
        atol=1e-5,
    )

    assert np.isclose(
        initial_d2l_ds2,
        0.0,
        atol=1e-5,
    )

    assert np.all(
        solution >= corridor_lower_bounds - 1e-5
    )

    assert np.all(
        solution <= corridor_upper_bounds + 1e-5
    )

    assert abs(solution[-1]) < 0.1
    
    assert len(trajectory.points) == num_points

    assert np.isclose(
        trajectory.points[0].s,
        12.0,
    )

    assert np.isclose(
        trajectory.points[-1].s,
        32.0,
    )

    assert np.isclose(
        trajectory.points[0].l,
        1.0,
        atol=1e-5,
    )

    assert np.isclose(
        trajectory.points[0].dl_ds,
        -0.05,
        atol=1e-5,
    )

    assert np.isclose(
        trajectory.points[0].d2l_ds2,
        0.0,
        atol=1e-5,
    )

    assert len(path) == num_points

    assert np.isclose(
        path[0].x,
        12.0,
        atol=1e-5,
    )

    assert np.isclose(
        path[0].y,
        1.0,
        atol=1e-5,
    )

    assert np.isclose(
        path[0].s,
        0.0,
        atol=1e-5,
    )

    assert np.isclose(
        path[-1].x,
        32.0,
        atol=1e-5,
    )

    assert np.isclose(
        path[-1].y,
        solution[-1],
        atol=1e-5,
    )

    assert path[-1].s >= 20.0

    print("QP solver nominal case passed.")
    print(
        f"  start_l={solution[0]:.4f}, "
        f"end_l={solution[-1]:.4f}"
    )
    print(
        f"  start_dl_ds={initial_dl_ds:.4f}, "
        f"start_d2l_ds2={initial_d2l_ds2:.4f}"
    )
    print("QP Frenet trajectory reconstruction passed.")
    print(
        f"  start_s={trajectory.points[0].s:.2f}, "
        f"end_s={trajectory.points[-1].s:.2f}"
    )
    print("QP Cartesian path conversion passed.")
    print(
        f"  start_xy=({path[0].x:.2f}, {path[0].y:.2f}), "
        f"end_xy=({path[-1].x:.2f}, {path[-1].y:.4f})"
    )
    print(
        f"  Cartesian path length={path[-1].s:.2f}"
    )
    print(
        f"  start_yaw={path[0].yaw:.4f}, "
        f"start_curvature={path[0].curvature:.4f}"
    )


def run_optimization_planner_nominal_case() -> None:
    num_points = 41
    resolution_m = 0.5

    reference_line = ReferenceLine([
        (0.0, 0.0),
        (10.0, 0.0),
        (20.0, 0.0),
        (30.0, 0.0),
        (40.0, 0.0),
    ])

    s_values = (
        12.0
        + np.arange(num_points)
        * resolution_m
    )

    corridor_lower_bounds = np.full(
        num_points,
        -2.1,
    )

    corridor_upper_bounds = np.full(
        num_points,
        2.1,
    )

    path = plan_optimized_path(
        reference_line=reference_line,
        s_values=s_values,
        corridor_lower_bounds=corridor_lower_bounds,
        corridor_upper_bounds=corridor_upper_bounds,
        resolution_m=resolution_m,
        initial_l=1.0,
        initial_dl_ds=-0.05,
        initial_d2l_ds2=0.0,
    )

    assert len(path) == num_points

    assert np.isclose(
        path[0].x,
        12.0,
        atol=1e-5,
    )

    assert np.isclose(
        path[0].y,
        1.0,
        atol=1e-5,
    )

    assert np.isclose(
        path[0].s,
        0.0,
        atol=1e-5,
    )

    assert abs(path[-1].y) < 0.1

    assert path[-1].s >= 20.0

    print("Optimization Planner nominal case passed.")
    print(
        f"  start=({path[0].x:.2f}, {path[0].y:.2f}), "
        f"end=({path[-1].x:.2f}, {path[-1].y:.4f})"
    )
    print(
        f"  path_length={path[-1].s:.2f}, "
        f"max_abs_curvature="
        f"{max(abs(point.curvature) for point in path):.4f}"
    )


def main() -> None:
    run_difference_matrix_case()
    run_qp_objective_case()
    run_nonzero_reference_case()
    run_qp_constraints_case()
    run_qp_solver_case()
    run_optimization_planner_nominal_case()

    print()
    print("All QP objective cases passed.")


if __name__ == "__main__":
    main()