from dataclasses import dataclass
import numpy as np
import osqp

from scipy import sparse
from src.common.types import (
    FrenetTrajectory,
    FrenetTrajectoryPoint,
    PathPoint,
)
from src.planning.frenet import (
    frenet_trajectory_to_path,
)
from src.planning.reference_line import (
    ReferenceLine,
)
from src.planning.lateral_corridor import (
    LateralCorridor,
    is_corridor_feasible,
)


@dataclass
class OptimizationPathResult:
    lateral_solution: np.ndarray
    trajectory: FrenetTrajectory
    path: list[PathPoint]
    objective_value: float


def build_difference_matrices(
    num_points: int,
    resolution_m: float,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    if num_points < 4:
        raise ValueError(
            "At least 4 points are required."
        )

    if resolution_m <= 0.0:
        raise ValueError(
            "resolution_m must be positive."
        )

    d1 = np.zeros(
        (num_points - 1, num_points)
    )

    for i in range(num_points - 1):
        d1[i, i] = -1.0 / resolution_m
        d1[i, i + 1] = 1.0 / resolution_m

    d2 = np.zeros(
        (num_points - 2, num_points)
    )

    resolution_squared = (
        resolution_m ** 2
    )

    for i in range(num_points - 2):
        d2[i, i] = 1.0 / resolution_squared
        d2[i, i + 1] = -2.0 / resolution_squared
        d2[i, i + 2] = 1.0 / resolution_squared

    d3 = np.zeros(
        (num_points - 3, num_points)
    )

    resolution_cubed = (
        resolution_m ** 3
    )

    for i in range(num_points - 3):
        d3[i, i] = -1.0 / resolution_cubed
        d3[i, i + 1] = 3.0 / resolution_cubed
        d3[i, i + 2] = -3.0 / resolution_cubed
        d3[i, i + 3] = 1.0 / resolution_cubed

    return d1, d2, d3


def build_qp_objective(
    num_points: int,
    resolution_m: float,
    lateral_reference_m: float,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
) -> tuple[
    np.ndarray,
    np.ndarray,
]:
    weights = [
        lateral_weight,
        first_derivative_weight,
        second_derivative_weight,
        third_derivative_weight,
    ]

    if any(weight < 0.0 for weight in weights):
        raise ValueError(
            "QP objective weights must not be negative."
        )

    d1, d2, d3 = build_difference_matrices(
        num_points=num_points,
        resolution_m=resolution_m,
    )

    identity = np.eye(
        num_points
    )

    quadratic_matrix = (
        lateral_weight
        * identity
        + first_derivative_weight
        * (d1.T @ d1)
        + second_derivative_weight
        * (d2.T @ d2)
        + third_derivative_weight
        * (d3.T @ d3)
    )

    p_matrix = (
        2.0
        * quadratic_matrix
    )

    lateral_reference = np.full(
        num_points,
        lateral_reference_m,
    )

    q_vector = (
        -2.0
        * lateral_weight
        * lateral_reference
    )

    return (
        p_matrix,
        q_vector,
    )


def build_qp_constraints(
    corridor_lower_bounds: np.ndarray,
    corridor_upper_bounds: np.ndarray,
    resolution_m: float,
    initial_l: float,
    initial_dl_ds: float,
    initial_d2l_ds2: float,
) -> tuple[
    np.ndarray,
    np.ndarray,
    np.ndarray,
]:
    lower_bounds = np.asarray(
        corridor_lower_bounds,
        dtype=float,
    )

    upper_bounds = np.asarray(
        corridor_upper_bounds,
        dtype=float,
    )

    if lower_bounds.ndim != 1 or upper_bounds.ndim != 1:
        raise ValueError(
            "Corridor bounds must be one-dimensional."
        )

    if lower_bounds.shape != upper_bounds.shape:
        raise ValueError(
            "Corridor bounds must have the same shape."
        )

    if np.any(lower_bounds > upper_bounds):
        raise ValueError(
            "Corridor lower bounds must not exceed upper bounds."
        )

    num_points = lower_bounds.size

    d1, d2, _ = build_difference_matrices(
        num_points=num_points,
        resolution_m=resolution_m,
    )

    identity = np.eye(
        num_points
    )

    a_matrix = np.vstack([
        identity,
        identity[0:1],
        d1[0:1],
        d2[0:1],
    ])

    lower_constraint_bounds = np.concatenate([
        lower_bounds,
        np.array([
            initial_l,
            initial_dl_ds,
            initial_d2l_ds2,
        ]),
    ])

    upper_constraint_bounds = np.concatenate([
        upper_bounds,
        np.array([
            initial_l,
            initial_dl_ds,
            initial_d2l_ds2,
        ]),
    ])

    return (
        a_matrix,
        lower_constraint_bounds,
        upper_constraint_bounds,
    )


def solve_qp(
    p_matrix: np.ndarray,
    q_vector: np.ndarray,
    a_matrix: np.ndarray,
    lower_bounds: np.ndarray,
    upper_bounds: np.ndarray,
) -> tuple[np.ndarray, float]:
    p_sparse = sparse.csc_matrix(
        np.triu(p_matrix)
    )

    a_sparse = sparse.csc_matrix(
        a_matrix
    )

    solver = osqp.OSQP()

    solver.setup(
        P=p_sparse,
        q=q_vector,
        A=a_sparse,
        l=lower_bounds,
        u=upper_bounds,
        verbose=False,
        eps_abs=1e-6,
        eps_rel=1e-6,
    )

    result = solver.solve()

    if result.info.status.lower() != "solved":
        raise RuntimeError(
            f"QP solver failed: {result.info.status}"
        )

    return (
        np.asarray(result.x, dtype=float),
        float(result.info.obj_val),
    )


def build_frenet_trajectory(
    s_values: np.ndarray,
    lateral_solution: np.ndarray,
    resolution_m: float,
) -> FrenetTrajectory:
    s_values = np.asarray(
        s_values,
        dtype=float,
    )

    lateral_solution = np.asarray(
        lateral_solution,
        dtype=float,
    )

    if s_values.ndim != 1 or lateral_solution.ndim != 1:
        raise ValueError(
            "s_values and lateral_solution must be one-dimensional."
        )

    if s_values.shape != lateral_solution.shape:
        raise ValueError(
            "s_values and lateral_solution must have the same shape."
        )

    if lateral_solution.size < 3:
        raise ValueError(
            "At least 3 points are required."
        )

    if resolution_m <= 0.0:
        raise ValueError(
            "resolution_m must be positive."
        )

    dl_ds = np.empty_like(
        lateral_solution
    )

    d2l_ds2 = np.empty_like(
        lateral_solution
    )

    dl_ds[0] = (
        lateral_solution[1]
        - lateral_solution[0]
    ) / resolution_m

    dl_ds[-1] = (
        lateral_solution[-1]
        - lateral_solution[-2]
    ) / resolution_m

    dl_ds[1:-1] = (
        lateral_solution[2:]
        - lateral_solution[:-2]
    ) / (2.0 * resolution_m)

    resolution_squared = (
        resolution_m ** 2
    )

    d2l_ds2[0] = (
        lateral_solution[0]
        - 2.0 * lateral_solution[1]
        + lateral_solution[2]
    ) / resolution_squared

    d2l_ds2[-1] = (
        lateral_solution[-1]
        - 2.0 * lateral_solution[-2]
        + lateral_solution[-3]
    ) / resolution_squared

    d2l_ds2[1:-1] = (
        lateral_solution[:-2]
        - 2.0 * lateral_solution[1:-1]
        + lateral_solution[2:]
    ) / resolution_squared

    points = [
        FrenetTrajectoryPoint(
            s=float(s_values[i]),
            l=float(lateral_solution[i]),
            dl_ds=float(dl_ds[i]),
            d2l_ds2=float(d2l_ds2[i]),
        )
        for i in range(lateral_solution.size)
    ]

    return FrenetTrajectory(
        points=points
    )


def solve_optimized_path(
    reference_line: ReferenceLine,
    s_values: np.ndarray,
    corridor_lower_bounds: np.ndarray,
    corridor_upper_bounds: np.ndarray,
    resolution_m: float,
    initial_l: float,
    initial_dl_ds: float,
    initial_d2l_ds2: float,
    lateral_reference_m: float = 0.0,
    lateral_weight: float = 0.2,
    first_derivative_weight: float = 1.0,
    second_derivative_weight: float = 5.0,
    third_derivative_weight: float = 20.0,
) -> OptimizationPathResult:
    num_points = len(s_values)

    if num_points != len(corridor_lower_bounds):
        raise ValueError(
            "s_values and corridor bounds must have the same length."
        )

    if num_points != len(corridor_upper_bounds):
        raise ValueError(
            "s_values and corridor bounds must have the same length."
        )

    p_matrix, q_vector = build_qp_objective(
        num_points=num_points,
        resolution_m=resolution_m,
        lateral_reference_m=lateral_reference_m,
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
    )

    (
        a_matrix,
        lower_bounds,
        upper_bounds,
    ) = build_qp_constraints(
        corridor_lower_bounds=corridor_lower_bounds,
        corridor_upper_bounds=corridor_upper_bounds,
        resolution_m=resolution_m,
        initial_l=initial_l,
        initial_dl_ds=initial_dl_ds,
        initial_d2l_ds2=initial_d2l_ds2,
    )

    (
        lateral_solution,
        objective_value,
    ) = solve_qp(
        p_matrix=p_matrix,
        q_vector=q_vector,
        a_matrix=a_matrix,
        lower_bounds=lower_bounds,
        upper_bounds=upper_bounds,
    )

    trajectory = build_frenet_trajectory(
        s_values=s_values,
        lateral_solution=lateral_solution,
        resolution_m=resolution_m,
    )

    path = frenet_trajectory_to_path(
        trajectory=trajectory,
        reference_line=reference_line,
    )

    return OptimizationPathResult(
        lateral_solution=lateral_solution,
        trajectory=trajectory,
        path=path,
        objective_value=objective_value,
    )


def solve_multi_corridor_path(
    reference_line: ReferenceLine,
    corridors: list[LateralCorridor],
    resolution_m: float,
    initial_l: float,
    initial_dl_ds: float,
    initial_d2l_ds2: float,
    lateral_reference_m: float = 0.0,
    lateral_weight: float = 0.2,
    first_derivative_weight: float = 1.0,
    second_derivative_weight: float = 5.0,
    third_derivative_weight: float = 20.0,
) -> OptimizationPathResult:
    if len(corridors) == 0:
        raise ValueError(
            "At least one corridor is required."
        )

    reference_s_values = corridors[0].s_values

    feasible_results = []

    for corridor in corridors:
        if (
            corridor.s_values.shape
            != reference_s_values.shape
            or not np.allclose(
                corridor.s_values,
                reference_s_values,
            )
        ):
            raise ValueError(
                "All corridors must share the same s_values."
            )

        if not is_corridor_feasible(
            corridor
        ):
            continue

        try:
            result = solve_optimized_path(
                reference_line=reference_line,
                s_values=corridor.s_values,
                corridor_lower_bounds=corridor.lower_bounds,
                corridor_upper_bounds=corridor.upper_bounds,
                resolution_m=resolution_m,
                initial_l=initial_l,
                initial_dl_ds=initial_dl_ds,
                initial_d2l_ds2=initial_d2l_ds2,
                lateral_reference_m=lateral_reference_m,
                lateral_weight=lateral_weight,
                first_derivative_weight=first_derivative_weight,
                second_derivative_weight=second_derivative_weight,
                third_derivative_weight=third_derivative_weight,
            )
        except RuntimeError:
            continue

        feasible_results.append(
            result
        )

    if len(feasible_results) == 0:
        raise RuntimeError(
            "No feasible optimization corridor found."
        )

    return min(
        feasible_results,
        key=lambda result: result.objective_value,
    )


def plan_optimized_path(
    reference_line: ReferenceLine,
    s_values: np.ndarray,
    corridor_lower_bounds: np.ndarray,
    corridor_upper_bounds: np.ndarray,
    resolution_m: float,
    initial_l: float,
    initial_dl_ds: float,
    initial_d2l_ds2: float,
    lateral_reference_m: float = 0.0,
    lateral_weight: float = 0.2,
    first_derivative_weight: float = 1.0,
    second_derivative_weight: float = 5.0,
    third_derivative_weight: float = 20.0,
) -> list[PathPoint]:
    result = solve_optimized_path(
        reference_line=reference_line,
        s_values=s_values,
        corridor_lower_bounds=corridor_lower_bounds,
        corridor_upper_bounds=corridor_upper_bounds,
        resolution_m=resolution_m,
        initial_l=initial_l,
        initial_dl_ds=initial_dl_ds,
        initial_d2l_ds2=initial_d2l_ds2,
        lateral_reference_m=lateral_reference_m,
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
    )

    return result.path