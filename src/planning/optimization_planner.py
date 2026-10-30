from dataclasses import dataclass
import math

import numpy as np
import osqp
from scipy import sparse

from src.common.types import (
    FrenetTrajectory,
    FrenetTrajectoryPoint,
    PathPoint,
    StaticObstacle,
    VehicleParams,
)
from src.config.planning import (
    DEFAULT_OPTIMIZATION_PLANNER_CONFIG,
)
from src.planning.frenet import (
    frenet_to_cartesian_geometry,
    frenet_trajectory_to_path,
)
from src.planning.hard_constraints import (
    is_curvature_feasible,
    is_obstacle_clearance_feasible,
    is_physical_path_feasible,
    is_road_boundary_feasible,
    obstacle_clearance,
)
from src.planning.lateral_corridor import (
    LateralCorridor,
    is_corridor_feasible,
    project_obstacle_to_frenet,
)
from src.planning.path_objective import (
    PathObjectiveQP,
    PathObjectiveWeights,
    evaluate_frenet_trajectory_objective,
)
from src.planning.reference_line import (
    ReferenceLine,
)


@dataclass
class OptimizationPathResult:
    lateral_solution: np.ndarray
    trajectory: FrenetTrajectory
    path: list[PathPoint]
    objective_value: float
    iterations: int = 0
    branch_name: str | None = None
    min_clearance_m: float | None = None
    max_curvature: float | None = None


def l_index(i: int, num_points: int) -> int:
    return i


def dl_index(i: int, num_points: int) -> int:
    return num_points + i


def ddl_index(i: int, num_points: int) -> int:
    return 2 * num_points + i


def build_explicit_qp_objective(
    num_points: int,
    resolution_m: float,
    lateral_reference_m: float,
    terminal_lateral_target_m: float,
    terminal_first_derivative_target: float,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
) -> PathObjectiveQP:
    if num_points < 2:
        raise ValueError("At least two points are required.")
    if resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive.")
    weights = (
        lateral_weight,
        first_derivative_weight,
        second_derivative_weight,
        third_derivative_weight,
        terminal_lateral_weight,
        terminal_first_derivative_weight,
    )
    if any(weight < 0.0 for weight in weights):
        raise ValueError("Objective weights must not be negative.")

    num_variables = 3 * num_points
    p_matrix = np.zeros((num_variables, num_variables), dtype=float)
    q_vector = np.zeros(num_variables, dtype=float)

    lateral_coefficient = lateral_weight / num_points
    for i in range(num_points):
        index = l_index(i, num_points)
        p_matrix[index, index] += 2.0 * lateral_coefficient
        q_vector[index] += -2.0 * lateral_coefficient * lateral_reference_m

    first_derivative_coefficient = first_derivative_weight / num_points
    for i in range(num_points):
        index = dl_index(i, num_points)
        p_matrix[index, index] += 2.0 * first_derivative_coefficient

    second_derivative_coefficient = second_derivative_weight / num_points
    for i in range(num_points):
        index = ddl_index(i, num_points)
        p_matrix[index, index] += 2.0 * second_derivative_coefficient

    jerk_coefficient = (
        third_derivative_weight
        / (num_points - 1)
        / (resolution_m ** 2)
    )
    for i in range(num_points - 1):
        index_0 = ddl_index(i, num_points)
        index_1 = ddl_index(i + 1, num_points)
        p_matrix[index_0, index_0] += 2.0 * jerk_coefficient
        p_matrix[index_1, index_1] += 2.0 * jerk_coefficient
        p_matrix[index_0, index_1] += -2.0 * jerk_coefficient
        p_matrix[index_1, index_0] += -2.0 * jerk_coefficient

    terminal_l_index = l_index(num_points - 1, num_points)
    p_matrix[terminal_l_index, terminal_l_index] += 2.0 * terminal_lateral_weight
    q_vector[terminal_l_index] += (
        -2.0 * terminal_lateral_weight * terminal_lateral_target_m
    )

    terminal_dl_index = dl_index(num_points - 1, num_points)
    p_matrix[terminal_dl_index, terminal_dl_index] += (
        2.0 * terminal_first_derivative_weight
    )
    q_vector[terminal_dl_index] += (
        -2.0
        * terminal_first_derivative_weight
        * terminal_first_derivative_target
    )

    constant = (
        lateral_weight * lateral_reference_m ** 2
        + terminal_lateral_weight * terminal_lateral_target_m ** 2
        + terminal_first_derivative_weight * terminal_first_derivative_target ** 2
    )

    p_matrix += 1e-9 * np.eye(num_variables)
    return PathObjectiveQP(
        p=p_matrix,
        q=q_vector,
        constant=float(constant),
    )


def build_explicit_qp_constraints(
    reference_line: ReferenceLine,
    s_values: np.ndarray,
    corridor_lower_bounds: np.ndarray,
    corridor_upper_bounds: np.ndarray,
    resolution_m: float,
    initial_l: float,
    initial_dl_ds: float,
    initial_d2l_ds2: float,
    max_curvature: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    s_values = np.asarray(s_values, dtype=float)
    lower_corridor = np.asarray(corridor_lower_bounds, dtype=float)
    upper_corridor = np.asarray(corridor_upper_bounds, dtype=float)

    if (
        s_values.ndim != 1
        or lower_corridor.ndim != 1
        or upper_corridor.ndim != 1
    ):
        raise ValueError("s_values and corridor bounds must be one-dimensional.")
    if not (s_values.shape == lower_corridor.shape == upper_corridor.shape):
        raise ValueError("s_values and corridor bounds must have the same shape.")
    if np.any(lower_corridor > upper_corridor):
        raise ValueError("Corridor lower bounds must not exceed upper bounds.")
    if resolution_m <= 0.0:
        raise ValueError("resolution_m must be positive.")
    if max_curvature <= 0.0:
        raise ValueError("max_curvature must be positive.")

    num_points = s_values.size
    num_variables = 3 * num_points
    rows: list[np.ndarray] = []
    lower_bounds: list[float] = []
    upper_bounds: list[float] = []

    def add_constraint(row: np.ndarray, lower: float, upper: float) -> None:
        rows.append(row)
        lower_bounds.append(lower)
        upper_bounds.append(upper)

    for i in range(num_points):
        row = np.zeros(num_variables, dtype=float)
        row[l_index(i, num_points)] = 1.0
        add_constraint(
            row=row,
            lower=float(lower_corridor[i]),
            upper=float(upper_corridor[i]),
        )

    initial_states = (
        (l_index(0, num_points), initial_l),
        (dl_index(0, num_points), initial_dl_ds),
        (ddl_index(0, num_points), initial_d2l_ds2),
    )
    for index, value in initial_states:
        row = np.zeros(num_variables, dtype=float)
        row[index] = 1.0
        add_constraint(row=row, lower=float(value), upper=float(value))

    ds = resolution_m
    ds_squared = ds ** 2
    for i in range(num_points - 1):
        row = np.zeros(num_variables, dtype=float)
        row[dl_index(i + 1, num_points)] = 1.0
        row[dl_index(i, num_points)] = -1.0
        row[ddl_index(i, num_points)] = -0.5 * ds
        row[ddl_index(i + 1, num_points)] = -0.5 * ds
        add_constraint(row=row, lower=0.0, upper=0.0)

        row = np.zeros(num_variables, dtype=float)
        row[l_index(i + 1, num_points)] = 1.0
        row[l_index(i, num_points)] = -1.0
        row[dl_index(i, num_points)] = -ds
        row[ddl_index(i, num_points)] = -ds_squared / 3.0
        row[ddl_index(i + 1, num_points)] = -ds_squared / 6.0
        add_constraint(row=row, lower=0.0, upper=0.0)

    for i, s_value in enumerate(s_values):
        reference_point = reference_line.query(float(s_value))
        reference_curvature = reference_point.curvature
        ddl_lower = -max_curvature - reference_curvature
        ddl_upper = max_curvature - reference_curvature

        row = np.zeros(num_variables, dtype=float)
        row[ddl_index(i, num_points)] = 1.0
        add_constraint(row=row, lower=ddl_lower, upper=ddl_upper)

    return (
        np.vstack(rows),
        np.asarray(lower_bounds, dtype=float),
        np.asarray(upper_bounds, dtype=float),
    )


def solve_qp(
    p_matrix: np.ndarray,
    q_vector: np.ndarray,
    a_matrix: np.ndarray,
    lower_bounds: np.ndarray,
    upper_bounds: np.ndarray,
    verbose: bool = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_verbose,
    eps_abs: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_abs,
    eps_rel: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_rel,
) -> tuple[np.ndarray, float]:
    solver = osqp.OSQP()
    solver.setup(
        P=sparse.csc_matrix(np.triu(p_matrix)),
        q=q_vector,
        A=sparse.csc_matrix(a_matrix),
        l=lower_bounds,
        u=upper_bounds,
        verbose=verbose,
        eps_abs=eps_abs,
        eps_rel=eps_rel,
    )
    result = solver.solve()
    if result.info.status.lower() not in ("solved", "solved inaccurate"):
        raise RuntimeError(f"QP solver failed: {result.info.status}")
    return np.asarray(result.x, dtype=float), float(result.info.obj_val)


def decode_explicit_solution(
    s_values: np.ndarray,
    solution: np.ndarray,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, FrenetTrajectory]:
    s_values = np.asarray(s_values, dtype=float)
    solution = np.asarray(solution, dtype=float)
    if s_values.ndim != 1:
        raise ValueError("s_values must be one-dimensional.")
    if solution.ndim != 1:
        raise ValueError("solution must be one-dimensional.")

    num_points = s_values.size
    if solution.size != 3 * num_points:
        raise ValueError("Explicit solution must contain 3 * num_points variables.")

    lateral = solution[0:num_points]
    first_derivative = solution[num_points:2 * num_points]
    second_derivative = solution[2 * num_points:3 * num_points]

    trajectory = FrenetTrajectory(
        points=[
            FrenetTrajectoryPoint(
                s=float(s_values[i]),
                l=float(lateral[i]),
                dl_ds=float(first_derivative[i]),
                d2l_ds2=float(second_derivative[i]),
            )
            for i in range(num_points)
        ]
    )
    return lateral, first_derivative, second_derivative, trajectory


def clearance_from_frenet_state(
    reference_line: ReferenceLine,
    vehicle_params: VehicleParams,
    obstacle: StaticObstacle,
    s: float,
    l: float,
    dl_ds: float,
) -> float:
    frenet_point = FrenetTrajectoryPoint(
        s=float(s),
        l=float(l),
        dl_ds=float(dl_ds),
        d2l_ds2=0.0,
    )
    x, y, yaw, _ = frenet_to_cartesian_geometry(
        frenet_point=frenet_point,
        reference_line=reference_line,
    )
    path_point = PathPoint(
        x=x,
        y=y,
        yaw=yaw,
        curvature=0.0,
        s=0.0,
    )
    return obstacle_clearance(
        path_point=path_point,
        vehicle_params=vehicle_params,
        obstacle=obstacle,
    )


def clearance_gradient(
    reference_line: ReferenceLine,
    vehicle_params: VehicleParams,
    obstacle: StaticObstacle,
    s: float,
    l: float,
    dl_ds: float,
    gradient_eps_l: float,
    gradient_eps_dl: float,
) -> tuple[float, float, float]:
    if gradient_eps_l <= 0.0 or gradient_eps_dl <= 0.0:
        raise ValueError("Gradient epsilons must be positive.")

    c0 = clearance_from_frenet_state(
        reference_line=reference_line,
        vehicle_params=vehicle_params,
        obstacle=obstacle,
        s=s,
        l=l,
        dl_ds=dl_ds,
    )
    c_l_plus = clearance_from_frenet_state(
        reference_line=reference_line,
        vehicle_params=vehicle_params,
        obstacle=obstacle,
        s=s,
        l=l + gradient_eps_l,
        dl_ds=dl_ds,
    )
    c_l_minus = clearance_from_frenet_state(
        reference_line=reference_line,
        vehicle_params=vehicle_params,
        obstacle=obstacle,
        s=s,
        l=l - gradient_eps_l,
        dl_ds=dl_ds,
    )
    grad_l = (c_l_plus - c_l_minus) / (2.0 * gradient_eps_l)

    c_dl_plus = clearance_from_frenet_state(
        reference_line=reference_line,
        vehicle_params=vehicle_params,
        obstacle=obstacle,
        s=s,
        l=l,
        dl_ds=dl_ds + gradient_eps_dl,
    )
    c_dl_minus = clearance_from_frenet_state(
        reference_line=reference_line,
        vehicle_params=vehicle_params,
        obstacle=obstacle,
        s=s,
        l=l,
        dl_ds=dl_ds - gradient_eps_dl,
    )
    grad_dl = (c_dl_plus - c_dl_minus) / (2.0 * gradient_eps_dl)
    return c0, grad_l, grad_dl


def build_initial_heading_constraints(
    reference_line: ReferenceLine,
    s_values: np.ndarray,
    vehicle_params: VehicleParams,
    obstacle: StaticObstacle,
    minimum_clearance_m: float,
    branch_name: str,
    body_sample_count: int,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if branch_name not in ("negative", "positive"):
        raise ValueError("branch_name must be 'negative' or 'positive'.")
    if minimum_clearance_m < 0.0:
        raise ValueError("minimum_clearance_m must not be negative.")
    if body_sample_count < 2:
        raise ValueError("body_sample_count must be at least 2.")

    num_points = len(s_values)
    num_variables = 3 * num_points
    frenet_obstacle = project_obstacle_to_frenet(
        obstacle=obstacle,
        reference_line=reference_line,
    )
    front_extent = vehicle_params.wheelbase_m + vehicle_params.front_overhang_m
    rear_extent = vehicle_params.rear_overhang_m
    half_width = vehicle_params.width_m / 2.0
    effective_radius = obstacle.radius_m + minimum_clearance_m
    body_x_samples = np.linspace(
        -rear_extent,
        front_extent,
        body_sample_count,
    )
    y_c = half_width if branch_name == "negative" else -half_width

    rows: list[np.ndarray] = []
    lower: list[float] = []
    upper: list[float] = []

    for i, s_value in enumerate(s_values):
        for x_c in body_x_samples:
            body_s = float(s_value) + float(x_c)
            delta_s = body_s - frenet_obstacle.s
            if abs(delta_s) >= effective_radius:
                continue

            circle_half_height = math.sqrt(
                max(0.0, effective_radius ** 2 - delta_s ** 2)
            )
            row = np.zeros(num_variables, dtype=float)
            row[l_index(i, num_points)] = 1.0
            row[dl_index(i, num_points)] = float(x_c)

            if branch_name == "negative":
                obstacle_boundary_l = frenet_obstacle.l - circle_half_height
                rows.append(row)
                lower.append(-np.inf)
                upper.append(obstacle_boundary_l - y_c)
            else:
                obstacle_boundary_l = frenet_obstacle.l + circle_half_height
                rows.append(row)
                lower.append(obstacle_boundary_l - y_c)
                upper.append(np.inf)

    if not rows:
        return (
            np.zeros((0, num_variables), dtype=float),
            np.zeros(0, dtype=float),
            np.zeros(0, dtype=float),
        )

    return (
        np.vstack(rows),
        np.asarray(lower, dtype=float),
        np.asarray(upper, dtype=float),
    )


def build_linearized_clearance_constraints(
    reference_line: ReferenceLine,
    s_values: np.ndarray,
    vehicle_params: VehicleParams,
    obstacle: StaticObstacle,
    minimum_clearance_m: float,
    nominal_l: np.ndarray,
    nominal_dl: np.ndarray,
    trust_l_m: float,
    trust_dl: float,
    gradient_eps_l: float,
    gradient_eps_dl: float,
    clearance_numerical_buffer_m: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    if trust_l_m <= 0.0:
        raise ValueError("trust_l_m must be positive.")
    if trust_dl <= 0.0:
        raise ValueError("trust_dl must be positive.")
    if clearance_numerical_buffer_m < 0.0:
        raise ValueError("clearance_numerical_buffer_m must not be negative.")

    target_clearance_m = minimum_clearance_m + clearance_numerical_buffer_m
    num_points = len(s_values)
    num_variables = 3 * num_points
    rows: list[np.ndarray] = []
    lower: list[float] = []
    upper: list[float] = []

    for i, s_value in enumerate(s_values):
        c0, grad_l, grad_dl = clearance_gradient(
            reference_line=reference_line,
            vehicle_params=vehicle_params,
            obstacle=obstacle,
            s=float(s_value),
            l=float(nominal_l[i]),
            dl_ds=float(nominal_dl[i]),
            gradient_eps_l=gradient_eps_l,
            gradient_eps_dl=gradient_eps_dl,
        )

        row = np.zeros(num_variables, dtype=float)
        row[l_index(i, num_points)] = 1.0
        rows.append(row)
        lower.append(float(nominal_l[i] - trust_l_m))
        upper.append(float(nominal_l[i] + trust_l_m))

        row = np.zeros(num_variables, dtype=float)
        row[dl_index(i, num_points)] = 1.0
        rows.append(row)
        lower.append(float(nominal_dl[i] - trust_dl))
        upper.append(float(nominal_dl[i] + trust_dl))

        possible_clearance_drop = (
            abs(grad_l) * trust_l_m
            + abs(grad_dl) * trust_dl
        )
        if c0 - possible_clearance_drop > target_clearance_m:
            continue

        gradient_norm = math.hypot(grad_l, grad_dl)
        if gradient_norm < 1e-8:
            if c0 < target_clearance_m:
                raise RuntimeError(
                    "Clearance linearization has zero gradient at a violating point: "
                    f"s={s_value:.3f}, clearance={c0:.6f}"
                )
            continue

        row = np.zeros(num_variables, dtype=float)
        row[l_index(i, num_points)] = grad_l
        row[dl_index(i, num_points)] = grad_dl
        required_lower = (
            target_clearance_m
            - c0
            + grad_l * nominal_l[i]
            + grad_dl * nominal_dl[i]
        )
        rows.append(row)
        lower.append(float(required_lower))
        upper.append(np.inf)

    return (
        np.vstack(rows),
        np.asarray(lower, dtype=float),
        np.asarray(upper, dtype=float),
    )


def evaluate_exact_path(
    reference_line: ReferenceLine,
    trajectory: FrenetTrajectory,
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
    max_curvature: float,
    minimum_clearance_m: float,
    road_min_l: float,
    road_max_l: float,
) -> tuple[list[PathPoint], bool, bool, bool, bool, float, float]:
    path = frenet_trajectory_to_path(
        trajectory=trajectory,
        reference_line=reference_line,
    )
    curvature_ok = is_curvature_feasible(
        path=path,
        max_curvature=max_curvature,
    )
    clearance_ok = is_obstacle_clearance_feasible(
        path=path,
        vehicle_params=vehicle_params,
        obstacles=obstacles,
        minimum_clearance_m=minimum_clearance_m,
    )
    road_ok = is_road_boundary_feasible(
        trajectory=trajectory,
        vehicle_params=vehicle_params,
        min_l=road_min_l,
        max_l=road_max_l,
    )
    exact_ok = is_physical_path_feasible(
        trajectory=trajectory,
        path=path,
        vehicle_params=vehicle_params,
        obstacles=obstacles,
        max_curvature=max_curvature,
        minimum_clearance_m=minimum_clearance_m,
        min_l=road_min_l,
        max_l=road_max_l,
    )

    if obstacles:
        min_clearance_m = min(
            obstacle_clearance(
                path_point=path_point,
                vehicle_params=vehicle_params,
                obstacle=obstacle,
            )
            for path_point in path
            for obstacle in obstacles
        )
    else:
        min_clearance_m = math.inf

    max_exact_curvature = max(abs(point.curvature) for point in path)
    return (
        path,
        exact_ok,
        curvature_ok,
        clearance_ok,
        road_ok,
        min_clearance_m,
        max_exact_curvature,
    )


def _canonical_objective(
    trajectory: FrenetTrajectory,
    resolution_m: float,
    lateral_reference_m: float,
    terminal_lateral_target_m: float,
    terminal_first_derivative_target: float,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
) -> float:
    weights = PathObjectiveWeights(
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=terminal_first_derivative_weight,
    )
    evaluation = evaluate_frenet_trajectory_objective(
        trajectory=trajectory,
        resolution_m=resolution_m,
        lateral_reference_m=lateral_reference_m,
        terminal_lateral_target_m=terminal_lateral_target_m,
        terminal_first_derivative_target=terminal_first_derivative_target,
        weights=weights,
    )
    return float(evaluation.total_cost)


def _solve_explicit_branch(
    reference_line: ReferenceLine,
    corridor: LateralCorridor,
    resolution_m: float,
    initial_l: float,
    initial_dl_ds: float,
    initial_d2l_ds2: float,
    lateral_reference_m: float,
    terminal_lateral_target_m: float,
    terminal_first_derivative_target: float,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
    max_curvature: float,
    minimum_clearance_m: float,
    road_min_l: float,
    road_max_l: float,
    branch_name: str | None,
    osqp_verbose: bool,
    osqp_eps_abs: float,
    osqp_eps_rel: float,
    sequential_max_iterations: int,
    clearance_numerical_buffer_m: float,
    trust_region_l_m: float,
    trust_region_dl: float,
    clearance_gradient_eps_l: float,
    clearance_gradient_eps_dl: float,
    heading_constraint_body_sample_count: int,
) -> OptimizationPathResult:
    s_values = corridor.s_values
    num_points = len(s_values)

    qp_objective = build_explicit_qp_objective(
        num_points=num_points,
        resolution_m=resolution_m,
        lateral_reference_m=lateral_reference_m,
        terminal_lateral_target_m=terminal_lateral_target_m,
        terminal_first_derivative_target=terminal_first_derivative_target,
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=terminal_first_derivative_weight,
    )

    base_a, base_lower, base_upper = build_explicit_qp_constraints(
        reference_line=reference_line,
        s_values=s_values,
        corridor_lower_bounds=corridor.lower_bounds,
        corridor_upper_bounds=corridor.upper_bounds,
        resolution_m=resolution_m,
        initial_l=initial_l,
        initial_dl_ds=initial_dl_ds,
        initial_d2l_ds2=initial_d2l_ds2,
        max_curvature=max_curvature,
    )

    if obstacles:
        if len(obstacles) != 1:
            raise ValueError(
                "Sequential Optimization Planner currently supports at most one static obstacle."
            )
        if branch_name is None:
            raise ValueError("Obstacle planning requires a branch name.")
        initial_a, initial_lower, initial_upper = build_initial_heading_constraints(
            reference_line=reference_line,
            s_values=s_values,
            vehicle_params=vehicle_params,
            obstacle=obstacles[0],
            minimum_clearance_m=minimum_clearance_m,
            branch_name=branch_name,
            body_sample_count=heading_constraint_body_sample_count,
        )
    else:
        initial_a = np.zeros((0, 3 * num_points), dtype=float)
        initial_lower = np.zeros(0, dtype=float)
        initial_upper = np.zeros(0, dtype=float)

    initial_solution, _ = solve_qp(
        p_matrix=qp_objective.p,
        q_vector=qp_objective.q,
        a_matrix=np.vstack([base_a, initial_a]),
        lower_bounds=np.concatenate([base_lower, initial_lower]),
        upper_bounds=np.concatenate([base_upper, initial_upper]),
        verbose=osqp_verbose,
        eps_abs=osqp_eps_abs,
        eps_rel=osqp_eps_rel,
    )

    nominal_l, nominal_dl, _, nominal_trajectory = decode_explicit_solution(
        s_values=s_values,
        solution=initial_solution,
    )
    (
        nominal_path,
        exact_ok,
        curvature_ok,
        clearance_ok,
        road_ok,
        min_clearance_m,
        max_exact_curvature,
    ) = evaluate_exact_path(
        reference_line=reference_line,
        trajectory=nominal_trajectory,
        vehicle_params=vehicle_params,
        obstacles=obstacles,
        max_curvature=max_curvature,
        minimum_clearance_m=minimum_clearance_m,
        road_min_l=road_min_l,
        road_max_l=road_max_l,
    )

    if exact_ok:
        objective_value = _canonical_objective(
            trajectory=nominal_trajectory,
            resolution_m=resolution_m,
            lateral_reference_m=lateral_reference_m,
            terminal_lateral_target_m=terminal_lateral_target_m,
            terminal_first_derivative_target=terminal_first_derivative_target,
            lateral_weight=lateral_weight,
            first_derivative_weight=first_derivative_weight,
            second_derivative_weight=second_derivative_weight,
            third_derivative_weight=third_derivative_weight,
            terminal_lateral_weight=terminal_lateral_weight,
            terminal_first_derivative_weight=terminal_first_derivative_weight,
        )
        return OptimizationPathResult(
            lateral_solution=nominal_l,
            trajectory=nominal_trajectory,
            path=nominal_path,
            objective_value=objective_value,
            iterations=0,
            branch_name=branch_name,
            min_clearance_m=min_clearance_m,
            max_curvature=max_exact_curvature,
        )

    if not obstacles:
        raise RuntimeError("Explicit QP result failed exact physical validation.")
    if not curvature_ok:
        raise RuntimeError("Nominal explicit QP path failed exact curvature validation.")
    if not road_ok:
        raise RuntimeError("Nominal explicit QP path failed exact road-boundary validation.")
    if clearance_ok:
        raise RuntimeError("Nominal path failed exact validation for an unexpected reason.")
    if sequential_max_iterations <= 0:
        raise ValueError("sequential_max_iterations must be positive.")

    final_trajectory = None
    final_path = None
    final_lateral = None
    iteration_count = 0

    for iteration in range(1, sequential_max_iterations + 1):
        linear_a, linear_lower, linear_upper = build_linearized_clearance_constraints(
            reference_line=reference_line,
            s_values=s_values,
            vehicle_params=vehicle_params,
            obstacle=obstacles[0],
            minimum_clearance_m=minimum_clearance_m,
            nominal_l=nominal_l,
            nominal_dl=nominal_dl,
            trust_l_m=trust_region_l_m,
            trust_dl=trust_region_dl,
            gradient_eps_l=clearance_gradient_eps_l,
            gradient_eps_dl=clearance_gradient_eps_dl,
            clearance_numerical_buffer_m=clearance_numerical_buffer_m,
        )
        new_solution, _ = solve_qp(
            p_matrix=qp_objective.p,
            q_vector=qp_objective.q,
            a_matrix=np.vstack([base_a, linear_a]),
            lower_bounds=np.concatenate([base_lower, linear_lower]),
            upper_bounds=np.concatenate([base_upper, linear_upper]),
            verbose=osqp_verbose,
            eps_abs=osqp_eps_abs,
            eps_rel=osqp_eps_rel,
        )
        new_l, new_dl, _, new_trajectory = decode_explicit_solution(
            s_values=s_values,
            solution=new_solution,
        )
        (
            new_path,
            exact_ok,
            curvature_ok,
            _,
            road_ok,
            min_clearance_m,
            max_exact_curvature,
        ) = evaluate_exact_path(
            reference_line=reference_line,
            trajectory=new_trajectory,
            vehicle_params=vehicle_params,
            obstacles=obstacles,
            max_curvature=max_curvature,
            minimum_clearance_m=minimum_clearance_m,
            road_min_l=road_min_l,
            road_max_l=road_max_l,
        )

        iteration_count = iteration
        if not curvature_ok:
            raise RuntimeError(
                "Sequential refinement produced an exact-curvature violation."
            )
        if not road_ok:
            raise RuntimeError(
                "Sequential refinement produced a road-boundary violation."
            )

        nominal_l = new_l
        nominal_dl = new_dl
        nominal_trajectory = new_trajectory

        if exact_ok:
            final_lateral = new_l
            final_trajectory = new_trajectory
            final_path = new_path
            break

    if final_trajectory is None or final_path is None or final_lateral is None:
        raise RuntimeError(
            "Maximum sequential clearance iterations reached without an exact-feasible path."
        )

    objective_value = _canonical_objective(
        trajectory=final_trajectory,
        resolution_m=resolution_m,
        lateral_reference_m=lateral_reference_m,
        terminal_lateral_target_m=terminal_lateral_target_m,
        terminal_first_derivative_target=terminal_first_derivative_target,
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=terminal_first_derivative_weight,
    )
    return OptimizationPathResult(
        lateral_solution=final_lateral,
        trajectory=final_trajectory,
        path=final_path,
        objective_value=objective_value,
        iterations=iteration_count,
        branch_name=branch_name,
        min_clearance_m=min_clearance_m,
        max_curvature=max_exact_curvature,
    )


def plan_matched_multi_corridor_path(
    reference_line: ReferenceLine,
    corridors: list[LateralCorridor],
    resolution_m: float,
    initial_l: float,
    initial_dl_ds: float,
    initial_d2l_ds2: float,
    lateral_reference_m: float,
    terminal_lateral_target_m: float,
    terminal_first_derivative_target: float,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
    max_second_derivative_abs: float,
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
    max_curvature: float,
    minimum_clearance_m: float,
    road_min_l: float,
    road_max_l: float,
    osqp_verbose: bool = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_verbose,
    osqp_eps_abs: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_abs,
    osqp_eps_rel: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_rel,
    sequential_max_iterations: int = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.sequential_max_iterations,
    clearance_numerical_buffer_m: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.clearance_numerical_buffer_m,
    trust_region_l_m: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.trust_region_l_m,
    trust_region_dl: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.trust_region_dl,
    clearance_gradient_eps_l: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.clearance_gradient_eps_l,
    clearance_gradient_eps_dl: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.clearance_gradient_eps_dl,
    heading_constraint_body_sample_count: int = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.heading_constraint_body_sample_count,
) -> OptimizationPathResult:
    """
    Production explicit-state sequential QP planner.

    max_second_derivative_abs is retained temporarily for API compatibility.
    The new planner uses vehicle-derived max_curvature instead of the old D2(l) proxy.
    """
    _ = max_second_derivative_abs

    if len(corridors) == 0:
        raise ValueError("At least one corridor is required.")
    if len(obstacles) > 1:
        raise ValueError(
            "Optimization Planner currently supports at most one static obstacle."
        )

    reference_s_values = corridors[0].s_values
    feasible_results: list[OptimizationPathResult] = []
    failure_reasons: list[str] = []

    for corridor_index, corridor in enumerate(corridors):
        if (
            corridor.s_values.shape != reference_s_values.shape
            or not np.allclose(corridor.s_values, reference_s_values)
        ):
            raise ValueError("All corridors must share the same s_values.")
        if not is_corridor_feasible(corridor):
            failure_reasons.append(f"corridor {corridor_index}: infeasible")
            continue

        branch_names: tuple[str | None, ...] = (
            ("negative", "positive") if obstacles else (None,)
        )

        for branch_name in branch_names:
            try:
                result = _solve_explicit_branch(
                    reference_line=reference_line,
                    corridor=corridor,
                    resolution_m=resolution_m,
                    initial_l=initial_l,
                    initial_dl_ds=initial_dl_ds,
                    initial_d2l_ds2=initial_d2l_ds2,
                    lateral_reference_m=lateral_reference_m,
                    terminal_lateral_target_m=terminal_lateral_target_m,
                    terminal_first_derivative_target=terminal_first_derivative_target,
                    lateral_weight=lateral_weight,
                    first_derivative_weight=first_derivative_weight,
                    second_derivative_weight=second_derivative_weight,
                    third_derivative_weight=third_derivative_weight,
                    terminal_lateral_weight=terminal_lateral_weight,
                    terminal_first_derivative_weight=terminal_first_derivative_weight,
                    vehicle_params=vehicle_params,
                    obstacles=obstacles,
                    max_curvature=max_curvature,
                    minimum_clearance_m=minimum_clearance_m,
                    road_min_l=road_min_l,
                    road_max_l=road_max_l,
                    branch_name=branch_name,
                    osqp_verbose=osqp_verbose,
                    osqp_eps_abs=osqp_eps_abs,
                    osqp_eps_rel=osqp_eps_rel,
                    sequential_max_iterations=sequential_max_iterations,
                    clearance_numerical_buffer_m=clearance_numerical_buffer_m,
                    trust_region_l_m=trust_region_l_m,
                    trust_region_dl=trust_region_dl,
                    clearance_gradient_eps_l=clearance_gradient_eps_l,
                    clearance_gradient_eps_dl=clearance_gradient_eps_dl,
                    heading_constraint_body_sample_count=heading_constraint_body_sample_count,
                )
            except RuntimeError as error:
                failure_reasons.append(
                    f"corridor {corridor_index}, branch {branch_name}: {error}"
                )
                continue
            feasible_results.append(result)

    if not feasible_results:
        details = "; ".join(failure_reasons) if failure_reasons else "no candidate generated"
        raise RuntimeError(
            "No physically feasible optimization path found: " + details
        )

    return min(feasible_results, key=lambda result: result.objective_value)

# -----------------------------------------------------------------------------
# Legacy public API compatibility
# -----------------------------------------------------------------------------
# These wrappers keep older diagnostic scripts importable while the production
# benchmark path uses plan_matched_multi_corridor_path above.


def solve_matched_optimized_path(
    reference_line: ReferenceLine,
    s_values: np.ndarray,
    corridor_lower_bounds: np.ndarray,
    corridor_upper_bounds: np.ndarray,
    resolution_m: float,
    initial_l: float,
    initial_dl_ds: float,
    initial_d2l_ds2: float,
    lateral_reference_m: float,
    terminal_lateral_target_m: float,
    terminal_first_derivative_target: float,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
    max_second_derivative_abs: float,
    osqp_verbose: bool = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_verbose,
    osqp_eps_abs: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_abs,
    osqp_eps_rel: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_rel,
) -> OptimizationPathResult:
    """Compatibility one-shot explicit QP without obstacle refinement."""

    qp_objective = build_explicit_qp_objective(
        num_points=len(s_values),
        resolution_m=resolution_m,
        lateral_reference_m=lateral_reference_m,
        terminal_lateral_target_m=terminal_lateral_target_m,
        terminal_first_derivative_target=terminal_first_derivative_target,
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=terminal_first_derivative_weight,
    )
    a_matrix, lower_bounds, upper_bounds = build_explicit_qp_constraints(
        reference_line=reference_line,
        s_values=s_values,
        corridor_lower_bounds=corridor_lower_bounds,
        corridor_upper_bounds=corridor_upper_bounds,
        resolution_m=resolution_m,
        initial_l=initial_l,
        initial_dl_ds=initial_dl_ds,
        initial_d2l_ds2=initial_d2l_ds2,
        max_curvature=max_second_derivative_abs,
    )
    solution, _ = solve_qp(
        p_matrix=qp_objective.p,
        q_vector=qp_objective.q,
        a_matrix=a_matrix,
        lower_bounds=lower_bounds,
        upper_bounds=upper_bounds,
        verbose=osqp_verbose,
        eps_abs=osqp_eps_abs,
        eps_rel=osqp_eps_rel,
    )
    lateral, _, _, trajectory = decode_explicit_solution(
        s_values=s_values,
        solution=solution,
    )
    path = frenet_trajectory_to_path(
        trajectory=trajectory,
        reference_line=reference_line,
    )
    objective_value = _canonical_objective(
        trajectory=trajectory,
        resolution_m=resolution_m,
        lateral_reference_m=lateral_reference_m,
        terminal_lateral_target_m=terminal_lateral_target_m,
        terminal_first_derivative_target=terminal_first_derivative_target,
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=terminal_first_derivative_weight,
    )
    return OptimizationPathResult(
        lateral_solution=lateral,
        trajectory=trajectory,
        path=path,
        objective_value=objective_value,
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
    lateral_reference_m: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.lateral_reference_m,
    lateral_weight: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.lateral_weight,
    first_derivative_weight: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.first_derivative_weight,
    second_derivative_weight: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.second_derivative_weight,
    third_derivative_weight: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.third_derivative_weight,
    osqp_verbose: bool = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_verbose,
    osqp_eps_abs: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_abs,
    osqp_eps_rel: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_rel,
) -> OptimizationPathResult:
    return solve_matched_optimized_path(
        reference_line=reference_line,
        s_values=s_values,
        corridor_lower_bounds=corridor_lower_bounds,
        corridor_upper_bounds=corridor_upper_bounds,
        resolution_m=resolution_m,
        initial_l=initial_l,
        initial_dl_ds=initial_dl_ds,
        initial_d2l_ds2=initial_d2l_ds2,
        lateral_reference_m=lateral_reference_m,
        terminal_lateral_target_m=lateral_reference_m,
        terminal_first_derivative_target=0.0,
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=0.0,
        terminal_first_derivative_weight=0.0,
        max_second_derivative_abs=(
            DEFAULT_OPTIMIZATION_PLANNER_CONFIG.max_second_derivative_abs
        ),
        osqp_verbose=osqp_verbose,
        osqp_eps_abs=osqp_eps_abs,
        osqp_eps_rel=osqp_eps_rel,
    )


def validate_optimization_result_physical(
    result: OptimizationPathResult,
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
    max_curvature: float,
    minimum_clearance_m: float,
    road_min_l: float,
    road_max_l: float,
) -> OptimizationPathResult:
    if not is_physical_path_feasible(
        trajectory=result.trajectory,
        path=result.path,
        vehicle_params=vehicle_params,
        obstacles=obstacles,
        max_curvature=max_curvature,
        minimum_clearance_m=minimum_clearance_m,
        min_l=road_min_l,
        max_l=road_max_l,
    ):
        raise RuntimeError(
            "Optimization result failed exact physical validation."
        )
    return result


def plan_matched_optimized_path(
    reference_line: ReferenceLine,
    s_values: np.ndarray,
    corridor_lower_bounds: np.ndarray,
    corridor_upper_bounds: np.ndarray,
    resolution_m: float,
    initial_l: float,
    initial_dl_ds: float,
    initial_d2l_ds2: float,
    lateral_reference_m: float,
    terminal_lateral_target_m: float,
    terminal_first_derivative_target: float,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
    max_second_derivative_abs: float,
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
    max_curvature: float,
    minimum_clearance_m: float,
    road_min_l: float,
    road_max_l: float,
    osqp_verbose: bool = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_verbose,
    osqp_eps_abs: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_abs,
    osqp_eps_rel: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_rel,
) -> OptimizationPathResult:
    corridor = LateralCorridor(
        s_values=np.asarray(s_values, dtype=float),
        lower_bounds=np.asarray(corridor_lower_bounds, dtype=float),
        upper_bounds=np.asarray(corridor_upper_bounds, dtype=float),
    )
    return plan_matched_multi_corridor_path(
        reference_line=reference_line,
        corridors=[corridor],
        resolution_m=resolution_m,
        initial_l=initial_l,
        initial_dl_ds=initial_dl_ds,
        initial_d2l_ds2=initial_d2l_ds2,
        lateral_reference_m=lateral_reference_m,
        terminal_lateral_target_m=terminal_lateral_target_m,
        terminal_first_derivative_target=terminal_first_derivative_target,
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=terminal_first_derivative_weight,
        max_second_derivative_abs=max_second_derivative_abs,
        vehicle_params=vehicle_params,
        obstacles=obstacles,
        max_curvature=max_curvature,
        minimum_clearance_m=minimum_clearance_m,
        road_min_l=road_min_l,
        road_max_l=road_max_l,
        osqp_verbose=osqp_verbose,
        osqp_eps_abs=osqp_eps_abs,
        osqp_eps_rel=osqp_eps_rel,
    )


def solve_multi_corridor_path(
    reference_line: ReferenceLine,
    corridors: list[LateralCorridor],
    resolution_m: float,
    initial_l: float,
    initial_dl_ds: float,
    initial_d2l_ds2: float,
    lateral_reference_m: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.lateral_reference_m,
    lateral_weight: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.lateral_weight,
    first_derivative_weight: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.first_derivative_weight,
    second_derivative_weight: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.second_derivative_weight,
    third_derivative_weight: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.third_derivative_weight,
    osqp_verbose: bool = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_verbose,
    osqp_eps_abs: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_abs,
    osqp_eps_rel: float = DEFAULT_OPTIMIZATION_PLANNER_CONFIG.osqp_eps_rel,
) -> OptimizationPathResult:
    results = []
    for corridor in corridors:
        if not is_corridor_feasible(corridor):
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
                osqp_verbose=osqp_verbose,
                osqp_eps_abs=osqp_eps_abs,
                osqp_eps_rel=osqp_eps_rel,
            )
        except RuntimeError:
            continue
        results.append(result)
    if not results:
        raise RuntimeError("No feasible optimization corridor found.")
    return min(results, key=lambda result: result.objective_value)
