from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class PathObjectiveTerms:
    lateral_deviation: float
    first_derivative: float
    second_derivative: float
    third_derivative: float
    terminal_lateral: float
    terminal_first_derivative: float


@dataclass(frozen=True)
class PathObjectiveEvaluation:
    terms: PathObjectiveTerms
    total_cost: float


@dataclass(frozen=True)
class PathObjectiveQP:
    p: np.ndarray
    q: np.ndarray
    constant: float


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

    resolution_squared = (
        resolution_m ** 2
    )

    d2 = np.zeros(
        (num_points - 2, num_points)
    )

    for i in range(num_points - 2):
        d2[i, i] = 1.0 / resolution_squared
        d2[i, i + 1] = -2.0 / resolution_squared
        d2[i, i + 2] = 1.0 / resolution_squared

    resolution_cubed = (
        resolution_m ** 3
    )

    d3 = np.zeros(
        (num_points - 3, num_points)
    )

    for i in range(num_points - 3):
        d3[i, i] = -1.0 / resolution_cubed
        d3[i, i + 1] = 3.0 / resolution_cubed
        d3[i, i + 2] = -3.0 / resolution_cubed
        d3[i, i + 3] = 1.0 / resolution_cubed

    return (
        d1,
        d2,
        d3,
    )


def validate_objective_weights(
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
) -> None:
    weights = (
        lateral_weight,
        first_derivative_weight,
        second_derivative_weight,
        third_derivative_weight,
        terminal_lateral_weight,
        terminal_first_derivative_weight,
    )

    if any(
        weight < 0.0
        for weight in weights
    ):
        raise ValueError(
            "Objective weights must not be negative."
        )


def evaluate_path_objective_terms(
    lateral_values: np.ndarray,
    resolution_m: float,
    lateral_reference_m: float,
    terminal_lateral_target_m: float,
    terminal_first_derivative_target: float,
) -> PathObjectiveTerms:
    lateral_values = np.asarray(
        lateral_values,
        dtype=float,
    )

    if lateral_values.ndim != 1:
        raise ValueError(
            "lateral_values must be one-dimensional."
        )

    num_points = lateral_values.size

    d1, d2, d3 = build_difference_matrices(
        num_points=num_points,
        resolution_m=resolution_m,
    )

    first_derivative = (
        d1 @ lateral_values
    )

    second_derivative = (
        d2 @ lateral_values
    )

    third_derivative = (
        d3 @ lateral_values
    )

    lateral_error = (
        lateral_values
        - lateral_reference_m
    )

    terminal_lateral_error = (
        lateral_values[-1]
        - terminal_lateral_target_m
    )

    terminal_first_derivative_error = (
        first_derivative[-1]
        - terminal_first_derivative_target
    )

    return PathObjectiveTerms(
        lateral_deviation=float(
            np.mean(
                lateral_error ** 2
            )
        ),
        first_derivative=float(
            np.mean(
                first_derivative ** 2
            )
        ),
        second_derivative=float(
            np.mean(
                second_derivative ** 2
            )
        ),
        third_derivative=float(
            np.mean(
                third_derivative ** 2
            )
        ),
        terminal_lateral=float(
            terminal_lateral_error ** 2
        ),
        terminal_first_derivative=float(
            terminal_first_derivative_error ** 2
        ),
    )


def weighted_path_objective(
    terms: PathObjectiveTerms,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
) -> float:
    validate_objective_weights(
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=(
            terminal_first_derivative_weight
        ),
    )

    return float(
        lateral_weight
        * terms.lateral_deviation
        + first_derivative_weight
        * terms.first_derivative
        + second_derivative_weight
        * terms.second_derivative
        + third_derivative_weight
        * terms.third_derivative
        + terminal_lateral_weight
        * terms.terminal_lateral
        + terminal_first_derivative_weight
        * terms.terminal_first_derivative
    )


def evaluate_path_objective(
    lateral_values: np.ndarray,
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
) -> PathObjectiveEvaluation:
    terms = evaluate_path_objective_terms(
        lateral_values=lateral_values,
        resolution_m=resolution_m,
        lateral_reference_m=lateral_reference_m,
        terminal_lateral_target_m=terminal_lateral_target_m,
        terminal_first_derivative_target=(
            terminal_first_derivative_target
        ),
    )

    total_cost = weighted_path_objective(
        terms=terms,
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=(
            terminal_first_derivative_weight
        ),
    )

    return PathObjectiveEvaluation(
        terms=terms,
        total_cost=total_cost,
    )


def build_path_objective_qp(
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
    validate_objective_weights(
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=(
            terminal_first_derivative_weight
        ),
    )

    d1, d2, d3 = build_difference_matrices(
        num_points=num_points,
        resolution_m=resolution_m,
    )

    identity = np.eye(
        num_points
    )

    lateral_reference = np.full(
        num_points,
        lateral_reference_m,
        dtype=float,
    )

    terminal_lateral_selector = np.zeros(
        num_points
    )
    terminal_lateral_selector[-1] = 1.0

    terminal_first_derivative_selector = (
        d1[-1]
    )

    quadratic = (
        lateral_weight
        / num_points
        * identity
        + first_derivative_weight
        / d1.shape[0]
        * (d1.T @ d1)
        + second_derivative_weight
        / d2.shape[0]
        * (d2.T @ d2)
        + third_derivative_weight
        / d3.shape[0]
        * (d3.T @ d3)
        + terminal_lateral_weight
        * np.outer(
            terminal_lateral_selector,
            terminal_lateral_selector,
        )
        + terminal_first_derivative_weight
        * np.outer(
            terminal_first_derivative_selector,
            terminal_first_derivative_selector,
        )
    )

    q = (
        -2.0
        * lateral_weight
        / num_points
        * lateral_reference
        - 2.0
        * terminal_lateral_weight
        * terminal_lateral_target_m
        * terminal_lateral_selector
        - 2.0
        * terminal_first_derivative_weight
        * terminal_first_derivative_target
        * terminal_first_derivative_selector
    )

    constant = (
        lateral_weight
        / num_points
        * float(
            lateral_reference
            @ lateral_reference
        )
        + terminal_lateral_weight
        * terminal_lateral_target_m ** 2
        + terminal_first_derivative_weight
        * terminal_first_derivative_target ** 2
    )

    p = (
        2.0
        * quadratic
    )

    return PathObjectiveQP(
        p=p,
        q=q,
        constant=float(
            constant
        ),
    )