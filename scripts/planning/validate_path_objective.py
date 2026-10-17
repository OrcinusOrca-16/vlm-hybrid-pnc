import math

import numpy as np

from src.planning.path_objective import (
    build_difference_matrices,
    build_path_objective_qp,
    evaluate_path_objective,
    evaluate_path_objective_terms,
    weighted_path_objective,
)


def validate_difference_matrices() -> None:
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

    np.testing.assert_allclose(
        d1,
        expected_d1,
    )

    np.testing.assert_allclose(
        d2,
        expected_d2,
    )

    np.testing.assert_allclose(
        d3,
        expected_d3,
    )

    print(
        "Shared difference matrices passed."
    )


def validate_objective_terms() -> None:
    lateral_values = np.array([
        0.0,
        1.0,
        2.0,
        3.0,
    ])

    terms = evaluate_path_objective_terms(
        lateral_values=lateral_values,
        resolution_m=1.0,
        lateral_reference_m=0.0,
        terminal_lateral_target_m=0.0,
        terminal_first_derivative_target=0.0,
    )

    assert math.isclose(
        terms.lateral_deviation,
        3.5,
        abs_tol=1e-12,
    )

    assert math.isclose(
        terms.first_derivative,
        1.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        terms.second_derivative,
        0.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        terms.third_derivative,
        0.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        terms.terminal_lateral,
        9.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        terms.terminal_first_derivative,
        1.0,
        abs_tol=1e-12,
    )

    print(
        "Matched objective terms passed."
    )

    print(
        f"  J_l={terms.lateral_deviation:.4f}"
    )
    print(
        f"  J_dl={terms.first_derivative:.4f}"
    )
    print(
        f"  J_d2l={terms.second_derivative:.4f}"
    )
    print(
        f"  J_d3l={terms.third_derivative:.4f}"
    )
    print(
        f"  J_terminal_l={terms.terminal_lateral:.4f}"
    )
    print(
        "  J_terminal_dl="
        f"{terms.terminal_first_derivative:.4f}"
    )


def validate_weighted_objective() -> None:
    lateral_values = np.array([
        0.0,
        1.0,
        2.0,
        3.0,
    ])

    terms = evaluate_path_objective_terms(
        lateral_values=lateral_values,
        resolution_m=1.0,
        lateral_reference_m=0.0,
        terminal_lateral_target_m=0.0,
        terminal_first_derivative_target=0.0,
    )

    total_cost = weighted_path_objective(
        terms=terms,
        lateral_weight=2.0,
        first_derivative_weight=3.0,
        second_derivative_weight=4.0,
        third_derivative_weight=5.0,
        terminal_lateral_weight=6.0,
        terminal_first_derivative_weight=7.0,
    )

    assert math.isclose(
        total_cost,
        71.0,
        abs_tol=1e-12,
    )

    print(
        "Weighted matched objective passed."
    )

    print(
        f"  total_cost={total_cost:.4f}"
    )


def validate_qp_equivalence() -> None:
    lateral_values = np.array([
        0.2,
        -0.1,
        0.4,
        0.8,
        0.3,
    ])

    resolution_m = 0.5

    evaluation = evaluate_path_objective(
        lateral_values=lateral_values,
        resolution_m=resolution_m,
        lateral_reference_m=0.25,
        terminal_lateral_target_m=0.1,
        terminal_first_derivative_target=-0.05,
        lateral_weight=2.0,
        first_derivative_weight=3.0,
        second_derivative_weight=4.0,
        third_derivative_weight=5.0,
        terminal_lateral_weight=6.0,
        terminal_first_derivative_weight=7.0,
    )

    qp = build_path_objective_qp(
        num_points=len(lateral_values),
        resolution_m=resolution_m,
        lateral_reference_m=0.25,
        terminal_lateral_target_m=0.1,
        terminal_first_derivative_target=-0.05,
        lateral_weight=2.0,
        first_derivative_weight=3.0,
        second_derivative_weight=4.0,
        third_derivative_weight=5.0,
        terminal_lateral_weight=6.0,
        terminal_first_derivative_weight=7.0,
    )

    qp_without_constant = float(
        0.5
        * lateral_values
        @ qp.p
        @ lateral_values
        + qp.q
        @ lateral_values
    )

    qp_full_cost = (
        qp_without_constant
        + qp.constant
    )

    assert math.isclose(
        evaluation.total_cost,
        qp_full_cost,
        rel_tol=1e-12,
        abs_tol=1e-12,
    )

    np.testing.assert_allclose(
        qp.p,
        qp.p.T,
        atol=1e-12,
    )

    print(
        "Direct objective / QP equivalence passed."
    )

    print(
        "  canonical_cost="
        f"{evaluation.total_cost:.6f}"
    )

    print(
        "  qp_without_constant="
        f"{qp_without_constant:.6f}"
    )

    print(
        f"  constant={qp.constant:.6f}"
    )

    print(
        f"  qp_full_cost={qp_full_cost:.6f}"
    )


def main() -> None:
    validate_difference_matrices()
    validate_objective_terms()
    validate_weighted_objective()
    validate_qp_equivalence()

    print()
    print(
        "All matched path objective cases passed."
    )


if __name__ == "__main__":
    main()