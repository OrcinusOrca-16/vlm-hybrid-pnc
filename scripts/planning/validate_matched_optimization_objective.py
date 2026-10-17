import math

import numpy as np

from src.planning.optimization_planner import (
    build_matched_qp_objective,
)
from src.planning.path_objective import (
    evaluate_path_objective,
)


LATERAL_VALUES = np.array([
    0.2,
    -0.1,
    0.4,
    0.8,
    0.3,
])

RESOLUTION_M = 0.5

LATERAL_REFERENCE_M = 0.25
TERMINAL_LATERAL_TARGET_M = 0.1
TERMINAL_FIRST_DERIVATIVE_TARGET = -0.05


def evaluate_case(
    case_name: str,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
) -> None:
    evaluation = evaluate_path_objective(
        lateral_values=LATERAL_VALUES,
        resolution_m=RESOLUTION_M,
        lateral_reference_m=LATERAL_REFERENCE_M,
        terminal_lateral_target_m=(
            TERMINAL_LATERAL_TARGET_M
        ),
        terminal_first_derivative_target=(
            TERMINAL_FIRST_DERIVATIVE_TARGET
        ),
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=(
            terminal_first_derivative_weight
        ),
    )

    qp = build_matched_qp_objective(
        num_points=len(LATERAL_VALUES),
        resolution_m=RESOLUTION_M,
        lateral_reference_m=LATERAL_REFERENCE_M,
        terminal_lateral_target_m=(
            TERMINAL_LATERAL_TARGET_M
        ),
        terminal_first_derivative_target=(
            TERMINAL_FIRST_DERIVATIVE_TARGET
        ),
        lateral_weight=lateral_weight,
        first_derivative_weight=first_derivative_weight,
        second_derivative_weight=second_derivative_weight,
        third_derivative_weight=third_derivative_weight,
        terminal_lateral_weight=terminal_lateral_weight,
        terminal_first_derivative_weight=(
            terminal_first_derivative_weight
        ),
    )

    qp_without_constant = float(
        0.5
        * LATERAL_VALUES
        @ qp.p
        @ LATERAL_VALUES
        + qp.q
        @ LATERAL_VALUES
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

    eigenvalues = np.linalg.eigvalsh(
        qp.p
    )

    minimum_eigenvalue = float(
        np.min(eigenvalues)
    )

    assert minimum_eigenvalue >= -1e-10

    print(
        f"{case_name} passed:"
    )

    print(
        "  canonical_cost="
        f"{evaluation.total_cost:.6f}"
    )

    print(
        "  qp_full_cost="
        f"{qp_full_cost:.6f}"
    )

    print(
        "  min_eigenvalue="
        f"{minimum_eigenvalue:.6e}"
    )


def validate_individual_terms() -> None:
    cases = [
        (
            "lateral deviation",
            (1.0, 0.0, 0.0, 0.0, 0.0, 0.0),
        ),
        (
            "first derivative",
            (0.0, 1.0, 0.0, 0.0, 0.0, 0.0),
        ),
        (
            "second derivative",
            (0.0, 0.0, 1.0, 0.0, 0.0, 0.0),
        ),
        (
            "third derivative",
            (0.0, 0.0, 0.0, 1.0, 0.0, 0.0),
        ),
        (
            "terminal lateral",
            (0.0, 0.0, 0.0, 0.0, 1.0, 0.0),
        ),
        (
            "terminal first derivative",
            (0.0, 0.0, 0.0, 0.0, 0.0, 1.0),
        ),
    ]

    for case_name, weights in cases:
        evaluate_case(
            case_name=case_name,
            lateral_weight=weights[0],
            first_derivative_weight=weights[1],
            second_derivative_weight=weights[2],
            third_derivative_weight=weights[3],
            terminal_lateral_weight=weights[4],
            terminal_first_derivative_weight=weights[5],
        )

    print(
        "All individual matched objective terms passed."
    )


def validate_combined_objective() -> None:
    evaluate_case(
        case_name="combined matched objective",
        lateral_weight=2.0,
        first_derivative_weight=3.0,
        second_derivative_weight=4.0,
        third_derivative_weight=5.0,
        terminal_lateral_weight=6.0,
        terminal_first_derivative_weight=7.0,
    )

    print(
        "Combined matched objective passed."
    )


def main() -> None:
    validate_individual_terms()
    validate_combined_objective()

    print()
    print(
        "All matched Optimization objective cases passed."
    )


if __name__ == "__main__":
    main()