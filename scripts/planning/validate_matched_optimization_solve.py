import math

import numpy as np

from src.planning.optimization_planner import (
    solve_matched_optimized_path,
)
from src.planning.path_objective import (
    evaluate_path_objective,
)
from src.planning.reference_line import (
    ReferenceLine,
)


def build_reference_line() -> ReferenceLine:
    return ReferenceLine([
        (0.0, 0.0),
        (20.0, 0.0),
        (40.0, 0.0),
    ])


def validate_matched_solve() -> None:
    reference_line = build_reference_line()

    resolution_m = 0.5

    s_values = np.arange(
        0.0,
        10.0 + resolution_m,
        resolution_m,
    )

    corridor_lower_bounds = np.full(
        len(s_values),
        -2.0,
    )

    corridor_upper_bounds = np.full(
        len(s_values),
        2.0,
    )

    result = solve_matched_optimized_path(
        reference_line=reference_line,
        s_values=s_values,
        corridor_lower_bounds=corridor_lower_bounds,
        corridor_upper_bounds=corridor_upper_bounds,
        resolution_m=resolution_m,
        initial_l=1.0,
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

    evaluation = evaluate_path_objective(
        lateral_values=result.lateral_solution,
        resolution_m=resolution_m,
        lateral_reference_m=0.0,
        terminal_lateral_target_m=0.0,
        terminal_first_derivative_target=0.0,
        lateral_weight=1.0,
        first_derivative_weight=1.0,
        second_derivative_weight=5.0,
        third_derivative_weight=20.0,
        terminal_lateral_weight=5.0,
        terminal_first_derivative_weight=5.0,
    )

    assert math.isclose(
        result.objective_value,
        evaluation.total_cost,
        rel_tol=1e-8,
        abs_tol=1e-8,
    )

    assert math.isclose(
        result.lateral_solution[0],
        1.0,
        abs_tol=1e-5,
    )

    assert len(result.trajectory.points) == len(
        s_values
    )

    assert len(result.path) == len(
        s_values
    )

    print(
        "Matched Optimization solve passed."
    )

    print(
        f"  start_l={result.lateral_solution[0]:.6f}"
    )

    print(
        f"  end_l={result.lateral_solution[-1]:.6f}"
    )

    print(
        "  solver/full objective="
        f"{result.objective_value:.6f}"
    )

    print(
        "  canonical objective="
        f"{evaluation.total_cost:.6f}"
    )


def validate_terminal_target_effect() -> None:
    reference_line = build_reference_line()

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

    positive_target = solve_matched_optimized_path(
        reference_line=reference_line,
        s_values=s_values,
        corridor_lower_bounds=lower_bounds,
        corridor_upper_bounds=upper_bounds,
        resolution_m=resolution_m,
        initial_l=0.0,
        initial_dl_ds=0.0,
        initial_d2l_ds2=0.0,
        lateral_reference_m=0.0,
        terminal_lateral_target_m=0.5,
        terminal_first_derivative_target=0.0,
        lateral_weight=0.1,
        first_derivative_weight=1.0,
        second_derivative_weight=5.0,
        third_derivative_weight=20.0,
        terminal_lateral_weight=20.0,
        terminal_first_derivative_weight=5.0,
        max_second_derivative_abs=1.0,
    )

    negative_target = solve_matched_optimized_path(
        reference_line=reference_line,
        s_values=s_values,
        corridor_lower_bounds=lower_bounds,
        corridor_upper_bounds=upper_bounds,
        resolution_m=resolution_m,
        initial_l=0.0,
        initial_dl_ds=0.0,
        initial_d2l_ds2=0.0,
        lateral_reference_m=0.0,
        terminal_lateral_target_m=-0.5,
        terminal_first_derivative_target=0.0,
        lateral_weight=0.1,
        first_derivative_weight=1.0,
        second_derivative_weight=5.0,
        third_derivative_weight=20.0,
        terminal_lateral_weight=20.0,
        terminal_first_derivative_weight=5.0,
        max_second_derivative_abs=1.0,
    )

    assert (
        positive_target.lateral_solution[-1]
        > 0.0
    )

    assert (
        negative_target.lateral_solution[-1]
        < 0.0
    )

    print(
        "Matched terminal target effect passed."
    )

    print(
        "  target=+0.50 -> "
        f"end_l={positive_target.lateral_solution[-1]:.6f}"
    )

    print(
        "  target=-0.50 -> "
        f"end_l={negative_target.lateral_solution[-1]:.6f}"
    )


def main() -> None:
    validate_matched_solve()
    validate_terminal_target_effect()

    print()
    print(
        "All matched Optimization solve cases passed."
    )


if __name__ == "__main__":
    main()