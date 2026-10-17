import math

import numpy as np

from src.planning.lateral_corridor import (
    LateralCorridor,
)
from src.planning.optimization_planner import (
    solve_matched_multi_corridor_path,
    solve_matched_optimized_path,
)
from src.planning.reference_line import (
    ReferenceLine,
)


def main() -> None:
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

    negative_lower = np.full(
        len(s_values),
        -2.0,
    )
    negative_upper = np.full(
        len(s_values),
        2.0,
    )

    positive_lower = np.full(
        len(s_values),
        -2.0,
    )
    positive_upper = np.full(
        len(s_values),
        2.0,
    )

    # Force two different middle branches.
    negative_upper[8:13] = -0.5
    positive_lower[8:13] = 1.0

    negative_corridor = LateralCorridor(
        s_values=s_values,
        lower_bounds=negative_lower,
        upper_bounds=negative_upper,
    )

    positive_corridor = LateralCorridor(
        s_values=s_values,
        lower_bounds=positive_lower,
        upper_bounds=positive_upper,
    )

    common_arguments = dict(
        reference_line=reference_line,
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

    negative_result = solve_matched_optimized_path(
        s_values=s_values,
        corridor_lower_bounds=negative_lower,
        corridor_upper_bounds=negative_upper,
        **common_arguments,
    )

    positive_result = solve_matched_optimized_path(
        s_values=s_values,
        corridor_lower_bounds=positive_lower,
        corridor_upper_bounds=positive_upper,
        **common_arguments,
    )

    selected_result = solve_matched_multi_corridor_path(
        corridors=[
            negative_corridor,
            positive_corridor,
        ],
        **common_arguments,
    )

    expected_cost = min(
        negative_result.objective_value,
        positive_result.objective_value,
    )

    assert math.isclose(
        selected_result.objective_value,
        expected_cost,
        rel_tol=1e-8,
        abs_tol=1e-8,
    )

    assert (
        negative_result.objective_value
        < positive_result.objective_value
    )

    assert math.isclose(
        selected_result.objective_value,
        negative_result.objective_value,
        rel_tol=1e-8,
        abs_tol=1e-8,
    )

    print(
        "Matched multi-corridor selection passed."
    )

    print(
        "  negative_cost="
        f"{negative_result.objective_value:.6f}"
    )

    print(
        "  positive_cost="
        f"{positive_result.objective_value:.6f}"
    )

    print(
        "  selected_cost="
        f"{selected_result.objective_value:.6f}"
    )

    print(
        "  selected_branch=negative"
    )


if __name__ == "__main__":
    main()