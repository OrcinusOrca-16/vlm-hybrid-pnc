import numpy as np

from src.planning.optimization_planner import (
    build_difference_matrices,
    build_matched_qp_constraints,
    solve_matched_optimized_path,
)
from src.planning.reference_line import (
    ReferenceLine,
)


def validate_constraint_construction() -> None:
    num_points = 8
    resolution_m = 0.5
    proxy_limit = 0.1

    lower_corridor = np.full(
        num_points,
        -2.0,
    )

    upper_corridor = np.full(
        num_points,
        2.0,
    )

    (
        a_matrix,
        lower_bounds,
        upper_bounds,
    ) = build_matched_qp_constraints(
        corridor_lower_bounds=lower_corridor,
        corridor_upper_bounds=upper_corridor,
        resolution_m=resolution_m,
        initial_l=0.0,
        initial_dl_ds=0.0,
        initial_d2l_ds2=0.0,
        max_second_derivative_abs=proxy_limit,
    )

    _, d2, _ = build_difference_matrices(
        num_points=num_points,
        resolution_m=resolution_m,
    )

    proxy_row_count = d2.shape[0]

    np.testing.assert_allclose(
        a_matrix[-proxy_row_count:],
        d2,
    )

    np.testing.assert_allclose(
        lower_bounds[-proxy_row_count:],
        -proxy_limit,
    )

    np.testing.assert_allclose(
        upper_bounds[-proxy_row_count:],
        proxy_limit,
    )

    print(
        "Matched D2 constraint construction passed."
    )

    print(
        f"  D2_rows={proxy_row_count}"
    )

    print(
        f"  proxy_limit={proxy_limit:.4f}"
    )


def validate_solution_respects_proxy() -> None:
    reference_line = ReferenceLine([
        (0.0, 0.0),
        (20.0, 0.0),
    ])

    resolution_m = 0.5
    proxy_limit = 0.05

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

    result = solve_matched_optimized_path(
        reference_line=reference_line,
        s_values=s_values,
        corridor_lower_bounds=lower_bounds,
        corridor_upper_bounds=upper_bounds,
        resolution_m=resolution_m,
        initial_l=0.0,
        initial_dl_ds=0.0,
        initial_d2l_ds2=0.0,
        lateral_reference_m=0.0,
        terminal_lateral_target_m=1.0,
        terminal_first_derivative_target=0.0,
        lateral_weight=0.1,
        first_derivative_weight=0.1,
        second_derivative_weight=0.1,
        third_derivative_weight=0.1,
        terminal_lateral_weight=100.0,
        terminal_first_derivative_weight=1.0,
        max_second_derivative_abs=proxy_limit,
    )

    _, d2, _ = build_difference_matrices(
        num_points=len(
            result.lateral_solution
        ),
        resolution_m=resolution_m,
    )

    second_derivative = (
        d2 @ result.lateral_solution
    )

    maximum_abs_d2 = float(
        np.max(
            np.abs(
                second_derivative
            )
        )
    )

    assert maximum_abs_d2 <= (
        proxy_limit
        + 1e-5
    )

    print(
        "Matched D2 curvature-proxy solve passed."
    )

    print(
        f"  limit={proxy_limit:.6f}"
    )

    print(
        f"  max_abs_D2={maximum_abs_d2:.6f}"
    )

    print(
        f"  end_l={result.lateral_solution[-1]:.6f}"
    )


def main() -> None:
    validate_constraint_construction()
    validate_solution_respects_proxy()

    print()
    print(
        "All matched curvature-proxy cases passed."
    )


if __name__ == "__main__":
    main()