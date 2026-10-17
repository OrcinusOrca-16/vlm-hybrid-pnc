import math

import numpy as np

from src.planning.frenet import (
    frenet_trajectories_to_paths,
)
from src.planning.path_objective import (
    evaluate_path_objective,
)
from src.planning.reference_line import (
    ReferenceLine,
)
from src.planning.sampling_planner import (
    generate_returning_frenet_candidates,
)
from src.planning.soft_cost import (
    matched_total_cost,
    select_best_matched_candidate,
)


def validate_wrapper_equivalence() -> None:
    candidates = generate_returning_frenet_candidates(
        start_s=12.0,
        start_l=0.0,
        start_dl_ds=0.0,
        start_d2l_ds2=0.0,
        bypass_lateral_positions=(-1.0,),
        bypass_lengths=(10.0,),
        total_length=28.0,
        final_lateral_positions=(0.5,),
        final_dl_ds_values=(0.05,),
        final_d2l_ds2=0.0,
        resolution=0.5,
    )

    trajectory = candidates[0]

    wrapper_cost = matched_total_cost(
        trajectory=trajectory,
        resolution_m=0.5,
        lateral_reference_m=0.0,
        terminal_lateral_target_m=0.0,
        terminal_first_derivative_target=0.0,
        lateral_weight=1.0,
        first_derivative_weight=2.0,
        second_derivative_weight=3.0,
        third_derivative_weight=4.0,
        terminal_lateral_weight=5.0,
        terminal_first_derivative_weight=6.0,
    )

    lateral_values = np.array(
        [
            point.l
            for point in trajectory.points
        ],
        dtype=float,
    )

    direct_evaluation = evaluate_path_objective(
        lateral_values=lateral_values,
        resolution_m=0.5,
        lateral_reference_m=0.0,
        terminal_lateral_target_m=0.0,
        terminal_first_derivative_target=0.0,
        lateral_weight=1.0,
        first_derivative_weight=2.0,
        second_derivative_weight=3.0,
        third_derivative_weight=4.0,
        terminal_lateral_weight=5.0,
        terminal_first_derivative_weight=6.0,
    )

    assert math.isclose(
        wrapper_cost,
        direct_evaluation.total_cost,
        rel_tol=1e-12,
        abs_tol=1e-12,
    )

    print(
        "Sampling matched-cost wrapper passed."
    )

    print(
        f"  wrapper_cost={wrapper_cost:.6f}"
    )

    print(
        "  canonical_cost="
        f"{direct_evaluation.total_cost:.6f}"
    )


def validate_terminal_cost_changes_selection() -> None:
    reference_line = ReferenceLine([
        (0.0, 0.0),
        (20.0, 0.0),
        (40.0, 0.0),
        (50.0, 0.0),
    ])

    candidates = generate_returning_frenet_candidates(
        start_s=12.0,
        start_l=0.0,
        start_dl_ds=0.0,
        start_d2l_ds2=0.0,
        bypass_lateral_positions=(-1.0,),
        bypass_lengths=(10.0,),
        total_length=28.0,
        final_lateral_positions=(
            0.0,
            0.5,
        ),
        final_dl_ds_values=(0.0,),
        final_d2l_ds2=0.0,
        resolution=0.5,
    )

    paths = frenet_trajectories_to_paths(
        trajectories=candidates,
        reference_line=reference_line,
    )

    candidate_pairs = list(
        zip(
            candidates,
            paths,
        )
    )

    without_terminal_cost = (
        select_best_matched_candidate(
            feasible_candidates=candidate_pairs,
            resolution_m=0.5,
            lateral_reference_m=0.0,
            terminal_lateral_target_m=0.0,
            terminal_first_derivative_target=0.0,
            lateral_weight=1.0,
            first_derivative_weight=0.0,
            second_derivative_weight=0.0,
            third_derivative_weight=0.0,
            terminal_lateral_weight=0.0,
            terminal_first_derivative_weight=0.0,
        )
    )

    with_terminal_cost = (
        select_best_matched_candidate(
            feasible_candidates=candidate_pairs,
            resolution_m=0.5,
            lateral_reference_m=0.0,
            terminal_lateral_target_m=0.0,
            terminal_first_derivative_target=0.0,
            lateral_weight=1.0,
            first_derivative_weight=0.0,
            second_derivative_weight=0.0,
            third_derivative_weight=0.0,
            terminal_lateral_weight=1.0,
            terminal_first_derivative_weight=0.0,
        )
    )

    assert without_terminal_cost is not None
    assert with_terminal_cost is not None

    without_terminal_trajectory = (
        without_terminal_cost[0]
    )

    with_terminal_trajectory = (
        with_terminal_cost[0]
    )

    assert math.isclose(
        without_terminal_trajectory.points[-1].l,
        0.5,
        abs_tol=1e-9,
    )

    assert math.isclose(
        with_terminal_trajectory.points[-1].l,
        0.0,
        abs_tol=1e-9,
    )

    print(
        "Sampling terminal-cost selection passed."
    )

    print(
        "  terminal_weight=0: "
        f"end_l={without_terminal_trajectory.points[-1].l:.2f}"
    )

    print(
        "  terminal_weight=1: "
        f"end_l={with_terminal_trajectory.points[-1].l:.2f}"
    )


def main() -> None:
    validate_wrapper_equivalence()
    validate_terminal_cost_changes_selection()

    print()
    print(
        "All matched Sampling cost cases passed."
    )


if __name__ == "__main__":
    main()