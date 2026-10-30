import numpy as np

from src.benchmark.tuning_scenario_suite import (
    build_tuning_lateral_recovery_scenario,
)
from src.config.planning import (
    DEFAULT_OPTIMIZATION_PLANNER_CONFIG,
)
from src.planning.frenet import (
    vehicle_state_to_frenet,
)
from src.planning.lateral_corridor import (
    build_base_lateral_corridor,
)
from src.planning.optimization_planner import (
    solve_matched_optimized_path,
    solve_explicit_matched_optimized_path,
)
from src.planning.path_objective import (
    evaluate_frenet_trajectory_objective,
)


def main() -> None:
    scenario = (
        build_tuning_lateral_recovery_scenario()
    )

    initial_state = (
        vehicle_state_to_frenet(
            vehicle_state=scenario.vehicle_state,
            reference_line=scenario.reference_line,
        )
    )

    corridor = (
        build_base_lateral_corridor(
            start_s=initial_state.s,
            reference_line=scenario.reference_line,
            vehicle_params=scenario.vehicle_params,
            road_min_l=scenario.road_min_l,
            road_max_l=scenario.road_max_l,
            planning_horizon_m=(
                scenario.planning_horizon_m
            ),
            resolution_m=scenario.resolution_m,
        )
    )

    cfg = (
        DEFAULT_OPTIMIZATION_PLANNER_CONFIG
    )

    common_arguments = dict(
        reference_line=scenario.reference_line,
        s_values=corridor.s_values,
        corridor_lower_bounds=(
            corridor.lower_bounds
        ),
        corridor_upper_bounds=(
            corridor.upper_bounds
        ),
        resolution_m=scenario.resolution_m,
        initial_l=initial_state.l,
        initial_dl_ds=initial_state.dl_ds,
        initial_d2l_ds2=initial_state.d2l_ds2,
        lateral_reference_m=(
            scenario.lateral_reference_m
        ),
        terminal_lateral_target_m=(
            scenario.terminal_lateral_target_m
        ),
        terminal_first_derivative_target=(
            scenario.terminal_first_derivative_target
        ),
        lateral_weight=scenario.lateral_weight,
        first_derivative_weight=(
            scenario.first_derivative_weight
        ),
        second_derivative_weight=(
            scenario.second_derivative_weight
        ),
        third_derivative_weight=(
            scenario.third_derivative_weight
        ),
        terminal_lateral_weight=(
            scenario.terminal_lateral_weight
        ),
        terminal_first_derivative_weight=(
            scenario.terminal_first_derivative_weight
        ),
        max_second_derivative_abs=(
            cfg.max_second_derivative_abs
        ),
    )

    old_result = (
        solve_matched_optimized_path(
            **common_arguments
        )
    )

    explicit_result = (
        solve_explicit_matched_optimized_path(
            **common_arguments
        )
    )

    weights = cfg.matched_objective_weights

    old_canonical = (
        evaluate_frenet_trajectory_objective(
            trajectory=old_result.trajectory,
            resolution_m=scenario.resolution_m,
            lateral_reference_m=(
                scenario.lateral_reference_m
            ),
            terminal_lateral_target_m=(
                scenario.terminal_lateral_target_m
            ),
            terminal_first_derivative_target=(
                scenario.terminal_first_derivative_target
            ),
            weights=weights,
        )
    )

    explicit_canonical = (
        evaluate_frenet_trajectory_objective(
            trajectory=explicit_result.trajectory,
            resolution_m=scenario.resolution_m,
            lateral_reference_m=(
                scenario.lateral_reference_m
            ),
            terminal_lateral_target_m=(
                scenario.terminal_lateral_target_m
            ),
            terminal_first_derivative_target=(
                scenario.terminal_first_derivative_target
            ),
            weights=weights,
        )
    )

    print(
        f"Scenario: {scenario.name}"
    )

    print(
        "\nInitial Frenet state:"
    )
    print(
        f"  l   = {initial_state.l:.6f}"
    )
    print(
        f"  dl  = {initial_state.dl_ds:.6f}"
    )
    print(
        f"  ddl = {initial_state.d2l_ds2:.6f}"
    )

    print(
        "\nCanonical objective:"
    )
    print(
        f"  old      = "
        f"{old_canonical.total_cost:.9f}"
    )
    print(
        f"  explicit = "
        f"{explicit_canonical.total_cost:.9f}"
    )

    print(
        "\nFirst 6 trajectory points:"
    )

    print(
        "  i      s       old_l     new_l"
        "     new_dl    new_ddl"
    )

    for i in range(
        min(
            6,
            len(
                explicit_result.trajectory.points
            ),
        )
    ):
        old_point = (
            old_result.trajectory.points[i]
        )

        new_point = (
            explicit_result.trajectory.points[i]
        )

        print(
            f"  {i:<2d} "
            f"{new_point.s:7.3f} "
            f"{old_point.l:10.6f} "
            f"{new_point.l:10.6f} "
            f"{new_point.dl_ds:10.6f} "
            f"{new_point.d2l_ds2:10.6f}"
        )

    # Verify the two piecewise-jerk state equations.
    points = (
        explicit_result.trajectory.points
    )

    max_dl_residual = 0.0
    max_l_residual = 0.0

    ds = scenario.resolution_m

    for i in range(
        len(points) - 1
    ):
        p0 = points[i]
        p1 = points[i + 1]

        dl_residual = (
            p1.dl_ds
            - p0.dl_ds
            - ds
            / 2.0
            * (
                p0.d2l_ds2
                + p1.d2l_ds2
            )
        )

        l_residual = (
            p1.l
            - p0.l
            - ds * p0.dl_ds
            - ds ** 2
            / 3.0
            * p0.d2l_ds2
            - ds ** 2
            / 6.0
            * p1.d2l_ds2
        )

        max_dl_residual = max(
            max_dl_residual,
            abs(dl_residual),
        )

        max_l_residual = max(
            max_l_residual,
            abs(l_residual),
        )

    print(
        "\nPiecewise-jerk dynamics residual:"
    )

    print(
        f"  max dl residual = "
        f"{max_dl_residual:.3e}"
    )

    print(
        f"  max l residual  = "
        f"{max_l_residual:.3e}"
    )


if __name__ == "__main__":
    main()