import math

from src.benchmark.canonical_scenario import (
    build_canonical_static_obstacle_scenario,
)
from src.benchmark.scenario_runner import (
    run_benchmark_scenario,
)
from src.planning.path_objective import (
    PathObjectiveWeights,
    evaluate_frenet_trajectory_objective,
)


ABS_TOLERANCE = 1e-9


def verify_canonical_objective(
    scenario,
    result,
) -> None:
    assert result.success
    assert result.trajectory is not None
    assert result.matched_objective_value is not None

    canonical_weights = PathObjectiveWeights(
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
            scenario
            .terminal_first_derivative_weight
        ),
    )

    expected = (
        evaluate_frenet_trajectory_objective(
            trajectory=result.trajectory,
            resolution_m=scenario.resolution_m,
            lateral_reference_m=(
                scenario.lateral_reference_m
            ),
            terminal_lateral_target_m=(
                scenario.terminal_lateral_target_m
            ),
            terminal_first_derivative_target=(
                scenario
                .terminal_first_derivative_target
            ),
            weights=canonical_weights,
        )
    )

    assert math.isclose(
        result.matched_objective_value,
        expected.total_cost,
        abs_tol=ABS_TOLERANCE,
    )


def main() -> None:
    scenario = (
        build_canonical_static_obstacle_scenario()
    )

    sampling_test_weights = PathObjectiveWeights(
        lateral_weight=2.0,
        first_derivative_weight=0.5,
        second_derivative_weight=2.0,
        third_derivative_weight=10.0,
        terminal_lateral_weight=8.0,
        terminal_first_derivative_weight=8.0,
    )

    optimization_test_weights = PathObjectiveWeights(
        lateral_weight=0.5,
        first_derivative_weight=2.0,
        second_derivative_weight=10.0,
        third_derivative_weight=30.0,
        terminal_lateral_weight=3.0,
        terminal_first_derivative_weight=3.0,
    )

    sampling_override_run = (
        run_benchmark_scenario(
            scenario=scenario,
            sampling_objective_weights=(
                sampling_test_weights
            ),
        )
    )

    verify_canonical_objective(
        scenario=scenario,
        result=(
            sampling_override_run
            .sampling_result
        ),
    )

    optimization_override_run = (
        run_benchmark_scenario(
            scenario=scenario,
            optimization_objective_weights=(
                optimization_test_weights
            ),
        )
    )

    verify_canonical_objective(
        scenario=scenario,
        result=(
            optimization_override_run
            .optimization_result
        ),
    )

    print(
        "Planner tuning / canonical objective "
        "separation passed."
    )
    print(
        "  Sampling planner weights can differ "
        "from canonical benchmark weights."
    )
    print(
        "  Optimization planner weights can differ "
        "from canonical benchmark weights."
    )


if __name__ == "__main__":
    main()