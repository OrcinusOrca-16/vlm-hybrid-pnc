from src.benchmark.evaluation_scenario_suite import (
    build_final_evaluation_scenarios,
)
from src.benchmark.scenario_suite import (
    build_stress_sweep_scenarios,
)
from src.benchmark.tuning_scenario_suite import (
    build_development_tuning_scenarios,
    build_tuning_scenarios,
)


EXPECTED_TUNING_NAMES = (
    "tuning_lateral_recovery",
    "tuning_obstacle_lateral_-0.75",
    "tuning_obstacle_lateral_+0.00",
    "tuning_obstacle_lateral_+0.75",
    "tuning_obstacle_longitudinal_18.00",
    "tuning_obstacle_longitudinal_34.00",
    "tuning_road_half_width_1.90",
    "tuning_curved_obstacle",
)

EXPECTED_EVALUATION_NAMES = (
    "evaluation_lateral_recovery",
    "evaluation_obstacle_lateral_-0.60",
    "evaluation_obstacle_lateral_+0.60",
    "evaluation_obstacle_longitudinal_19.00",
    "evaluation_obstacle_longitudinal_35.00",
    "evaluation_road_half_width_2.10",
    "evaluation_road_half_width_1.85",
    "evaluation_curved_nominal",
    "evaluation_curved_obstacle",
)


def main() -> None:
    tuning_scenarios = (
        build_tuning_scenarios()
    )

    stress_scenarios = (
        build_stress_sweep_scenarios()
    )

    development_scenarios = (
        build_development_tuning_scenarios()
    )

    evaluation_scenarios = (
        build_final_evaluation_scenarios()
    )

    tuning_names = tuple(
        scenario.name
        for scenario in tuning_scenarios
    )

    stress_names = tuple(
        scenario.name
        for scenario in stress_scenarios
    )

    development_names = tuple(
        scenario.name
        for scenario in development_scenarios
    )

    evaluation_names = tuple(
        scenario.name
        for scenario in evaluation_scenarios
    )

    assert tuning_names == EXPECTED_TUNING_NAMES
    assert evaluation_names == EXPECTED_EVALUATION_NAMES

    assert len(tuning_scenarios) == 8
    assert len(stress_scenarios) == 19
    assert len(development_scenarios) == 27
    assert len(evaluation_scenarios) == 9

    assert development_names == (
        tuning_names
        + stress_names
    )

    assert len(set(development_names)) == len(
        development_names
    )

    assert len(set(evaluation_names)) == len(
        evaluation_names
    )

    assert set(development_names).isdisjoint(
        evaluation_names
    )

    print(
        "Benchmark scenario split passed."
    )
    print(
        "  dedicated tuning scenarios="
        f"{len(tuning_scenarios)}"
    )
    print(
        "  reused stress scenarios="
        f"{len(stress_scenarios)}"
    )
    print(
        "  total development/tuning scenarios="
        f"{len(development_scenarios)}"
    )
    print(
        "  held-out evaluation scenarios="
        f"{len(evaluation_scenarios)}"
    )
    print(
        "  planner execution on evaluation set: "
        "intentionally skipped"
    )


if __name__ == "__main__":
    main()