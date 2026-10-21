from __future__ import annotations

import csv
import json

from dataclasses import asdict, replace
from pathlib import Path

import scripts.planning.run_optimization_tuning as tuning
from src.planning.path_objective import PathObjectiveWeights


OUTPUT_DIR = Path("outputs/tuning")

RESULT_CSV_PATH = (
    OUTPUT_DIR
    / "optimization_tuning_reproduction_results.csv"
)

SUMMARY_JSON_PATH = (
    OUTPUT_DIR
    / "optimization_tuning_reproduction_summary.json"
)


# ------------------------------------------------------------
# Frozen historical configs
#
# IMPORTANT:
# DEFAULT_OPTIMIZATION_PLANNER_CONFIG is already tuned now.
# Therefore the original pre-tuning weights must be recorded
# explicitly here so that this experiment remains reproducible.
# ------------------------------------------------------------

PRE_TUNE_WEIGHTS = PathObjectiveWeights(
    lateral_weight=1.0,
    first_derivative_weight=1.0,
    second_derivative_weight=5.0,
    third_derivative_weight=20.0,
    terminal_lateral_weight=5.0,
    terminal_first_derivative_weight=5.0,
)

FROZEN_TUNED_WEIGHTS = PathObjectiveWeights(
    lateral_weight=0.5,
    first_derivative_weight=1.0,
    second_derivative_weight=5.0,
    third_derivative_weight=20.0,
    terminal_lateral_weight=5.0,
    terminal_first_derivative_weight=5.0,
)


# ------------------------------------------------------------
# Stage A
# Original coarse geometry / feasibility sweep.
# ------------------------------------------------------------

COARSE_RESOLUTION_VALUES_M = (
    0.25,
    0.5,
    1.0,
)

COARSE_MAX_D2_VALUES = (
    0.25,
    0.5,
    1.0,
    2.0,
)

COARSE_INFLATION_MARGIN_VALUES_M = (
    0.0,
    0.05,
    0.10,
    0.15,
    0.20,
)

COARSE_CORRIDOR_EPSILON_VALUES_M = (
    0.0,
    0.01,
    0.05,
)


# ------------------------------------------------------------
# Stage B
# Refinement used after the coarse study showed that
# margin and objective weights interact.
# ------------------------------------------------------------

REFINEMENT_MARGIN_VALUES_M = (
    0.05,
    0.055,
    0.06,
    0.065,
    0.07,
    0.075,
    0.08,
    0.085,
    0.09,
)

WEIGHT_FACTORS = (
    0.5,
    2.0,
)

WEIGHT_FIELDS = (
    "lateral_weight",
    "first_derivative_weight",
    "second_derivative_weight",
    "third_derivative_weight",
    "terminal_lateral_weight",
    "terminal_first_derivative_weight",
)


# ------------------------------------------------------------
# Stage C
# Solver tolerance study on the frozen tuned geometry/objective.
# ------------------------------------------------------------

SOLVER_TOLERANCE_PROFILES = (
    (
        "pair_1e-4",
        1e-4,
        1e-4,
    ),
    (
        "pair_1e-5",
        1e-5,
        1e-5,
    ),
    (
        "pair_1e-6",
        1e-6,
        1e-6,
    ),
    (
        "pair_1e-7",
        1e-7,
        1e-7,
    ),
    (
        "abs_1e-4_rel_1e-6",
        1e-4,
        1e-6,
    ),
    (
        "abs_1e-6_rel_1e-4",
        1e-6,
        1e-4,
    ),
)

SOLVER_RUNTIME_REPEATS = 5


def make_config(
    *,
    name: str,
    resolution_m: float,
    max_second_derivative_abs: float,
    obstacle_inflation_safety_margin_m: float,
    corridor_affected_epsilon_m: float,
    planner_objective_weights: PathObjectiveWeights,
    osqp_eps_abs: float = 1e-6,
    osqp_eps_rel: float = 1e-6,
) -> tuning.OptimizationTuningConfig:
    return tuning.OptimizationTuningConfig(
        name=name,
        resolution_m=resolution_m,
        max_second_derivative_abs=(
            max_second_derivative_abs
        ),
        obstacle_inflation_safety_margin_m=(
            obstacle_inflation_safety_margin_m
        ),
        corridor_affected_epsilon_m=(
            corridor_affected_epsilon_m
        ),
        planner_objective_weights=(
            planner_objective_weights
        ),
        osqp_eps_abs=osqp_eps_abs,
        osqp_eps_rel=osqp_eps_rel,
    )


def build_coarse_configs(
) -> tuple[
    tuning.OptimizationTuningConfig,
    ...,
]:
    configs = []

    for resolution_m in (
        COARSE_RESOLUTION_VALUES_M
    ):
        for d2_bound in (
            COARSE_MAX_D2_VALUES
        ):
            for margin_m in (
                COARSE_INFLATION_MARGIN_VALUES_M
            ):
                for corridor_epsilon_m in (
                    COARSE_CORRIDOR_EPSILON_VALUES_M
                ):
                    configs.append(
                        make_config(
                            name=(
                                f"r{resolution_m:.2f}"
                                f"__d2_{d2_bound:.2f}"
                                f"__margin_{margin_m:.3f}"
                                f"__eps_{corridor_epsilon_m:.2f}"
                            ),
                            resolution_m=(
                                resolution_m
                            ),
                            max_second_derivative_abs=(
                                d2_bound
                            ),
                            obstacle_inflation_safety_margin_m=(
                                margin_m
                            ),
                            corridor_affected_epsilon_m=(
                                corridor_epsilon_m
                            ),
                            planner_objective_weights=(
                                PRE_TUNE_WEIGHTS
                            ),
                        )
                    )

    return tuple(configs)


def build_weight_profiles(
) -> tuple[
    tuple[
        str,
        PathObjectiveWeights,
    ],
    ...,
]:
    profiles = [
        (
            "canonical",
            PRE_TUNE_WEIGHTS,
        )
    ]

    for field_name in (
        WEIGHT_FIELDS
    ):
        original_value = getattr(
            PRE_TUNE_WEIGHTS,
            field_name,
        )

        for factor in (
            WEIGHT_FACTORS
        ):
            profiles.append(
                (
                    (
                        field_name
                        + "_x"
                        + f"{factor:.1f}"
                    ),
                    replace(
                        PRE_TUNE_WEIGHTS,
                        **{
                            field_name: (
                                original_value
                                * factor
                            )
                        },
                    ),
                )
            )

    return tuple(profiles)


def build_interaction_configs(
) -> tuple[
    tuning.OptimizationTuningConfig,
    ...,
]:
    configs = []

    for margin_m in (
        REFINEMENT_MARGIN_VALUES_M
    ):
        for (
            profile_name,
            weights,
        ) in build_weight_profiles():
            configs.append(
                make_config(
                    name=(
                        "r0.50"
                        "__d2_1.00"
                        f"__margin_{margin_m:.3f}"
                        "__eps_0.01"
                        f"__{profile_name}"
                    ),
                    resolution_m=0.5,
                    max_second_derivative_abs=(
                        1.0
                    ),
                    obstacle_inflation_safety_margin_m=(
                        margin_m
                    ),
                    corridor_affected_epsilon_m=(
                        0.01
                    ),
                    planner_objective_weights=(
                        weights
                    ),
                )
            )

    return tuple(configs)


def build_solver_configs(
) -> tuple[
    tuning.OptimizationTuningConfig,
    ...,
]:
    configs = []

    for (
        profile_name,
        eps_abs,
        eps_rel,
    ) in (
        SOLVER_TOLERANCE_PROFILES
    ):
        configs.append(
            make_config(
                name=(
                    "frozen_tuned"
                    f"__{profile_name}"
                ),
                resolution_m=0.5,
                max_second_derivative_abs=(
                    1.0
                ),
                obstacle_inflation_safety_margin_m=(
                    0.05
                ),
                corridor_affected_epsilon_m=(
                    0.01
                ),
                planner_objective_weights=(
                    FROZEN_TUNED_WEIGHTS
                ),
                osqp_eps_abs=(
                    eps_abs
                ),
                osqp_eps_rel=(
                    eps_rel
                ),
            )
        )

    return tuple(configs)


def evaluate_stage(
    *,
    stage_name: str,
    configs: tuple[
        tuning.OptimizationTuningConfig,
        ...,
    ],
    scenarios,
    csv_rows: list[dict],
    repeats: int = 1,
) -> tuple[
    list[tuning.ConfigEvaluation],
    int,
    set[str],
    list[tuning.ConfigEvaluation],
    list[tuning.ConfigEvaluation],
]:
    evaluations = []

    print()
    print(stage_name)

    for (
        index,
        config,
    ) in enumerate(
        configs,
        start=1,
    ):
        print(
            f"  [{index}/"
            f"{len(configs)}] "
            f"{config.name}"
        )

        evaluation = (
            tuning.evaluate_config(
                config=config,
                scenarios=scenarios,
                repeats=repeats,
            )
        )

        evaluations.append(
            evaluation
        )

        tuning.append_case_rows(
            rows=csv_rows,
            stage=stage_name,
            evaluation=evaluation,
        )

    (
        max_success,
        common_names,
        max_coverage_group,
    ) = (
        tuning.configure_common_objective(
            evaluations
        )
    )

    frontier = (
        tuning.pareto_frontier(
            max_coverage_group
        )
    )

    tuning.print_stage_result(
        title=(
            f"{stage_name} result"
        ),
        evaluations=evaluations,
        max_success=max_success,
        common_names=common_names,
        frontier=frontier,
    )

    return (
        evaluations,
        max_success,
        common_names,
        max_coverage_group,
        frontier,
    )


def build_stage_summary(
    *,
    evaluations: list[
        tuning.ConfigEvaluation
    ],
    max_success: int,
    common_names: set[str],
    max_coverage_group: list[
        tuning.ConfigEvaluation
    ],
    frontier: list[
        tuning.ConfigEvaluation
    ],
) -> dict:
    return {
        "max_success_count": (
            max_success
        ),
        "common_comparison_scenarios": sorted(
            common_names
        ),
        "max_coverage_config_names": [
            evaluation.config.name
            for evaluation
            in max_coverage_group
        ],
        "pareto_config_names": [
            evaluation.config.name
            for evaluation
            in frontier
        ],
        "configs": [
            tuning.evaluation_summary(
                evaluation
            )
            for evaluation
            in evaluations
        ],
    }


def write_csv(
    rows: list[dict],
) -> None:
    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    if not rows:
        raise RuntimeError(
            "No tuning rows were generated."
        )

    with RESULT_CSV_PATH.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=list(
                rows[0].keys()
            ),
        )

        writer.writeheader()
        writer.writerows(
            rows
        )


def write_json(
    summary: dict,
) -> None:
    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    with SUMMARY_JSON_PATH.open(
        "w",
        encoding="utf-8",
    ) as file:
        json.dump(
            summary,
            file,
            indent=2,
            ensure_ascii=False,
            allow_nan=False,
        )


def main() -> None:
    (
        all_scenarios,
        active_scenarios,
        invalid_initial_states,
    ) = tuning.prepare_scenarios()

    csv_rows = []

    summary = {
        "metadata": {
            "purpose": (
                "Reproduce the Optimization Planner "
                "tuning study from frozen "
                "pre-tuning parameters."
            ),
            "development_scenarios": len(
                all_scenarios
            ),
            "active_tuning_scenarios": len(
                active_scenarios
            ),
            "held_out_evaluation_scenarios": (
                "NOT IMPORTED / NOT EXECUTED"
            ),
            "solver_runtime_repeats": (
                SOLVER_RUNTIME_REPEATS
            ),
            "pre_tune_weights": asdict(
                PRE_TUNE_WEIGHTS
            ),
            "frozen_tuned_weights": asdict(
                FROZEN_TUNED_WEIGHTS
            ),
        },
        "initial_state_invalid_scenarios": (
            invalid_initial_states
        ),
    }

    print(
        "Optimization Planner tuning reproduction"
    )

    print(
        "  held-out evaluation scenarios="
        "NOT IMPORTED / NOT EXECUTED"
    )

    # --------------------------------------------------------
    # Stage A
    # Full coarse geometry / feasibility sweep.
    # --------------------------------------------------------

    (
        coarse_evaluations,
        coarse_max_success,
        coarse_common_names,
        coarse_max_group,
        coarse_frontier,
    ) = evaluate_stage(
        stage_name=(
            "A_coarse_geometry"
        ),
        configs=(
            build_coarse_configs()
        ),
        scenarios=(
            active_scenarios
        ),
        csv_rows=(
            csv_rows
        ),
    )

    summary[
        "stage_a_coarse_geometry"
    ] = build_stage_summary(
        evaluations=(
            coarse_evaluations
        ),
        max_success=(
            coarse_max_success
        ),
        common_names=(
            coarse_common_names
        ),
        max_coverage_group=(
            coarse_max_group
        ),
        frontier=(
            coarse_frontier
        ),
    )

    # --------------------------------------------------------
    # Stage B
    # Targeted margin x objective-weight interaction.
    # --------------------------------------------------------

    (
        interaction_evaluations,
        interaction_max_success,
        interaction_common_names,
        interaction_max_group,
        interaction_frontier,
    ) = evaluate_stage(
        stage_name=(
            "B_margin_weight_interaction"
        ),
        configs=(
            build_interaction_configs()
        ),
        scenarios=(
            active_scenarios
        ),
        csv_rows=(
            csv_rows
        ),
    )

    summary[
        "stage_b_margin_weight_interaction"
    ] = build_stage_summary(
        evaluations=(
            interaction_evaluations
        ),
        max_success=(
            interaction_max_success
        ),
        common_names=(
            interaction_common_names
        ),
        max_coverage_group=(
            interaction_max_group
        ),
        frontier=(
            interaction_frontier
        ),
    )

    # --------------------------------------------------------
    # Stage C
    # Solver tolerance / runtime on the frozen tuned config.
    # --------------------------------------------------------

    (
        solver_evaluations,
        solver_max_success,
        solver_common_names,
        solver_max_group,
        solver_frontier,
    ) = evaluate_stage(
        stage_name=(
            "C_solver_tolerance"
        ),
        configs=(
            build_solver_configs()
        ),
        scenarios=(
            active_scenarios
        ),
        csv_rows=(
            csv_rows
        ),
        repeats=(
            SOLVER_RUNTIME_REPEATS
        ),
    )

    summary[
        "stage_c_solver_tolerance"
    ] = build_stage_summary(
        evaluations=(
            solver_evaluations
        ),
        max_success=(
            solver_max_success
        ),
        common_names=(
            solver_common_names
        ),
        max_coverage_group=(
            solver_max_group
        ),
        frontier=(
            solver_frontier
        ),
    )

    write_csv(
        csv_rows
    )

    write_json(
        summary
    )

    print()
    print(
        "Optimization tuning reproduction completed."
    )

    print(
        "  CSV: "
        f"{RESULT_CSV_PATH}"
    )

    print(
        "  JSON: "
        f"{SUMMARY_JSON_PATH}"
    )

    print(
        "  No config was automatically recommended."
    )

    print(
        "  Held-out evaluation scenarios "
        "were never imported or executed."
    )


if __name__ == "__main__":
    main()