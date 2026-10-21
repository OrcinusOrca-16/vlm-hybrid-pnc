from __future__ import annotations

import csv
import json
import re

from dataclasses import asdict, dataclass, field, replace
from pathlib import Path
from statistics import mean

from src.benchmark.metrics import (
    evaluate_planner_benchmark_metrics,
)
from src.benchmark.optimization_adapter import (
    run_optimization_benchmark_adapter,
)
from src.benchmark.scenario import BenchmarkScenario
from src.benchmark.tuning_scenario_suite import (
    build_development_tuning_scenarios,
)
from src.common.types import PathPoint
from src.config.planning import (
    DEFAULT_OPTIMIZATION_PLANNER_CONFIG,
)
from src.planning.frenet import vehicle_state_to_frenet
from src.planning.hard_constraints import obstacle_clearance
from src.planning.path_objective import PathObjectiveWeights


OUTPUT_DIR = Path("outputs/tuning")

RESULT_CSV_PATH = (
    OUTPUT_DIR
    / "optimization_tuning_refinement_results.csv"
)

SUMMARY_JSON_PATH = (
    OUTPUT_DIR
    / "optimization_tuning_refinement_summary.json"
)


RESOLUTION_VALUES_M = (
    0.5,
)

MAX_D2_VALUES = (
    1.0,
)

INFLATION_MARGIN_VALUES_M = (
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

CORRIDOR_EPSILON_VALUES_M = (
    0.01,
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

SOLVER_TOLERANCE_PROFILES = (
    (
        "pair_1e-6",
        1e-6,
        1e-6,
    ),
)

SOLVER_RUNTIME_REPEATS = 5


@dataclass(frozen=True)
class OptimizationTuningConfig:
    name: str = field(
        compare=False
    )

    resolution_m: float
    max_second_derivative_abs: float

    obstacle_inflation_safety_margin_m: float
    corridor_affected_epsilon_m: float

    planner_objective_weights: PathObjectiveWeights

    osqp_eps_abs: float
    osqp_eps_rel: float


@dataclass(frozen=True)
class CaseOutcome:
    success: bool

    planning_time_ms: float
    failure_reason: str | None

    canonical_objective: float | None

    terminal_lateral_error_m: float | None
    terminal_slope_error: float | None

    rms_lateral_deviation_m: float | None

    cartesian_path_length_m: float | None
    minimum_exact_obstacle_clearance_m: float | None

    max_abs_cartesian_curvature_per_m: float | None
    rms_cartesian_curvature_per_m: float | None
    rms_curvature_rate_per_m2: float | None

    corridor_infeasible_count: int | None
    qp_failure_count: int | None
    exact_validation_failure_count: int | None


@dataclass
class ConfigEvaluation:
    config: OptimizationTuningConfig

    outcomes: dict[
        str,
        CaseOutcome,
    ]

    comparison_objective: float | None = None

    @property
    def success_names(
        self,
    ) -> set[str]:
        return {
            name
            for name, outcome
            in self.outcomes.items()
            if outcome.success
        }

    @property
    def success_count(
        self,
    ) -> int:
        return len(
            self.success_names
        )

    @property
    def total_count(
        self,
    ) -> int:
        return len(
            self.outcomes
        )

    @property
    def failure_names(
        self,
    ) -> tuple[str, ...]:
        return tuple(
            name
            for name, outcome
            in self.outcomes.items()
            if not outcome.success
        )

    @property
    def mean_runtime_ms(
        self,
    ) -> float:
        return mean(
            outcome.planning_time_ms
            for outcome
            in self.outcomes.values()
        )

    @property
    def mean_success_objective(
        self,
    ) -> float | None:
        values = [
            outcome.canonical_objective
            for outcome
            in self.outcomes.values()
            if (
                outcome.success
                and outcome.canonical_objective
                is not None
            )
        ]

        if not values:
            return None

        return mean(
            values
        )


def mean_optional(
    values: list[
        float | None
    ],
) -> float | None:
    valid = [
        value
        for value in values
        if value is not None
    ]

    if not valid:
        return None

    return mean(
        valid
    )


def min_optional(
    values: list[
        float | None
    ],
) -> float | None:
    valid = [
        value
        for value in values
        if value is not None
    ]

    if not valid:
        return None

    return min(
        valid
    )


def max_optional(
    values: list[
        float | None
    ],
) -> float | None:
    valid = [
        value
        for value in values
        if value is not None
    ]

    if not valid:
        return None

    return max(
        valid
    )


def parse_failure_counts(
    reason: str | None,
) -> tuple[
    int | None,
    int | None,
    int | None,
]:
    if reason is None:
        return (
            None,
            None,
            None,
        )

    corridor_match = re.search(
        r"corridor infeasible=(\d+)",
        reason,
    )

    qp_match = re.search(
        r"QP failures=(\d+)",
        reason,
    )

    exact_match = re.search(
        r"exact validation failures=(\d+)",
        reason,
    )

    return (
        (
            int(
                corridor_match.group(1)
            )
            if corridor_match
            else None
        ),
        (
            int(
                qp_match.group(1)
            )
            if qp_match
            else None
        ),
        (
            int(
                exact_match.group(1)
            )
            if exact_match
            else None
        ),
    )


def initial_state_violation_reason(
    scenario: BenchmarkScenario,
) -> str | None:
    state = scenario.vehicle_state

    if (
        abs(
            state.curvature
        )
        > scenario.max_curvature
    ):
        return (
            "initial curvature exceeds "
            "scenario max curvature"
        )

    initial_frenet = (
        vehicle_state_to_frenet(
            vehicle_state=state,
            reference_line=(
                scenario.reference_line
            ),
        )
    )

    half_width = (
        scenario.vehicle_params.width_m
        / 2.0
    )

    allowed_min_l = (
        scenario.road_min_l
        + half_width
    )

    allowed_max_l = (
        scenario.road_max_l
        - half_width
    )

    if (
        allowed_min_l
        > allowed_max_l
    ):
        return (
            "road is narrower than vehicle"
        )

    if (
        initial_frenet.l
        < allowed_min_l
        or initial_frenet.l
        > allowed_max_l
    ):
        return (
            "initial vehicle state violates "
            "road boundary"
        )

    initial_path_point = PathPoint(
        x=state.x,
        y=state.y,
        yaw=state.yaw,
        curvature=state.curvature,
        s=0.0,
    )

    for obstacle in scenario.obstacles:
        clearance = obstacle_clearance(
            path_point=(
                initial_path_point
            ),
            vehicle_params=(
                scenario.vehicle_params
            ),
            obstacle=obstacle,
        )

        if (
            clearance
            <= scenario.minimum_clearance_m
        ):
            return (
                "initial obstacle clearance "
                f"{clearance:.4f} m <= "
                "required "
                f"{scenario.minimum_clearance_m:.4f} m"
            )

    return None


def prepare_scenarios(
) -> tuple[
    tuple[BenchmarkScenario, ...],
    tuple[BenchmarkScenario, ...],
    dict[str, str],
]:
    scenarios = (
        build_development_tuning_scenarios()
    )

    names = [
        scenario.name
        for scenario in scenarios
    ]

    if (
        len(names)
        != len(set(names))
    ):
        raise RuntimeError(
            "Development scenario names "
            "must be unique."
        )

    invalid_initial_states = {}
    active_scenarios = []

    for scenario in scenarios:
        reason = (
            initial_state_violation_reason(
                scenario
            )
        )

        if reason is not None:
            invalid_initial_states[
                scenario.name
            ] = reason
            continue

        active_scenarios.append(
            scenario
        )

    return (
        scenarios,
        tuple(
            active_scenarios
        ),
        invalid_initial_states,
    )


def baseline_config(
    name: str,
) -> OptimizationTuningConfig:
    base = (
        DEFAULT_OPTIMIZATION_PLANNER_CONFIG
    )

    return OptimizationTuningConfig(
        name=name,
        resolution_m=(
            base.resolution_m
        ),
        max_second_derivative_abs=(
            base.max_second_derivative_abs
        ),
        obstacle_inflation_safety_margin_m=(
            base
            .obstacle_inflation_safety_margin_m
        ),
        corridor_affected_epsilon_m=(
            base.corridor_affected_epsilon_m
        ),
        planner_objective_weights=(
            base.matched_objective_weights
        ),
        osqp_eps_abs=(
            base.osqp_eps_abs
        ),
        osqp_eps_rel=(
            base.osqp_eps_rel
        ),
    )


def build_stage_a_configs(
) -> tuple[
    OptimizationTuningConfig,
    ...,
]:
    base = baseline_config(
        "baseline"
    )

    configs = []

    for resolution_m in (
        RESOLUTION_VALUES_M
    ):
        for d2_bound in (
            MAX_D2_VALUES
        ):
            for margin_m in (
                INFLATION_MARGIN_VALUES_M
            ):
                for corridor_epsilon_m in (
                    CORRIDOR_EPSILON_VALUES_M
                ):
                    name = (
                        f"r{resolution_m:.2f}"
                        f"__d2_{d2_bound:.2f}"
                        f"__margin_{margin_m:.3f}"
                        f"__eps_{corridor_epsilon_m:.2f}"
                    )

                    configs.append(
                        replace(
                            base,
                            name=name,
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
                        )
                    )

    return tuple(
        configs
    )


def build_weight_profiles(
) -> tuple[
    tuple[
        str,
        PathObjectiveWeights,
    ],
    ...,
]:
    canonical = (
        DEFAULT_OPTIMIZATION_PLANNER_CONFIG
        .matched_objective_weights
    )

    profiles = [
        (
            "canonical",
            canonical,
        )
    ]

    for field_name in (
        WEIGHT_FIELDS
    ):
        original_value = getattr(
            canonical,
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
                        canonical,
                        **{
                            field_name: (
                                original_value
                                * factor
                            )
                        },
                    ),
                )
            )

    return tuple(
        profiles
    )


def build_stage_b_configs(
    stage_a_max_coverage: list[
        ConfigEvaluation
    ],
) -> tuple[
    OptimizationTuningConfig,
    ...,
]:
    profiles = (
        build_weight_profiles()
    )

    configs = []

    for evaluation in (
        stage_a_max_coverage
    ):
        base = evaluation.config

        for (
            profile_name,
            weights,
        ) in profiles:
            configs.append(
                replace(
                    base,
                    name=(
                        base.name
                        + "__"
                        + profile_name
                    ),
                    planner_objective_weights=(
                        weights
                    ),
                )
            )

    return tuple(
        configs
    )


def build_stage_c_configs(
    stage_b_pareto: list[
        ConfigEvaluation
    ],
) -> tuple[
    OptimizationTuningConfig,
    ...,
]:
    configs = []

    for evaluation in (
        stage_b_pareto
    ):
        base = evaluation.config

        for (
            profile_name,
            eps_abs,
            eps_rel,
        ) in (
            SOLVER_TOLERANCE_PROFILES
        ):
            configs.append(
                replace(
                    base,
                    name=(
                        base.name
                        + "__"
                        + profile_name
                    ),
                    osqp_eps_abs=(
                        eps_abs
                    ),
                    osqp_eps_rel=(
                        eps_rel
                    ),
                )
            )

    return tuple(
        configs
    )


def run_one_scenario(
    config: OptimizationTuningConfig,
    scenario: BenchmarkScenario,
    repeats: int = 1,
) -> CaseOutcome:
    if repeats <= 0:
        raise ValueError(
            "repeats must be positive."
        )

    results = []

    for _ in range(repeats):
        result = (
            run_optimization_benchmark_adapter(
                reference_line=(
                    scenario.reference_line
                ),
                vehicle_state=(
                    scenario.vehicle_state
                ),
                vehicle_params=(
                    scenario.vehicle_params
                ),
                obstacles=list(
                    scenario.obstacles
                ),
                road_min_l=(
                    scenario.road_min_l
                ),
                road_max_l=(
                    scenario.road_max_l
                ),
                max_curvature=(
                    scenario.max_curvature
                ),
                minimum_clearance_m=(
                    scenario.minimum_clearance_m
                ),
                planning_horizon_m=(
                    scenario.planning_horizon_m
                ),
                resolution_m=(
                    config.resolution_m
                ),
                lateral_reference_m=(
                    scenario.lateral_reference_m
                ),
                terminal_lateral_target_m=(
                    scenario
                    .terminal_lateral_target_m
                ),
                terminal_first_derivative_target=(
                    scenario
                    .terminal_first_derivative_target
                ),
                lateral_weight=(
                    scenario.lateral_weight
                ),
                first_derivative_weight=(
                    scenario
                    .first_derivative_weight
                ),
                second_derivative_weight=(
                    scenario
                    .second_derivative_weight
                ),
                third_derivative_weight=(
                    scenario
                    .third_derivative_weight
                ),
                terminal_lateral_weight=(
                    scenario
                    .terminal_lateral_weight
                ),
                terminal_first_derivative_weight=(
                    scenario
                    .terminal_first_derivative_weight
                ),
                max_second_derivative_abs=(
                    config
                    .max_second_derivative_abs
                ),
                obstacle_inflation_safety_margin_m=(
                    config
                    .obstacle_inflation_safety_margin_m
                ),
                corridor_affected_epsilon_m=(
                    config
                    .corridor_affected_epsilon_m
                ),
                osqp_eps_abs=(
                    config.osqp_eps_abs
                ),
                osqp_eps_rel=(
                    config.osqp_eps_rel
                ),
                planner_objective_weights=(
                    config
                    .planner_objective_weights
                ),
            )
        )

        results.append(
            result
        )

    success_values = {
        result.success
        for result in results
    }

    if len(success_values) != 1:
        raise RuntimeError(
            "Repeated solver runs changed "
            "success/failure status."
        )

    representative = results[0]

    if not representative.success:
        reasons = {
            result.failure_reason
            for result in results
        }

        if len(reasons) != 1:
            raise RuntimeError(
                "Repeated solver runs changed "
                "failure reason."
            )

    mean_runtime_ms = mean(
        result.planning_time_ms
        for result in results
    )

    metrics = (
        evaluate_planner_benchmark_metrics(
            result=representative,
            vehicle_params=(
                scenario.vehicle_params
            ),
            obstacles=list(
                scenario.obstacles
            ),
            lateral_reference_m=(
                scenario.lateral_reference_m
            ),
            terminal_lateral_target_m=(
                scenario
                .terminal_lateral_target_m
            ),
            terminal_first_derivative_target=(
                scenario
                .terminal_first_derivative_target
            ),
        )
    )

    (
        corridor_count,
        qp_count,
        exact_count,
    ) = parse_failure_counts(
        representative.failure_reason
    )

    return CaseOutcome(
        success=(
            representative.success
        ),
        planning_time_ms=(
            mean_runtime_ms
        ),
        failure_reason=(
            representative.failure_reason
        ),
        canonical_objective=(
            metrics
            .canonical_matched_objective_value
        ),
        terminal_lateral_error_m=(
            metrics.terminal_lateral_error_m
        ),
        terminal_slope_error=(
            metrics.terminal_slope_error
        ),
        rms_lateral_deviation_m=(
            metrics.rms_lateral_deviation_m
        ),
        cartesian_path_length_m=(
            metrics.cartesian_path_length_m
        ),
        minimum_exact_obstacle_clearance_m=(
            metrics
            .minimum_exact_obstacle_clearance_m
        ),
        max_abs_cartesian_curvature_per_m=(
            metrics
            .max_abs_cartesian_curvature_per_m
        ),
        rms_cartesian_curvature_per_m=(
            metrics
            .rms_cartesian_curvature_per_m
        ),
        rms_curvature_rate_per_m2=(
            metrics
            .rms_curvature_rate_per_m2
        ),
        corridor_infeasible_count=(
            corridor_count
        ),
        qp_failure_count=(
            qp_count
        ),
        exact_validation_failure_count=(
            exact_count
        ),
    )


def evaluate_config(
    config: OptimizationTuningConfig,
    scenarios: tuple[
        BenchmarkScenario,
        ...,
    ],
    repeats: int = 1,
) -> ConfigEvaluation:
    outcomes = {}

    for scenario in scenarios:
        outcomes[
            scenario.name
        ] = run_one_scenario(
            config=config,
            scenario=scenario,
            repeats=repeats,
        )

    return ConfigEvaluation(
        config=config,
        outcomes=outcomes,
    )


def configure_common_objective(
    evaluations: list[
        ConfigEvaluation
    ],
) -> tuple[
    int,
    set[str],
    list[ConfigEvaluation],
]:
    if not evaluations:
        raise ValueError(
            "No evaluations supplied."
        )

    max_success = max(
        evaluation.success_count
        for evaluation in evaluations
    )

    max_coverage_group = [
        evaluation
        for evaluation in evaluations
        if (
            evaluation.success_count
            == max_success
        )
    ]

    common_names = set(
        max_coverage_group[
            0
        ].success_names
    )

    for evaluation in (
        max_coverage_group[1:]
    ):
        common_names &= (
            evaluation.success_names
        )

    if not common_names:
        raise RuntimeError(
            "Maximum-coverage configs have "
            "no common successful scenarios."
        )

    for evaluation in (
        max_coverage_group
    ):
        values = [
            evaluation.outcomes[
                name
            ].canonical_objective
            for name
            in common_names
        ]

        if any(
            value is None
            for value in values
        ):
            raise RuntimeError(
                "Common-success scenario "
                "has no canonical objective."
            )

        evaluation.comparison_objective = (
            mean(
                value
                for value in values
                if value is not None
            )
        )

    return (
        max_success,
        common_names,
        max_coverage_group,
    )


def dominates(
    candidate: ConfigEvaluation,
    reference: ConfigEvaluation,
) -> bool:
    if (
        candidate.comparison_objective
        is None
        or reference.comparison_objective
        is None
    ):
        raise RuntimeError(
            "Comparison objective missing."
        )

    candidate_values = (
        candidate.comparison_objective,
        candidate.mean_runtime_ms,
    )

    reference_values = (
        reference.comparison_objective,
        reference.mean_runtime_ms,
    )

    no_worse = all(
        candidate_value
        <= reference_value
        for (
            candidate_value,
            reference_value,
        ) in zip(
            candidate_values,
            reference_values,
        )
    )

    strictly_better = any(
        candidate_value
        < reference_value
        for (
            candidate_value,
            reference_value,
        ) in zip(
            candidate_values,
            reference_values,
        )
    )

    return (
        no_worse
        and strictly_better
    )


def pareto_frontier(
    max_coverage_group: list[
        ConfigEvaluation
    ],
) -> list[
    ConfigEvaluation
]:
    frontier = []

    for reference in (
        max_coverage_group
    ):
        is_dominated = any(
            dominates(
                candidate=candidate,
                reference=reference,
            )
            for candidate
            in max_coverage_group
            if candidate is not reference
        )

        if not is_dominated:
            frontier.append(
                reference
            )

    frontier.sort(
        key=lambda evaluation: (
            evaluation.mean_runtime_ms,
            (
                evaluation
                .comparison_objective
                if evaluation
                .comparison_objective
                is not None
                else float("inf")
            ),
        )
    )

    return frontier


def evaluation_summary(
    evaluation: ConfigEvaluation,
) -> dict:
    successful = [
        outcome
        for outcome
        in evaluation.outcomes.values()
        if outcome.success
    ]

    config = evaluation.config

    return {
        "name": config.name,
        "resolution_m": (
            config.resolution_m
        ),
        "max_second_derivative_abs": (
            config
            .max_second_derivative_abs
        ),
        "obstacle_inflation_safety_margin_m": (
            config
            .obstacle_inflation_safety_margin_m
        ),
        "corridor_affected_epsilon_m": (
            config
            .corridor_affected_epsilon_m
        ),
        "planner_objective_weights": (
            asdict(
                config
                .planner_objective_weights
            )
        ),
        "osqp_eps_abs": (
            config.osqp_eps_abs
        ),
        "osqp_eps_rel": (
            config.osqp_eps_rel
        ),
        "success_count": (
            evaluation.success_count
        ),
        "total_count": (
            evaluation.total_count
        ),
        "failure_names": list(
            evaluation.failure_names
        ),
        "mean_runtime_ms": (
            evaluation.mean_runtime_ms
        ),
        "mean_success_objective": (
            evaluation
            .mean_success_objective
        ),
        "comparison_objective": (
            evaluation
            .comparison_objective
        ),
        "mean_rms_lateral_deviation_m": (
            mean_optional(
                [
                    outcome
                    .rms_lateral_deviation_m
                    for outcome
                    in successful
                ]
            )
        ),
        "minimum_clearance_over_successes_m": (
            min_optional(
                [
                    outcome
                    .minimum_exact_obstacle_clearance_m
                    for outcome
                    in successful
                ]
            )
        ),
        "maximum_curvature_over_successes_per_m": (
            max_optional(
                [
                    outcome
                    .max_abs_cartesian_curvature_per_m
                    for outcome
                    in successful
                ]
            )
        ),
        "mean_rms_curvature_per_m": (
            mean_optional(
                [
                    outcome
                    .rms_cartesian_curvature_per_m
                    for outcome
                    in successful
                ]
            )
        ),
        "mean_rms_curvature_rate_per_m2": (
            mean_optional(
                [
                    outcome
                    .rms_curvature_rate_per_m2
                    for outcome
                    in successful
                ]
            )
        ),
    }


def append_case_rows(
    rows: list[dict],
    stage: str,
    evaluation: ConfigEvaluation,
) -> None:
    config = evaluation.config
    weights = (
        config.planner_objective_weights
    )

    for (
        scenario_name,
        outcome,
    ) in evaluation.outcomes.items():
        rows.append(
            {
                "stage": stage,
                "config_name": (
                    config.name
                ),
                "scenario_name": (
                    scenario_name
                ),
                "resolution_m": (
                    config.resolution_m
                ),
                "max_second_derivative_abs": (
                    config
                    .max_second_derivative_abs
                ),
                "obstacle_inflation_safety_margin_m": (
                    config
                    .obstacle_inflation_safety_margin_m
                ),
                "corridor_affected_epsilon_m": (
                    config
                    .corridor_affected_epsilon_m
                ),
                "lateral_weight": (
                    weights.lateral_weight
                ),
                "first_derivative_weight": (
                    weights
                    .first_derivative_weight
                ),
                "second_derivative_weight": (
                    weights
                    .second_derivative_weight
                ),
                "third_derivative_weight": (
                    weights
                    .third_derivative_weight
                ),
                "terminal_lateral_weight": (
                    weights
                    .terminal_lateral_weight
                ),
                "terminal_first_derivative_weight": (
                    weights
                    .terminal_first_derivative_weight
                ),
                "osqp_eps_abs": (
                    config.osqp_eps_abs
                ),
                "osqp_eps_rel": (
                    config.osqp_eps_rel
                ),
                "success": (
                    outcome.success
                ),
                "planning_time_ms": (
                    outcome.planning_time_ms
                ),
                "canonical_objective": (
                    outcome.canonical_objective
                ),
                "terminal_lateral_error_m": (
                    outcome
                    .terminal_lateral_error_m
                ),
                "terminal_slope_error": (
                    outcome
                    .terminal_slope_error
                ),
                "rms_lateral_deviation_m": (
                    outcome
                    .rms_lateral_deviation_m
                ),
                "cartesian_path_length_m": (
                    outcome
                    .cartesian_path_length_m
                ),
                "minimum_exact_obstacle_clearance_m": (
                    outcome
                    .minimum_exact_obstacle_clearance_m
                ),
                "max_abs_cartesian_curvature_per_m": (
                    outcome
                    .max_abs_cartesian_curvature_per_m
                ),
                "rms_cartesian_curvature_per_m": (
                    outcome
                    .rms_cartesian_curvature_per_m
                ),
                "rms_curvature_rate_per_m2": (
                    outcome
                    .rms_curvature_rate_per_m2
                ),
                "corridor_infeasible_count": (
                    outcome
                    .corridor_infeasible_count
                ),
                "qp_failure_count": (
                    outcome
                    .qp_failure_count
                ),
                "exact_validation_failure_count": (
                    outcome
                    .exact_validation_failure_count
                ),
                "failure_reason": (
                    outcome.failure_reason
                ),
            }
        )


def print_stage_result(
    title: str,
    evaluations: list[
        ConfigEvaluation
    ],
    max_success: int,
    common_names: set[str],
    frontier: list[
        ConfigEvaluation
    ],
) -> None:
    print()
    print(title)

    print(
        "  configs="
        f"{len(evaluations)}"
    )

    print(
        "  maximum success coverage="
        f"{max_success}/"
        f"{evaluations[0].total_count}"
    )

    print(
        "  common comparison scenarios="
        f"{len(common_names)}"
    )

    print(
        "  Pareto configs="
        f"{len(frontier)}"
    )

    print()

    print(
        "Config"
        "                                             "
        "| success"
        " | common obj"
        " | mean ms"
    )

    print(
        "-" * 100
    )

    for evaluation in frontier:
        objective = (
            evaluation
            .comparison_objective
        )

        if objective is None:
            raise RuntimeError(
                "Missing comparison objective."
            )

        print(
            f"{evaluation.config.name:<50}"
            " | "
            f"{evaluation.success_count:>2}/"
            f"{evaluation.total_count:<2}"
            "   | "
            f"{objective:>10.6f}"
            " | "
            f"{evaluation.mean_runtime_ms:>7.3f}"
        )


def write_csv(
    rows: list[dict],
) -> None:
    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    fieldnames = (
        "stage",
        "config_name",
        "scenario_name",
        "resolution_m",
        "max_second_derivative_abs",
        "obstacle_inflation_safety_margin_m",
        "corridor_affected_epsilon_m",
        "lateral_weight",
        "first_derivative_weight",
        "second_derivative_weight",
        "third_derivative_weight",
        "terminal_lateral_weight",
        "terminal_first_derivative_weight",
        "osqp_eps_abs",
        "osqp_eps_rel",
        "success",
        "planning_time_ms",
        "canonical_objective",
        "terminal_lateral_error_m",
        "terminal_slope_error",
        "rms_lateral_deviation_m",
        "cartesian_path_length_m",
        "minimum_exact_obstacle_clearance_m",
        "max_abs_cartesian_curvature_per_m",
        "rms_cartesian_curvature_per_m",
        "rms_curvature_rate_per_m2",
        "corridor_infeasible_count",
        "qp_failure_count",
        "exact_validation_failure_count",
        "failure_reason",
    )

    with RESULT_CSV_PATH.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(
            file,
            fieldnames=fieldnames,
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
    ) = prepare_scenarios()

    csv_rows = []

    summary = {
        "metadata": {
            "development_scenarios": (
                len(
                    all_scenarios
                )
            ),
            "active_tuning_scenarios": (
                len(
                    active_scenarios
                )
            ),
            "held_out_evaluation_scenarios": (
                "NOT IMPORTED / NOT EXECUTED"
            ),
            "solver_runtime_repeats": (
                SOLVER_RUNTIME_REPEATS
            ),
        },
        "initial_state_invalid_scenarios": (
            invalid_initial_states
        ),
    }

    print(
        "Optimization Planner tuning experiment"
    )

    print(
        "  development scenarios="
        f"{len(all_scenarios)}"
    )

    print(
        "  active tuning scenarios="
        f"{len(active_scenarios)}"
    )

    print(
        "  held-out evaluation scenarios="
        "NOT IMPORTED / NOT EXECUTED"
    )

    if invalid_initial_states:
        print()
        print(
            "Initial-state-invalid scenarios:"
        )

        for (
            name,
            reason,
        ) in (
            invalid_initial_states.items()
        ):
            print(
                f"  {name}: {reason}"
            )

    # ------------------------------------------------------------
    # Stage A
    # Geometry / feasibility sweep
    # ------------------------------------------------------------

    print()
    print(
        "Stage A: geometry / feasibility sweep"
    )

    stage_a_configs = (
        build_stage_a_configs()
    )

    stage_a_evaluations = []

    for (
        index,
        config,
    ) in enumerate(
        stage_a_configs,
        start=1,
    ):
        print(
            f"  [{index}/"
            f"{len(stage_a_configs)}] "
            f"{config.name}"
        )

        evaluation = evaluate_config(
            config=config,
            scenarios=(
                active_scenarios
            ),
        )

        stage_a_evaluations.append(
            evaluation
        )

        append_case_rows(
            rows=csv_rows,
            stage="A_geometry",
            evaluation=evaluation,
        )

    (
        stage_a_max_success,
        stage_a_common_names,
        stage_a_max_group,
    ) = configure_common_objective(
        stage_a_evaluations
    )

    stage_a_pareto = (
        pareto_frontier(
            stage_a_max_group
        )
    )

    print_stage_result(
        title="Stage A result",
        evaluations=(
            stage_a_evaluations
        ),
        max_success=(
            stage_a_max_success
        ),
        common_names=(
            stage_a_common_names
        ),
        frontier=(
            stage_a_pareto
        ),
    )

    summary[
        "stage_a_geometry"
    ] = {
        "max_success_count": (
            stage_a_max_success
        ),
        "common_comparison_scenarios": sorted(
            stage_a_common_names
        ),
        "max_coverage_config_names": [
            evaluation.config.name
            for evaluation
            in stage_a_max_group
        ],
        "pareto_config_names": [
            evaluation.config.name
            for evaluation
            in stage_a_pareto
        ],
        "configs": [
            evaluation_summary(
                evaluation
            )
            for evaluation
            in stage_a_evaluations
        ],
    }

    # ------------------------------------------------------------
    # Stage B
    # Objective-weight sweep.
    #
    # Keep every Stage-A maximum-coverage structure here.
    # We intentionally do not allow the script to choose a single
    # structural configuration.
    # ------------------------------------------------------------

    print()
    print(
        "Stage B: matched objective-weight sweep"
    )

    stage_b_configs = (
        build_stage_b_configs(
            stage_a_evaluations
        )
    )

    stage_b_evaluations = []

    for (
        index,
        config,
    ) in enumerate(
        stage_b_configs,
        start=1,
    ):
        print(
            f"  [{index}/"
            f"{len(stage_b_configs)}] "
            f"{config.name}"
        )

        evaluation = evaluate_config(
            config=config,
            scenarios=(
                active_scenarios
            ),
        )

        stage_b_evaluations.append(
            evaluation
        )

        append_case_rows(
            rows=csv_rows,
            stage="B_objective",
            evaluation=evaluation,
        )

    (
        stage_b_max_success,
        stage_b_common_names,
        stage_b_max_group,
    ) = configure_common_objective(
        stage_b_evaluations
    )

    stage_b_pareto = (
        pareto_frontier(
            stage_b_max_group
        )
    )

    print_stage_result(
        title="Stage B result",
        evaluations=(
            stage_b_evaluations
        ),
        max_success=(
            stage_b_max_success
        ),
        common_names=(
            stage_b_common_names
        ),
        frontier=(
            stage_b_pareto
        ),
    )

    summary[
        "stage_b_objective"
    ] = {
        "max_success_count": (
            stage_b_max_success
        ),
        "common_comparison_scenarios": sorted(
            stage_b_common_names
        ),
        "pareto_config_names": [
            evaluation.config.name
            for evaluation
            in stage_b_pareto
        ],
        "configs": [
            evaluation_summary(
                evaluation
            )
            for evaluation
            in stage_b_evaluations
        ],
    }

    # ------------------------------------------------------------
    # Stage C
    # Solver tolerance / runtime.
    #
    # Repeated wall-clock measurements are used here because this
    # stage explicitly studies numerical tolerance vs runtime.
    # ------------------------------------------------------------

    print()
    print(
        "Stage C: OSQP tolerance / runtime sweep"
    )

    stage_c_configs = (
        build_stage_c_configs(
            stage_b_pareto
        )
    )

    stage_c_evaluations = []

    for (
        index,
        config,
    ) in enumerate(
        stage_c_configs,
        start=1,
    ):
        print(
            f"  [{index}/"
            f"{len(stage_c_configs)}] "
            f"{config.name}"
        )

        evaluation = evaluate_config(
            config=config,
            scenarios=(
                active_scenarios
            ),
            repeats=(
                SOLVER_RUNTIME_REPEATS
            ),
        )

        stage_c_evaluations.append(
            evaluation
        )

        append_case_rows(
            rows=csv_rows,
            stage="C_solver",
            evaluation=evaluation,
        )

    (
        stage_c_max_success,
        stage_c_common_names,
        stage_c_max_group,
    ) = configure_common_objective(
        stage_c_evaluations
    )

    stage_c_pareto = (
        pareto_frontier(
            stage_c_max_group
        )
    )

    print_stage_result(
        title="Stage C result",
        evaluations=(
            stage_c_evaluations
        ),
        max_success=(
            stage_c_max_success
        ),
        common_names=(
            stage_c_common_names
        ),
        frontier=(
            stage_c_pareto
        ),
    )

    summary[
        "stage_c_solver"
    ] = {
        "max_success_count": (
            stage_c_max_success
        ),
        "common_comparison_scenarios": sorted(
            stage_c_common_names
        ),
        "pareto_config_names": [
            evaluation.config.name
            for evaluation
            in stage_c_pareto
        ],
        "configs": [
            evaluation_summary(
                evaluation
            )
            for evaluation
            in stage_c_evaluations
        ],
    }

    write_csv(
        csv_rows
    )

    write_json(
        summary
    )

    print()
    print(
        "Optimization tuning experiment completed."
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