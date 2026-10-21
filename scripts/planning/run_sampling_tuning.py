from __future__ import annotations

import csv
import json

from dataclasses import (
    asdict,
    dataclass,
    field,
    replace,
)
from pathlib import Path
from statistics import mean

from src.benchmark.metrics import (
    PlannerBenchmarkMetrics,
    evaluate_planner_benchmark_metrics,
)
from src.benchmark.planner_benchmark import (
    PlannerBenchmarkResult,
)
from src.benchmark.sampling_adapter import (
    run_sampling_benchmark_adapter,
)
from src.benchmark.scenario import (
    BenchmarkScenario,
)
from src.benchmark.tuning_scenario_suite import (
    build_development_tuning_scenarios,
)
from src.common.types import (
    FrenetTrajectory,
    PathPoint,
)
from src.config.planning import (
    DEFAULT_SAMPLING_PLANNER_CONFIG,
)
from src.planning.frenet import (
    frenet_trajectory_to_path,
    vehicle_state_to_frenet,
)
from src.planning.hard_constraints import (
    filter_feasible_candidates,
    obstacle_clearance,
)
from src.planning.path_objective import (
    PathObjectiveWeights,
    evaluate_frenet_trajectory_objective,
)
from src.planning.sampling_planner import (
    generate_returning_frenet_candidates_from_vehicle_state,
)
from src.planning.soft_cost import (
    select_best_matched_candidate,
)


OUTPUT_DIR = Path(
    "outputs/tuning"
)

RESULT_CSV_PATH = (
    OUTPUT_DIR
    / "sampling_tuning_results.csv"
)

SUMMARY_JSON_PATH = (
    OUTPUT_DIR
    / "sampling_tuning_summary.json"
)


# Terminal-state sampling can multiply candidate count very quickly.
# This is only a runtime guard, not a tuning decision criterion.
MAX_TERMINAL_CANDIDATES = 500


WEIGHT_FIELDS = (
    "lateral_weight",
    "first_derivative_weight",
    "second_derivative_weight",
    "third_derivative_weight",
    "terminal_lateral_weight",
    "terminal_first_derivative_weight",
)

WEIGHT_FACTORS = (
    0.5,
    2.0,
)


@dataclass(frozen=True)
class SamplingTuningConfig:
    # Display label only.
    # Ignore the name when comparing / hashing configs so identical
    # parameter sets can reuse cached planner runs.
    name: str = field(
        compare=False
    )

    bypass_lateral_positions_m: tuple[
        float,
        ...,
    ]

    bypass_lengths_m: tuple[
        float,
        ...,
    ]

    final_lateral_offsets_m: tuple[
        float,
        ...,
    ] = (
        0.0,
    )

    final_slope_offsets: tuple[
        float,
        ...,
    ] = (
        0.0,
    )

    @property
    def candidate_count(
        self,
    ) -> int:
        return (
            len(
                self.bypass_lateral_positions_m
            )
            * len(
                self.bypass_lengths_m
            )
            * len(
                self.final_lateral_offsets_m
            )
            * len(
                self.final_slope_offsets
            )
        )


@dataclass(frozen=True)
class CaseOutcome:
    success: bool

    planning_time_ms: (
        float
        | None
    )

    failure_reason: (
        str
        | None
    )

    canonical_objective: (
        float
        | None
    )

    terminal_lateral_error_m: (
        float
        | None
    )

    terminal_slope_error: (
        float
        | None
    )

    rms_lateral_deviation_m: (
        float
        | None
    )

    cartesian_path_length_m: (
        float
        | None
    )

    minimum_exact_obstacle_clearance_m: (
        float
        | None
    )

    max_abs_cartesian_curvature_per_m: (
        float
        | None
    )

    rms_cartesian_curvature_per_m: (
        float
        | None
    )

    rms_curvature_rate_per_m2: (
        float
        | None
    )


@dataclass
class ConfigEvaluation:
    config: SamplingTuningConfig

    outcomes: dict[
        str,
        CaseOutcome,
    ]

    comparison_objective: (
        float
        | None
    ) = None

    @property
    def success_names(
        self,
    ) -> set[str]:
        return {
            name
            for (
                name,
                outcome,
            ) in self.outcomes.items()
            if outcome.success
        }

    @property
    def failure_names(
        self,
    ) -> tuple[str, ...]:
        return tuple(
            name
            for (
                name,
                outcome,
            ) in self.outcomes.items()
            if not outcome.success
        )

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
    def mean_runtime_ms(
        self,
    ) -> float:
        runtimes = [
            outcome.planning_time_ms
            for outcome
            in self.outcomes.values()
            if (
                outcome.planning_time_ms
                is not None
            )
        ]

        if not runtimes:
            raise RuntimeError(
                "No planner runtimes available."
            )

        return mean(
            runtimes
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


_RUN_CACHE: dict[
    tuple[
        SamplingTuningConfig,
        str,
        PathObjectiveWeights,
    ],
    CaseOutcome,
] = {}


_CANDIDATE_CACHE: dict[
    tuple[
        SamplingTuningConfig,
        str,
    ],
    list[
        tuple[
            FrenetTrajectory,
            list[PathPoint],
        ]
    ],
] = {}


def symmetric_samples(
    max_abs_value: float,
    interval: float,
) -> tuple[float, ...]:
    if max_abs_value <= 0.0:
        raise ValueError(
            "max_abs_value must be positive."
        )

    if interval <= 0.0:
        raise ValueError(
            "interval must be positive."
        )

    step_count_float = (
        max_abs_value
        / interval
    )

    step_count = round(
        step_count_float
    )

    if abs(
        step_count_float
        - step_count
    ) > 1e-9:
        raise ValueError(
            "max_abs_value must be divisible "
            "by interval."
        )

    return tuple(
        round(
            index
            * interval,
            10,
        )
        for index in range(
            -step_count,
            step_count + 1,
        )
    )


def build_structure_configs(
) -> tuple[
    SamplingTuningConfig,
    ...,
]:
    lateral_profiles = (
        (
            "r1.50_d0.50",
            symmetric_samples(
                max_abs_value=1.5,
                interval=0.5,
            ),
        ),
        (
            "r2.00_d0.50",
            symmetric_samples(
                max_abs_value=2.0,
                interval=0.5,
            ),
        ),
        (
            "r2.00_d0.40",
            symmetric_samples(
                max_abs_value=2.0,
                interval=0.4,
            ),
        ),
        (
            "r2.00_d0.25",
            symmetric_samples(
                max_abs_value=2.0,
                interval=0.25,
            ),
        ),
    )

    bypass_length_profiles = (
        (
            "baseline",
            (
                10.0,
                14.0,
                18.0,
            ),
        ),
        (
            "efficient",
            (
                8.0,
                22.0,
            ),
        ),
        (
            "balanced",
            (
                8.0,
                14.0,
                22.0,
            ),
        ),
        (
            "mid_dense",
            (
                8.0,
                12.0,
                16.0,
                22.0,
            ),
        ),
        (
            "dense",
            (
                6.0,
                10.0,
                14.0,
                18.0,
                22.0,
            ),
        ),
    )

    configs = []

    for (
        lateral_name,
        lateral_positions,
    ) in lateral_profiles:
        for (
            length_name,
            bypass_lengths,
        ) in bypass_length_profiles:
            configs.append(
                SamplingTuningConfig(
                    name=(
                        lateral_name
                        + "__"
                        + length_name
                    ),
                    bypass_lateral_positions_m=(
                        lateral_positions
                    ),
                    bypass_lengths_m=(
                        bypass_lengths
                    ),
                )
            )

    return tuple(
        configs
    )


def build_terminal_variants(
    base_config: SamplingTuningConfig,
) -> tuple[
    SamplingTuningConfig,
    ...,
]:
    terminal_profiles = (
        (
            "terminal_strict",
            (
                0.0,
            ),
            (
                0.0,
            ),
        ),
        (
            "terminal_lateral",
            (
                -0.25,
                0.0,
                0.25,
            ),
            (
                0.0,
            ),
        ),
        (
            "terminal_slope",
            (
                0.0,
            ),
            (
                -0.02,
                0.0,
                0.02,
            ),
        ),
        (
            "terminal_full",
            (
                -0.25,
                0.0,
                0.25,
            ),
            (
                -0.02,
                0.0,
                0.02,
            ),
        ),
    )

    variants = []

    for (
        terminal_name,
        lateral_offsets,
        slope_offsets,
    ) in terminal_profiles:
        config = SamplingTuningConfig(
            name=(
                base_config.name
                + "__"
                + terminal_name
            ),
            bypass_lateral_positions_m=(
                base_config
                .bypass_lateral_positions_m
            ),
            bypass_lengths_m=(
                base_config
                .bypass_lengths_m
            ),
            final_lateral_offsets_m=(
                lateral_offsets
            ),
            final_slope_offsets=(
                slope_offsets
            ),
        )

        if (
            config.candidate_count
            <= MAX_TERMINAL_CANDIDATES
        ):
            variants.append(
                config
            )
        else:
            print(
                "  skip "
                f"{config.name}: "
                f"{config.candidate_count} "
                "candidates exceeds guard "
                f"{MAX_TERMINAL_CANDIDATES}"
            )

    return tuple(
        variants
    )


def unique_configs(
    configs: list[
        SamplingTuningConfig
    ],
) -> list[
    SamplingTuningConfig
]:
    result = []

    for config in configs:
        if config not in result:
            result.append(
                config
            )

    return result


def canonical_weights_from_scenario(
    scenario: BenchmarkScenario,
) -> PathObjectiveWeights:
    return PathObjectiveWeights(
        lateral_weight=(
            scenario.lateral_weight
        ),
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


def initial_state_violation_reason(
    scenario: BenchmarkScenario,
) -> str | None:
    state = (
        scenario.vehicle_state
    )

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

    for obstacle in (
        scenario.obstacles
    ):
        clearance = (
            obstacle_clearance(
                path_point=(
                    initial_path_point
                ),
                vehicle_params=(
                    scenario.vehicle_params
                ),
                obstacle=obstacle,
            )
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
        len(
            names
        )
        != len(
            set(
                names
            )
        )
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


def outcome_from_metrics(
    result: PlannerBenchmarkResult,
    metrics: PlannerBenchmarkMetrics,
) -> CaseOutcome:
    return CaseOutcome(
        success=result.success,
        planning_time_ms=(
            result.planning_time_ms
        ),
        failure_reason=(
            result.failure_reason
        ),
        canonical_objective=(
            metrics
            .canonical_matched_objective_value
        ),
        terminal_lateral_error_m=(
            metrics
            .terminal_lateral_error_m
        ),
        terminal_slope_error=(
            metrics
            .terminal_slope_error
        ),
        rms_lateral_deviation_m=(
            metrics
            .rms_lateral_deviation_m
        ),
        cartesian_path_length_m=(
            metrics
            .cartesian_path_length_m
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
    )


def run_one_scenario(
    config: SamplingTuningConfig,
    scenario: BenchmarkScenario,
    planner_weights: PathObjectiveWeights,
) -> CaseOutcome:
    cache_key = (
        config,
        scenario.name,
        planner_weights,
    )

    cached = (
        _RUN_CACHE.get(
            cache_key
        )
    )

    if cached is not None:
        return cached

    final_lateral_positions = tuple(
        scenario
        .terminal_lateral_target_m
        + offset
        for offset in (
            config.final_lateral_offsets_m
        )
    )

    final_slope_values = tuple(
        scenario
        .terminal_first_derivative_target
        + offset
        for offset in (
            config.final_slope_offsets
        )
    )

    obstacles = list(
        scenario.obstacles
    )

    result = (
        run_sampling_benchmark_adapter(
            reference_line=(
                scenario.reference_line
            ),
            vehicle_state=(
                scenario.vehicle_state
            ),
            vehicle_params=(
                scenario.vehicle_params
            ),
            obstacles=obstacles,
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
            bypass_lateral_positions=(
                config
                .bypass_lateral_positions_m
            ),
            bypass_lengths=(
                config.bypass_lengths_m
            ),
            total_length=(
                scenario.planning_horizon_m
            ),
            final_lateral_positions=(
                final_lateral_positions
            ),
            final_dl_ds_values=(
                final_slope_values
            ),
            final_d2l_ds2=(
                DEFAULT_SAMPLING_PLANNER_CONFIG
                .returning_final_d2l_ds2
            ),
            resolution_m=(
                scenario.resolution_m
            ),
            planner_objective_weights=(
                planner_weights
            ),
        )
    )

    metrics = (
        evaluate_planner_benchmark_metrics(
            result=result,
            vehicle_params=(
                scenario.vehicle_params
            ),
            obstacles=obstacles,
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

    outcome = (
        outcome_from_metrics(
            result=result,
            metrics=metrics,
        )
    )

    _RUN_CACHE[
        cache_key
    ] = outcome

    return outcome


def evaluate_config(
    config: SamplingTuningConfig,
    scenarios: tuple[
        BenchmarkScenario,
        ...,
    ],
    planner_weights: PathObjectiveWeights,
) -> ConfigEvaluation:
    outcomes = {}

    for scenario in scenarios:
        outcomes[
            scenario.name
        ] = (
            run_one_scenario(
                config=config,
                scenario=scenario,
                planner_weights=(
                    planner_weights
                ),
            )
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
        max_coverage_group[
            1:
        ]
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
            for name in (
                common_names
            )
        ]

        if any(
            value is None
            for value in values
        ):
            raise RuntimeError(
                "Common-success scenario "
                "is missing canonical objective."
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
            "Pareto comparison objective "
            "has not been configured."
        )

    candidate_values = (
        candidate.comparison_objective,
        candidate.mean_runtime_ms,
        candidate.config.candidate_count,
    )

    reference_values = (
        reference.comparison_objective,
        reference.mean_runtime_ms,
        reference.config.candidate_count,
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
            evaluation
            .config
            .candidate_count,
            evaluation.mean_runtime_ms,
        )
    )

    return frontier


def mean_optional(
    values: list[
        float | None
    ],
) -> float | None:
    valid_values = [
        value
        for value in values
        if value is not None
    ]

    if not valid_values:
        return None

    return mean(
        valid_values
    )


def min_optional(
    values: list[
        float | None
    ],
) -> float | None:
    valid_values = [
        value
        for value in values
        if value is not None
    ]

    if not valid_values:
        return None

    return min(
        valid_values
    )


def max_optional(
    values: list[
        float | None
    ],
) -> float | None:
    valid_values = [
        value
        for value in values
        if value is not None
    ]

    if not valid_values:
        return None

    return max(
        valid_values
    )


def config_parameters_dict(
    config: SamplingTuningConfig,
) -> dict:
    return {
        "candidate_count": (
            config.candidate_count
        ),
        "bypass_lateral_positions_m": (
            list(
                config
                .bypass_lateral_positions_m
            )
        ),
        "bypass_lengths_m": (
            list(
                config.bypass_lengths_m
            )
        ),
        "final_lateral_offsets_m": (
            list(
                config
                .final_lateral_offsets_m
            )
        ),
        "final_slope_offsets": (
            list(
                config.final_slope_offsets
            )
        ),
    }


def evaluation_summary(
    evaluation: ConfigEvaluation,
) -> dict:
    successful_outcomes = [
        outcome
        for outcome
        in evaluation.outcomes.values()
        if outcome.success
    ]

    return {
        "name": (
            evaluation.config.name
        ),
        **config_parameters_dict(
            evaluation.config
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
                    in successful_outcomes
                ]
            )
        ),
        "minimum_clearance_over_successes_m": (
            min_optional(
                [
                    outcome
                    .minimum_exact_obstacle_clearance_m
                    for outcome
                    in successful_outcomes
                ]
            )
        ),
        "maximum_curvature_over_successes_per_m": (
            max_optional(
                [
                    outcome
                    .max_abs_cartesian_curvature_per_m
                    for outcome
                    in successful_outcomes
                ]
            )
        ),
        "mean_rms_curvature_per_m": (
            mean_optional(
                [
                    outcome
                    .rms_cartesian_curvature_per_m
                    for outcome
                    in successful_outcomes
                ]
            )
        ),
        "mean_rms_curvature_rate_per_m2": (
            mean_optional(
                [
                    outcome
                    .rms_curvature_rate_per_m2
                    for outcome
                    in successful_outcomes
                ]
            )
        ),
    }


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
    print(
        title
    )

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
        "                                    "
        "| cand"
        " | success"
        " | common obj"
        " | mean ms"
    )

    print(
        "-" * 91
    )

    for evaluation in frontier:
        if (
            evaluation.comparison_objective
            is None
        ):
            raise RuntimeError(
                "Missing comparison objective."
            )

        print(
            f"{evaluation.config.name:<41}"
            " | "
            f"{evaluation.config.candidate_count:>4}"
            " | "
            f"{evaluation.success_count:>2}/"
            f"{evaluation.total_count:<2}"
            "   | "
            f"{evaluation.comparison_objective:>10.6f}"
            " | "
            f"{evaluation.mean_runtime_ms:>7.2f}"
        )


def build_candidate_pool(
    config: SamplingTuningConfig,
    scenario: BenchmarkScenario,
) -> list[
    tuple[
        FrenetTrajectory,
        list[PathPoint],
    ]
]:
    cache_key = (
        config,
        scenario.name,
    )

    cached = (
        _CANDIDATE_CACHE.get(
            cache_key
        )
    )

    if cached is not None:
        return cached

    final_lateral_positions = tuple(
        scenario
        .terminal_lateral_target_m
        + offset
        for offset in (
            config.final_lateral_offsets_m
        )
    )

    final_slope_values = tuple(
        scenario
        .terminal_first_derivative_target
        + offset
        for offset in (
            config.final_slope_offsets
        )
    )

    trajectories = (
        generate_returning_frenet_candidates_from_vehicle_state(
            vehicle_state=(
                scenario.vehicle_state
            ),
            reference_line=(
                scenario.reference_line
            ),
            bypass_lateral_positions=(
                config
                .bypass_lateral_positions_m
            ),
            bypass_lengths=(
                config.bypass_lengths_m
            ),
            total_length=(
                scenario.planning_horizon_m
            ),
            final_lateral_positions=(
                final_lateral_positions
            ),
            final_dl_ds_values=(
                final_slope_values
            ),
            final_d2l_ds2=(
                DEFAULT_SAMPLING_PLANNER_CONFIG
                .returning_final_d2l_ds2
            ),
            resolution=(
                scenario.resolution_m
            ),
        )
    )

    paths = [
        frenet_trajectory_to_path(
            trajectory=trajectory,
            reference_line=(
                scenario.reference_line
            ),
        )
        for trajectory in trajectories
    ]

    feasible_candidates = (
        filter_feasible_candidates(
            trajectories=trajectories,
            paths=paths,
            vehicle_params=(
                scenario.vehicle_params
            ),
            obstacles=list(
                scenario.obstacles
            ),
            max_curvature=(
                scenario.max_curvature
            ),
            minimum_clearance_m=(
                scenario.minimum_clearance_m
            ),
            min_l=(
                scenario.road_min_l
            ),
            max_l=(
                scenario.road_max_l
            ),
        )
    )

    _CANDIDATE_CACHE[
        cache_key
    ] = feasible_candidates

    return feasible_candidates


def build_weight_profiles(
) -> tuple[
    tuple[
        str,
        PathObjectiveWeights,
    ],
    ...,
]:
    canonical_weights = (
        DEFAULT_SAMPLING_PLANNER_CONFIG
        .matched_objective_weights
    )

    profiles = [
        (
            "canonical",
            canonical_weights,
        )
    ]

    for field_name in (
        WEIGHT_FIELDS
    ):
        original_value = getattr(
            canonical_weights,
            field_name,
        )

        for factor in (
            WEIGHT_FACTORS
        ):
            weights = replace(
                canonical_weights,
                **{
                    field_name: (
                        original_value
                        * factor
                    )
                },
            )

            profiles.append(
                (
                    (
                        field_name
                        + "_x"
                        + f"{factor:.1f}"
                    ),
                    weights,
                )
            )

    return tuple(
        profiles
    )


def evaluate_cached_weight_profile(
    config: SamplingTuningConfig,
    scenarios: tuple[
        BenchmarkScenario,
        ...,
    ],
    profile_name: str,
    planner_weights: PathObjectiveWeights,
) -> tuple[
    dict,
    list[
        tuple[
            BenchmarkScenario,
            CaseOutcome,
        ]
    ],
]:
    case_results = []

    for scenario in scenarios:
        feasible_candidates = (
            build_candidate_pool(
                config=config,
                scenario=scenario,
            )
        )

        if not feasible_candidates:
            outcome = CaseOutcome(
                success=False,
                planning_time_ms=None,
                failure_reason=(
                    "No physically feasible "
                    "candidate in cached pool."
                ),
                canonical_objective=None,
                terminal_lateral_error_m=None,
                terminal_slope_error=None,
                rms_lateral_deviation_m=None,
                cartesian_path_length_m=None,
                minimum_exact_obstacle_clearance_m=None,
                max_abs_cartesian_curvature_per_m=None,
                rms_cartesian_curvature_per_m=None,
                rms_curvature_rate_per_m2=None,
            )

            case_results.append(
                (
                    scenario,
                    outcome,
                )
            )

            continue

        best_candidate = (
            select_best_matched_candidate(
                feasible_candidates=(
                    feasible_candidates
                ),
                resolution_m=(
                    scenario.resolution_m
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
                    planner_weights
                    .lateral_weight
                ),
                first_derivative_weight=(
                    planner_weights
                    .first_derivative_weight
                ),
                second_derivative_weight=(
                    planner_weights
                    .second_derivative_weight
                ),
                third_derivative_weight=(
                    planner_weights
                    .third_derivative_weight
                ),
                terminal_lateral_weight=(
                    planner_weights
                    .terminal_lateral_weight
                ),
                terminal_first_derivative_weight=(
                    planner_weights
                    .terminal_first_derivative_weight
                ),
            )
        )

        if best_candidate is None:
            raise RuntimeError(
                "Matched Sampling selection "
                "unexpectedly returned None."
            )

        (
            best_trajectory,
            best_path,
            _planner_cost,
        ) = best_candidate

        canonical_evaluation = (
            evaluate_frenet_trajectory_objective(
                trajectory=(
                    best_trajectory
                ),
                resolution_m=(
                    scenario.resolution_m
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
                weights=(
                    canonical_weights_from_scenario(
                        scenario
                    )
                ),
            )
        )

        result = (
            PlannerBenchmarkResult.succeeded(
                planner_name="sampling",
                trajectory=(
                    best_trajectory
                ),
                path=best_path,
                planning_time_ms=0.0,
                matched_objective_value=(
                    canonical_evaluation
                    .total_cost
                ),
            )
        )

        metrics = (
            evaluate_planner_benchmark_metrics(
                result=result,
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

        outcome = (
            outcome_from_metrics(
                result=result,
                metrics=metrics,
            )
        )

        outcome = replace(
            outcome,
            planning_time_ms=None,
        )

        case_results.append(
            (
                scenario,
                outcome,
            )
        )

    successful_outcomes = [
        outcome
        for (
            _scenario,
            outcome,
        ) in case_results
        if outcome.success
    ]

    summary = {
        "config_name": (
            config.name
        ),
        "profile_name": (
            profile_name
        ),
        "weights": asdict(
            planner_weights
        ),
        "success_count": len(
            successful_outcomes
        ),
        "total_count": len(
            case_results
        ),
        "mean_canonical_objective": (
            mean_optional(
                [
                    outcome
                    .canonical_objective
                    for outcome
                    in successful_outcomes
                ]
            )
        ),
        "mean_rms_lateral_deviation_m": (
            mean_optional(
                [
                    outcome
                    .rms_lateral_deviation_m
                    for outcome
                    in successful_outcomes
                ]
            )
        ),
        "minimum_clearance_over_successes_m": (
            min_optional(
                [
                    outcome
                    .minimum_exact_obstacle_clearance_m
                    for outcome
                    in successful_outcomes
                ]
            )
        ),
        "maximum_curvature_over_successes_per_m": (
            max_optional(
                [
                    outcome
                    .max_abs_cartesian_curvature_per_m
                    for outcome
                    in successful_outcomes
                ]
            )
        ),
        "mean_rms_curvature_per_m": (
            mean_optional(
                [
                    outcome
                    .rms_cartesian_curvature_per_m
                    for outcome
                    in successful_outcomes
                ]
            )
        ),
        "mean_rms_curvature_rate_per_m2": (
            mean_optional(
                [
                    outcome
                    .rms_curvature_rate_per_m2
                    for outcome
                    in successful_outcomes
                ]
            )
        ),
    }

    return (
        summary,
        case_results,
    )


def append_case_rows(
    rows: list[dict],
    stage: str,
    config: SamplingTuningConfig,
    evaluation: ConfigEvaluation,
    weight_profile: str,
) -> None:
    for (
        scenario_name,
        outcome,
    ) in evaluation.outcomes.items():
        rows.append(
            {
                "stage": stage,
                "run_type": (
                    "full_planner"
                ),
                "config_name": (
                    config.name
                ),
                "weight_profile": (
                    weight_profile
                ),
                "candidate_count": (
                    config.candidate_count
                ),
                "bypass_lateral_positions_m": (
                    repr(
                        config
                        .bypass_lateral_positions_m
                    )
                ),
                "bypass_lengths_m": (
                    repr(
                        config
                        .bypass_lengths_m
                    )
                ),
                "final_lateral_offsets_m": (
                    repr(
                        config
                        .final_lateral_offsets_m
                    )
                ),
                "final_slope_offsets": (
                    repr(
                        config
                        .final_slope_offsets
                    )
                ),
                "scenario_name": (
                    scenario_name
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
                "failure_reason": (
                    outcome.failure_reason
                ),
            }
        )


def append_weight_case_rows(
    rows: list[dict],
    config: SamplingTuningConfig,
    profile_name: str,
    case_results: list[
        tuple[
            BenchmarkScenario,
            CaseOutcome,
        ]
    ],
) -> None:
    for (
        scenario,
        outcome,
    ) in case_results:
        rows.append(
            {
                "stage": (
                    "C_weight_sensitivity"
                ),
                "run_type": (
                    "cached_candidate_selection"
                ),
                "config_name": (
                    config.name
                ),
                "weight_profile": (
                    profile_name
                ),
                "candidate_count": (
                    config.candidate_count
                ),
                "bypass_lateral_positions_m": (
                    repr(
                        config
                        .bypass_lateral_positions_m
                    )
                ),
                "bypass_lengths_m": (
                    repr(
                        config
                        .bypass_lengths_m
                    )
                ),
                "final_lateral_offsets_m": (
                    repr(
                        config
                        .final_lateral_offsets_m
                    )
                ),
                "final_slope_offsets": (
                    repr(
                        config
                        .final_slope_offsets
                    )
                ),
                "scenario_name": (
                    scenario.name
                ),
                "success": (
                    outcome.success
                ),
                "planning_time_ms": None,
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
                "failure_reason": (
                    outcome.failure_reason
                ),
            }
        )


def write_results_csv(
    rows: list[dict],
) -> None:
    OUTPUT_DIR.mkdir(
        parents=True,
        exist_ok=True,
    )

    fieldnames = (
        "stage",
        "run_type",
        "config_name",
        "weight_profile",
        "candidate_count",
        "bypass_lateral_positions_m",
        "bypass_lengths_m",
        "final_lateral_offsets_m",
        "final_slope_offsets",
        "scenario_name",
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


def write_summary_json(
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

    canonical_planner_weights = (
        DEFAULT_SAMPLING_PLANNER_CONFIG
        .matched_objective_weights
    )

    csv_rows = []

    summary = {
        "metadata": {
            "raw_development_scenarios": (
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
            "max_terminal_candidates_guard": (
                MAX_TERMINAL_CANDIDATES
            ),
        },
        "initial_state_invalid_scenarios": (
            invalid_initial_states
        ),
    }

    print(
        "Sampling Planner experiment runner"
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

    # ================================================================
    # Stage A
    # Full structural sweep
    # ================================================================

    print()
    print(
        "Stage A: full structural sweep"
    )

    structure_configs = (
        build_structure_configs()
    )

    stage_a_evaluations = []

    for (
        index,
        config,
    ) in enumerate(
        structure_configs,
        start=1,
    ):
        print(
            "  "
            f"[{index}/"
            f"{len(structure_configs)}] "
            f"{config.name} "
            f"({config.candidate_count} candidates)"
        )

        evaluation = (
            evaluate_config(
                config=config,
                scenarios=(
                    active_scenarios
                ),
                planner_weights=(
                    canonical_planner_weights
                ),
            )
        )

        stage_a_evaluations.append(
            evaluation
        )

        append_case_rows(
            rows=csv_rows,
            stage="A_structure",
            config=config,
            evaluation=evaluation,
            weight_profile="canonical",
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
        title=(
            "Stage A result"
        ),
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
        "stage_a_structure"
    ] = {
        "max_success_count": (
            stage_a_max_success
        ),
        "total_active_scenarios": (
            len(
                active_scenarios
            )
        ),
        "common_comparison_scenarios": sorted(
            stage_a_common_names
        ),
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

    # ================================================================
    # Stage B
    # Terminal-state density sweep only around the mathematical
    # Stage-A Pareto frontier.
    # ================================================================

    print()
    print(
        "Stage B: terminal-state sweep "
        "on Stage-A Pareto configs"
    )

    terminal_configs = []

    for evaluation in (
        stage_a_pareto
    ):
        terminal_configs.extend(
            build_terminal_variants(
                evaluation.config
            )
        )

    terminal_configs = (
        unique_configs(
            terminal_configs
        )
    )

    stage_b_evaluations = []

    for (
        index,
        config,
    ) in enumerate(
        terminal_configs,
        start=1,
    ):
        print(
            "  "
            f"[{index}/"
            f"{len(terminal_configs)}] "
            f"{config.name} "
            f"({config.candidate_count} candidates)"
        )

        evaluation = (
            evaluate_config(
                config=config,
                scenarios=(
                    active_scenarios
                ),
                planner_weights=(
                    canonical_planner_weights
                ),
            )
        )

        stage_b_evaluations.append(
            evaluation
        )

        append_case_rows(
            rows=csv_rows,
            stage="B_terminal",
            config=config,
            evaluation=evaluation,
            weight_profile="canonical",
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
        title=(
            "Stage B result"
        ),
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
        "stage_b_terminal"
    ] = {
        "max_success_count": (
            stage_b_max_success
        ),
        "total_active_scenarios": (
            len(
                active_scenarios
            )
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

    # ================================================================
    # Stage C
    # Weight sensitivity.
    #
    # Candidate generation / physical validation are cached.
    # Only candidate selection changes when weights change.
    # ================================================================

    print()
    print(
        "Stage C: objective-weight sensitivity "
        "on Stage-B Pareto configs"
    )

    weight_profiles = (
        build_weight_profiles()
    )

    stage_c_summaries = []

    for (
        config_index,
        evaluation,
    ) in enumerate(
        stage_b_pareto,
        start=1,
    ):
        config = (
            evaluation.config
        )

        print()
        print(
            "  Config "
            f"[{config_index}/"
            f"{len(stage_b_pareto)}]: "
            f"{config.name}"
        )

        for (
            profile_index,
            (
                profile_name,
                planner_weights,
            ),
        ) in enumerate(
            weight_profiles,
            start=1,
        ):
            print(
                "    "
                f"[{profile_index}/"
                f"{len(weight_profiles)}] "
                f"{profile_name}"
            )

            (
                profile_summary,
                case_results,
            ) = (
                evaluate_cached_weight_profile(
                    config=config,
                    scenarios=(
                        active_scenarios
                    ),
                    profile_name=(
                        profile_name
                    ),
                    planner_weights=(
                        planner_weights
                    ),
                )
            )

            stage_c_summaries.append(
                profile_summary
            )

            append_weight_case_rows(
                rows=csv_rows,
                config=config,
                profile_name=(
                    profile_name
                ),
                case_results=(
                    case_results
                ),
            )

    summary[
        "stage_c_weight_sensitivity"
    ] = {
        "note": (
            "No automatic recommendation is made. "
            "Candidate generation is fixed and cached; "
            "only candidate selection changes with weights."
        ),
        "profiles": (
            stage_c_summaries
        ),
    }

    # ================================================================
    # Save complete experiment data
    # ================================================================

    write_results_csv(
        csv_rows
    )

    write_summary_json(
        summary
    )

    print()
    print(
        "Sampling tuning experiment completed."
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