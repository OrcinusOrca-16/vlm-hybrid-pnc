from __future__ import annotations

import argparse
import csv
import json

from dataclasses import asdict
from pathlib import Path
from statistics import mean

from scripts.planning import run_sampling_tuning as legacy
from src.benchmark.scenario import BenchmarkScenario
from src.common.types import FrenetTrajectory, PathPoint
from src.config.planning import DEFAULT_SAMPLING_PLANNER_CONFIG
from src.planning.frenet import frenet_trajectory_to_path
from src.planning.hard_constraints import (
    is_curvature_feasible,
    is_obstacle_clearance_feasible,
    is_road_boundary_feasible,
)
from src.planning.path_objective import PathObjectiveWeights
from src.planning.sampling_planner import (
    generate_returning_frenet_candidates_from_vehicle_state,
)


OUTPUT_DIR = Path("outputs/tuning")
GUIDED_RESULT_CSV_PATH = OUTPUT_DIR / "sampling_tuning_guided_results.csv"
GUIDED_SUMMARY_JSON_PATH = OUTPUT_DIR / "sampling_tuning_guided_summary.json"


class CandidateDiagnostics:
    def __init__(
        self,
        generated_candidate_count: int,
        feasible_candidate_count: int,
        curvature_reject_count: int,
        clearance_reject_count: int,
        road_boundary_reject_count: int,
    ) -> None:
        self.generated_candidate_count = generated_candidate_count
        self.feasible_candidate_count = feasible_candidate_count
        self.curvature_reject_count = curvature_reject_count
        self.clearance_reject_count = clearance_reject_count
        self.road_boundary_reject_count = road_boundary_reject_count

    @property
    def feasible_ratio(self) -> float:
        if self.generated_candidate_count == 0:
            return 0.0
        return self.feasible_candidate_count / self.generated_candidate_count

    def to_dict(self) -> dict:
        return {
            "generated_candidate_count": self.generated_candidate_count,
            "feasible_candidate_count": self.feasible_candidate_count,
            "feasible_ratio": self.feasible_ratio,
            "curvature_reject_count": self.curvature_reject_count,
            "clearance_reject_count": self.clearance_reject_count,
            "road_boundary_reject_count": self.road_boundary_reject_count,
        }


_DIAGNOSTIC_CACHE: dict[
    tuple[legacy.SamplingTuningConfig, str],
    CandidateDiagnostics,
] = {}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Run agent-facing Sampling Planner tuning stages with "
            "candidate-level feasibility diagnostics."
        )
    )
    parser.add_argument(
        "--mode",
        choices=("structure", "strict"),
        default="structure",
        help=(
            "structure: run Stage A only for diagnosis; "
            "strict: keep strict terminal states and continue with "
            "weight sensitivity on the Stage-A Pareto configs."
        ),
    )
    return parser.parse_args()


def candidate_diagnostics(
    config: legacy.SamplingTuningConfig,
    scenario: BenchmarkScenario,
) -> CandidateDiagnostics:
    cache_key = (config, scenario.name)
    cached = _DIAGNOSTIC_CACHE.get(cache_key)
    if cached is not None:
        return cached

    final_lateral_positions = tuple(
        scenario.terminal_lateral_target_m + offset
        for offset in config.final_lateral_offsets_m
    )
    final_slope_values = tuple(
        scenario.terminal_first_derivative_target + offset
        for offset in config.final_slope_offsets
    )

    trajectories = generate_returning_frenet_candidates_from_vehicle_state(
        vehicle_state=scenario.vehicle_state,
        reference_line=scenario.reference_line,
        bypass_lateral_positions=config.bypass_lateral_positions_m,
        bypass_lengths=config.bypass_lengths_m,
        total_length=scenario.planning_horizon_m,
        final_lateral_positions=final_lateral_positions,
        final_dl_ds_values=final_slope_values,
        final_d2l_ds2=(
            DEFAULT_SAMPLING_PLANNER_CONFIG.returning_final_d2l_ds2
        ),
        resolution=scenario.resolution_m,
    )

    paths = [
        frenet_trajectory_to_path(
            trajectory=trajectory,
            reference_line=scenario.reference_line,
        )
        for trajectory in trajectories
    ]

    feasible_count = 0
    curvature_reject_count = 0
    clearance_reject_count = 0
    road_boundary_reject_count = 0

    for trajectory, path in zip(trajectories, paths):
        # Match the exact hard-constraint order used by
        # is_physical_path_feasible(): curvature -> clearance -> road.
        if not is_curvature_feasible(
            path=path,
            max_curvature=scenario.max_curvature,
        ):
            curvature_reject_count += 1
            continue

        if not is_obstacle_clearance_feasible(
            path=path,
            vehicle_params=scenario.vehicle_params,
            obstacles=list(scenario.obstacles),
            minimum_clearance_m=scenario.minimum_clearance_m,
        ):
            clearance_reject_count += 1
            continue

        if not is_road_boundary_feasible(
            trajectory=trajectory,
            vehicle_params=scenario.vehicle_params,
            min_l=scenario.road_min_l,
            max_l=scenario.road_max_l,
        ):
            road_boundary_reject_count += 1
            continue

        feasible_count += 1

    diagnostics = CandidateDiagnostics(
        generated_candidate_count=len(trajectories),
        feasible_candidate_count=feasible_count,
        curvature_reject_count=curvature_reject_count,
        clearance_reject_count=clearance_reject_count,
        road_boundary_reject_count=road_boundary_reject_count,
    )

    classified_count = (
        diagnostics.feasible_candidate_count
        + diagnostics.curvature_reject_count
        + diagnostics.clearance_reject_count
        + diagnostics.road_boundary_reject_count
    )
    if classified_count != diagnostics.generated_candidate_count:
        raise RuntimeError(
            "Candidate diagnostic counts do not sum to the generated "
            "candidate count."
        )

    _DIAGNOSTIC_CACHE[cache_key] = diagnostics
    return diagnostics


def aggregate_candidate_diagnostics(
    config: legacy.SamplingTuningConfig,
    scenarios: tuple[BenchmarkScenario, ...],
) -> dict:
    case_diagnostics = {
        scenario.name: candidate_diagnostics(config, scenario)
        for scenario in scenarios
    }

    generated = sum(
        item.generated_candidate_count
        for item in case_diagnostics.values()
    )
    feasible = sum(
        item.feasible_candidate_count
        for item in case_diagnostics.values()
    )

    return {
        "generated_candidate_total": generated,
        "feasible_candidate_total": feasible,
        "feasible_ratio": 0.0 if generated == 0 else feasible / generated,
        "curvature_reject_total": sum(
            item.curvature_reject_count
            for item in case_diagnostics.values()
        ),
        "clearance_reject_total": sum(
            item.clearance_reject_count
            for item in case_diagnostics.values()
        ),
        "road_boundary_reject_total": sum(
            item.road_boundary_reject_count
            for item in case_diagnostics.values()
        ),
        "cases": {
            name: diagnostics.to_dict()
            for name, diagnostics in case_diagnostics.items()
        },
    }


def run_structure_stage(
    active_scenarios: tuple[BenchmarkScenario, ...],
    planner_weights: PathObjectiveWeights,
) -> tuple[
    list[legacy.ConfigEvaluation],
    list[legacy.ConfigEvaluation],
    set[str],
    dict,
]:
    evaluations = []
    diagnostics_by_config = {}

    configs = legacy.build_structure_configs()
    print("Stage A: structural sweep with candidate diagnostics")

    for index, config in enumerate(configs, start=1):
        print(
            f"  [{index}/{len(configs)}] {config.name} "
            f"({config.candidate_count} candidates)"
        )
        evaluation = legacy.evaluate_config(
            config=config,
            scenarios=active_scenarios,
            planner_weights=planner_weights,
        )
        evaluations.append(evaluation)
        diagnostics_by_config[config.name] = aggregate_candidate_diagnostics(
            config=config,
            scenarios=active_scenarios,
        )

    (
        max_success,
        common_names,
        max_coverage_group,
    ) = legacy.configure_common_objective(evaluations)
    frontier = legacy.pareto_frontier(max_coverage_group)

    legacy.print_stage_result(
        title="Stage A result",
        evaluations=evaluations,
        max_success=max_success,
        common_names=common_names,
        frontier=frontier,
    )

    summary = {
        "max_success_count": max_success,
        "total_active_scenarios": len(active_scenarios),
        "common_comparison_scenarios": sorted(common_names),
        "pareto_config_names": [
            evaluation.config.name
            for evaluation in frontier
        ],
        "configs": [
            {
                **legacy.evaluation_summary(evaluation),
                **diagnostics_by_config[evaluation.config.name],
            }
            for evaluation in evaluations
        ],
    }

    return evaluations, frontier, common_names, summary


def run_strict_weight_stage(
    stage_a_pareto: list[legacy.ConfigEvaluation],
    active_scenarios: tuple[BenchmarkScenario, ...],
) -> dict:
    print()
    print(
        "Stage C: objective-weight sensitivity with strict terminal states"
    )

    profiles = legacy.build_weight_profiles()
    summaries = []

    for config_index, evaluation in enumerate(stage_a_pareto, start=1):
        config = evaluation.config
        print(
            f"  Config [{config_index}/{len(stage_a_pareto)}]: "
            f"{config.name}"
        )
        for profile_index, (profile_name, weights) in enumerate(
            profiles,
            start=1,
        ):
            print(
                f"    [{profile_index}/{len(profiles)}] {profile_name}"
            )
            profile_summary, _case_results = (
                legacy.evaluate_cached_weight_profile(
                    config=config,
                    scenarios=active_scenarios,
                    profile_name=profile_name,
                    planner_weights=weights,
                )
            )
            summaries.append(profile_summary)

    return {
        "note": (
            "Strict terminal states retained after Stage-A diagnostics. "
            "Candidate generation is fixed/cached during weight sensitivity; "
            "only candidate selection changes with weights."
        ),
        "profiles": summaries,
    }


def write_guided_outputs(
    summary: dict,
    stage_a_evaluations: list[legacy.ConfigEvaluation],
    active_scenarios: tuple[BenchmarkScenario, ...],
) -> None:
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    with GUIDED_SUMMARY_JSON_PATH.open(
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

    rows = []
    scenario_by_name = {
        scenario.name: scenario
        for scenario in active_scenarios
    }

    for evaluation in stage_a_evaluations:
        config = evaluation.config
        for scenario_name, outcome in evaluation.outcomes.items():
            diagnostics = candidate_diagnostics(
                config=config,
                scenario=scenario_by_name[scenario_name],
            )
            rows.append(
                {
                    "config_name": config.name,
                    "candidate_count": config.candidate_count,
                    "scenario_name": scenario_name,
                    "success": outcome.success,
                    "canonical_objective": outcome.canonical_objective,
                    "planning_time_ms": outcome.planning_time_ms,
                    "failure_reason": outcome.failure_reason,
                    **diagnostics.to_dict(),
                }
            )

    fieldnames = (
        "config_name",
        "candidate_count",
        "scenario_name",
        "success",
        "canonical_objective",
        "planning_time_ms",
        "failure_reason",
        "generated_candidate_count",
        "feasible_candidate_count",
        "feasible_ratio",
        "curvature_reject_count",
        "clearance_reject_count",
        "road_boundary_reject_count",
    )

    with GUIDED_RESULT_CSV_PATH.open(
        "w",
        newline="",
        encoding="utf-8",
    ) as file:
        writer = csv.DictWriter(file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()

    (
        all_scenarios,
        active_scenarios,
        invalid_initial_states,
    ) = legacy.prepare_scenarios()

    planner_weights = (
        DEFAULT_SAMPLING_PLANNER_CONFIG.matched_objective_weights
    )

    summary = {
        "metadata": {
            "runner": "guided",
            "mode": args.mode,
            "raw_development_scenarios": len(all_scenarios),
            "active_tuning_scenarios": len(active_scenarios),
            "held_out_evaluation_scenarios": (
                "NOT IMPORTED / NOT EXECUTED"
            ),
        },
        "initial_state_invalid_scenarios": invalid_initial_states,
    }

    print("Sampling Planner guided tuning runner")
    print(f"  mode={args.mode}")
    print(f"  development scenarios={len(all_scenarios)}")
    print(f"  active tuning scenarios={len(active_scenarios)}")
    print("  held-out evaluation scenarios=NOT IMPORTED / NOT EXECUTED")

    (
        stage_a_evaluations,
        stage_a_pareto,
        _stage_a_common_names,
        stage_a_summary,
    ) = run_structure_stage(
        active_scenarios=active_scenarios,
        planner_weights=planner_weights,
    )
    summary["stage_a_structure"] = stage_a_summary

    if args.mode == "strict":
        summary["stage_c_weight_sensitivity"] = run_strict_weight_stage(
            stage_a_pareto=stage_a_pareto,
            active_scenarios=active_scenarios,
        )
    else:
        summary["next_step"] = {
            "decision": (
                "Inspect feasible ratio, pruning composition, and failure "
                "cases. Keep strict terminal states unless evidence suggests "
                "that strict rejoin conditions are limiting feasibility."
            ),
            "strict_command": (
                "python -m scripts.planning.run_sampling_tuning_guided "
                "--mode strict"
            ),
            "expanded_terminal_command": (
                "python -m scripts.planning.run_sampling_tuning"
            ),
        }

    write_guided_outputs(
        summary=summary,
        stage_a_evaluations=stage_a_evaluations,
        active_scenarios=active_scenarios,
    )

    print()
    print("Guided Sampling tuning stage completed.")
    print(f"  CSV: {GUIDED_RESULT_CSV_PATH}")
    print(f"  JSON: {GUIDED_SUMMARY_JSON_PATH}")
    print("  No config was automatically recommended.")
    print("  Held-out evaluation scenarios were never imported or executed.")


if __name__ == "__main__":
    main()
