from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if old not in text:
        raise RuntimeError(f"Patch anchor not found: {label}")
    return text.replace(old, new, 1)


script_path = Path("scripts/planning/run_sampling_tuning.py")
text = script_path.read_text(encoding="utf-8")

text = replace_once(
    text,
    "from __future__ import annotations\n\nimport csv\nimport json\n",
    "from __future__ import annotations\n\nimport argparse\nimport csv\nimport json\n",
    "argparse import",
)

text = replace_once(
    text,
    "from src.planning.hard_constraints import (\n    filter_feasible_candidates,\n    obstacle_clearance,\n)\n",
    "from src.planning.hard_constraints import (\n    filter_feasible_candidates,\n    is_curvature_feasible,\n    is_obstacle_clearance_feasible,\n    is_road_boundary_feasible,\n    obstacle_clearance,\n)\n",
    "hard-constraint diagnostic imports",
)

text = replace_once(
    text,
    "\n\n@dataclass(frozen=True)\nclass CaseOutcome:\n",
    "\n\n@dataclass(frozen=True)\nclass CandidateDiagnostics:\n    generated_candidate_count: int\n    feasible_candidate_count: int\n    curvature_reject_count: int\n    clearance_reject_count: int\n    road_boundary_reject_count: int\n\n    @property\n    def feasible_ratio(self) -> float:\n        if self.generated_candidate_count <= 0:\n            return 0.0\n\n        return (\n            self.feasible_candidate_count\n            / self.generated_candidate_count\n        )\n\n\n@dataclass(frozen=True)\nclass CaseOutcome:\n",
    "CandidateDiagnostics dataclass",
)

text = replace_once(
    text,
    "    rms_curvature_rate_per_m2: (\n        float\n        | None\n    )\n\n\n@dataclass\nclass ConfigEvaluation:\n",
    "    rms_curvature_rate_per_m2: (\n        float\n        | None\n    )\n\n    generated_candidate_count: (\n        int\n        | None\n    ) = None\n\n    feasible_candidate_count: (\n        int\n        | None\n    ) = None\n\n    feasible_ratio: (\n        float\n        | None\n    ) = None\n\n    curvature_reject_count: (\n        int\n        | None\n    ) = None\n\n    clearance_reject_count: (\n        int\n        | None\n    ) = None\n\n    road_boundary_reject_count: (\n        int\n        | None\n    ) = None\n\n\n@dataclass\nclass ConfigEvaluation:\n",
    "CaseOutcome diagnostic fields",
)

text = replace_once(
    text,
    "        return mean(\n            values\n        )\n\n\n_RUN_CACHE: dict[\n",
    "        return mean(\n            values\n        )\n\n    @property\n    def mean_feasible_ratio(\n        self,\n    ) -> float | None:\n        values = [\n            outcome.feasible_ratio\n            for outcome\n            in self.outcomes.values()\n            if outcome.feasible_ratio is not None\n        ]\n\n        if not values:\n            return None\n\n        return mean(\n            values\n        )\n\n\n_RUN_CACHE: dict[\n",
    "ConfigEvaluation mean feasible ratio",
)

text = replace_once(
    text,
    "] = {}\n\n\ndef symmetric_samples(\n",
    "] = {}\n\n\n_CANDIDATE_DIAGNOSTIC_CACHE: dict[\n    tuple[\n        SamplingTuningConfig,\n        str,\n    ],\n    CandidateDiagnostics,\n] = {}\n\n\ndef symmetric_samples(\n",
    "candidate diagnostic cache",
)

text = replace_once(
    text,
    "    return CaseOutcome(\n        success=result.success,\n",
    "    return CaseOutcome(\n        success=result.success,\n",
    "outcome anchor check",
)

text = replace_once(
    text,
    "    )\n\n\ndef run_one_scenario(\n    config: SamplingTuningConfig,\n",
    "    )\n\n\ndef attach_candidate_diagnostics(\n    outcome: CaseOutcome,\n    diagnostics: CandidateDiagnostics,\n) -> CaseOutcome:\n    return replace(\n        outcome,\n        generated_candidate_count=(\n            diagnostics.generated_candidate_count\n        ),\n        feasible_candidate_count=(\n            diagnostics.feasible_candidate_count\n        ),\n        feasible_ratio=(\n            diagnostics.feasible_ratio\n        ),\n        curvature_reject_count=(\n            diagnostics.curvature_reject_count\n        ),\n        clearance_reject_count=(\n            diagnostics.clearance_reject_count\n        ),\n        road_boundary_reject_count=(\n            diagnostics.road_boundary_reject_count\n        ),\n    )\n\n\ndef run_one_scenario(\n    config: SamplingTuningConfig,\n",
    "attach diagnostics helper",
)

text = replace_once(
    text,
    "    outcome = (\n        outcome_from_metrics(\n            result=result,\n            metrics=metrics,\n        )\n    )\n\n    _RUN_CACHE[\n",
    "    outcome = (\n        outcome_from_metrics(\n            result=result,\n            metrics=metrics,\n        )\n    )\n\n    diagnostics = (\n        get_candidate_diagnostics(\n            config=config,\n            scenario=scenario,\n        )\n    )\n\n    outcome = (\n        attach_candidate_diagnostics(\n            outcome=outcome,\n            diagnostics=diagnostics,\n        )\n    )\n\n    _RUN_CACHE[\n",
    "run_one_scenario diagnostics",
)

text = replace_once(
    text,
    "        \"comparison_objective\": (\n            evaluation\n            .comparison_objective\n        ),\n        \"mean_rms_lateral_deviation_m\": (\n",
    "        \"comparison_objective\": (\n            evaluation\n            .comparison_objective\n        ),\n        \"mean_feasible_candidate_count\": (\n            mean_optional(\n                [\n                    (\n                        float(\n                            outcome\n                            .feasible_candidate_count\n                        )\n                        if outcome.feasible_candidate_count\n                        is not None\n                        else None\n                    )\n                    for outcome\n                    in evaluation.outcomes.values()\n                ]\n            )\n        ),\n        \"mean_feasible_ratio\": (\n            evaluation.mean_feasible_ratio\n        ),\n        \"total_generated_candidate_count\": sum(\n            outcome.generated_candidate_count or 0\n            for outcome\n            in evaluation.outcomes.values()\n        ),\n        \"total_feasible_candidate_count\": sum(\n            outcome.feasible_candidate_count or 0\n            for outcome\n            in evaluation.outcomes.values()\n        ),\n        \"total_curvature_reject_count\": sum(\n            outcome.curvature_reject_count or 0\n            for outcome\n            in evaluation.outcomes.values()\n        ),\n        \"total_clearance_reject_count\": sum(\n            outcome.clearance_reject_count or 0\n            for outcome\n            in evaluation.outcomes.values()\n        ),\n        \"total_road_boundary_reject_count\": sum(\n            outcome.road_boundary_reject_count or 0\n            for outcome\n            in evaluation.outcomes.values()\n        ),\n        \"mean_rms_lateral_deviation_m\": (\n",
    "evaluation summary diagnostics",
)

text = replace_once(
    text,
    "\n\ndef build_candidate_pool(\n    config: SamplingTuningConfig,\n",
    "\n\ndef compute_candidate_diagnostics(\n    trajectories: list[FrenetTrajectory],\n    paths: list[list[PathPoint]],\n    scenario: BenchmarkScenario,\n) -> CandidateDiagnostics:\n    if len(trajectories) != len(paths):\n        raise ValueError(\n            \"trajectories and paths must have the same length.\"\n        )\n\n    curvature_reject_count = 0\n    clearance_reject_count = 0\n    road_boundary_reject_count = 0\n    feasible_candidate_count = 0\n\n    for (\n        trajectory,\n        path,\n    ) in zip(\n        trajectories,\n        paths,\n    ):\n        if not is_curvature_feasible(\n            path=path,\n            max_curvature=(\n                scenario.max_curvature\n            ),\n        ):\n            curvature_reject_count += 1\n            continue\n\n        if not is_obstacle_clearance_feasible(\n            path=path,\n            vehicle_params=(\n                scenario.vehicle_params\n            ),\n            obstacles=list(\n                scenario.obstacles\n            ),\n            minimum_clearance_m=(\n                scenario.minimum_clearance_m\n            ),\n        ):\n            clearance_reject_count += 1\n            continue\n\n        if not is_road_boundary_feasible(\n            trajectory=trajectory,\n            vehicle_params=(\n                scenario.vehicle_params\n            ),\n            min_l=(\n                scenario.road_min_l\n            ),\n            max_l=(\n                scenario.road_max_l\n            ),\n        ):\n            road_boundary_reject_count += 1\n            continue\n\n        feasible_candidate_count += 1\n\n    diagnostics = CandidateDiagnostics(\n        generated_candidate_count=len(\n            trajectories\n        ),\n        feasible_candidate_count=(\n            feasible_candidate_count\n        ),\n        curvature_reject_count=(\n            curvature_reject_count\n        ),\n        clearance_reject_count=(\n            clearance_reject_count\n        ),\n        road_boundary_reject_count=(\n            road_boundary_reject_count\n        ),\n    )\n\n    classified_count = (\n        diagnostics.feasible_candidate_count\n        + diagnostics.curvature_reject_count\n        + diagnostics.clearance_reject_count\n        + diagnostics.road_boundary_reject_count\n    )\n\n    if (\n        classified_count\n        != diagnostics.generated_candidate_count\n    ):\n        raise RuntimeError(\n            \"Candidate diagnostic counts do not sum to generated count.\"\n        )\n\n    return diagnostics\n\n\ndef build_candidate_pool(\n    config: SamplingTuningConfig,\n",
    "candidate diagnostic classifier",
)

text = replace_once(
    text,
    "    feasible_candidates = (\n        filter_feasible_candidates(\n            trajectories=trajectories,\n            paths=paths,\n            vehicle_params=(\n                scenario.vehicle_params\n            ),\n            obstacles=list(\n                scenario.obstacles\n            ),\n            max_curvature=(\n                scenario.max_curvature\n            ),\n            minimum_clearance_m=(\n                scenario.minimum_clearance_m\n            ),\n            min_l=(\n                scenario.road_min_l\n            ),\n            max_l=(\n                scenario.road_max_l\n            ),\n        )\n    )\n\n    _CANDIDATE_CACHE[\n        cache_key\n    ] = feasible_candidates\n\n    return feasible_candidates\n\n\ndef build_weight_profiles(\n",
    "    feasible_candidates = (\n        filter_feasible_candidates(\n            trajectories=trajectories,\n            paths=paths,\n            vehicle_params=(\n                scenario.vehicle_params\n            ),\n            obstacles=list(\n                scenario.obstacles\n            ),\n            max_curvature=(\n                scenario.max_curvature\n            ),\n            minimum_clearance_m=(\n                scenario.minimum_clearance_m\n            ),\n            min_l=(\n                scenario.road_min_l\n            ),\n            max_l=(\n                scenario.road_max_l\n            ),\n        )\n    )\n\n    diagnostics = (\n        compute_candidate_diagnostics(\n            trajectories=trajectories,\n            paths=paths,\n            scenario=scenario,\n        )\n    )\n\n    if (\n        diagnostics.feasible_candidate_count\n        != len(feasible_candidates)\n    ):\n        raise RuntimeError(\n            \"Candidate diagnostics disagree with physical validator.\"\n        )\n\n    _CANDIDATE_CACHE[\n        cache_key\n    ] = feasible_candidates\n\n    _CANDIDATE_DIAGNOSTIC_CACHE[\n        cache_key\n    ] = diagnostics\n\n    return feasible_candidates\n\n\ndef get_candidate_diagnostics(\n    config: SamplingTuningConfig,\n    scenario: BenchmarkScenario,\n) -> CandidateDiagnostics:\n    cache_key = (\n        config,\n        scenario.name,\n    )\n\n    build_candidate_pool(\n        config=config,\n        scenario=scenario,\n    )\n\n    diagnostics = (\n        _CANDIDATE_DIAGNOSTIC_CACHE.get(\n            cache_key\n        )\n    )\n\n    if diagnostics is None:\n        raise RuntimeError(\n            \"Candidate diagnostics missing after pool construction.\"\n        )\n\n    return diagnostics\n\n\ndef build_weight_profiles(\n",
    "candidate pool diagnostics",
)

text = replace_once(
    text,
    "        feasible_candidates = (\n            build_candidate_pool(\n                config=config,\n                scenario=scenario,\n            )\n        )\n\n        if not feasible_candidates:\n",
    "        feasible_candidates = (\n            build_candidate_pool(\n                config=config,\n                scenario=scenario,\n            )\n        )\n\n        diagnostics = (\n            get_candidate_diagnostics(\n                config=config,\n                scenario=scenario,\n            )\n        )\n\n        if not feasible_candidates:\n",
    "cached weight diagnostics lookup",
)

text = replace_once(
    text,
    "            case_results.append(\n                (\n                    scenario,\n                    outcome,\n                )\n            )\n\n            continue\n",
    "            outcome = (\n                attach_candidate_diagnostics(\n                    outcome=outcome,\n                    diagnostics=diagnostics,\n                )\n            )\n\n            case_results.append(\n                (\n                    scenario,\n                    outcome,\n                )\n            )\n\n            continue\n",
    "failed cached weight outcome diagnostics",
)

text = replace_once(
    text,
    "        outcome = replace(\n            outcome,\n            planning_time_ms=None,\n        )\n\n        case_results.append(\n",
    "        outcome = replace(\n            outcome,\n            planning_time_ms=None,\n        )\n\n        outcome = (\n            attach_candidate_diagnostics(\n                outcome=outcome,\n                diagnostics=diagnostics,\n            )\n        )\n\n        case_results.append(\n",
    "successful cached weight outcome diagnostics",
)

for row_anchor in (
    "                \"success\": (\n                    outcome.success\n                ),\n                \"planning_time_ms\": (\n                    outcome.planning_time_ms\n                ),\n",
    "                \"success\": (\n                    outcome.success\n                ),\n                \"planning_time_ms\": None,\n",
):
    replacement = row_anchor.replace(
        "                \"planning_time_ms\"",
        "                \"generated_candidate_count\": (\n                    outcome.generated_candidate_count\n                ),\n                \"feasible_candidate_count\": (\n                    outcome.feasible_candidate_count\n                ),\n                \"feasible_ratio\": (\n                    outcome.feasible_ratio\n                ),\n                \"curvature_reject_count\": (\n                    outcome.curvature_reject_count\n                ),\n                \"clearance_reject_count\": (\n                    outcome.clearance_reject_count\n                ),\n                \"road_boundary_reject_count\": (\n                    outcome.road_boundary_reject_count\n                ),\n                \"planning_time_ms\"",
        1,
    )
    text = replace_once(
        text,
        row_anchor,
        replacement,
        "CSV diagnostic columns",
    )

text = replace_once(
    text,
    "        \"success\",\n        \"planning_time_ms\",\n",
    "        \"success\",\n        \"generated_candidate_count\",\n        \"feasible_candidate_count\",\n        \"feasible_ratio\",\n        \"curvature_reject_count\",\n        \"clearance_reject_count\",\n        \"road_boundary_reject_count\",\n        \"planning_time_ms\",\n",
    "CSV fieldnames diagnostics",
)

text = replace_once(
    text,
    "\ndef main() -> None:\n    (\n        all_scenarios,\n",
    "\ndef parse_args() -> argparse.Namespace:\n    parser = argparse.ArgumentParser(\n        description=(\n            \"Run Sampling Planner tuning experiments.\"\n        )\n    )\n\n    parser.add_argument(\n        \"--stop-after-structure\",\n        action=\"store_true\",\n        help=(\n            \"Run Stage A only, write diagnostics, and stop.\"\n        ),\n    )\n\n    parser.add_argument(\n        \"--skip-terminal\",\n        action=\"store_true\",\n        help=(\n            \"Skip terminal-state expansion and carry Stage-A Pareto configs into weight sensitivity.\"\n        ),\n    )\n\n    return parser.parse_args()\n\n\ndef main() -> None:\n    args = parse_args()\n\n    (\n        all_scenarios,\n",
    "CLI parser",
)

text = replace_once(
    text,
    "            \"max_terminal_candidates_guard\": (\n                MAX_TERMINAL_CANDIDATES\n            ),\n",
    "            \"max_terminal_candidates_guard\": (\n                MAX_TERMINAL_CANDIDATES\n            ),\n            \"stop_after_structure\": (\n                args.stop_after_structure\n            ),\n            \"skip_terminal\": (\n                args.skip_terminal\n            ),\n",
    "metadata CLI modes",
)

text = replace_once(
    text,
    "    summary[\n        \"stage_a_structure\"\n    ] = {\n",
    "    summary[\n        \"stage_a_structure\"\n    ] = {\n",
    "stage A summary anchor check",
)

text = replace_once(
    text,
    "    }\n\n    # ================================================================\n    # Stage B\n",
    "    }\n\n    if args.stop_after_structure:\n        write_results_csv(\n            csv_rows\n        )\n\n        write_summary_json(\n            summary\n        )\n\n        print()\n        print(\n            \"Sampling tuning Stage A completed.\"\n        )\n        print(\n            \"  CSV: \"\n            f\"{RESULT_CSV_PATH}\"\n        )\n        print(\n            \"  JSON: \"\n            f\"{SUMMARY_JSON_PATH}\"\n        )\n        print(\n            \"  Held-out evaluation scenarios \"\n            \"were never imported or executed.\"\n        )\n        return\n\n    # ================================================================\n    # Stage B\n",
    "stop after Stage A",
)

text = replace_once(
    text,
    "    print()\n    print(\n        \"Stage B: terminal-state sweep \"\n        \"on Stage-A Pareto configs\"\n    )\n\n    terminal_configs = []\n\n    for evaluation in (\n        stage_a_pareto\n    ):\n        terminal_configs.extend(\n            build_terminal_variants(\n                evaluation.config\n            )\n        )\n",
    "    print()\n\n    if args.skip_terminal:\n        print(\n            \"Stage B: terminal-state sweep skipped; \"\n            \"carrying Stage-A Pareto configs forward\"\n        )\n    else:\n        print(\n            \"Stage B: terminal-state sweep \"\n            \"on Stage-A Pareto configs\"\n        )\n\n    terminal_configs = []\n\n    for evaluation in (\n        stage_a_pareto\n    ):\n        if args.skip_terminal:\n            terminal_configs.append(\n                evaluation.config\n            )\n        else:\n            terminal_configs.extend(\n                build_terminal_variants(\n                    evaluation.config\n                )\n            )\n",
    "conditional terminal stage",
)

text = replace_once(
    text,
    "        \"total_active_scenarios\": (\n            len(\n                active_scenarios\n            )\n        ),\n        \"common_comparison_scenarios\": sorted(\n            stage_b_common_names\n        ),\n",
    "        \"total_active_scenarios\": (\n            len(\n                active_scenarios\n            )\n        ),\n        \"terminal_sweep_skipped\": (\n            args.skip_terminal\n        ),\n        \"common_comparison_scenarios\": sorted(\n            stage_b_common_names\n        ),\n",
    "Stage B skipped metadata",
)

script_path.write_text(text, encoding="utf-8")


skill_path = Path(
    ".agents/skills/sampling-planner-tuning/SKILL.md"
)
skill = skill_path.read_text(encoding="utf-8")

skill = replace_once(
    skill,
    "Treat `docs/roadmap.md` as the source of truth for current phase/status wording.\n\nRecord the starting Git commit and inspect `git status --short` before edits. Preserve unrelated user changes.\n\n## Scenario isolation: mandatory\n",
    "Treat `docs/roadmap.md` as the source of truth for current phase/status wording.\n\nRecord the starting Git commit and inspect `git status --short` before edits. Preserve unrelated user changes.\n\nThe tuning runner supports two evidence-driven control modes:\n\n- `python -m scripts.planning.run_sampling_tuning --stop-after-structure` runs Stage A only and writes candidate-level feasibility diagnostics.\n- `python -m scripts.planning.run_sampling_tuning --skip-terminal` carries the Stage-A Pareto frontier directly into weight sensitivity when terminal-state expansion is not justified.\n\nRunning the script without flags preserves the full historical A -> B -> C reproduction workflow.\n\n## Pre-tuning sanity check\n\nBefore tuning sampling density or search-space geometry, confirm that the current reference/Frenet pipeline is a sound basis for comparison. At minimum:\n\n- the reference line is sufficiently smooth for Frenet polynomial generation;\n- Frenet projection is locally well behaved for the active scenarios;\n- initial states are physically valid or explicitly excluded with a recorded reason;\n- planner resolution and physical-validation density are understood.\n\nUse the repository's existing reference-line/Frenet validation scripts when relevant. If these checks pass, record that fact and continue; do not turn this into a separate tuning project.\n\n## Scenario isolation: mandatory\n",
    "skill precheck and runner modes",
)

skill = replace_once(
    skill,
    "- candidate count versus runtime trade-off.\n\nDo not silently change common physical constraints, benchmark definitions, vehicle geometry, canonical evaluator formulas, scenario definitions, or the maneuver intent just to improve a result.\n",
    "- candidate count versus runtime trade-off.\n\nThe current planner is a spatial path sampler over `l(s)`. Do not introduce time-horizon, target-speed, or longitudinal-state sampling unless the planner scope is explicitly extended to spatiotemporal trajectory planning.\n\nStructured, obstacle-aware, or scenario-conditioned sampling is optional future work. Consider it when scenario diversity, horizon length, or candidate budget makes uniform sampling inefficient; do not add it automatically to the current two-stage bypass-and-return planner.\n\nDo not silently change common physical constraints, benchmark definitions, vehicle geometry, canonical evaluator formulas, scenario definitions, or the maneuver intent just to improve a result.\n",
    "skill spatial-scope rule",
)

skill = replace_once(
    skill,
    "For each config, record at least:\n\n- candidate count;\n- success count and failure names;\n- common-success canonical objective where comparable;\n- minimum clearance;\n- max and RMS curvature;\n- curvature-rate metric;\n- runtime.\n",
    "For each config, record at least:\n\n- generated candidate count;\n- feasible candidate count and feasible ratio;\n- exclusive pruning breakdown in validator order: curvature rejects, clearance rejects, then road-boundary rejects;\n- success count and failure names;\n- common-success canonical objective where comparable;\n- minimum clearance;\n- max and RMS curvature;\n- curvature-rate metric;\n- runtime.\n\nUse candidate-level diagnostics to distinguish search-density limitations from candidate-family limitations. If a denser grid does not improve feasible ratio or materially change the pruning breakdown, do not keep increasing density without a new hypothesis.\n",
    "skill Stage A diagnostics",
)

skill = replace_once(
    skill,
    "Historical evidence from this repository suggests bypass-length diversity can provide more value than simply densifying the lateral grid. Treat this as a prior, not a permanent rule: rerun the evidence if scenarios or planner formulation have materially changed.\n\n### Stage B — Terminal-state sampling\n\nAfter a promising structural family is identified, test terminal target density:\n",
    "Historical evidence from this repository suggests bypass-length diversity can provide more value than simply densifying the lateral grid. Treat this as a prior, not a permanent rule: rerun the evidence if scenarios or planner formulation have materially changed.\n\nStart a new tuning investigation with `--stop-after-structure` when practical so the Stage-A failure and pruning evidence can determine whether terminal expansion is worth running.\n\n### Stage B — Conditional terminal-state sampling\n\nKeep the strict terminal state by default. Expand terminal target density only when Stage-A failure analysis gives a concrete reason to suspect that strict rejoin constraints are limiting feasible candidates. If no such evidence exists, use `--skip-terminal` and move directly to weight sensitivity.\n\nWhen terminal expansion is justified, test:\n",
    "conditional terminal Stage B",
)

skill = replace_once(
    skill,
    "- experiments performed;\n- maximum coverage before and after;\n- final config and what changed;\n",
    "- experiments performed;\n- maximum coverage before and after;\n- candidate-level feasible ratio and dominant pruning categories for important configs;\n- whether terminal-state expansion was run or skipped, and why;\n- final config and what changed;\n",
    "skill final report diagnostics",
)

skill_path.write_text(skill, encoding="utf-8")

print("Sampling tuning runner and skill patched successfully.")
