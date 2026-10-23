from pathlib import Path


def replace_once(text: str, old: str, new: str, label: str) -> str:
    if old not in text:
        raise RuntimeError(f"Patch anchor not found: {label}")
    return text.replace(old, new, 1)


script_path = Path("scripts/planning/run_sampling_tuning.py")
text = script_path.read_text(encoding="utf-8")

text = replace_once(
    text,
    "_CANDIDATE_DIAGNOSTIC_CACHE: dict[\n    tuple[\n        SamplingTuningConfig,\n        str,\n    ],\n    CandidateDiagnostics,\n] = {}\n\n\ndef symmetric_samples(\n",
    "_CANDIDATE_DIAGNOSTIC_CACHE: dict[\n    tuple[\n        SamplingTuningConfig,\n        str,\n    ],\n    CandidateDiagnostics,\n] = {}\n\n\n# Candidate-level pruning diagnostics intentionally re-run candidate\n# generation/validation. Keep them opt-in so the historical full tuning\n# reproduction retains its original wall-clock cost.\n_COLLECT_CANDIDATE_DIAGNOSTICS = False\n\n\ndef symmetric_samples(\n",
    "diagnostic mode flag",
)

text = replace_once(
    text,
    "    diagnostics = (\n        get_candidate_diagnostics(\n            config=config,\n            scenario=scenario,\n        )\n    )\n\n    outcome = (\n        attach_candidate_diagnostics(\n            outcome=outcome,\n            diagnostics=diagnostics,\n        )\n    )\n\n    _RUN_CACHE[\n",
    "    if _COLLECT_CANDIDATE_DIAGNOSTICS:\n        diagnostics = (\n            get_candidate_diagnostics(\n                config=config,\n                scenario=scenario,\n            )\n        )\n\n        outcome = (\n            attach_candidate_diagnostics(\n                outcome=outcome,\n                diagnostics=diagnostics,\n            )\n        )\n\n    _RUN_CACHE[\n",
    "gate run-one diagnostics",
)

text = replace_once(
    text,
    "    successful_outcomes = [\n        outcome\n        for outcome\n        in evaluation.outcomes.values()\n        if outcome.success\n    ]\n\n    return {\n",
    "    successful_outcomes = [\n        outcome\n        for outcome\n        in evaluation.outcomes.values()\n        if outcome.success\n    ]\n\n    diagnostic_outcomes = [\n        outcome\n        for outcome\n        in evaluation.outcomes.values()\n        if outcome.generated_candidate_count is not None\n    ]\n\n    return {\n",
    "diagnostic outcomes",
)

text = replace_once(
    text,
    "        \"total_generated_candidate_count\": sum(\n            outcome.generated_candidate_count or 0\n            for outcome\n            in evaluation.outcomes.values()\n        ),\n        \"total_feasible_candidate_count\": sum(\n            outcome.feasible_candidate_count or 0\n            for outcome\n            in evaluation.outcomes.values()\n        ),\n        \"total_curvature_reject_count\": sum(\n            outcome.curvature_reject_count or 0\n            for outcome\n            in evaluation.outcomes.values()\n        ),\n        \"total_clearance_reject_count\": sum(\n            outcome.clearance_reject_count or 0\n            for outcome\n            in evaluation.outcomes.values()\n        ),\n        \"total_road_boundary_reject_count\": sum(\n            outcome.road_boundary_reject_count or 0\n            for outcome\n            in evaluation.outcomes.values()\n        ),\n",
    "        \"total_generated_candidate_count\": (\n            sum(\n                outcome.generated_candidate_count or 0\n                for outcome\n                in diagnostic_outcomes\n            )\n            if diagnostic_outcomes\n            else None\n        ),\n        \"total_feasible_candidate_count\": (\n            sum(\n                outcome.feasible_candidate_count or 0\n                for outcome\n                in diagnostic_outcomes\n            )\n            if diagnostic_outcomes\n            else None\n        ),\n        \"total_curvature_reject_count\": (\n            sum(\n                outcome.curvature_reject_count or 0\n                for outcome\n                in diagnostic_outcomes\n            )\n            if diagnostic_outcomes\n            else None\n        ),\n        \"total_clearance_reject_count\": (\n            sum(\n                outcome.clearance_reject_count or 0\n                for outcome\n                in diagnostic_outcomes\n            )\n            if diagnostic_outcomes\n            else None\n        ),\n        \"total_road_boundary_reject_count\": (\n            sum(\n                outcome.road_boundary_reject_count or 0\n                for outcome\n                in diagnostic_outcomes\n            )\n            if diagnostic_outcomes\n            else None\n        ),\n",
    "optional diagnostic totals",
)

text = replace_once(
    text,
    "    diagnostics = (\n        compute_candidate_diagnostics(\n            trajectories=trajectories,\n            paths=paths,\n            scenario=scenario,\n        )\n    )\n\n    if (\n        diagnostics.feasible_candidate_count\n        != len(feasible_candidates)\n    ):\n        raise RuntimeError(\n            \"Candidate diagnostics disagree with physical validator.\"\n        )\n\n    _CANDIDATE_CACHE[\n        cache_key\n    ] = feasible_candidates\n\n    _CANDIDATE_DIAGNOSTIC_CACHE[\n        cache_key\n    ] = diagnostics\n",
    "    if _COLLECT_CANDIDATE_DIAGNOSTICS:\n        diagnostics = (\n            compute_candidate_diagnostics(\n                trajectories=trajectories,\n                paths=paths,\n                scenario=scenario,\n            )\n        )\n\n        if (\n            diagnostics.feasible_candidate_count\n            != len(feasible_candidates)\n        ):\n            raise RuntimeError(\n                \"Candidate diagnostics disagree with physical validator.\"\n            )\n\n        _CANDIDATE_DIAGNOSTIC_CACHE[\n            cache_key\n        ] = diagnostics\n\n    _CANDIDATE_CACHE[\n        cache_key\n    ] = feasible_candidates\n",
    "gate pool diagnostics",
)

text = replace_once(
    text,
    "    build_candidate_pool(\n        config=config,\n        scenario=scenario,\n    )\n\n    diagnostics = (\n",
    "    if not _COLLECT_CANDIDATE_DIAGNOSTICS:\n        raise RuntimeError(\n            \"Candidate diagnostics were not enabled for this run.\"\n        )\n\n    build_candidate_pool(\n        config=config,\n        scenario=scenario,\n    )\n\n    diagnostics = (\n",
    "diagnostics enabled guard",
)

text = replace_once(
    text,
    "        diagnostics = (\n            get_candidate_diagnostics(\n                config=config,\n                scenario=scenario,\n            )\n        )\n\n        if not feasible_candidates:\n",
    "        diagnostics = None\n\n        if _COLLECT_CANDIDATE_DIAGNOSTICS:\n            diagnostics = (\n                get_candidate_diagnostics(\n                    config=config,\n                    scenario=scenario,\n                )\n            )\n\n        if not feasible_candidates:\n",
    "gate cached-profile diagnostics",
)

text = replace_once(
    text,
    "            outcome = (\n                attach_candidate_diagnostics(\n                    outcome=outcome,\n                    diagnostics=diagnostics,\n                )\n            )\n\n            case_results.append(\n",
    "            if diagnostics is not None:\n                outcome = (\n                    attach_candidate_diagnostics(\n                        outcome=outcome,\n                        diagnostics=diagnostics,\n                    )\n                )\n\n            case_results.append(\n",
    "gate failed cached-profile attach",
)

text = replace_once(
    text,
    "        outcome = (\n            attach_candidate_diagnostics(\n                outcome=outcome,\n                diagnostics=diagnostics,\n            )\n        )\n\n        case_results.append(\n",
    "        if diagnostics is not None:\n            outcome = (\n                attach_candidate_diagnostics(\n                    outcome=outcome,\n                    diagnostics=diagnostics,\n                )\n            )\n\n        case_results.append(\n",
    "gate successful cached-profile attach",
)

text = replace_once(
    text,
    "def main() -> None:\n    args = parse_args()\n\n    (\n",
    "def main() -> None:\n    global _COLLECT_CANDIDATE_DIAGNOSTICS\n\n    args = parse_args()\n\n    # Stage-A-only mode is the diagnostic entry point used by the skill.\n    # Full reproduction and skip-terminal runs keep the historical cost.\n    _COLLECT_CANDIDATE_DIAGNOSTICS = (\n        args.stop_after_structure\n    )\n\n    (\n",
    "enable diagnostics from Stage-A-only mode",
)

text = replace_once(
    text,
    "            \"skip_terminal\": (\n                args.skip_terminal\n            ),\n",
    "            \"skip_terminal\": (\n                args.skip_terminal\n            ),\n            \"candidate_diagnostics_enabled\": (\n                _COLLECT_CANDIDATE_DIAGNOSTICS\n            ),\n",
    "diagnostics metadata",
)

script_path.write_text(text, encoding="utf-8")


skill_path = Path(".agents/skills/sampling-planner-tuning/SKILL.md")
skill = skill_path.read_text(encoding="utf-8")

skill = replace_once(
    skill,
    "- `python -m scripts.planning.run_sampling_tuning --stop-after-structure` runs Stage A only and writes candidate-level feasibility diagnostics.\n- `python -m scripts.planning.run_sampling_tuning --skip-terminal` carries the Stage-A Pareto frontier directly into weight sensitivity when terminal-state expansion is not justified.\n\nRunning the script without flags preserves the full historical A -> B -> C reproduction workflow.\n",
    "- `python -m scripts.planning.run_sampling_tuning --stop-after-structure` runs Stage A only and enables the extra candidate-level feasibility/pruning diagnostics.\n- `python -m scripts.planning.run_sampling_tuning --skip-terminal` carries the Stage-A Pareto frontier directly into weight sensitivity when terminal-state expansion is not justified.\n\nRunning the script without flags preserves the full historical A -> B -> C reproduction workflow and does not pay the extra diagnostic re-validation cost.\n",
    "skill diagnostic mode wording",
)

skill = replace_once(
    skill,
    "Use Pareto reasoning. For example, if a denser config has the same coverage and only a small path-quality advantage but materially increases candidates/runtime, the lower-complexity config may be the better engineering choice.\n\nDo not invent a fixed acceptable percentage improvement unless the user or repository already defines one.\n",
    "Use Pareto reasoning. For example, if a denser config has the same coverage and only a small path-quality advantage but materially increases candidates/runtime, the lower-complexity config may be the better engineering choice.\n\nDo not treat a Pareto-membership flip caused only by small single-run runtime noise as evidence of a better configuration. Repeat timing or prefer deterministic candidate-count/quality evidence when coverage and objective are otherwise unchanged.\n\nDo not invent a fixed acceptable percentage improvement unless the user or repository already defines one.\n",
    "skill runtime-noise safeguard",
)

skill_path.write_text(skill, encoding="utf-8")

print("Sampling diagnostics-mode patch applied.")
