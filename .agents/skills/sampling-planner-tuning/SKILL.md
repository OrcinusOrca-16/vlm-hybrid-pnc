---
name: sampling-planner-tuning
description: Autonomously tune the Sampling Planner in vlm-hybrid-pnc using the repository tuning engine, development scenarios, canonical metrics, iterative targeted sweeps, regression checks, config freeze, and a selectable Git finish mode. Never use held-out final-evaluation scenarios for tuning and never push.
---

# Sampling Planner Tuning

Use this skill when the user asks an agent to tune, retune, calibrate, optimize, or systematically benchmark the Sampling Planner in the `vlm-hybrid-pnc` repository.

The skill owns the tuning-engineer loop. The Python scripts own experiment execution and measurement.

Do not ask the user to manually run each sweep and paste results when the agent can run the scripts and inspect their CSV/JSON output directly.

## Required inputs

Resolve these from the user request or repository state:

- repository root;
- tuning goal, if the user specifies one;
- `git_finish_mode`: `unstaged`, `staged`, or `committed`.

If `git_finish_mode` is omitted, use `unstaged`.

Never `git push`.

## Repository contract

Before changing anything, read the current repository. Do not rely on stale remembered values.

Read at minimum:

1. `docs/roadmap.md`
2. `src/config/planning.py`
3. `src/benchmark/tuning_scenario_suite.py`
4. `scripts/planning/run_sampling_tuning.py`
5. `src/benchmark/sampling_adapter.py`
6. `src/planning/sampling_planner.py`
7. `src/planning/hard_constraints.py`
8. `src/planning/path_objective.py`
9. `src/planning/soft_cost.py`
10. the canonical, discrete-scenario, stress and runtime benchmark scripts used by the current roadmap.

Treat `docs/roadmap.md` as the source of truth for current phase/status wording.

Record the starting Git commit and inspect `git status --short` before edits. Preserve unrelated user changes.

The tuning runner supports two evidence-driven control modes:

- `python -m scripts.planning.run_sampling_tuning --stop-after-structure` runs Stage A only and enables the extra candidate-level feasibility/pruning diagnostics.
- `python -m scripts.planning.run_sampling_tuning --skip-terminal` carries the Stage-A Pareto frontier directly into weight sensitivity when terminal-state expansion is not justified.

Running the script without flags preserves the full historical A -> B -> C reproduction workflow and does not pay the extra diagnostic re-validation cost.

## Pre-tuning sanity check

Before tuning sampling density or search-space geometry, confirm that the current reference/Frenet pipeline is a sound basis for comparison. At minimum:

- the reference line is sufficiently smooth for Frenet polynomial generation;
- Frenet projection is locally well behaved for the active scenarios;
- initial states are physically valid or explicitly excluded with a recorded reason;
- planner resolution and physical-validation density are understood.

Use the repository's existing reference-line/Frenet validation scripts when relevant. If these checks pass, record that fact and continue; do not turn this into a separate tuning project.

## Scenario isolation: mandatory

Tuning may use only scenarios explicitly exposed by the repository's development/tuning suite, currently through `build_development_tuning_scenarios()` in `src/benchmark/tuning_scenario_suite.py`.

Do not:

- import final-evaluation scenario builders during tuning;
- execute held-out evaluation scenarios;
- inspect held-out scenario parameter values to guide search;
- grep/search for hidden evaluation values merely to infer what would score well;
- change evaluation scenarios to make a planner pass;
- use final-evaluation outcomes to select tuning parameters.

If a runner unexpectedly imports or executes held-out data, stop, fix the isolation issue, and invalidate results from that run.

## What may be tuned

Sampling-specific parameters may include, as supported by the current repository:

- bypass lateral range;
- lateral sampling interval / lateral sample positions;
- bypass-length sample set;
- final lateral target sampling density;
- final slope target sampling density;
- matched objective weights for `l`, `l'`, `l''`, `l'''`, terminal `l`, terminal `l'`;
- candidate count versus runtime trade-off.

The current planner is a spatial path sampler over `l(s)`. Do not introduce time-horizon, target-speed, or longitudinal-state sampling unless the planner scope is explicitly extended to spatiotemporal trajectory planning.

Structured, obstacle-aware, or scenario-conditioned sampling is optional future work. Consider it when scenario diversity, horizon length, or candidate budget makes uniform sampling inefficient; do not add it automatically to the current two-stage bypass-and-return planner.

Do not silently change common physical constraints, benchmark definitions, vehicle geometry, canonical evaluator formulas, scenario definitions, or the maneuver intent just to improve a result.

## Tuning objective hierarchy

Use this ordering when judging configurations:

1. **Physical success / coverage** on active development tuning scenarios.
2. **Canonical objective on a common-success scenario set.**
3. **Physical path quality**, including clearance, curvature, curvature rate, terminal errors, and lateral deviation.
4. **Runtime and candidate count.**
5. **Simplicity and robustness** of the final configuration.

Coverage is evaluated before objective value.

Never compare `mean_success_objective` between configs when those means are computed from different success subsets. A config that fails difficult scenarios can appear to have an artificially better mean objective.

When comparing equal-coverage candidates, evaluate canonical objective over the intersection of scenarios that all compared configs successfully solve. Use the repository's existing common-objective/Pareto machinery when available.

Do not create an arbitrary weighted meta-score or magic threshold just to force a single winner.

## Experiment-loop behavior

Operate autonomously in a loop:

1. inspect the latest result;
2. state the engineering hypothesis internally or in the run log;
3. choose the smallest targeted sweep that can test that hypothesis;
4. modify only the experiment configuration needed for that sweep;
5. run the experiment;
6. parse CSV/JSON outputs and failure cases;
7. compare coverage first, then common-success canonical objective and physical metrics;
8. decide whether the evidence supports another targeted sweep, a Pareto decision, or freeze;
9. repeat until further search is no longer justified.

Do not repeatedly ask the user which obvious next sweep to run when repository evidence is sufficient to decide.

## Recommended search sequence

Adapt this to the current code, but preserve the conceptual ordering.

### Stage A — Structural search space

Tune the candidate geometry before fine objective tuning.

Explore combinations such as:

- bypass lateral range;
- lateral interval / sample density;
- bypass-length diversity.

For each config, record at least:

- generated candidate count;
- feasible candidate count and feasible ratio;
- exclusive pruning breakdown in validator order: curvature rejects, clearance rejects, then road-boundary rejects;
- success count and failure names;
- common-success canonical objective where comparable;
- minimum clearance;
- max and RMS curvature;
- curvature-rate metric;
- runtime.

Use candidate-level diagnostics to distinguish search-density limitations from candidate-family limitations. If a denser grid does not improve feasible ratio or materially change the pruning breakdown, do not keep increasing density without a new hypothesis.

Prefer a targeted structural sweep over blindly exploding all dimensions at once.

Historical evidence from this repository suggests bypass-length diversity can provide more value than simply densifying the lateral grid. Treat this as a prior, not a permanent rule: rerun the evidence if scenarios or planner formulation have materially changed.

Start a new tuning investigation with `--stop-after-structure` when practical so the Stage-A failure and pruning evidence can determine whether terminal expansion is worth running.

### Stage B — Conditional terminal-state sampling

Keep the strict terminal state by default. Expand terminal target density only when Stage-A failure analysis gives a concrete reason to suspect that strict rejoin constraints are limiting feasible candidates. If no such evidence exists, use `--skip-terminal` and move directly to weight sensitivity.

When terminal expansion is justified, test:

- strict final `l=target`, final `l'=target`;
- lateral terminal offsets when justified;
- slope offsets when justified;
- combined terminal variants when candidate budget permits.

Guard candidate explosion. A candidate-cap mechanism is a runtime protection, not a decision criterion.

If added terminal density does not improve coverage or produces only dominated path-quality/runtime outcomes, keep the simpler terminal sampling.

### Stage C — Matched objective weights

Tune the six matched objective weights only after candidate geometry is reasonably stable:

- `l`
- `l'`
- `l''`
- `l'''`
- terminal `l`
- terminal `l'`

Prefer systematic perturbations around the current baseline, such as multiplicative factors already supported by the tuning runner, before inventing a large arbitrary search.

Where candidate pools are unchanged, reuse cached candidate generation / feasibility work if the runner supports it. Weight sweeps should not regenerate expensive geometry unnecessarily.

If weight perturbations do not change selections or yield only negligible trade-offs, keep the simpler/current weights rather than changing them for cosmetic reasons.

### Stage D — Candidate count versus runtime

For non-dominated structural candidates, compare:

- coverage;
- common-success canonical objective;
- physical quality;
- candidate count;
- measured runtime.

Use Pareto reasoning. For example, if a denser config has the same coverage and only a small path-quality advantage but materially increases candidates/runtime, the lower-complexity config may be the better engineering choice.

Do not treat a Pareto-membership flip caused only by small single-run runtime noise as evidence of a better configuration. Repeat timing or prefer deterministic candidate-count/quality evidence when coverage and objective are otherwise unchanged.

Do not invent a fixed acceptable percentage improvement unless the user or repository already defines one.

## Failure analysis

For every maximum-coverage frontier candidate, inspect failure names rather than only the aggregate count.

Separate at least these concepts when the data supports them:

- initial-state-invalid scenarios;
- curvature pruning;
- clearance pruning;
- road-boundary pruning;
- search-space limitation / no sampled feasible candidate;
- genuine implementation error.

Do not label a scenario globally infeasible unless the repository evidence mathematically or geometrically proves that claim. Prefer precise wording such as `persistent geometry/search-family-limited` when that is all the evidence supports.

If a scenario is invalid at the initial state, exclude it from active tuning coverage only when the repository's tuning methodology explicitly performs that exclusion and records the reason.

## Numerical and methodological safeguards

- The canonical evaluator must remain independent of planner-internal tuning choices.
- Do not change hard-constraint definitions during a parameter sweep unless the task is explicitly to fix a validated bug.
- Do not change both the objective formulation and the tuning config in one opaque step. Keep formulation changes separate from parameter tuning.
- Keep benchmark timing boundaries consistent while comparing runtime.
- If measurement noise appears meaningful, repeat runtime measurements rather than over-interpreting a single run.
- Do not automatically select a config solely because a generic Pareto helper happened to order it first.

## Stop conditions

A tuning loop may stop and freeze when the evidence supports all of the following:

- no targeted follow-up sweep has a reasonable hypothesis for improving maximum coverage or resolving an important trade-off;
- remaining candidates are dominated or represent an explainable Pareto trade-off;
- the chosen config is not relying on a methodological confound;
- runtime is measured with the same benchmark timing scope;
- the final config is simple enough to justify and explain in an interview;
- all required development regressions pass.

If two or more Pareto candidates remain genuinely incomparable because the choice requires a product preference that is not encoded in the repository, stop and present the trade-off instead of inventing a preference.

## Freeze procedure

When a final Sampling config is justified:

1. update only the intended `SamplingPlannerConfig` fields in `src/config/planning.py`;
2. keep experiment/reproduction scripts able to explain how the choice was obtained;
3. do not commit generated tuning CSV/JSON if the repository intentionally ignores outputs;
4. rerun the current regression set required by the roadmap, including canonical, discrete-scenario, stress and formal runtime benchmarks;
5. inspect the new failure set and confirm no unexplained regression;
6. update `docs/roadmap.md` only when the corresponding work is genuinely complete;
7. do not run held-out final evaluation as part of tuning/freeze.

The skill may create or refine tuning automation scripts when needed, but prefer reusable experiment runners over throwaway one-off scripts. Clean up exploratory artifacts before finishing unless they are intentionally retained as reproduction tooling.

## Regression interpretation

A frozen configuration is not accepted merely because the tuning runner reports a better score.

Confirm:

- canonical benchmark passes;
- discrete benchmark passes;
- stress benchmark behavior is understood;
- runtime benchmark is stable enough to report;
- failure names are consistent with the engineering explanation;
- the canonical comparison metric still uses the common benchmark weights rather than planner-internal tuned weights.

If regression changes expose a new bug, return to the experiment loop after fixing the bug. Do not tune around it.

## Git finish modes

Never push.

### Mode: `unstaged`

This is the default.

After all validation:

- leave task edits unstaged;
- run `git status --short`;
- summarize the modified files and important diff;
- do not run `git add`;
- do not commit;
- do not push.

### Mode: `staged`

After all validation:

- inspect `git diff`;
- stage only files belonging to this tuning task with explicit paths;
- do not include unrelated pre-existing changes;
- show `git status --short`;
- do not commit;
- do not push.

Avoid `git add .` if unrelated changes exist.

### Mode: `committed`

After all validation:

- inspect the final diff;
- stage only task files;
- create a local commit with a concise message, for example `Tune and freeze Sampling Planner` or the roadmap-specific equivalent;
- report the commit SHA;
- show `git status --short`;
- do not push.

Never amend, reset, stash, discard, or overwrite unrelated user work without explicit permission.

## Final report

Return a compact engineering report containing:

- starting commit;
- tuning scenario count and any excluded invalid-initial-state cases;
- experiments performed;
- maximum coverage before and after;
- candidate-level feasible ratio and dominant pruning categories for important configs;
- whether terminal-state expansion was run or skipped, and why;
- final config and what changed;
- canonical/common-success objective comparison;
- key clearance/smoothness/runtime trade-offs;
- remaining failure cases with careful wording;
- regression results;
- Git finish mode and resulting repository state;
- explicit statement that held-out final-evaluation scenarios were not used;
- explicit statement that no push was performed.

## Historical repository notes

Use these only as priors. Verify against the current repository before acting.

- Sampling tuning previously favored a moderate candidate set with strong bypass-length diversity rather than maximum lateral densification.
- Extra terminal-state density previously increased runtime/candidate count without improving coverage enough to justify the complexity.
- Weight perturbations previously had limited impact on the selected candidate family.

If the planner, scenarios, objective, hard constraints, or benchmark methodology have changed since those observations, rerun the relevant evidence instead of preserving the old answer by assumption.
