---
name: optimization-planner-tuning
description: Autonomously tune the Optimization Planner in vlm-hybrid-pnc using the repository tuning engine, development scenarios, targeted geometry/objective/solver sweeps, methodological safeguards, regression checks, config freeze, and a selectable Git finish mode. Never use held-out final-evaluation scenarios for tuning and never push.
---

# Optimization Planner Tuning

Use this skill when the user asks an agent to tune, retune, calibrate, optimize, or systematically benchmark the Optimization Planner in the `vlm-hybrid-pnc` repository.

The skill owns the tuning-engineer loop. The Python tuning scripts own experiment execution, logging, CSV/JSON output, and reusable evaluation utilities.

Do not ask the user to manually run each sweep and paste results when the agent can execute the scripts and inspect the output itself.

## Required inputs

Resolve these from the user request or repository state:

- repository root;
- tuning goal, if explicitly constrained by the user;
- `git_finish_mode`: `unstaged`, `staged`, or `committed`.

If `git_finish_mode` is omitted, use `unstaged`.

Never `git push`.

## Repository contract

Before changing anything, read the current repository. Do not rely on stale remembered values.

Read at minimum:

1. `docs/roadmap.md`
2. `src/config/planning.py`
3. `src/benchmark/tuning_scenario_suite.py`
4. `scripts/planning/run_optimization_tuning.py`
5. `scripts/planning/run_optimization_tuning_reproduction.py`, if present
6. `src/benchmark/optimization_adapter.py`
7. `src/planning/optimization_planner.py`
8. `src/planning/lateral_corridor.py`
9. `src/planning/hard_constraints.py`
10. `src/planning/path_objective.py`
11. canonical, discrete-scenario, stress and runtime benchmark scripts required by the current roadmap.

Treat `docs/roadmap.md` as the source of truth for current phase/status wording.

Record the starting commit and inspect `git status --short`. Preserve unrelated user changes.

## Scenario isolation: mandatory

Tuning may use only scenarios explicitly exposed by the repository's development/tuning suite, currently through `build_development_tuning_scenarios()` in `src/benchmark/tuning_scenario_suite.py`.

Do not:

- import final-evaluation scenario builders during tuning;
- execute held-out final-evaluation scenarios;
- inspect held-out scenario parameter values to guide tuning;
- grep/search for hidden evaluation values merely to infer what would score well;
- change held-out scenarios;
- use final-evaluation outcomes to select parameters.

If a tuning runner unexpectedly imports or executes held-out data, stop, fix the isolation issue, and invalidate the affected run.

## What may be tuned

Optimization-specific parameters may include, as supported by the current repository:

- longitudinal discretization resolution;
- `D2` / curvature-proxy hard bound;
- obstacle inflation safety margin;
- corridor affected epsilon / corridor parameters;
- matched objective weights for `l`, `l'`, `l''`, `l'''`, terminal `l`, terminal `l'`;
- OSQP `eps_abs` and `eps_rel`;
- runtime-related solver settings only when they do not compromise the physical interpretation of success.

Do not silently alter benchmark definitions, common physical constraints, vehicle geometry, canonical evaluator formulas, scenario definitions, or maneuver intent just to improve results.

## Tuning objective hierarchy

Use this ordering:

1. **Physical success / coverage** on active development tuning scenarios.
2. **Canonical objective on a common-success scenario set.**
3. **Physical path quality**, especially minimum clearance, curvature, curvature rate, terminal errors, and lateral deviation.
4. **Runtime.**
5. **Simplicity, numerical robustness, and explainability.**

Coverage comes before objective value.

Never compare mean objective values computed over different success subsets. A config that fails hard scenarios may appear to have a lower mean simply because those difficult cases disappeared from the average.

For equal-coverage candidates, compare canonical objective over the common-success intersection using the repository's existing common-objective/Pareto machinery where possible.

Do not invent a weighted meta-score or arbitrary threshold merely to force a winner.

## Critical methodological safeguard: resolution versus validation density

Before tuning `resolution_m`, inspect the current physical validator and path reconstruction pipeline.

If the same planner resolution also determines the spacing of points used by final physical validation, then planner discretization and validation density are coupled.

In that state:

- do **not** interpret a coarser resolution's higher success rate as genuine planner improvement without further evidence;
- a coarse path can appear to pass because between-point clearance/curvature violations are not sampled;
- keep resolution fixed during unrelated parameter refinement;
- either decouple final validation to a fixed fine resolution or perform a fair fixed-density validation before choosing resolution based on success rate.

If this coupling is discovered during tuning, treat it as a methodological confound, not as a parameter win.

Do not tune around the confound.

Historical note: this repository previously showed exactly this issue when `1.0 m` discretization appeared to improve coverage compared with `0.5 m`. The final tuning decision intentionally returned to a fair fixed `0.5 m` comparison before freezing other parameters. Re-verify the current implementation rather than assuming the issue still exists.

## Critical numerical safeguard: solver tolerance

A looser OSQP tolerance may slightly move a solution across a hard physical threshold.

Therefore:

- never count `FAIL -> PASS` caused only by tiny tolerance-level drift as a meaningful planner improvement without inspecting the physical margin;
- if a successful path clears the hard threshold by only a very small numerical amount, compare it with the strict-tolerance solution and inspect whether the difference is physically meaningful;
- keep a stricter tolerance when runtime improvement is small and path metrics are otherwise unchanged;
- use repeated runtime measurements when comparing solver tolerances.

Do not use solver looseness as a substitute for better planning geometry or objective design.

Historical note: this repository previously observed a looser tolerance producing a success with clearance only tens of micrometers above the threshold. That result was intentionally rejected as a tuning improvement.

## Experiment-loop behavior

Operate autonomously:

1. inspect the latest config, output and failure set;
2. formulate the next engineering hypothesis;
3. choose the smallest targeted sweep that can test it;
4. modify experiment ranges or a reusable tuning driver;
5. run the experiment;
6. parse CSV/JSON outputs and failure reasons;
7. compare coverage first, then common-success objective and physical metrics;
8. check for parameter interactions or methodological confounds;
9. decide whether another targeted sweep is justified;
10. freeze only when the result is robust and explainable.

Do not repeatedly ask the user which obvious next experiment to run when the data already points to it.

## Recommended search sequence

Adapt to the current repository, but preserve this conceptual ordering.

### Stage A — Geometry / feasibility sweep

Start with planner geometry and hard-constraint approximations:

- resolution;
- `D2` curvature proxy bound;
- obstacle inflation margin;
- corridor epsilon.

Use a coarse but systematic sweep first. Record:

- success count and failure names;
- corridor-infeasible counts where available;
- QP failures;
- exact-validation rejects;
- common-success canonical objective;
- minimum clearance;
- max/RMS curvature;
- curvature-rate metric;
- runtime.

Do not conclude that a parameter is irrelevant merely because several values tie on aggregate coverage. Inspect whether the bound is inactive, whether failure modes differ, and whether a later objective interaction can activate the parameter.

When one dimension is methodologically confounded, fix it to a fair value while refining the other dimensions.

### Stage B — Objective and parameter interaction

Once a reasonable geometry family is identified, tune the six matched objective weights:

- `l`
- `l'`
- `l''`
- `l'''`
- terminal `l`
- terminal `l'`

Use systematic perturbations around the baseline before inventing an unconstrained search.

Do not assume one-factor-at-a-time results are sufficient when the data shows interaction.

If a weight improves coverage only under a particular margin/corridor setting, run a targeted interaction sweep such as:

`geometry subset x weight profiles`

rather than selecting the best Stage-A config first and permanently discarding the rest.

Historical note: this repository previously found a strong interaction between obstacle inflation margin and the internal matched lateral-deviation weight. A lower `lateral_weight` improved obstacle-avoidance coverage while retaining strict solver tolerance. Treat this as a prior, not a permanent truth.

### Stage C — Solver tolerance versus runtime

Only after geometry and objective are stable, compare OSQP tolerance profiles.

Use repeated runtime measurement. Compare:

- coverage stability across repeats;
- failure reason stability;
- mean/median runtime when available;
- canonical path objective;
- clearance and smoothness metrics;
- numerical proximity to hard thresholds.

If a looser tolerance yields only a small runtime gain or produces threshold-sensitive success flips, prefer the strict robust setting.

### Optional Stage D — Narrow refinement

If the best family is clear but one meaningful trade-off remains, perform a small local sweep around that family.

Examples:

- margin refinement between two coarse values;
- a small interaction grid around one weight and one corridor parameter;
- a targeted D2 bound refinement if the bound is demonstrably active.

Do not restart a huge Cartesian search unless the evidence justifies it.

## Failure analysis

For maximum-coverage and Pareto candidates, inspect scenario-level failure reasons.

Separate, when supported by the runner:

- initial-state-invalid scenarios;
- corridor infeasible;
- QP failure/infeasible status;
- exact physical validator reject;
- road-width geometry/search-family limitation;
- implementation error.

Do not label a scenario globally infeasible without proof. Use precise wording such as `persistent geometry/search-family-limited` when that is the strongest supported conclusion.

If a scenario is invalid at the initial state, exclude it from active tuning coverage only when the tuning methodology explicitly checks and records the reason.

## Canonical objective independence

The planner may use tuned internal objective weights while the benchmark reports a fixed canonical comparison objective.

Preserve that separation.

Do not modify the canonical evaluator to make the tuned planner look better. The adapter should use planner-internal weights for solving and canonical benchmark weights for cross-planner comparison.

If those roles become conflated, fix the implementation before continuing tuning.

## Experiment scripts and reproduction

Prefer reusable experiment automation.

- `scripts/planning/run_optimization_tuning.py` may serve as the shared tuning engine / current experiment runner.
- `scripts/planning/run_optimization_tuning_reproduction.py` may preserve the historical full study from frozen pre-tuning parameters.

Do not overwrite historical pre-tuning constants in a reproduction script with values from the already-tuned current config.

When adding a new experiment driver, make its purpose explicit and avoid duplicating thousands of lines of engine logic if it can safely reuse the shared runner.

Generated CSV/JSON outputs should remain outside Git if the repository intentionally ignores them.

## Stop conditions

Freeze when the evidence supports all of the following:

- no reasonable targeted follow-up has a clear hypothesis for improving maximum coverage or resolving an important trade-off;
- remaining candidates are dominated or form an explainable Pareto set;
- the selected candidate is not benefiting from resolution/validator coupling, solver numerical looseness, or another methodological confound;
- failure cases are understood well enough to explain;
- runtime is measured consistently;
- the chosen config is simple and interview-explainable;
- all development regressions pass.

If multiple Pareto candidates remain genuinely incomparable because the choice depends on a product preference not encoded in the repository, stop and report the trade-off rather than inventing that preference.

## Freeze procedure

When a final Optimization config is justified:

1. update only intended `OptimizationPlannerConfig` fields in `src/config/planning.py`;
2. preserve the distinction between legacy/native objective fields and matched planner objective fields;
3. keep the tuning/reproduction scripts able to explain the experiments;
4. rerun canonical, discrete-scenario, stress and formal runtime regressions required by the current roadmap;
5. inspect exact failure names and failure categories;
6. update `docs/roadmap.md` only when the corresponding work is genuinely complete;
7. do not run held-out final evaluation during tuning/freeze.

Be especially careful when `OptimizationPlannerConfig` contains both older/native fields and `matched_objective_weights`. Change the field actually consumed by the matched benchmark planner; do not change similarly named legacy fields by accident.

## Regression interpretation

A config is not frozen merely because one tuning summary says it is better.

Confirm:

- canonical benchmark passes;
- discrete scenarios pass;
- stress benchmark behavior is understood;
- formal runtime benchmark is recorded;
- failure cases are stable and explainable;
- the canonical comparison metric remains independent of planner-internal weights;
- any solver-tolerance result is numerically robust.

If regressions expose an implementation bug or a methodological confound, fix that issue and rerun the affected tuning stage rather than tuning around it.

## Git finish modes

Never push.

### Mode: `unstaged`

Default mode.

After all validation:

- leave task edits unstaged;
- run `git status --short`;
- summarize modified files and important diff;
- do not run `git add`;
- do not commit;
- do not push.

### Mode: `staged`

After validation:

- inspect `git diff`;
- stage only files belonging to this task using explicit paths;
- preserve unrelated pre-existing changes;
- show `git status --short`;
- do not commit;
- do not push.

Avoid blind `git add .` when unrelated changes exist.

### Mode: `committed`

After validation:

- inspect the final diff;
- stage only task files;
- create a local commit with a concise message such as `Tune and freeze Optimization Planner` or the roadmap-specific equivalent;
- report the commit SHA;
- show `git status --short`;
- do not push.

Never amend, reset, stash, discard, or overwrite unrelated user work without explicit permission.

## Final report

Return a compact engineering report containing:

- starting commit;
- development/tuning scenario count and invalid-initial-state exclusions;
- experiment stages performed;
- maximum coverage before and after;
- final config and exactly what changed;
- common-success canonical objective comparison;
- key clearance/smoothness/runtime trade-offs;
- remaining failure cases with precise wording;
- regression results;
- any methodological issue found and how it was resolved;
- Git finish mode and repository state;
- explicit statement that held-out final-evaluation scenarios were not used;
- explicit statement that no push was performed.

## Historical repository notes

Use these as priors only. Verify against the current implementation before acting.

- A coarse `1.0 m` resolution previously appeared to improve success, but final validation density was coupled to planner resolution; that apparent gain was not accepted without a fair validation basis.
- Obstacle-inflation margin showed a real trade-off: too small could lead to exact-validation obstacle rejects, while too large could make narrow corridors infeasible.
- Objective weights interacted with geometry. A lower matched lateral-deviation weight previously improved robust coverage.
- Loose OSQP tolerance once produced a near-threshold success that was rejected as a numerical boundary effect.
- The final historical config kept strict `1e-6` solver tolerances and achieved the improved coverage through planner objective/geometry choices instead of numerical looseness.

If planner formulation, scenario suite, validator, corridor construction or benchmark methodology changes, rerun the relevant evidence instead of preserving historical conclusions by assumption.
