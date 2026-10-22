# Autonomous Driving Planner Tuning Skills

This folder contains agent-facing tuning skills for the `vlm-hybrid-pnc` repository.

The goal is to replace the manual loop of:

`run sweep -> inspect CSV/JSON -> reason about the next sweep -> edit config -> rerun`

with a controlled AI tuning loop that preserves the benchmark methodology already established in the repository.

## Contents

- `sampling-planner-tuning/SKILL.md`  
  - Sampling Planner search-space, terminal-state, matched-objective, candidate-count, runtime and freeze workflow.  
- `optimization-planner-tuning/SKILL.md`  
  - Optimization Planner geometry/corridor, objective interaction, OSQP tolerance, runtime and freeze workflow.

The Python tuning scripts remain the experiment engine. The skill acts as the tuning engineer: it reads the repository, designs targeted sweeps, runs experiments, interprets the outputs, chooses the next experiment, validates the final configuration, and performs the requested Git finishing action.

## Core principles shared by both skills

1. Read the repository before changing anything. `docs/roadmap.md` is the source of truth for phase/status wording.  
2. Use only development/tuning scenarios during tuning. Held-out final-evaluation scenarios must not be imported, executed, inspected for parameter values, or used to guide tuning.  
3. Compare physical success/coverage before objective value.  
4. Never compare mean objective values computed over different success subsets. Use a common-success scenario set when comparing path objective among configs.  
5. Keep the canonical benchmark evaluator independent from planner-internal tuning weights.  
6. Prefer Pareto reasoning over arbitrary scalar thresholds. Do not invent a magic score or automatic threshold just to pick a winner.  
7. Treat tuning scripts as experiment automation, not as an oracle that automatically chooses a config.  
8. Do not tune around a suspected implementation bug. If a bug or methodological confound is discovered, fix or isolate it first, rerun the relevant experiment, and document the effect.  
9. Do not use solver numerical looseness as a fake planning improvement. Boundary flips caused by tolerance-level numerical drift require special scrutiny.  
10. Never `git push` from these skills.

## Git finish modes

Each skill accepts one of three finishing modes. If the user does not specify a mode, default to `unstaged`.

### `unstaged`

Finish after all edits and validation with the worktree modified but nothing newly staged by the agent.

Required final checks:

- show `git status --short`  
- show the important diff summary  
- do not run `git add`  
- do not commit  
- do not push

### `staged`

After validation, stage only the files that belong to the tuning task.

Required final checks:

- run `git add` with explicit file paths, not a blind `git add .` when unrelated changes exist  
- show `git status --short`  
- do not commit  
- do not push

### `committed`

After validation, stage only the task files and create a local commit with a concise descriptive message.

Required final checks:

- verify the diff before staging  
- stage only task files  
- create the commit  
- show the resulting commit SHA and `git status --short`  
- do not push

If unrelated user changes are already present, preserve them. Never discard, reset, stash, amend, or include unrelated work without explicit permission.

## Example prompts

Sampling:

`Use the sampling-planner-tuning skill to tune the Sampling Planner in this repo. Continue targeted experiments until you can justify a frozen config. Do not touch held-out evaluation scenarios. git_finish_mode=staged.`

Optimization:

`Use the optimization-planner-tuning skill to tune the Optimization Planner. Investigate interactions instead of assuming one-factor tuning is sufficient. Keep solver tolerance strict unless the evidence justifies a change. git_finish_mode=committed. Never push.`

Conservative review-only finish:

`Run the Optimization tuning skill and stop with all changes unstaged so I can review the diff first.`

## Intended agent behavior

The agent should continue the experiment loop autonomously when the evidence clearly suggests the next targeted test. It should not repeatedly ask the user to run scripts or manually relay results that the agent can execute and read itself.

The agent should stop and report instead of guessing when:

- the tuning methodology becomes invalid or ambiguous;  
- a code bug is likely driving the result;  
- the requested next step would expose held-out evaluation data;  
- multiple Pareto candidates remain genuinely incomparable and choosing one would require a product preference not encoded in the repository;  
- destructive Git action would be required.

## Repository assumptions

These skills were authored for:

`OrcinusOrca-16/vlm-hybrid-pnc`

They intentionally reference the current project structure, including:

- `docs/roadmap.md`  
- `src/config/planning.py`  
- `src/benchmark/tuning_scenario_suite.py`  
- `scripts/planning/run_sampling_tuning.py`  
- `scripts/planning/run_optimization_tuning.py`  
- `scripts/planning/run_optimization_tuning_reproduction.py`  
- canonical/discrete/stress/runtime benchmark scripts

The agent must still read the current versions of these files at execution time. Historical tuning conclusions are hints and evidence, not immutable rules.

## Future extension

The same architecture can later be extended with a third `mpc-tuning` skill for Phase 15: the Python scripts remain the experiment engine, while the skill owns experiment design, analysis, iterative search, regression, freeze and Git finishing policy.