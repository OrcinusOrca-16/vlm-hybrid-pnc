# Planning scripts

Planning utilities are grouped by purpose so the safe development path is obvious.

- `validation/`: regression and component checks. Use these during normal development and cleanup.
- `development/`: tuning, diagnostics, studies, and development visualizations.
- `evaluation/`: frozen/final evaluation, runtime benchmarking, and final figure generation. Do not use these to tune planner parameters.

## Main development checks

```bash
python -m scripts.planning.validation.validate_planner_redesign
python -m scripts.planning.validation.validate_planner_redesign_development
```

## Final/frozen evaluation only

```bash
python -m scripts.planning.evaluation.run_final_planner_evaluation
python -m scripts.planning.evaluation.run_canonical_runtime_benchmark
python -m scripts.planning.evaluation.build_planner_scenario_figures
python -m scripts.planning.evaluation.build_planner_summary_figure
```

## Development examples

```bash
python -m scripts.planning.development.run_optimization_tuning
python -m scripts.planning.development.run_sampling_tuning
python -m scripts.planning.development.diagnose_canonical_optimization
```
