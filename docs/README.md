# Documentation index

Use [root README](../README.md) for the portfolio overview.

| Question | Human-readable document |
| --- | --- |
| What is implemented / next? | [Roadmap](roadmap.md) |
| Planner feasibility, Cost, runtime and curvature? | [Planning benchmark](reports/planning_benchmark.md) |
| Safe Speed, Tracking Error, Steering Rate, feedback effort? | [Planning & Control benchmark](reports/planning_control_benchmark.md) |
| What did the 108-candidate Sampling development experiment test? | [Sampling development study](development/sampling_trackability_study.md) |

## Raw results and source code

| Data/source | Link |
| --- | --- |
| Planning per-scenario data | [CSV](../outputs/planning_benchmark/final_planner_evaluation.csv) · [JSON](../outputs/planning_benchmark/final_planner_evaluation_summary.json) |
| Planning & Control per-scenario data | [CSV](../outputs/planning_control_benchmark/final_planning_control_benchmark.csv) · [JSON](../outputs/planning_control_benchmark/final_planning_control_benchmark_summary.json) |
| Illustrated figure galleries (all images visible) | [Planning](../outputs/planning_benchmark/README.md) · [Planning & Control](../outputs/planning_control_benchmark/README.md) · [Development](../outputs/development/README.md) |
| Planning evaluation script | [run_final_planner_evaluation.py](../scripts/planning/evaluation/run_final_planner_evaluation.py) |
| Control evaluation metrics | [run_final_planning_control_benchmark.py](../scripts/mpc/run_final_planning_control_benchmark.py) |
| Safe-speed calculations | [src/control/path_tracking.py](../src/control/path_tracking.py) |
| Historical Sampling configuration | [JSON](development/sampling_trackability_config.json) · [command record](development/sampling_trackability_commands.md) |

`.md` files explain methods and outcomes; `.csv` files contain experiment tables (Excel); `.json` files are machine-readable summaries/configuration; `.png/.svg` are figures; `.py` are code. The historical commands are a record, **not** a one-click executable script.

The nine-case benchmark is a **frozen, previously inspected evaluation suite**, not a pristine unseen held-out suite. The DEVELOPMENT 108-candidate proposal is not the published 45-candidate final baseline.
