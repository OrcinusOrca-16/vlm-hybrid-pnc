# Historical Sampling Trackability Commands

This is a PowerShell **historical command record** from 2026-11-01, not a one-click current reproduction script. Baseline and tuned commands were run against differing code states; verify each command and historical module path before using it. Original base commit: `159d72baccf124e85f5642cdff0ca52fd1201d25`.

See [development study](sampling_trackability_study.md) and [configuration snapshot](sampling_trackability_config.json).

```powershell
# Historical execution record, not a one-click replay script.
# Baseline adapter/runtime commands ran before production edits.
# For historical production baseline, use starting commit 159d72baccf124e85f5642cdff0ca52fd1201d25.
# Run from repository root using its .venv.
.\.venv\Scripts\python.exe -m scripts.planning.validation.validate_reference_line
.\.venv\Scripts\python.exe -m scripts.planning.validation.validate_frenet
.\.venv\Scripts\python.exe -m scripts.planning.development.run_sampling_trackability_tuning > outputs\sampling_trackability_initial.log
.\.venv\Scripts\python.exe -u -m scripts.planning.development.run_sampling_development_study --phase broad > outputs/sampling_study_broad.log
.\.venv\Scripts\python.exe -u -m scripts.planning.development.run_sampling_development_study --phase refine > outputs/sampling_study_refine.log
.\.venv\Scripts\python.exe -m scripts.planning.evaluation.run_canonical_runtime_benchmark > outputs/sampling_runtime_baseline.log
.\.venv\Scripts\python.exe -u -m scripts.planning.validation.validate_sampling_trackability_development --label baseline --expected 22 > outputs/sampling_baseline_regression.log
.\.venv\Scripts\python.exe -u -m scripts.planning.development.run_sampling_development_study --phase diagnose > outputs/sampling_study_diagnose.log
.\.venv\Scripts\python.exe -u -m scripts.planning.development.analyze_sampling_study > outputs/sampling_study_targeted.log
.\.venv\Scripts\python.exe -u -m scripts.planning.development.run_sampling_development_study --phase junction > outputs/sampling_study_junction.log
.\.venv\Scripts\python.exe -u -m scripts.planning.validation.validate_sampling_dense > outputs/sampling_study_dense_validation.log
.\.venv\Scripts\python.exe -u -m scripts.planning.development.finalize_sampling_study > outputs/sampling_study_finalists.log
.\.venv\Scripts\python.exe -u -m scripts.planning.validation.validate_sampling_trackability_development --label tuned --expected 22 > outputs/sampling_tuned_regression.log
.\.venv\Scripts\python.exe -m scripts.planning.validation.validate_sampling_trackability_features

$taskModules=@('validate_sampling_planner','validate_sampling_benchmark_adapter','validate_matched_sampling_cost','validate_tuning_objective_separation','validate_common_physical_validator','validate_canonical_benchmark','validate_discrete_scenario_benchmark','validate_stress_benchmark','validate_runtime_benchmark')
$taskResults=@()
foreach($taskModule in $taskModules){
    & .\.venv\Scripts\python.exe -m ('scripts.planning.'+$taskModule) *> ('outputs/sampling_'+$taskModule+'.log')
    $taskCode=$LASTEXITCODE
    $taskResults+=@{module=$taskModule;exit_code=$taskCode}
    Write-Output ($taskModule+' exit='+$taskCode)
}
$taskResults | ConvertTo-Json | Set-Content outputs/sampling_development_checks.json
if(@($taskResults | Where-Object {$_.exit_code -ne 0}).Count){exit 1}

.\.venv\Scripts\python.exe -m scripts.planning.validation.validate_matched_sampling_cost > outputs/sampling_validate_matched_sampling_cost.log
.\.venv\Scripts\python.exe -m scripts.planning.evaluation.run_canonical_runtime_benchmark > outputs/sampling_runtime_tuned.log
.\.venv\Scripts\python.exe -m scripts.mpc.validate_sampling_path_tracking > outputs/sampling_closed_loop.log
.\.venv\Scripts\python.exe -m scripts.planning.development.diagnose_sampling_junction_slope > outputs/sampling_junction_legacy_regression.log
.\.venv\Scripts\python.exe -c "from scripts.planning.development.run_sampling_tuning import build_structure_configs, run_one_scenario; from src.benchmark.tuning_scenario_suite import build_development_tuning_scenarios; from src.config.planning import DEFAULT_SAMPLING_PLANNER_CONFIG; c=build_structure_configs()[0]; s=build_development_tuning_scenarios()[0]; r=run_one_scenario(c,s,DEFAULT_SAMPLING_PLANNER_CONFIG.matched_objective_weights); print(c.name,c.candidate_count,r.success); assert r.success"
.\.venv\Scripts\python.exe -m scripts.planning.development.build_sampling_report
.\.venv\Scripts\python.exe -u -m scripts.planning.validation.validate_sampling_trackability_development --label tuned_verified --expected 22 --verify-recommendation > outputs/sampling_tuned_verified.log
git rev-parse HEAD
git status --short
git diff --check
git diff --stat
```
