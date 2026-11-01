"""Package measured DEVELOPMENT evidence into a reviewable report."""
import json
import math
from collections import Counter
from dataclasses import replace
from pathlib import Path
from statistics import mean

from scripts.planning.run_sampling_development_study import OUT, scenarios, measure


def main():
    broad=json.loads((OUT/'broad.json').read_text())
    baseline=next(r for r in broad if r['name']=='r2.00_d0.50/dense/strict/w0.0')
    finalists=json.loads((OUT/'finalists.json').read_text())
    chosen=next(r for r in finalists if r['name']=='d05/pruned/s005/w4.0/r0.0')
    ss=scenarios()
    fine_baseline={}
    for s in ss:
        if s.name not in baseline['selected']:continue
        dense=replace(s,resolution_m=.05)
        object.__setattr__(dense,'_study_initial',s._study_initial)
        fine_baseline[s.name]=measure(dense,tuple(baseline['selected'][s.name]['params']))
    assert all(all(r['flags']) for r in fine_baseline.values())
    chosen_dense={r['name']:r['row'] for r in json.loads((OUT/'finalist_dense_rows.json').read_text())
                  if r['name'] in chosen['selected'] and r['params']==chosen['selected'][r['name']]['params']}
    stats={}
    for label,coarse,fine in [('baseline',baseline,fine_baseline),('recommended',chosen,chosen_dense)]:
        stats[label]=dict(coarse={k:v for k,v in coarse.items() if k!='selected'},
            dense=dict(success=len(fine), mean_speed=mean(r['speed'] for r in fine.values()),
                       min_speed=min(r['speed'] for r in fine.values()),
                       clearance=min(r['clearance'] for r in fine.values()),
                       curvature_margin=min(r['curvature_margin'] for r in fine.values()),
                       steering_margin=min(r['steering_margin'] for r in fine.values())),
            mean_rms_curvature=mean(r['rms_k'] for r in coarse['selected'].values()),
            mean_rms_curvature_rate=mean(r['rms_krate'] for r in coarse['selected'].values()),
            max_terminal_l=max(abs(r['terminal_l']) for r in coarse['selected'].values()),
            max_terminal_dl=max(abs(r['terminal_dl']) for r in coarse['selected'].values()),
            max_terminal_d2=max(abs(r['terminal_d2']) for r in coarse['selected'].values()))
    (OUT/'comparison.json').write_text(json.dumps(stats,indent=2))
    freeze=dict(status='recommended DEVELOPMENT configuration; awaiting user review before final evaluation',
        starting_commit='159d72baccf124e85f5642cdff0ca52fd1201d25',
        lateral_positions=chosen['lat'],bypass_lengths=chosen['lengths'],junction_slopes=chosen['slopes'],
        junction_second_derivative=0.,terminal_state=[0.,0.,0.],trackability_weight=4.,
        geometric_weights_unchanged=True,hard_constraints_unchanged=True,
        development_success=22,development_total=26,failures=chosen['failures'],
        selected={n:dict(params=r['params'],speed=r['speed'],geo=r['geo'],clearance=r['clearance'] if math.isfinite(r['clearance']) else None)
                  for n,r in chosen['selected'].items()})
    Path('docs/sampling_trackability_development_config.json').write_text(json.dumps(freeze,indent=2,allow_nan=False))
    alternatives=[baseline,
        next(r for r in finalists if r['name']=='d05/tiny/s005/w4.0/r0.0'), chosen,
        next(r for r in finalists if r['name']=='d05/base/s005/w4.0/r0.0'),
        next(r for r in finalists if r['name']=='nonuniform12/tiny/s0075/w4.0/r0.0'),
        next(r for r in broad if r['name']=='r2.00_d0.40/dense/slope_015/w7.5')]
    table='| 配置 | 候选数 | 粗网格覆盖 | 平均安全速度 | 早障碍速度 | 平均几何代价 | 最小净空 |\n|---|---:|---:|---:|---:|---:|---:|\n'
    for r in alternatives:
        table+=f"| `{r['name']}` | {r['count']} | {r['success']}/26 | {r['mean_speed']:.4f} | {r['early_speed']:.4f} | {r['geo']:.6f} | {r['clearance']:.6f} |\n"
    fine_table='| 0.05 m 诊断 | 基线 | 推荐配置 |\n|---|---:|---:|\n'
    for name,key in [('成功路径','success'),('平均安全速度 m/s','mean_speed'),('最小安全速度 m/s','min_speed'),
                     ('最小净空 m','clearance'),('最小曲率裕量 1/m','curvature_margin'),('最小转角裕量 rad','steering_margin')]:
        a,b=stats['baseline']['dense'][key],stats['recommended']['dense'][key]
        fine_table+=f'| {name} | {a:.6f} | {b:.6f} |\n'
    commands=r'''# 在 E:\file\work\P&C Portfolio\vlm-hybrid-pnc 执行；使用仓库 .venv。
.\.venv\Scripts\python.exe -m scripts.planning.validate_reference_line
.\.venv\Scripts\python.exe -m scripts.planning.validate_frenet
.\.venv\Scripts\python.exe -m scripts.planning.run_sampling_trackability_tuning > outputs\sampling_trackability_initial.log
.\.venv\Scripts\python.exe -u -m scripts.planning.run_sampling_development_study --phase broad > outputs/sampling_study_broad.log
.\.venv\Scripts\python.exe -u -m scripts.planning.run_sampling_development_study --phase refine > outputs/sampling_study_refine.log
.\.venv\Scripts\python.exe -m scripts.planning.run_canonical_runtime_benchmark > outputs/sampling_runtime_baseline.log
.\.venv\Scripts\python.exe -u -m scripts.planning.validate_sampling_trackability_development --label baseline --expected 22 > outputs/sampling_baseline_regression.log
.\.venv\Scripts\python.exe -u -m scripts.planning.run_sampling_development_study --phase diagnose > outputs/sampling_study_diagnose.log
.\.venv\Scripts\python.exe -u -m scripts.planning.analyze_sampling_study > outputs/sampling_study_targeted.log
.\.venv\Scripts\python.exe -u -m scripts.planning.run_sampling_development_study --phase junction > outputs/sampling_study_junction.log
.\.venv\Scripts\python.exe -u -m scripts.planning.check_sampling_dense_validation > outputs/sampling_study_dense_validation.log
.\.venv\Scripts\python.exe -u -m scripts.planning.finalize_sampling_study > outputs/sampling_study_finalists.log
.\.venv\Scripts\python.exe -u -m scripts.planning.validate_sampling_trackability_development --label tuned --expected 22 > outputs/sampling_tuned_regression.log
.\.venv\Scripts\python.exe -m scripts.planning.validate_sampling_trackability_features

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

.\.venv\Scripts\python.exe -m scripts.planning.validate_matched_sampling_cost > outputs/sampling_validate_matched_sampling_cost.log
.\.venv\Scripts\python.exe -m scripts.planning.run_canonical_runtime_benchmark > outputs/sampling_runtime_tuned.log
.\.venv\Scripts\python.exe -m scripts.mpc.validate_sampling_path_tracking > outputs/sampling_closed_loop.log
.\.venv\Scripts\python.exe -m scripts.planning.diagnose_sampling_junction_slope > outputs/sampling_junction_legacy_regression.log
.\.venv\Scripts\python.exe -c "from scripts.planning.run_sampling_tuning import build_structure_configs, run_one_scenario; from src.benchmark.tuning_scenario_suite import build_development_tuning_scenarios; from src.config.planning import DEFAULT_SAMPLING_PLANNER_CONFIG; c=build_structure_configs()[0]; s=build_development_tuning_scenarios()[0]; r=run_one_scenario(c,s,DEFAULT_SAMPLING_PLANNER_CONFIG.matched_objective_weights); print(c.name,c.candidate_count,r.success); assert r.success"
.\.venv\Scripts\python.exe -m scripts.planning.build_sampling_report
.\.venv\Scripts\python.exe -u -m scripts.planning.validate_sampling_trackability_development --label tuned_verified --expected 22 --verify-recommendation > outputs/sampling_tuned_verified.log
git rev-parse HEAD
git status --short
git diff --check
git diff --stat
'''
    Path('docs/sampling_trackability_commands.ps1').write_text(commands,encoding='utf-8')
    report=f'''Sampling Planner DEVELOPMENT 调优报告（2026-11-01）

工作仓库：`E:\\file\\work\\P&C Portfolio\\vlm-hybrid-pnc`。起始提交：`159d72baccf124e85f5642cdff0ca52fd1201d25`；开始时工作树干净。推荐参数已实现，等待用户审阅后决定最终冻结及最终评测。本次没有运行最终评测，没有构建、调用或读取 held-out 场景定义，没有使用被禁止的道路场景调参；没有修改场景、物理阈值、车辆参数、MPC 权重/时域或 Optimization Planner 生产逻辑。

1. **基线与评价口径。** DEVELOPMENT 共 26 个场景，未移除任何初始状态无效场景。基线 22/26，45 条候选，平均安全速度 4.312302 m/s，最小/早障碍速度 0.897230 m/s，平均几何代价 0.5275265，最小净空 0.5363775 m。四个失败场景见下文。参考线与 Frenet 验证均通过；生产路径及硬约束检查间隔为 0.5 m，额外 0.05 m 复核仅用于 DEVELOPMENT 诊断。几何代价沿用独立 canonical evaluator；均值对比使用相同 22 个成功场景。23/26 的均值与 22/26 不直接排名。

2. **实验及迭代。** 先复现已有联合搜索，再缓存逐候选几何和独立约束标志。广搜覆盖 0.5/0.4/0.25 m 横向网格、5 组绕行长度、零及 ±0.05/±0.10/±0.15 斜率与权重 0–25；随后添加保留 ±1.0 和 ±1.5 的非均匀采样、长度至 26 m、0.025 斜率精化及 ±0.3 诊断。重点诊断对每个窄路/晚障碍场景测试 14,040 组零二阶导数候选（长度 4–27 m）。零二阶导数饱和后，单独测试交接二阶导数 0、±0.025、±0.05、±0.1：晚障碍 2,744 组，两个窄路各 3,773 组。最后对低候选数方案进行 3/3.5/4/4.5/5/6/7.5/10 权重精化，并消融连续净空软项 `r*(0.5/clearance)^2`，r=0/0.25/0.5/1。所有参数筛选仅使用 DEVELOPMENT 数据。终端始终 (0,0,0)。

{table}

3. **关键的细网格发现。** 晚障碍 `[-1.6,22,0.15,0]` 粗网格净空 0.500642 m，细网格只有 0.485458 m；细网格曲率裕量只剩 0.002266 1/m。更广诊断中的 108 条粗网格晚障碍可行候选（包括两条非零交接二阶导数候选）在 0.05 m 复核时全部失败。27 m 绕行、1 m 返回段甚至出现粗网格安全速度 5 m/s 的混叠假象。早障碍 `[-1.5,8,0,0]` 的净空从 0.514243 降到 0.475886 m；`[-1,6,-0.1,0]` 从 0.527376 降到 0.498691 m。它们均未进入推荐配置。±0.075 方案虽可通过，但最小净空约 0.5085 m，裕量明显偏低。停止为这些边缘粗网格 PASS 增加复杂度。

4. **推荐配置及理由。** 横向 `[-2,-1.5,-1,-0.5,0,0.5,1,1.5,2]` m；绕行长度 `[6,10,14,18]` m；交接斜率 `[-0.05,0,0.05]`；交接二阶导数 0；终端 (0,0,0)；`J_select=J_geo+4*J_speed`；原几何权重不变。108 条候选。22/26 的覆盖及失败集合均保持，所有成功路径通过细网格复核。平均安全速度提升约 5.8%，早障碍约 12.2%，几何代价约增加 4.9%；最小净空保持，曲率及转角裕量改善。权重 3.5–5 处于相同选择平台，取 4；不选择最大权重。22 m 长度未被选中，删除它将 135 条降至 108 条，选中路径及全部质量指标不变。保留 10 m 长度可把 81 条方案的平均几何代价从 0.607591 降到 0.553479。非均匀或更密网格可改善部分几何代价，但增加数量并降低最小净空。净空软项没有提供足够收益，未引入生产。±0.05 已改善早障碍，未推广更大斜率或非零二阶导数。

{fine_table}

细网格下安全速度仍较低：早障碍基线 0.683968、推荐 0.776710 m/s。生产和下游仍使用其实际离散路径的 steering-gradient 速度规则，未把 5 m/s 跟踪可行性改成规划硬约束。细网格数据不代表连续时间保证，也未偷偷修改共享 validator。若未来改变插值或分辨率，必须重新做速度限制与净空验证。

5. **其他质量指标。** 原网格下平均 RMS 曲率 {stats['baseline']['mean_rms_curvature']:.6f} → {stats['recommended']['mean_rms_curvature']:.6f} 1/m，平均 RMS 曲率变化率 {stats['baseline']['mean_rms_curvature_rate']:.6f} → {stats['recommended']['mean_rms_curvature_rate']:.6f} 1/m²。最大终端横向/一阶/二阶误差分别为 {stats['recommended']['max_terminal_l']:.3g}、{stats['recommended']['max_terminal_dl']:.3g}、{stats['recommended']['max_terminal_d2']:.3g}，均为数值舍入量级。基线候选总计 1170：曲率拒绝 207、净空拒绝 762、道路拒绝 37、可行 164（14.02%）；推荐总计 2808：曲率拒绝 310、净空拒绝 1947、道路拒绝 116、可行 435（15.49%）。拒绝计数按曲率→净空→道路顺序互斥归类。

6. **剩余失败原因。** `obstacle_longitudinal_16.00`：初始精确净空 −0.087689 m，已经违反 0.5 m 要求，无法通过后续路径消除初始违规，仍计 FAIL。`road_half_width_1.75`：采样家族中同时满足道路及曲率约束的最大净空为 0.45 m，净空可行候选会越道路边界；密度和交接二阶导数均未改善，属于所测家族的道路/净空冲突，未声称已数学证明所有路径均不可行。`road_half_width_1.50`：同类限制，诊断最大净空约 0.20 m。`obstacle_longitudinal_36.00`：固定终端和短返回段在当前两段 quintic 家族下受曲率/净空联合限制；新增粗网格解未通过细网格验证。继续搜索的收益不支持候选数量膨胀。

7. **运行时间。** 完整生产适配器的 canonical 基准（5 次预热、50 次测量，不使用诊断缓存）：基线均值 188.3557 ms、中位 188.3082、P95 195.0896；推荐均值 441.9098 ms、中位 440.7716、P95 453.5068。候选数 45→108（2.4 倍），平均时间约 2.35 倍。26 场景单次回归均值 188.997→448.330 ms。增加的运行时间是真实代价，此结果不等于已满足高频实时规划需求。缓存搜索时间从未作为生产时间。

8. **验证结果。** Sampling 生成器、适配器、matched cost、canonical objective separation、共享物理 validator、canonical 基准、7 场景离散基准、19 场景 stress、runtime 工具及新增交接/软成本集成验证最终全部通过。两项历史夹具显式固定零交接斜率以维持原来的测试意图；matched-cost 测试首次因新增采样改变选择而失败，修正夹具后通过。新增调优脚本最初遇到 NumPy bool JSON 序列化错误，改为 Python bool 后重跑。未使用失败或未完成运行的数据作推荐依据。旧几何调优 runner 显式保留零交接/零跟踪权重，以维持候选计数、缓存和选路语义一致；旧交接诊断亦显式复现零斜率基线。

已有 Sampling→curvature-aware MPC 闭环开发验证也通过：早障碍 v_ref=1.006821 m/s，最大横向误差 0.0175 m、最终横向误差约 0、最大转角 0.3963 rad、最大实际转角速率 0.5055 rad/s；弯道障碍 v_ref=5 m/s，最大横向误差 0.0285 m、最终误差 −0.0137 m、最大转角 0.1619 rad、最大速率 0.5001 rad/s。存在约 0.0055 rad/s 的数值超调，处于原测试已存在的 0.02 rad/s 容差内；没有更改 0.5 rad/s 限值或任何控制配置。该测试从安全参考速度直接初始化，未验证从 5 m/s 减速的瞬态；也没有将闭环轨迹描述为已做全程碰撞验证。

9. **精确生产改动。** `src/config/planning.py`：推荐长度、斜率、跟踪权重；`src/planning/sampling_planner.py`：交接斜率参数、三层接口透传及采样循环，保持交接 C² 连续、终端零状态；`src/planning/soft_cost.py`：可选非负路径软偏好回调和归一化速度缺额函数；`src/benchmark/sampling_adapter.py`：从共享车辆配置取得转角/速率限值、按实际车辆轴距计算速度偏好并接入选择。Planning 不依赖 MPC config；canonical 结果仍由独立几何 evaluator 计算。下游 min(target,safe) 防护和 Optimization Planner 不变。其他改动为实验/验证脚本、两项历史测试夹具、文档及生成数据忽略规则。

10. **复现与交付状态。** 完整实验/验证命令见 `docs/sampling_trackability_commands.ps1`；参数和各场景选择见 `docs/sampling_trackability_development_config.json`。逐候选缓存、所有摘要及细网格证据在 `outputs/sampling_development_study/`，日志在 `outputs/sampling_*.log`；它们是本地生成的开发证据，未纳入 Git。工作树保持 unstaged；没有 add、commit 或 push。最终 held-out run 留给用户审阅冻结后执行。
'''
    Path('docs/sampling_trackability_development.md').write_text(report,encoding='utf-8')
    print(json.dumps({k:v['dense'] for k,v in stats.items()},indent=2))


if __name__=='__main__':main()
