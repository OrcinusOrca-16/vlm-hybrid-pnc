# VLM-Guided Hybrid Autonomous Driving Planner — 开发路线

## Phase 0 — 项目初始化

- [x] 创建 GitHub 仓库
- [x] 配置 Python 开发环境
- [x] 配置 Git / SSH
- [x] 创建基础项目目录

---

## Phase 1 — Planning & Control 基线系统

### 1. 通用数据结构

建立 Planning & Control 模块之间统一的数据接口。

- [x] `VehicleState`
- [x] `ScenarioState`
- [x] `BehaviorType`
- [x] `BehaviorCommand`
- [x] `TrajectoryPoint`
- [x] `ControlCommand`

### 2. 基础场景建模

- [x] 正常直行
- [x] 前车减速
- [x] 行人横穿

### 3. Rule-based Behavior Planner

- [x] `KEEP_LANE`
- [x] `SLOW_DOWN`
- [x] `STOP`

### 4. 基础参考轨迹生成

- [x] 固定 `dt` 的直线参考轨迹
- [x] KEEP_LANE / SLOW_DOWN / STOP 验证
- [x] 加速度 / 减速度约束
- [x] 位置积分
- [x] 停车距离检查
- [x] 参考速度与参考加速度输出

### 5. Kinematic Bicycle Model

- [x] 推导 `x / y / yaw / speed` 状态方程
- [x] 前轮转角 + 纵向加速度输入
- [x] 固定 `dt` 离散化
- [x] 直线匀速验证
- [x] 加速 / 减速验证
- [x] 固定转角转弯验证

### 6. Longitudinal PID

- [x] 速度误差闭环
- [x] PID 控制器
- [x] 加速度限幅
- [x] anti-windup
- [x] 与车辆模型闭环
- [x] 加入 reference-acceleration feedforward
- [x] 加速 / 减速 / 目标变化验证

### 7. Lateral MPC

- [x] 定义 lateral / heading error
- [x] 推导线性化运动学自行车误差模型
- [x] 离散状态空间模型
- [x] 多步预测模型
- [x] Q / R quadratic cost
- [x] steering-angle constraint
- [x] steering-rate constraint
- [x] QP formulation
- [x] OSQP solve
- [x] receding-horizon first-control application
- [x] 初始 lateral-error recovery
- [x] 初始 heading-error recovery
- [x] baseline MPC calibration

### 8. Planning & Control 闭环仿真

- [x] Reference Trajectory → PID / MPC
- [x] PID → acceleration
- [x] MPC → steering
- [x] ControlCommand → vehicle model
- [x] VehicleState feedback
- [x] KEEP_LANE closed loop
- [x] SLOW_DOWN closed loop
- [x] STOP closed loop

### 9. 基础评价与可视化

- [x] speed MAE / RMSE / max error
- [x] lateral MAE / RMSE / max error
- [x] heading RMSE / max error
- [x] steering-rate / smoothness
- [x] 统一 P&C summary figure

---

## Phase 2 — Classical Trajectory Planning

### 10. Reference Line

建立统一 smooth Reference Line。

- [x] 离散 Reference Point
- [x] cumulative arc length `s`
- [x] yaw
- [x] curvature
- [x] cubic spline `x(s) / y(s)`
- [x] curvature derivative `κ'`
- [x] `ReferenceLine.query(s)`
- [x] straight / circular / varying-curvature validation

### 11. Cartesian ↔ Frenet / SL

- [x] XY → SL
- [x] SL → XY
- [x] raw-waypoint coarse localization
- [x] spline projection
- [x] smooth Reference Line geometry
- [x] straight / endpoint / circular / varying-curvature round-trip validation

### 12. Sampling-based Planner

基于 Frenet Sampling 生成空间 Candidate Paths，并使用统一物理约束筛选。

#### Frenet geometry

- [x] `s / l / l' / l''`
- [x] `FrenetTrajectoryPoint / FrenetTrajectory`
- [x] Candidate yaw
- [x] `1 - κ_r l` geometry
- [x] Candidate curvature
- [x] `κ_r'`
- [x] Frenet → Cartesian geometry
- [x] `PathPoint`

#### Candidate generation

- [x] target lateral positions
- [x] target maneuver lengths
- [x] quintic polynomial
- [x] ego initial `l / l' / l''`
- [x] bypass-and-return candidate family
- [x] common planning horizon
- [x] terminal state sampling

#### Physical constraints

- [x] vehicle footprint
- [x] static circular obstacle
- [x] exact sampled-pose obstacle clearance
- [x] exact sampled Cartesian curvature check
- [x] Frenet-based road-boundary pruning
- [x] infeasible candidate removal

#### Objective / selection

- [x] lateral deviation
- [x] `l' / l'' / l'''` smoothness terms
- [x] terminal `l / l'`
- [x] canonical matched objective
- [x] minimum-cost feasible candidate selection

#### Validation

- [x] straight nominal
- [x] curved Reference Line
- [x] lateral recovery
- [x] static obstacle avoidance
- [x] candidate visualization
- [x] planning time
- [x] minimum clearance
- [x] maximum curvature
- [x] path length
- [x] planning success rate

---

### 13. Optimization-based Path Planner — Initial Version

第一版 Optimization Planner 使用 `l`-only convex QP + finite differences + Frenet obstacle corridors。

这一阶段保留在路线图中作为后续 redesign 的历史基线。

#### Initial QP state / objective

- [x] optimization variable `l`
- [x] `D1` finite-difference approximation of `l'`
- [x] `D2` finite-difference approximation of `l''`
- [x] `D3` finite-difference approximation of `l'''`
- [x] quadratic lateral / smoothness objective
- [x] terminal objective
- [x] OSQP `P / q`

#### Initial constraints

- [x] road lateral bounds
- [x] initial `l` constraint
- [x] initial `l'` finite-difference constraint
- [x] initial `l''` finite-difference constraint
- [x] `D2` curvature-related proxy bound
- [x] OSQP `A / lower / upper`

#### Initial obstacle treatment

- [x] obstacle Frenet projection
- [x] obstacle inflation
- [x] negative / positive lateral corridor branches
- [x] corridor feasibility check
- [x] multi-corridor QP solve
- [x] final physical post-validation

#### Initial validation

- [x] nominal solve
- [x] curved Reference Line
- [x] lateral maneuver
- [x] static obstacle avoidance
- [x] corridor / optimized-path visualization

---

### 14. Sampling vs Optimization Unified Benchmark

建立相同 Behavior intent 下的公平 planner comparison。

#### 14.1 Centralized Configuration

- [x] `src/config/` centralized configuration
- [x] vehicle geometry
- [x] road / planning constraints
- [x] Sampling parameters
- [x] Optimization parameters
- [x] OSQP / numerical tolerance
- [x] benchmark runtime settings

#### 14.2 Matched Objective

统一 geometric objective：

`J = w_l J_l + w_dl J_dl + w_d2l J_d2l + w_d3l J_d3l + w_terminal_l J_terminal_l + w_terminal_dl J_terminal_dl`

- [x] `l`
- [x] `l'`
- [x] `l''`
- [x] `l'''`
- [x] terminal `l`
- [x] terminal `l'`
- [x] mean-square normalization
- [x] planner-independent canonical objective evaluator

#### 14.3 Shared physical acceptance

两种 Planner 使用同一最终 acceptance contract：

- [x] exact sampled Cartesian curvature check
- [x] exact sampled-pose footprint-to-obstacle clearance
- [x] Frenet-based road-boundary validation
- [x] initial-state consistency

> 注意：production road-boundary check 不是 rotated full-footprint exact containment。它使用 Frenet centerline ± half-width contract。后续额外加入 rotated-corner diagnostic，但该 diagnostic 不改变 production PASS / FAIL。

#### 14.4 Unified benchmark interface

- [x] `PlannerBenchmarkResult`
- [x] success / failure
- [x] Frenet trajectory
- [x] Cartesian path
- [x] planning time
- [x] failure reason
- [x] canonical matched objective
- [x] Sampling adapter
- [x] Optimization adapter

#### 14.5 Common metrics

- [x] planning success
- [x] canonical objective
- [x] terminal lateral error
- [x] terminal slope error
- [x] RMS lateral deviation
- [x] Cartesian path length
- [x] minimum exact obstacle clearance
- [x] maximum absolute Cartesian curvature
- [x] RMS curvature
- [x] RMS curvature rate
- [x] planning computation time

#### 14.6 Benchmark visualization

- [x] Cartesian overlay
- [x] Frenet `s-l` overlay
- [x] road boundaries
- [x] original obstacle
- [x] Sampling best path
- [x] Optimization path

#### 14.7 Runtime / stress benchmark

- [x] warm-up / repeated timing
- [x] full Sampling pipeline timing
- [x] full Optimization pipeline timing
- [x] nominal / curved / obstacle / narrow-road scenarios
- [x] obstacle lateral sweep
- [x] obstacle longitudinal sweep
- [x] road-width sweep
- [x] failure recording

#### 14.8 Tuning methodology

- [x] independent tuning scenario set
- [x] freeze Sampling candidate/config search space
- [x] freeze Optimization tuning parameters
- [x] avoid tuning on final evaluation scenarios

#### 14.9 Original frozen evaluation

Original 9-scenario evaluation result before physical-consistency redesign:

- Sampling: `8 / 9`
- Optimization: `8 / 9`
- shared failure: `evaluation_road_half_width_1.85`

The `1.85 m` case was already part of the original evaluation geometry; it was not added later.

---

### 15. Optimization Planner Physical-Consistency Redesign

这一阶段由 **Planning → MPC / vehicle consistency** 触发，而不是为了提高 benchmark 分数。

#### 15.1 Vehicle-derived curvature limit

旧 benchmark 使用过宽松的：

`κ_max = 1.0 1/m`

根据车辆参数：

- wheelbase `L = 2.7 m`
- max steering `δ_max = 0.5 rad`

改为：

`κ_max = tan(δ_max) / L ≈ 0.202334 1/m`

- [x] 使用 vehicle config 推导 curvature limit
- [x] Planning acceptance 与 vehicle/MPC physical capability 对齐
- [x] exact Cartesian curvature post-validation

#### 15.2 Root-cause diagnosis

真实曲率限制暴露旧 Optimization formulation 的结构性问题。

- [x] 对比 Sampling 与 Optimization 在相同 physical constraints 下的 feasibility
- [x] 确认 close-obstacle scene 并非天然物理不可行
- [x] 识别 `l`-only + finite-difference initial constraints 的 early-path coupling / start-locking
- [x] 识别 rectangular Frenet corridor 对 vehicle heading freedom 的保守性
- [x] 判断问题属于 formulation limitation，而非简单 objective-weight tuning

#### 15.3 Explicit Frenet state

新 Optimization state：

`x = [l, l', l'']`

- [x] explicit `l`
- [x] explicit `l'`
- [x] explicit `l''`
- [x] direct initial-state constraints

#### 15.4 Piecewise-constant-jerk dynamics

- [x] `j_i = (l''_{i+1} - l''_i) / Δs`
- [x] `l'_{i+1} = l'_i + Δs/2 (l''_i + l''_{i+1})`
- [x] `l_{i+1} = l_i + Δs l'_i + Δs²/3 l''_i + Δs²/6 l''_{i+1}`
- [x] linear equality constraints for QP

#### 15.5 Heading-aware obstacle geometry

- [x] Frenet heading relation `tan(Δψ) = l' / (1 - κ_r l)`
- [x] heading-aware nominal obstacle constraints
- [x] exact rotated vehicle rectangle vs circular obstacle clearance
- [x] sampled-pose exact clearance validation

#### 15.6 Sequential convex QP refinement

- [x] numerical clearance gradient w.r.t. `l`
- [x] numerical clearance gradient w.r.t. `l'`
- [x] local linearized clearance constraint
- [x] trust region in `l`
- [x] trust region in `l'`
- [x] iterative QP refinement
- [x] exact revalidation after each solve

Frozen redesign settings include:

- `sequential_max_iterations = 6`
- `clearance_numerical_buffer_m = 0.001`
- `trust_region_l_m = 0.15`
- `trust_region_dl = 0.08`
- clearance gradient epsilon `= 1e-4`

#### 15.7 Development-only regression

- [x] dedicated `validate_planner_redesign_development.py`
- [x] development regression does not build or run final evaluation scenarios
- [x] required development/tuning scenarios: `7 / 7 PASS`
- [x] stress envelope: `19` cases
- [x] production-valid stress successes: `16`
- [x] capability-boundary / infeasible stress cases: `3`
- [x] near-boundary diagnostics
- [x] independent rotated-footprint road diagnostic

#### 15.8 Road-boundary limitation documented

Production road acceptance remains:

`Frenet centerline ± vehicle half-width`

- [x] keep shared production road contract unchanged for both planners
- [x] add rotated-corner diagnostic warning
- [x] document cases where rotated corners can exceed the simplified road contract
- [ ] future: exact rotated-footprint production road-boundary constraint

Correct project wording:

> Exact sampled-pose obstacle clearance and Cartesian curvature validation, with Frenet-based road-boundary validation.

#### 15.9 Post-redesign frozen evaluation

Same 9 scenario geometries rerun after redesign:

- Sampling: `8 / 9`
- Optimization: `9 / 9`
- Sampling failure: `evaluation_road_half_width_1.85`
- Optimization: no evaluation failure

Methodological note:

Because some evaluation cases were inspected during diagnosis, this set is better described as a **frozen evaluation suite** rather than a pristine unseen held-out set.

- [x] do not expand/tune Sampling candidate set after observing `1.85 m` result
- [x] compare path quality only on common-success scenarios
- [x] archive previous figures
- [x] save source provenance
- [x] update benchmark documentation

---

### 16. Curvature-aware MPC + Planning Integration

Phase 1 MPC 保留为 baseline。当前 spatial planned path 接入 common curvature-aware controller；纵向 speed-transition dynamics 暂不纳入本阶段范围。

- [x] 从 planned path 获取 reference curvature
- [x] steering curvature feedforward
- [x] MPC feedback correction
- [x] spatial path → constant safe reference speed
- [x] curved-path tracking validation
- [x] Sampling path + common MPC
- [x] Optimization path + common MPC
- [x] 检查 planner curvature 与 MPC steering constraint 一致性
- [x] 评估 horizon / Q / R tuning 必要性：当前验证无系统搜索必要
- [ ] Future: planner-level steering-rate / curvature-rate feasibility at design speed

---

### 17. Final Planning & Control Benchmark

使用统一 controller 比较两种 Planner 的闭环系统表现。

- [x] Sampling Planner + common MPC
- [x] Optimization Planner + common MPC
- [x] lateral tracking error
- [x] heading error
- [x] steering / steering rate
- [x] trajectory quality
- [x] planning computation time
- [x] controller feasibility
- [x] failure cases

Final frozen 9-scenario result:

- Sampling Planner: `8 / 9` planning success; all `8 / 8` successful plans tracked successfully.
- Optimization Planner: `9 / 9` planning success and closed-loop tracking success.
- Common-success comparisons use the same controller and an auxiliary common-speed diagnostic.
- Final interpretation and limitations are documented in `docs/final_planning_control_findings.md`.

---

## Phase 3 — C++ Core Engineering

将核心数据结构、Reference Line、Frenet、车辆模型、MPC 和 Planner 迁移到 C++，并验证 Python / C++ 数值一致性。

## Phase 4 — Dynamic Scenarios & System Evaluation

加入动态车辆、行人、lead-vehicle slowdown、cut-in 等场景，并建立 collision rate、task completion、minimum distance 等系统级指标。

## Phase 5 — Hybrid Planner（Optional）

Sampling 提供可行解 / initial guess，再由 Optimization refinement，并与两种独立 Planner 做统一 Benchmark。

## Phase 6 — Safety & Fallback

建立独立 deterministic safety checker、planning failure fallback 和安全介入统计。

## Phase 7 — VLM Behavior Layer

VLM 只负责高层 `KEEP_LANE / SLOW_DOWN / STOP / CAUTIOUS` 等 `BehaviorCommand`，不直接生成 control command。

## Phase 8 — Rule-based vs VLM Evaluation

在相同场景和相同下游 Planning & Control 下比较规则决策与 VLM 的正确率、延迟、安全介入和 failure cases。

## Phase 9 — Portfolio Evaluation & Documentation

- [ ] README 重构
- [ ] 系统架构图
- [ ] Planner benchmark figure
- [x] Planning–Control integration results
- [x] failure-case documentation
- [ ] interview / presentation material
- [ ] GIF / demo
