# VLM-Guided Hybrid Autonomous Driving Planner — 开发路线

## Phase 0 — 项目初始化
- [x] 创建 GitHub 仓库
- [x] 配置 Ubuntu / Python 开发环境
- [x] 配置 Git 和 SSH
- [x] 创建基础项目目录


## Phase 1 — Planning & Control 基线系统
### 1. 通用数据结构
定义 Planning & Control 各模块之间统一使用的数据接口。
包括：
- [x] VehicleState
- [x] ScenarioState
- [x] BehaviorType
- [x] BehaviorCommand
- [x] TrajectoryPoint

### 2. 基础场景建模
使用 VehicleState 和 ScenarioState 构造可重复运行的基础驾驶测试场景。
包括：
- [x] 正常直行
- [x] 前车减速
- [x] 行人横穿

### 3. 规则行为决策器
根据 VehicleState 和 ScenarioState 使用确定性规则生成 BehaviorCommand。
第一版支持：
- [x] KEEP_LANE
- [x] SLOW_DOWN
- [x] STOP

### 4. 简单参考轨迹生成器
根据 VehicleState 和 BehaviorCommand，沿直线路径生成由多个 TrajectoryPoint 组成的参考轨迹和目标速度曲线。
- [x] 固定 `dt` 的直线参考轨迹
- [x] KEEP_LANE / SLOW_DOWN / STOP 验证
- [x] 加速度 / 减速度约束
- [x] 更合理的位置积分
- [x] 停车距离检查
- [x] 三种行为统一验证

### 5. 运动学自行车模型
使用前轮转角和纵向加速度作为输入，根据运动学自行车模型更新车辆的位置、航向角和速度。
- [x] 定义 ControlCommand
- [x] 理解运动学自行车模型的状态与控制输入
- [x] 推导 x / y / yaw / speed 状态方程
- [x] 使用固定 `dt` 离散化车辆模型
- [x] 实现单步车辆状态更新
- [x] 验证直线匀速运动
- [x] 验证纵向加速 / 减速
- [x] 验证固定转角下的转弯运动

### 6. 纵向 PID 控制
根据参考速度和当前车辆速度计算纵向加速度指令，使车辆跟踪目标速度。
包括：
- [x] 理解纵向速度控制闭环
- [x] 定义速度误差
- [x] 理解 P / I / D 三项作用
- [x] 实现 PIDController
- [x] 加速度输出限幅
- [x] 实现积分项防饱和
- [x] 与运动学自行车模型闭环连接
- [x] 验证加速跟踪
- [x] 验证减速跟踪
- [x] 验证目标速度变化
- [x] 比较不同 Kp / 时间常数的响应
- [x] 讨论 PI / PID 是否有必要

### 7. 横向 MPC 控制
根据 VehicleState 和参考轨迹建立预测模型和优化问题，计算前轮转角，实现横向轨迹跟踪。
- [x] 理解横向控制闭环
- [x] 定义横向误差和航向误差
- [x] 推导横向误差状态方程
- [x] 线性化运动学自行车模型
- [x] 离散化并写成状态空间
- [x] 构造 MPC 多步预测模型
- [x] 定义状态误差和转向代价
- [x] 加入转角约束
- [x] 转换为 QP
- [x] 实现 MPCController
- [x] 与运动学自行车模型闭环连接
- [x] 加入转角变化量约束（扩展）
- [x] 再接回 MPCController
- [x] 验证直线轨迹跟踪
- [x] 验证存在初始横向偏差
- [x] 验证存在初始航向偏差

### 8. Planning & Control 闭环仿真
将参考轨迹、纵向 PID、横向 MPC 和车辆模型连接成完整 Planning & Control 闭环。
- [x] 根据当前时间获取参考轨迹点
- [x] 计算纵向速度误差
- [x] 计算横向误差和航向误差
- [x] PID 生成加速度指令
- [x] MPC 生成转向指令
- [x] 合成 ControlCommand
- [x] 更新车辆状态
- [x] 完整闭环循环
- [x] 验证 KEEP_LANE
- [x] 验证 SLOW_DOWN
- [x] 加入纵向参考加速度 feedforward
- [x] 验证 STOP

### 9. 基础评价与可视化
对 Planning & Control 闭环的跟踪性能和控制输出进行定量评价。
- [x] 记录完整闭环仿真数据
- [x] 绘制统一 P&C summary
- [x] 计算速度 MAE / RMSE / max error
- [x] 计算横向 MAE / RMSE / max error
- [x] 计算航向 RMSE / max error
- [x] 评价 steering rate / smoothness
- [x] 汇总 KEEP_LANE / SLOW_DOWN / STOP 场景结果

### MPC Baseline Calibration
得到一套稳定、合理、可重复的 baseline MPC 参数。
- [x] 记录当前 MPC 参数
- [x] 建立 baseline evaluation cases
- [x] 检查 lateral tracking error
- [x] 检查 heading error
- [x] 检查 steering smoothness
- [x] 检查 steering-rate
- [x] 检查 constraint violation
- [x] 检查 prediction horizon
- [x] 检查不同速度下稳定性
- [x] 冻结 baseline MPC config

## Phase 2 — 传统轨迹规划
### 10. Reference Line
建立统一、平滑的道路参考线，为 Frenet 坐标和 Sampling Planner 提供稳定几何信息。
已完成离散版本：
- [x] 定义 ReferencePoint 数据结构
- [x] 离散二维点表示 Reference Line
- [x] 计算累计弧长 s
- [x] 计算 yaw
- [x] 计算 curvature
- [x] 按 s 查询 / 线性插值
- [x] 验证直线和圆弧 Reference Line
升级为成熟实现：
- [x] 使用 cubic spline 建立 x(s) / y(s)
- [x] 由 spline 导数计算 yaw
- [x] 由 spline 导数计算 curvature
- [x] 计算 curvature derivative κ'
- [x] ReferencePoint 增加 curvature_derivative
- [x] 统一 ReferenceLine query(s)
- [x] 验证 straight / circular / smooth curved path

### 11. Cartesian ↔ Frenet / SL 坐标转换
建立基于同一 Smooth Reference Line 的 XY ↔ SL 坐标转换。
已完成基础版本：
- [x] 理解 XY ↔ SL 几何意义
- [x] 定义 FrenetPoint
- [x] 实现 polyline XY → SL
- [x] 实现 polyline SL → XY
- [x] 验证 XY → SL → XY
升级：
- [x] 保存 raw waypoints，用于 coarse localization
- [x] XY → SL 改为 raw-waypoint coarse localization + spline ternary-search projection
- [x] SL → XY 改为 spline Reference Line
- [x] 使用统一 smooth ReferenceLine position / yaw 几何信息
- [x] 验证 straight / endpoint / circular / varying-curvature round-trip

### 12. Sampling-based Planner
基于 Frenet Sampling 生成多条空间 Candidate Path，
参考 Autoware Path Sampler / Apollo Frenet / Werling 方法。
#### Frenet Geometry
- [x] 理解 Frenet trajectory representation
- [x] 定义 s / l / l' / l''
- [x] 定义 FrenetTrajectoryPoint / FrenetTrajectory
- [x] 推导 Candidate position
- [x] 推导 Candidate yaw
- [x] 理解 1 - κ_r l
- [x] 推导 Candidate curvature
- [x] 理解 κ_r'
- [x] 实现 Apollo-style Frenet → Cartesian geometry
- [x] 定义 Cartesian PathPoint
- [x] 输出 x / y / yaw / curvature / s

#### Lateral Sampling
第一版只做空间 Path Sampling，不进行 longitudinal trajectory optimization。
- [x] 定义 target lateral positions
- [x] 定义 target path lengths
- [x] 定义 sampling resolution
- [x] 使用 quintic polynomial 连接 initial / target state
- [x] 生成多条 Frenet Candidate Paths
- [x] 转换全部 Candidate 到 Cartesian
- [x] 根据 ego state 计算初始 l / l' / l''

#### Hard Constraints
参考 Autoware hard constraints。
- [x] 定义 VehicleParams / ego footprint
- [x] 定义 StaticObstacle geometry
- [x] curvature limit
- [x] vehicle footprint collision checking
- [x] minimum obstacle distance
- [x] drivable-area / road-boundary checking
- [x] 剔除 infeasible Candidates

#### Soft Cost
第一版固定使用：
- [x] lateral deviation cost
- [x] curvature cost
- [x] path length / progress cost
- [x] weighted total cost
- [x] 选择 minimum-cost Candidate

#### Validation & Benchmark
- [x] straight nominal driving
- [x] curved Reference Line
- [x] lateral maneuver
- [x] static obstacle avoidance
- [x] 可视化全部 Candidates 和 Best Path
- [ ] planning computation time
- [ ] minimum obstacle clearance
- [ ] maximum curvature
- [ ] path length
- [ ] planning success rate

### 13. Optimization-based Path Planner
使用 Frenet corridor + convex QP 直接优化离散 lateral path，
与 Sampling Planner 形成明确算法对照。

#### QP State & Objective
- [x] 定义离散优化变量 `l`
- [x] 建立 `l'` finite-difference matrix
- [x] 建立 `l''` finite-difference matrix
- [x] 建立 `l'''` finite-difference matrix
- [x] 定义 lateral reference / deviation cost
- [x] 定义 `l' / l'' / l'''` smoothness cost
- [x] 将 quadratic cost 转换为标准 QP `P / q`

#### QP Constraints & Nominal Solve
- [x] 将 lateral bounds 写成 inequality constraints
- [x] 加入 initial `l` equality constraint
- [x] 加入 initial `l'` equality constraint
- [x] 加入 initial `l''` equality constraint
- [x] 构造 OSQP 所需 `A / lower / upper`
- [x] 使用 OSQP 求解单个 convex QP
- [x] 处理 solver failure / infeasible status

#### Path Reconstruction & Output
- [x] 从 optimized `l` 恢复 `l' / l''`
- [x] 输出统一 `FrenetTrajectory`
- [x] Frenet → Cartesian
- [x] 输出统一 `PathPoint`
- [x] 封装 nominal Optimization Planner pipeline

#### Lateral Corridor
- [x] 沿 Reference Line 离散 planning horizon
- [x] 根据 road boundary + vehicle half-width 建立基础 lateral bounds
- [x] 将 StaticObstacle 从 Cartesian 投影到 Frenet / SL
- [x] 考虑 ego footprint + minimum clearance 对 obstacle 进行 inflation
- [x] 构造 obstacle 左绕 / 右绕两套 lateral corridor
- [x] 检查 corridor feasibility

#### Multi-corridor Optimization
- [x] 对每个 feasible corridor 分别构造并求解 QP
- [x] 丢弃 infeasible / solver-failure corridor
- [x] 从 feasible solutions 中选择 minimum-objective path

#### Validation
- [x] QP difference matrices
- [x] QP objective `P / q`
- [x] QP constraints `A / lower / upper`
- [x] OSQP nominal solve
- [x] Frenet trajectory reconstruction
- [x] straight nominal Cartesian Path
- [x] curved Reference Line
- [x] lateral maneuver
- [x] static obstacle avoidance
- [x] corridor / optimized path visualization

### 14. Sampling vs Optimization Planner Benchmark

在固定 Behavior intent 下，对 Sampling-based Planner 和
Optimization-based Planner 进行公平、可重复的统一 Benchmark。

Benchmark 固定 maneuver intent：
- 绕开静态障碍物
- 最终回到 Reference Line
- 不比较 Behavior Planner 本身
- 重点比较离散 Sampling 与连续 Optimization 的差异

#### 14.0 Freeze Pre-Benchmark Baseline
- [x] 完成 Sampling Planner 基础实现
- [x] 完成两段式 bypass-and-return Sampling
- [x] 完成 Sampling hard-constraint validation
- [x] 完成 Sampling visualization
- [x] 完成 Optimization Planner
- [x] 完成 obstacle corridor branching
- [x] 完成 multi-corridor QP solve
- [x] 完成 Optimization validation / visualization
- [x] 冻结 pre-benchmark baseline commits

#### 14.1 Centralized Configuration
将 Planning / Benchmark 使用的可配置参数集中管理，
避免 planner / validation / benchmark scripts 中散落 hardcoded 参数。

- [x] 建立 `src/config/` package
- [x] 集中 Vehicle geometry 参数
- [x] 集中 Road / planning constraint 参数
- [x] 集中 Sampling Planner 参数
- [x] 集中 Optimization Planner 参数
- [x] 集中 corridor / obstacle inflation 参数
- [x] 集中 OSQP / numerical tolerance 参数
- [ ] 集中 Benchmark runtime 参数
- [x] Planner source 不再包含 tuning parameter hardcode
- [ ] Benchmark / visualization 统一引用 config
- [x] 保留 validation oracle / assertion tolerance 作为 test-local 常量
- [x] 重新运行全部 Sampling / Optimization regression tests
- [x] 确认 config refactor 不改变原有数值结果

#### 14.2 Matched Planner Formulation
让两个 Planner 尽量解决相同的几何优化问题，
只保留 Sampling 与 continuous optimization 的核心求解方式差异。

共同 soft objective：

`J = w_l J_l
   + w_dl J_dl
   + w_d2l J_d2l
   + w_d3l J_d3l
   + w_terminal_l J_terminal_l
   + w_terminal_dl J_terminal_dl`

- [ ] `l`：mean squared lateral deviation
- [ ] `l'`：mean squared lateral slope / heading-related cost
- [ ] `l''`：mean squared curvature-related cost
- [ ] `l'''`：mean squared curvature-change / smoothness cost
- [ ] terminal `l`：planning horizon 末端 lateral error
- [ ] terminal `l'`：planning horizon 末端 heading / slope error
- [ ] 使用 mean-square normalization，降低 discretization 点数变化对 cost scale 的影响
- [ ] matched benchmark 中移除 Sampling-only path-length/progress cost

#### 14.3 Generalize Returning Sampling
使 terminal objective 在 Sampling 中真正参与 candidate selection，
而不是所有 candidate 固定 terminal state。

- [ ] terminal `l` 支持多个 sampled target
- [ ] terminal `l'` 支持多个 sampled target
- [ ] 保留 terminal `l''` 的合理边界条件
- [ ] 保证两段 quintic junction 的 `l / l' / l''` 连续
- [ ] 保证所有 candidate 使用相同 planning horizon
- [ ] 验证 candidate 数量及 terminal-state coverage
- [ ] 验证原 bypass-and-return behavior 仍可生成

#### 14.4 Matched Optimization Objective
将相同六项 geometric objective 转换为 Optimization Planner 的 QP。

- [ ] `l` cost → identity quadratic term
- [ ] `l'` cost → `D1^T D1`
- [ ] `l''` cost → `D2^T D2`
- [ ] `l'''` cost → `D3^T D3`
- [ ] terminal `l` cost → terminal quadratic term
- [ ] terminal `l'` cost → terminal D1 quadratic term
- [ ] 构造统一 `P / q`
- [ ] 验证 Sampling / Optimization 各 cost 项物理意义一致
- [ ] 验证 QP 保持 convex / PSD

#### 14.5 Matched Hard Constraints
两种 Planner 使用相同的最终物理可行性标准。

共同 physical requirements：
- road boundary
- minimum obstacle clearance
- maximum Cartesian curvature
- initial-state consistency

Sampling：
- [ ] 保留 exact Cartesian curvature pruning
- [ ] 保留 exact vehicle-footprint obstacle clearance
- [ ] 保留 road-boundary pruning

Optimization：
- [ ] 保留 road / obstacle convex corridor
- [ ] 加入 `D2` curvature-proxy hard bound
- [ ] 保持 QP convex
- [ ] QP 求解后转换为 Cartesian
- [ ] 使用与 Sampling 相同的 exact physical validator

统一最终 validation：
- [ ] exact road-boundary check
- [ ] exact footprint-to-obstacle clearance
- [ ] exact Cartesian max curvature
- [ ] 任一物理约束失败则 planner result 判定为 failure

#### 14.6 Unified Benchmark Interface
建立与 planner 类型无关的 Benchmark 输出接口。

- [ ] 定义统一 `PlannerBenchmarkResult`
- [ ] planner name
- [ ] success / failure
- [ ] Frenet trajectory
- [ ] Cartesian path
- [ ] planning time
- [ ] failure reason
- [ ] Sampling adapter
- [ ] Optimization adapter

#### 14.7 Common Benchmark Metrics
禁止直接比较两个 Planner 内部 objective value，
统一使用 planner-independent physical metrics。

- [ ] planning success
- [ ] terminal lateral error
- [ ] terminal slope error
- [ ] RMS lateral deviation
- [ ] Cartesian path length
- [ ] minimum exact obstacle clearance
- [ ] maximum absolute Cartesian curvature
- [ ] RMS Cartesian curvature
- [ ] RMS curvature rate
- [ ] planning computation time

#### 14.8 Canonical Single-Scenario Benchmark
先使用一个固定 static-obstacle case 验证完整 Benchmark pipeline。

统一：
- [ ] Reference Line
- [ ] initial VehicleState
- [ ] vehicle geometry
- [ ] road boundary
- [ ] obstacle geometry
- [ ] minimum clearance
- [ ] max curvature
- [ ] planning horizon
- [ ] discretization resolution
- [ ] maneuver intent

输出：
- [ ] Sampling metrics
- [ ] Optimization metrics
- [ ] unified comparison table
- [ ] 确认 metric evaluator 数值正确

#### 14.9 Benchmark Visualization
只显示共同物理世界和最终 planner output，
不混入各 planner 内部的 obstacle approximation。

- [ ] Cartesian overlay：Sampling vs Optimization
- [ ] Frenet `s-l` overlay
- [ ] Reference Line
- [ ] road boundary
- [ ] original obstacle
- [ ] Sampling best path
- [ ] Optimization best path

#### 14.10 Runtime Benchmark
统一完整 planner pipeline 的计时范围。

- [ ] 定义 warm-up runs
- [ ] 定义 repeated runs
- [ ] Sampling 计时包含 candidate generation / pruning / selection
- [ ] Optimization 计时包含 corridor / QP build / solve / reconstruction
- [ ] mean runtime
- [ ] median runtime
- [ ] P95 runtime
- [ ] min / max runtime

#### 14.11 Multi-Scenario & Stress Benchmark
在 canonical case 正确后扩展测试覆盖。

- [ ] nominal straight
- [ ] lateral recovery
- [ ] static obstacle
- [ ] mirrored obstacle
- [ ] curved Reference Line
- [ ] curved Reference Line + obstacle
- [ ] narrower road
- [ ] obstacle lateral-position sweep
- [ ] obstacle longitudinal-position sweep
- [ ] road-width sweep
- [ ] 统计 planning success rate
- [ ] 记录 failure cases

#### 14.12 Planner Tuning
Baseline Benchmark 完成后，再分别优化两个 Planner。

Sampling tuning：
- [ ] `l / l' / l'' / l'''` weights
- [ ] terminal `l / l'` weights
- [ ] lateral sampling interval
- [ ] bypass lateral range
- [ ] bypass-length sampling
- [ ] terminal-state sampling density
- [ ] candidate count vs runtime

Optimization tuning：
- [ ] `l / l' / l'' / l'''` weights
- [ ] terminal `l / l'` weights
- [ ] longitudinal discretization resolution
- [ ] curvature-proxy bound
- [ ] corridor parameters
- [ ] solver tolerance vs runtime

Tuning methodology：
- [ ] 建立独立 tuning scenario set
- [ ] 两个 Planner 分别调到合理最佳性能
- [ ] 冻结 tuned configs
- [ ] 不使用 final evaluation scenarios 调参

#### 14.13 Final Planner Evaluation
使用冻结后的 tuned configs，
在未参与调参的 evaluation scenario set 上重新比较。

- [ ] Sampling tuned result
- [ ] Optimization tuned result
- [ ] path quality comparison
- [ ] smoothness comparison
- [ ] safety margin comparison
- [ ] success-rate comparison
- [ ] computation-time comparison
- [ ] 分析 Sampling density trade-off
- [ ] 分析 convex Optimization approximation trade-off
- [ ] 总结两种 Planner 的优缺点
- [ ] 保存 CSV / figures / benchmark summary

后续 Phase 15 / 16 再进行 Curvature-aware MPC 和
same-controller closed-loop Planning & Control Benchmark。

### 15. Curvature-aware MPC + Systematic Tuning
Phase 1 MPC 保留为冻结 Baseline。
增加曲率前馈：
- [ ] 从 planned path 获取 reference curvature
- [ ] 加入 steering curvature feedforward
- [ ] 保留 MPC feedback correction
- [ ] 验证 curved-path tracking
- [ ] 再系统搜索 horizon / Q / R
- [ ] 比较 Baseline MPC 与 Tuned MPC

### 16. Final Planning & Control Benchmark
使用统一 Tuned MPC 重新运行：
- [ ] Sampling Planner
- [ ] Optimization Planner
- [ ] tracking error
- [ ] heading error
- [ ] steering / steering rate
- [ ] trajectory quality
- [ ] planning computation time
- [ ] failure cases

## Phase 3 — C++ Core Engineering
将数据结构、Reference Line、Frenet、车辆模型、MPC 以及两种核心 Planner 迁移到 C++，并验证 Python / C++ 数值一致性。

## Phase 4 — Dynamic Scenarios & System Evaluation
加入动态车辆、行人、lead-vehicle slowdown 和 cut-in 等场景，建立 collision rate、task completion、minimum distance 等系统级指标。

## Phase 5 — Hybrid Planner（Optional）
使用 Sampling 提供可行解 / 初值，再由 Optimization refinement，并与前两种 Planner 做统一 Benchmark。

## Phase 6 — Safety & Fallback
建立独立于高层智能模块的 deterministic safety checker、planning failure fallback 和安全介入统计。

## Phase 7 — VLM Behavior Layer
接入小型 VLM，仅负责 KEEP_LANE / SLOW_DOWN / STOP / CAUTIOUS 等高层 BehaviorCommand，不让 VLM 直接生成控制量。

## Phase 8 — Rule-based vs VLM Evaluation
在相同场景和相同下游 Planning & Control 下比较规则决策与 VLM 的正确率、延迟、安全介入和 failure cases。

## Phase 9 — Portfolio Evaluation & Documentation
整理自动化测试、Failure Case、系统架构、Planner Benchmark、P&C Benchmark、图表、GIF / Demo 和技术文档。

## Phase 10 — ROS 2 Integration（Optional）
最后将 Behavior、Planning、Control 和 Simulation 节点化，用 topic / rosbag / RViz 展示完整系统。
