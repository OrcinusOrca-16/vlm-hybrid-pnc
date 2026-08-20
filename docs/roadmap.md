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
建立道路参考线，并定义参考线的位置、航向角和曲率等基本信息。
- [x] 理解 Reference Line 在 Planning 中的作用
- [x] 定义 ReferencePoint 数据结构
- [x] 用离散点表示二维参考线
- [x] 计算累计弧长 s
- [x] 计算参考线 yaw
- [x] 计算参考线 curvature
- [x] 构造完整 Reference Line
- [x] 实现两个 ReferencePoint 之间的线性插值
- [x] 实现按 s 查询 ReferencePoint
- [x] 验证直线 Reference Line
- [x] 验证曲线 Reference Line

### 11. Cartesian ↔ Frenet / SL 坐标转换
建立全局 XY 坐标与道路局部 SL 坐标之间的转换。
- [x] 理解 XY ↔ SL 转换的几何意义
- [x] 定义 FrenetPoint 数据结构
- [x] 将 XY 点投影到 Reference Line
- [x] 计算纵向坐标 s
- [x] 计算有符号横向坐标 l
- [x] 实现 XY → SL
- [x] 推导 SL → XY
- [x] 实现 SL → XY
- [x] 验证直线 Reference Line
- [x] 验证曲线 Reference Line
- [x] 验证 XY → SL → XY 往返一致性

### 12. Sampling-based Planner
基于 Frenet / Lattice 思路生成多条候选轨迹，
完成第一个真正具备绕障能力的 Motion Planner。
- [x] 理解 Sampling / Lattice Planner 的整体流程
- [x] 定义 Frenet trajectory representation

- [x] 理解 Frenet trajectory 与单个 FrenetPoint 的区别
- [x] 定义纵向变量 s
- [x] 定义横向函数 l(s)
- [x] 定义 l' / l''
- [x] 确定候选轨迹需要保存的数据结构

- [x] 推导 Frenet trajectory → Cartesian trajectory
- [x] 推导 trajectory position
- [x] 推导 trajectory yaw
- [x] 理解 1 - κ_r l 的几何意义
- [ ] 推导 / 计算 trajectory curvature
- [ ] 将 Frenet trajectory 转换为 Cartesian trajectory

- [ ] 定义 lateral sampling space
- [ ] 定义目标横向位置 l_target
- [ ] 定义 longitudinal sampling range
- [ ] 确定 planning horizon
- [ ] 确定 sampling resolution

- [ ] 使用多项式生成 lateral candidate
- [ ] 生成多条 Frenet candidate trajectories
- [ ] 将全部候选轨迹转换到 Cartesian

- [ ] 检查 road boundary
- [ ] 检查最大 curvature
- [ ] 检查基本运动学可行性
- [ ] 剔除 infeasible trajectories

- [ ] 定义静态 obstacle representation
- [ ] 实现 candidate trajectory collision checking
- [ ] 计算 minimum obstacle clearance

- [ ] 定义 trajectory cost
- [ ] lateral target cost
- [ ] smoothness cost
- [ ] curvature cost
- [ ] obstacle / clearance cost
- [ ] trajectory length / progress cost
- [ ] 选择 minimum-cost trajectory

- [ ] 验证 straight nominal driving
- [ ] 验证 curved reference line
- [ ] 验证 static obstacle avoidance
- [ ] 验证 lateral maneuver
- [ ] 可视化全部 candidate 与 best trajectory
- [ ] 建立 Sampling Planner 基础评价指标

### 13. Optimization-based Planner
使用 QP / 数值优化直接求解平滑轨迹，与 Sampling Planner 形成两种不同 Planning 方法。

### 14. Sampling vs Optimization Planner Benchmark
在相同 Reference Line、场景、车辆模型和冻结的 Baseline MPC 下比较两种规划器的轨迹质量、成功率、计算时间和下游控制表现。

### 15. MPC Systematic Tuning
在 Planner Benchmark 建立之后系统搜索 horizon、Q、R 等参数，并比较 Baseline MPC 与 Tuned MPC。

### 16. Final Planning & Control Benchmark
使用 Tuned MPC 重新运行两种 Planner，分析 trajectory quality 如何影响 tracking、steering 和整体 P&C 性能。

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