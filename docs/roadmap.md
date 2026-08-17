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
- [x] 实现单个 segment 的 XY 投影
- [x] 验证 segment 端点 clamp
- [x] 遍历整条 Reference Line 找最近 segment
- [x] 计算纵向坐标 s
- [x] 计算有符号横向坐标 l
- [x] 实现 XY → SL
- [x] 推导 SL → XY
- [ ] 实现 SL → XY

### 12. 候选轨迹生成
使用多项式等方法在 Frenet 空间生成多条满足不同横向和纵向目标的候选轨迹。

### 13. 轨迹约束检查
检查速度、加速度、加加速度、曲率等约束，剔除不满足车辆运动要求的轨迹。

### 14. 碰撞检测
根据车辆和障碍物的位置判断候选轨迹是否存在碰撞风险。

### 15. Cost Function 与轨迹选择
综合安全性、舒适性、轨迹平滑度和目标偏差等代价，从候选轨迹中选择最优轨迹。

### 16. 静态绕障场景
加入道路施工或车道受阻场景，验证 Frenet Planner 能否生成安全绕障轨迹。

### 17. 完整 Planning & Control 闭环
将 Behavior Planner、Frenet Trajectory Planner、MPC 和车辆模型连接，形成完整传统 P&C 系统。

## Phase 3 — VLM 与混合式决策架构
### 18. VLM 行为决策器
输入道路图像和车辆结构化信息，由视觉语言模型输出 KEEP_LANE、SLOW_DOWN、STOP、YIELD 等高层驾驶行为。

### 19. 结构化 JSON 接口
将自然语言模型输出转换为固定 JSON 数据结构，并处理格式错误、缺失字段和非法值。

### 20. VLM 输出验证
检查行为类型、目标速度、置信度以及其他输出参数是否满足接口要求。

### 21. 确定性安全检查器
使用独立于 VLM 的确定性安全规则检查 BehaviorCommand，并在必要时覆盖危险或异常决策。

### 22. Rule vs VLM 对比
在相同场景下比较规则行为决策和 VLM 行为决策的正确率、推理延迟、安全性和失败模式。

## Phase 4 — 动态驾驶场景与系统实验
### 23. 动态障碍物建模
为其他车辆和行人增加位置、速度及运动方向等随时间变化的状态。

### 24. TTC 与碰撞风险评价
计算 Time-to-Collision 等指标，用于判断动态场景中的潜在碰撞风险。

### 25. 前车减速与行人横穿实验
在动态仿真中验证减速、停车以及安全检查策略。

### 26. 前方车辆切入实验
加入 Cut-in 场景，验证 Behavior、Planning 和 Control 对动态交通变化的响应。

### 27. 系统级评价
统计行为判断正确率、安全检查介入次数、VLM 推理延迟、跟踪误差、最大加速度、最大加加速度、最小障碍物距离和碰撞情况。

## Phase 5 — 测试与作品集完善
### 28. 自动化测试
为数据接口、行为规则、车辆模型、规划器和控制器增加单元测试与场景测试。

### 29. Failure Case 分析
记录 VLM 错误判断、Safety Checker 介入、规划失败和控制失败案例，并分析原因。

### 30. README 与技术文档
整理系统架构、数学模型、算法设计、实验设置和主要技术决策。

### 31. Demo 与实验结果
制作轨迹图、误差曲线、Rule vs VLM 对比结果、GIF 和演示视频。

## Phase 6 — C++ 工程化
### 32. 核心数据结构 C++ 化
将 VehicleState、BehaviorCommand、TrajectoryPoint 等共享数据结构从 Python 迁移到 C++，建立清晰的类型和模块接口。

### 33. 车辆模型与控制器 C++ 化
将运动学自行车模型、纵向 PID 和横向 MPC 迁移到 C++，并验证与 Python 版本结果一致。

### 34. 轨迹规划器 C++ 化
将 Reference Line、Frenet 坐标转换、候选轨迹生成、约束检查、碰撞检测和 Cost Function 等核心 Planning 模块迁移到 C++。

### 35. Python / C++ 结果对比
使用相同场景和参数运行 Python 与 C++ 实现，对比轨迹、控制输入和评价指标，验证迁移正确性。

### 36. C++ 工程结构与测试
使用 CMake 组织项目，并为主要 Planning & Control 模块增加单元测试。

## Phase 7 — ROS 2 扩展
### 37. ROS 2 节点化
将 Behavior、Safety、Planning、Control 和 Vehicle Simulation 等模块拆分成 ROS 2 节点。

### 38. ROS 2 Topic 接口
使用 topic 连接车辆状态、场景信息、行为指令、参考轨迹和控制指令。

### 39. rosbag 与可视化
支持 rosbag 回放，并使用 RViz 或其他工具展示车辆、轨迹和障碍物。

## 当前进度
当前阶段：
- [x] Phase 1 — Planning & Control 基线系统

下一步：
- [ ] 10. Reference Line