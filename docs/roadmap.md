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

### 5. 运动学自行车模型
使用前轮转角和纵向加速度作为输入，根据运动学自行车模型更新车辆的位置、航向角和速度。

### 6. 纵向 PID 控制
根据目标速度与实际速度误差计算纵向加速度，实现速度闭环跟踪。

### 7. 横向 MPC 控制
根据 VehicleState 和参考轨迹建立预测模型和优化问题，计算前轮转角，实现横向轨迹跟踪。

### 8. Planning & Control 闭环仿真
连接：
Scenario
→ Behavior Planner
→ Trajectory Generator
→ PID / MPC
→ Vehicle Model
→ VehicleState Feedback
形成完整的 Planning & Control 反馈闭环。

### 9. 基础评价与可视化
计算并展示：
- XY 实际轨迹与参考轨迹
- 横向误差
- 航向误差
- 速度误差
- 前轮转角
- 纵向加速度

## Phase 2 — 传统轨迹规划
### 10. Reference Line
建立道路参考线，并定义参考线的位置、航向角和曲率等基本信息。

### 11. Cartesian ↔ Frenet / SL 坐标转换
实现车辆和障碍物在 XY 坐标系与 Frenet / SL 坐标系之间的转换。

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
- [ ] Phase 1 — Planning & Control 基线系统

正在进行：
- [x] 1. 通用数据结构
- [x] 2. 基础场景建模
- [x] 3. 规则行为决策器

下一步：
- [ ] 4. 简单参考轨迹生成器