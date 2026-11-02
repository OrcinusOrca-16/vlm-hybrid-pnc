# Algorithm Review Notes

这组文档用于面试前复习本仓库的 Planning & Control 核心算法。目标不是写成教材，而是做到：

1. 能用直觉解释算法为什么存在；
2. 能写出关键数学关系；
3. 能指出仓库中的代码对应位置；
4. 面试官追问设计取舍时，能说明优点、限制和后续改进方向。

## 推荐复习顺序

1. [Reference Line 与 Frenet](01_frenet_reference_line.md)
2. [Quintic Polynomial 与 Sampling Planner](02_quintic_sampling.md)
3. [QP 与 Optimization Planner](03_qp_optimization.md)
4. [Explicit Frenet State 与 Piecewise Jerk](04_explicit_state_piecewise_jerk.md)
5. [Vehicle Footprint 与 Obstacle Clearance](05_vehicle_footprint_clearance.md)
6. [Taylor Linearization、SCP 与 Trust Region](06_scp_trust_region.md)
7. [Kinematic Bicycle Model](07_kinematic_bicycle_model.md)
8. [Lateral MPC](08_lateral_mpc.md)
9. [Planning → Control Integration](09_planning_control_integration.md)

## 一条主线记住整个项目

```text
Reference Line
    ↓
Cartesian ↔ Frenet
    ↓
Sampling / Optimization Planning
    ↓
Exact Physical Validation
    ↓
Curvature-aware MPC
    ↓
Kinematic Bicycle Model
    ↓
Closed-loop Metrics
```

Sampling 与 Optimization 的核心差别：

- Sampling：先生成许多完整候选，再做物理筛选；
- Optimization：直接在连续变量上求一个低成本解，并通过约束保证可行性。

Optimization redesign 的核心因果链：

```text
vehicle-derived curvature limit
    ↓
旧 l-only formulation 的结构问题暴露
    ↓
explicit [l, l', l''] + piecewise jerk
    ↓
旧 rectangular obstacle corridor 的几何近似问题暴露
    ↓
exact rotated-footprint clearance + SCP
```

## 面试复习方法

每一章至少做到三件事：

- 不看代码，能用 30–60 秒说明算法；
- 能手写最关键的 1–3 个公式；
- 能打开对应源码，说明主要函数输入、输出和作用。

不需要背所有参数，也不需要逐行背代码。重点是设计逻辑、约束关系和验证方法。
