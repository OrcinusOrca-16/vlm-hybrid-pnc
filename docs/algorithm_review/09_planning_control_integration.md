# 09 — Planning → Control Integration

## 1. 为什么 Planning benchmark 之后还要接 Control

Planner 的 objective 和 controller 真正关心的 quantity 并不完全一样。

Planner 主要优化：

$$
l,\quad l',\quad l'',\quad l'''
$$

Controller / vehicle 真正直接感受到：

- Cartesian curvature；
- steering angle；
- steering rate；
- lateral error；
- heading error；
- feedback correction。

所以：

> planning-domain cost 更低，不自动代表 closed-loop 更好跟。

这就是最终 Planning + Control benchmark 的主要意义。

## 2. Planning 输出到 Control 的接口

完整流程：

```text
planner
  ↓
Frenet trajectory
  ↓
Cartesian path [x, y, yaw, curvature]
  ↓
physical validation
  ↓
reference curvature / path geometry
  ↓
constant safe-speed interface
  ↓
curvature-aware MPC
  ↓
steering command
  ↓
kinematic bicycle model
  ↓
closed-loop tracking metrics
```

两种 planner 最终都经过同一个 controller，这样才能把“planner 差异”和“controller 差异”分开。

## 3. 为什么 curvature 是关键接口

由 bicycle geometry：

$$
\delta_{ff}=\arctan(L\kappa_{ref})
$$

所以 planner path 的 curvature 直接决定 nominal steering demand。

这也解释了为什么项目后来必须把 planner curvature constraint 和 steering capability 对齐。

## 4. Constant Safe Speed 的意义

同一条 path 在低速时可能很好跟，但速度提高后 steering-rate demand 和 tracking difficulty 会增加。

项目因此根据 path steering demand 给出一个 constant safe reference speed，用相同 controller 测试每条 path 的可跟踪性。

这个 interface 不是完整 longitudinal planning，而是为了把问题简化为：

> 在当前 steering constraints 下，这条 spatial path 以多大 constant speed 能稳定跟踪？

## 5. 为什么还做 same-speed comparison

如果 Sampling 和 Optimization 各自使用不同 safe speed，仅比较 RMS tracking error 不完全公平。

因此 final benchmark 还增加 common-speed diagnostic：

> 在相同速度下，把两条 path 交给同一个 controller，比 lateral RMS、heading error 和 feedback correction。

这样可以更直接观察 path geometry 对 trackability 的影响。

## 6. 最终结果应该怎么解释

Frozen planning-only suite：

- Sampling：8 / 9 planning success；
- Optimization：9 / 9 planning success；
- Optimization mean runtime 约 46 ms；
- Sampling mean runtime 约 191 ms。

Planning + Control：

- 所有成功规划的 path 都能被 common MPC 成功跟踪；
- Optimization 是整体更强的 planner；
- 但 Sampling 在部分场景可以产生更容易跟踪的 path。

这两个结论并不矛盾。

## 7. 一个重要 trade-off

Optimization 的 canonical objective 通常更低，但在一些场景它会更积极地利用允许的 state / curvature 空间。

Sampling candidate family 反而可能因为 quintic shape 较固定，产生较低 peak curvature 或更温和的 steering profile。

因此：

$$
\text{lower planner cost}\not\Rightarrow\text{lower control effort}
$$

这说明 planner objective 和 controller-oriented metrics 之间存在 alignment gap。

## 8. 两个代表案例

### `evaluation_obstacle_longitudinal_19.00`

Optimization 在这个 case 中同时表现出：

- 更低 planning objective；
- 较低 max curvature；
- 更高可跟踪速度。

这是“planning improvement 同时带来 control improvement”的例子。

### `evaluation_curved_obstacle`

Sampling 虽然 planning objective 更高，但 path curvature 更温和，最终：

- safe speed 更高；
- same-speed tracking error 更低；
- feedback demand 更小。

这是“planner objective 和 controller trackability 排名不同”的代表案例。

## 9. 这个结果对后续 planner design 的启发

如果未来继续优化，可以考虑在 planning objective 中加入更 controller-aligned 的 quantity，例如：

- curvature rate；
- steering-rate proxy；
- design-speed feasibility；
- tracking-oriented cost。

但当前 portfolio 没有继续 tuning，因为 final frozen result 不应该再变成 tuning loop。

## 10. 代码与结果对应

主要代码：

- [`src/planning/frenet.py`](../../src/planning/frenet.py)
  - Frenet → Cartesian geometry
- [`src/planning/hard_constraints.py`](../../src/planning/hard_constraints.py)
  - physical acceptance
- [`src/control/curvature_aware_mpc.py`](../../src/control/curvature_aware_mpc.py)
  - curvature feedforward + MPC correction
- [`src/control/path_tracking.py`](../../src/control/path_tracking.py)
  - path tracking utilities
- [`src/vehicle/kinematic_bicycle.py`](../../src/vehicle/kinematic_bicycle.py)
  - closed-loop plant

最终分析：

- [`docs/final_planning_control_findings.md`](../final_planning_control_findings.md)
- [`outputs/final_evaluation/`](../../outputs/final_evaluation/)
- [`outputs/final_planning_control/`](../../outputs/final_planning_control/)

## 11. 面试 30 秒讲法

> 我最后没有只停在 planner benchmark，而是把 Sampling 和 Optimization 的 path 都接到同一个 curvature-aware MPC 和同一个 kinematic bicycle model。结果发现 Optimization 在 planning success、cost 和 runtime 上整体更强，但 Sampling 在部分场景反而更容易被 controller 跟踪。这说明 Frenet planning objective 和 steering / closed-loop metrics 并不完全对齐，是这个项目一个比较重要的系统级结论。

## 12. 常见追问

**Q：是不是说明 Sampling 比 Optimization 更好？**  
A：不是。当前 frozen suite 中 Optimization planning coverage 是 9/9，Sampling 是 8/9；只是 controller trackability 是另一个维度，Sampling 在部分 common-success cases 更好。

**Q：为什么不根据 final control result 再重新 tuning planner？**  
A：因为 final suite 已经被观察过，再继续针对它 tuning 会削弱 evaluation discipline。更合理的是把 controller-aligned objective 作为下一轮独立 development cycle。

**Q：量产系统还缺什么？**  
A：动态障碍与 prediction、连续 collision checking、更真实 vehicle / actuator dynamics、fallback / safety architecture 等都还没有在当前 portfolio 中实现。