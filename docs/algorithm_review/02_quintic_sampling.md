# 02 — Quintic Polynomial 与 Sampling Planner

## 1. 直觉

Sampling Planner 的思路不是“直接求唯一最优解”，而是：

1. 先生成一批完整、平滑的候选路径；
2. 每条候选都转换到 Cartesian；
3. 用统一物理约束筛掉不可行候选；
4. 在剩余候选中选 cost 最低的一条。

核心问题因此变成：怎样快速生成“满足起点和终点状态”的平滑横向曲线。

## 2. 为什么用五次多项式

定义横向 profile：

$$
l(s)=a_0+a_1s+a_2s^2+a_3s^3+a_4s^4+a_5s^5
$$

六个系数正好可以满足六个边界条件：

$$
[l(0),l'(0),l''(0)]
$$

和

$$
[l(S),l'(S),l''(S)]
$$

因此可以同时控制：

- 起始横向位置；
- 起始 slope；
- 起始 second derivative；
- 终点横向位置；
- 终点 slope；
- 终点 second derivative。

这比只指定起终点位置更适合车辆路径，因为方向和曲率趋势也能连续衔接。

## 3. 系数怎么求

前三个系数直接由初始状态得到：

$$
a_0=l_0
$$

$$
a_1=l'_0
$$

$$
a_2=\frac12 l''_0
$$

剩下 $a_3,a_4,a_5$ 通过终点三个条件形成一个 $3\times3$ 线性方程组求解。

项目中直接用 `np.linalg.solve()` 完成。

## 4. 一个容易混淆的点

即使：

$$
l'(0)=0,\qquad l''(0)=0
$$

也不代表路径必须在起点后保持直线一段时间。

因为五次多项式仍然可以有：

$$
l'''(0)\neq0
$$

于是对很小的 $\Delta s$：

$$
l(\Delta s)-l_0\approx\frac16l'''(0)\Delta s^3
$$

所以 Sampling candidate 可以在严格满足初始 slope / second derivative 的同时，起点之后立即开始 lateral motion。

这也是后来诊断旧 Optimization Planner early-knot locking 时的重要对照。

## 5. 两段式 bypass-and-return

当前障碍绕行 candidate 不是只做一次 lateral shift，而是两段：

```text
initial state
    ↓
quintic bypass segment
    ↓
bypass / junction state
    ↓
quintic return segment
    ↓
return to lane center
```

第一段从当前 Frenet state 到 bypass lateral position；第二段从 junction state 回到最终 lane state。

连接点直接继承上一段的：

$$
[l,l',l'']
$$

因此两段之间保持状态连续。

## 6. Sampling 搜索空间

Frozen obstacle-bypass family 的代表配置是：

$$
9\ \text{lateral positions}\times5\ \text{bypass lengths}=45\ \text{candidates}
$$

Sampling 的优点是：

- 每条候选完整、直观；
- 一个候选失败可以直接丢掉；
- 容易可视化和诊断；
- 不需要求解非凸连续优化问题。

主要限制是：

> 它只能搜索预定义 candidate family。

如果真正可行路径位于两个离散 candidate 之间，Sampling 可能失败，即使连续空间里存在解。

## 7. Candidate 的完整处理流程

```text
vehicle state
    ↓
convert to initial Frenet state
    ↓
generate quintic candidates
    ↓
sample [l, l', l''] along s
    ↓
Frenet → Cartesian path
    ↓
physical feasibility filter
    ↓
canonical objective / trackability selection
    ↓
best feasible candidate
```

这里要注意：Sampling candidate 生成阶段本身并不等于最终安全性。最终仍然要通过真实 Cartesian curvature、vehicle footprint clearance 和 road boundary 检查。

## 8. 代码对应

主要文件：

- [`src/planning/quintic_polynomial.py`](../../src/planning/quintic_polynomial.py)
  - `QuinticPolynomial`
  - `position()`
  - `first_derivative()`
  - `second_derivative()`
- [`src/planning/sampling_planner.py`](../../src/planning/sampling_planner.py)
  - `sample_quintic_candidate()`
  - `generate_returning_frenet_candidate()`
  - `generate_returning_frenet_candidates()`
  - `generate_frenet_candidates()`
- [`src/planning/frenet.py`](../../src/planning/frenet.py)
  - `frenet_trajectory_to_path()`

## 9. 面试 30 秒讲法

> Sampling Planner 用 quintic polynomial 在 Frenet 空间生成完整候选路径。五次多项式正好能同时满足起终点的 $l,l',l''$ 六个边界条件，所以 candidate 的位置、方向趋势和 curvature-related state 都可以平滑连接。然后我把所有候选转换到 Cartesian，用统一物理约束过滤，最后从 feasible candidates 中选 cost 最低的一条。

## 10. 常见追问

**Q：为什么不是 cubic？**  
A：cubic 只有四个系数，无法同时独立满足起终点的 position、first derivative、second derivative 六个条件。

**Q：Sampling 为什么会失败在 narrow road，而 Optimization 能过？**  
A：因为 frozen Sampling 只搜索固定 candidate family；连续空间里存在可行解不代表离散 candidate 集里一定有。

**Q：为什么不无限增加 candidates？**  
A：搜索覆盖率会提高，但 runtime 也线性增长，而且候选设计仍然是人工离散化。