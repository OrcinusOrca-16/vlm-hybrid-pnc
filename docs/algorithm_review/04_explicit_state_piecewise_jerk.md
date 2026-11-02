# 04 — Explicit Frenet State 与 Piecewise Jerk

## 1. 为什么把 state 改成 `[l, l', l'']`

旧 Optimization Planner 只直接优化：

$$
[l_0,l_1,\dots,l_N]
$$

然后通过 finite differences 近似 $l'$、$l''$、$l'''$。

问题出在 initial derivative constraints 也通过 early $l_i$ 间接施加。以简单 forward difference 为例：

$$
l'_0\approx\frac{l_1-l_0}{\Delta s}=0
$$

会得到：

$$
l_1=l_0
$$

如果再要求：

$$
l''_0\approx\frac{l_2-2l_1+l_0}{\Delta s^2}=0
$$

结合 $l_1=l_0$，得到：

$$
l_2=l_0
$$

于是起点附近可能近似形成：

$$
l_0=l_1=l_2
$$

这并不是物理上“初始 $l'=0,l''=0$”的唯一含义，而是离散 formulation 额外造成的 early-knot coupling。

## 2. Explicit state 的核心改变

新的 state 直接包含：

$$
x_i=[l_i,l'_i,l''_i]
$$

初始条件可以直接写成：

$$
l_0=l_{ego},\qquad l'_0=l'_{ego},\qquad l''_0=l''_{ego}
$$

不需要通过 $l_1,l_2$ 反推初始导数。

这意味着：

> 初始 slope 和 second derivative 可以严格为 0，但后续 knot 不需要被人为锁成同一个 lateral position。

## 3. Piecewise jerk 是什么

这里 jerk 指：

$$
j=\frac{d^3l}{ds^3}
$$

是 lateral profile 对空间弧长 $s$ 的三阶导数，不是车辆对时间的 passenger-comfort jerk。

在每个 segment $[s_i,s_{i+1}]$ 内假设：

$$
j_i=\text{constant}
$$

这就是 piecewise-constant jerk。

## 4. 从 jerk 推状态转移

因为：

$$
\frac{dl''}{ds}=j_i
$$

在长度 $\Delta s$ 上积分：

$$
l''_{i+1}=l''_i+j_i\Delta s
$$

再积分一次：

$$
l'_{i+1}=l'_i+l''_i\Delta s+\frac12j_i\Delta s^2
$$

再积分：

$$
l_{i+1}=l_i+l'_i\Delta s+\frac12l''_i\Delta s^2+\frac16j_i\Delta s^3
$$

由第一式：

$$
j_i=\frac{l''_{i+1}-l''_i}{\Delta s}
$$

代回后得到项目里更方便用于 QP 的 linear equality constraints：

$$
l'_{i+1}=l'_i+\frac{\Delta s}{2}(l''_i+l''_{i+1})
$$

以及：

$$
l_{i+1}=l_i+\Delta s\,l'_i+\frac{\Delta s^2}{3}l''_i+\frac{\Delta s^2}{6}l''_{i+1}
$$

## 5. 为什么这个模型适合 QP

关键点：

- state 是 physically meaningful 的 $l,l',l''$；
- segment dynamics 是 linear equality；
- jerk 可以从相邻 $l''$ 直接得到；
- smoothness cost 仍然可以保持 quadratic。

因此可以提高 formulation 语义，同时不放弃 convex QP 结构。

## 6. 为什么 `l'=0, l''=0` 仍然能立即开始横移

只要：

$$
j_0\neq0
$$

那么：

$$
l(\Delta s)-l_0=\frac16j_0\Delta s^3
$$

所以即使起点：

$$
l'_0=0,\qquad l''_0=0
$$

路径仍然可以在起点之后立刻产生 lateral displacement。

这正是 explicit state 相对旧 l-only finite-difference initial constraints 的主要好处之一。

## 7. A/B 结果应该怎么理解

严格 retrospective A/B 中只替换：

- l-only state → explicit $[l,l',l'']$；
- finite-difference initial derivatives → direct state constraints；
- 加入 piecewise-jerk propagation。

在 lateral recovery case 中：

- first meaningful lateral motion：约 `1.5 m → 0.5 m`；
- canonical cost：约下降 27.7%。

但一些 obstacle cases 反而失败。

这不意味着 explicit state 有问题，而是说明：

> state/dynamics 改善后，旧 rectangular corridor 的几何近似成为新的主要瓶颈。

所以完整 redesign 是两步：

$$
\text{state/dynamics fix}+\text{obstacle geometry/refinement fix}
$$

## 8. 代码对应

主要文件：

- [`src/planning/optimization_planner.py`](../../src/planning/optimization_planner.py)
  - explicit state indexing
  - `build_explicit_qp_objective()`
  - `build_explicit_qp_constraints()`
  - `decode_explicit_solution()`

## 9. 面试 30 秒讲法

> 旧 optimizer 只优化 $l$，然后用 finite differences 重建导数。这样 initial $l'=0,l''=0$ 会间接约束前几个 lateral knots，造成不必要的 early-path coupling。后来我把 state 改成 explicit $[l,l',l'']$，并假设每段 spatial jerk 为常数，积分得到线性的 state-transition constraints。这样既保留 QP 结构，也能直接表达真实初始 Frenet state。

## 10. 常见追问

**Q：piecewise jerk 是为了 passenger comfort 吗？**  
A：这里主要不是 temporal comfort jerk，而是 spatial profile 的三阶导数，用来建立平滑、线性的 Frenet state dynamics。

**Q：为什么最高阶假设常数？**  
A：这样 segment 内状态转移可以解析积分，得到简单的线性关系，特别适合离散优化。

**Q：finite difference 一定不好吗？**  
A：不是。问题是这个项目旧 formulation 中 `l-only + initial derivative constraints` 的组合引入了额外 early-knot coupling；不是说 finite difference 普遍不可用。