# 06 — Taylor Linearization、SCP 与 Trust Region

## 1. 为什么 QP 之后还需要 SCP

Optimization Planner 希望保持 QP 结构，但真实 rotated-footprint clearance 是 nonlinear constraint：

$$
c(l,l')\ge d_{min}
$$

其中 $c$ 不只是 lateral position 的函数，因为 $l'$ 会改变车辆相对 Reference Line 的 heading，从而改变旋转后的 footprint。

OSQP 不能直接处理这种一般 nonlinear constraint。

解决办法是 Sequential Convex Programming（SCP）：

> 在当前 nominal path 附近把 nonlinear constraint 线性化，解一个局部 convex QP，再更新 nominal path，重复这个过程。

## 2. 一阶 Taylor 展开

设当前 iterate 为：

$$
(l_i^k,l_i'^k)
$$

真实 clearance：

$$
c_i=c(l_i,l_i')
$$

在当前点做一阶 Taylor approximation：

$$
c(l_i,l_i')\approx c_i^k+\frac{\partial c}{\partial l}\bigg|_k(l_i-l_i^k)+\frac{\partial c}{\partial l'}\bigg|_k(l_i'-l_i'^k)
$$

然后把安全要求写成 affine constraint：

$$
c_i^k+g_l(l_i-l_i^k)+g_{l'}(l_i'-l_i'^k)\ge d_{min}
$$

这样就可以重新放回 QP。

## 3. 为什么项目用 numerical gradient

项目没有手推 rectangle-circle nearest-feature 对 $l,l'$ 的完整解析导数，而是用 central difference 估计：

$$
\frac{\partial c}{\partial l}\approx\frac{c(l+\epsilon,l')-c(l-\epsilon,l')}{2\epsilon}
$$

$$
\frac{\partial c}{\partial l'}\approx\frac{c(l,l'+\epsilon)-c(l,l'-\epsilon)}{2\epsilon}
$$

好处：

- 和 exact validator 使用同一套 geometry；
- 实现简单；
- 不需要处理最近点落在 corner / edge 时的分段解析导数。

代价：

- 每次 gradient 需要额外 clearance evaluation；
- 对 epsilon 敏感；
- feature switching 附近可能有数值噪声。

## 4. 为什么需要 Trust Region

Taylor approximation 只在 nominal point 附近可靠。

如果一次 QP step 走得太远，线性 approximation 可能完全失真。

因此限制：

$$
|l_i-l_i^k|\le\Delta_l
$$

$$
|l_i'-l_i'^k|\le\Delta_{l'}
$$

这就是 trust region。

直觉上：

> 我允许 optimizer 沿着当前局部几何“修一点”，但不允许它根据一个局部切线一次跳到很远的位置。

## 5. 完整 SCP 流程

```text
heading-aware / corridor seed
        ↓
solve seed QP
        ↓
reconstruct Cartesian path
        ↓
exact footprint clearance
        ↓
PASS ? ───── yes → return path
        │
        no
        ↓
numerical clearance gradients
        ↓
linearized clearance constraints
        + trust region
        ↓
solve next QP
        ↓
exact revalidation
        ↓
repeat until pass / iteration limit / infeasible
```

注意：

- linearized constraint 只是局部近似；
- final acceptance 仍然依赖 exact physical validation；
- SCP 成功不代表原始 nonconvex problem 的 global optimum。

## 6. Seed 为什么也重要

SCP 是 local method，所以 starting point 会影响结果。

项目先用 heading-aware / branch-aware constraint 提供一个合理的 bypass side 和 nominal path，再在这个局部区域做 exact-clearance refinement。

因此三个角色要区分：

1. seed / initial branch：提供可用初始猜测；
2. SCP：把 exact nonlinear clearance 局部转成 repeated affine constraints；
3. exact validator：最终安全 gate。

## 7. 为什么 redesign 后 runtime 变高

旧 optimizer 一次 convex QP 大约只有几毫秒级。

新版本还要：

- 重建 Cartesian geometry；
- 多次 exact clearance evaluation；
- numerical gradient；
- 反复 solve QP。

所以平均 runtime 从旧版约 7.6 ms 上升到当前约 46 ms，但仍明显快于 frozen Sampling 的约 191 ms。

## 8. 代码对应

主要文件：

- [`src/planning/optimization_planner.py`](../../src/planning/optimization_planner.py)
  - `clearance_from_frenet_state()`
  - `clearance_gradient()`
  - `build_initial_heading_constraints()`
  - `build_linearized_clearance_constraints()`
  - `_solve_explicit_branch()`
  - `evaluate_exact_path()`
- [`src/planning/hard_constraints.py`](../../src/planning/hard_constraints.py)
  - exact physical clearance implementation

## 9. 面试 30 秒讲法

> Rotated vehicle-footprint clearance 对 Frenet $l,l'$ 是 nonlinear 的，所以我没有直接把它塞进 OSQP，而是用 SCP。在当前 path 上计算 exact clearance，用 numerical gradient 做一阶 Taylor linearization，再加 trust region 限制每次更新范围，反复解 QP。每次更新以后都重新做 exact physical validation，所以 linearized constraint 只是求解手段，不是最终安全判定。

## 10. 常见追问

**Q：为什么不用 Newton / SQP？**  
A：SCP 和 SQP 思路接近；当前实现重点是把 nonlinear constraint 转成 OSQP 能处理的 affine constraints，同时保持已有 QP objective / dynamics 结构。

**Q：为什么 trust region 不设得很大？**  
A：太大会削弱局部线性 approximation 的可信度；太小则收敛慢甚至难以脱离当前 nominal path，是典型 accuracy / progress trade-off。

**Q：SCP 一定收敛吗？**  
A：不保证。它依赖 initial guess、local geometry、trust region 和 iteration limit，所以 production code 仍需要明确 infeasible / non-success handling。