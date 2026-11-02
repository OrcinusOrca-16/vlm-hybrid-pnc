# 03 — QP 与 Optimization Planner

## 1. 直觉

Optimization Planner 不预先枚举完整路径，而是把路径离散成一组 decision variables，然后通过一个优化问题直接求“最合适的那条”。

典型形式：

$$
\min_x \frac12 x^T P x + q^T x
$$

subject to

$$
l \le Ax \le u
$$

如果 objective 是 quadratic，constraints 是 linear，这就是 Quadratic Programming（QP）。

项目使用 OSQP 求解。

## 2. 为什么 QP 很适合 path planning

很多 path objective 本身就可以写成平方和：

$$
J = w_l\|l\|^2+w_{l'}\|l'\|^2+w_{l''}\|l''\|^2+w_{l'''}\|l'''\|^2+J_{terminal}
$$

平方和天然就是 quadratic cost。

很多约束也可以做成 linear：

- road boundary；
- initial state；
- piecewise-jerk dynamics；
- convex local curvature proxy；
- trust region；
- 线性化后的 clearance constraint。

所以可以保留较强可解释性和较快求解速度。

## 3. $P$ 和 $q$ 是什么

OSQP 目标：

$$
\frac12x^TPx+q^Tx
$$

例如一个简单 lateral cost：

$$
w_l\sum_i l_i^2
$$

可以写进 $P$ 的对角项。

如果 terminal target 不是 0：

$$
w_t(l_N-l_{target})^2
$$

展开后会同时产生 quadratic term 和 linear term：

$$
w_tl_N^2-2w_tl_{target}l_N+\text{constant}
$$

其中 constant 对 argmin 没影响，可以忽略。

## 4. Constraint matrix 的统一表达

OSQP 使用：

$$
l_b \le Ax \le u_b
$$

一个 equality constraint：

$$
a^Tx=b
$$

只需要设置：

$$
l_b=u_b=b
$$

一个 upper-bound inequality：

$$
a^Tx\le b
$$

可以设 lower bound 为 $-\infty$。

因此 initial-state、dynamics、road boundary 等不同约束最终都能统一堆叠到一个 sparse matrix 里。

## 5. Original Optimization Planner 的问题

旧 formulation 的 decision variable 只有：

$$
x=[l_0,l_1,\dots,l_N]
$$

然后通过 finite-difference matrices 重建：

$$
l'\approx D_1l,
\quad l''\approx D_2l,
\quad l'''\approx D_3l
$$

这种方式本身不是错误，但旧实现里 initial derivative constraints 也通过 early samples 间接施加，导致起点附近自由度被过度耦合。

此外 obstacle avoidance 使用 heading-unaware rectangular Frenet corridor，和最终 rotated-footprint geometry 并不完全一致。

所以后来 redesign 不是简单“调权重”，而是改 formulation。

## 6. Canonical objective 为什么重要

Sampling 和 Optimization 内部求解方式不同，如果直接拿各自 solver 的 raw objective 比较并不公平。

项目因此有 planner-independent canonical objective，用统一定义重新评价两种 planner 的最终 Frenet path。

这使 benchmark 比较的是：

> 同一条最终路径，在同一 objective 定义下的表现。

而不是比较两个 solver 内部不同的数值尺度。

## 7. QP 的边界

QP 的核心优势也是限制：约束最好保持 convex / linear。

但真实 rotated vehicle footprint clearance 对 $l,l'$ 是 nonlinear 的，所以不能简单一次性直接塞进 OSQP。

这就是后面为什么需要 SCP：

> 在当前 nominal path 附近，把 nonlinear constraint 局部线性化，然后反复解 QP。

## 8. 代码对应

主要文件：

- [`src/planning/optimization_planner.py`](../../src/planning/optimization_planner.py)
  - `build_explicit_qp_objective()`
  - `build_explicit_qp_constraints()`
  - `solve_qp()`
  - `_solve_explicit_branch()`
- [`src/planning/path_objective.py`](../../src/planning/path_objective.py)
  - common / canonical path objective
- [`src/planning/lateral_corridor.py`](../../src/planning/lateral_corridor.py)
  - corridor-related geometry

## 9. 面试 30 秒讲法

> Optimization Planner 把 Frenet path discretize 成 decision variables，用 quadratic objective 表达 lateral deviation 和 smoothness，用 linear constraints 表达 initial state、road boundary 和 dynamics，再用 OSQP 解 QP。后来我没有继续在旧 l-only formulation 上调参，而是把状态改成 explicit $[l,l',l'']$，因为旧 finite-difference initial constraints 会人为限制起点附近自由度。

## 10. 常见追问

**Q：为什么选 OSQP？**  
A：问题是 sparse convex QP，OSQP 对这种结构成熟、接口直接，也方便重复求解。

**Q：QP 是否保证全局最优？**  
A：对单个 convex QP 是的；但完整系统里 obstacle exact clearance 是 nonlinear，SCP 只保证局部迭代，不等价于原始非凸问题全局最优。

**Q：为什么不直接 nonlinear optimization？**  
A：可以，但当前项目更重视可解释结构、稳定求解和 Planning/Control 面试相关性，所以保留 QP 主体，再用 SCP 处理必要的 nonlinear geometry。