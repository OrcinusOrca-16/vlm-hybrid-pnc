# 01 — Reference Line 与 Frenet

## 1. 直觉

Cartesian 坐标直接描述世界中的位置：

$$
(x, y)
$$

Frenet 坐标改成相对于道路中心线描述：

$$
(s, l)
$$

其中：

- $s$：沿 Reference Line 前进的弧长；
- $l$：相对 Reference Line 的横向偏移。

这样一条弯路在 Frenet 中会被“拉直”。例如绕障碍问题不再主要是同时设计 $x(s),y(s)$，而是设计：

$$
l(s): 0 \rightarrow 1.0 \rightarrow 0
$$

也就是“沿道路向前走时，横向偏移怎么变化”。

## 2. Reference Line 为什么需要平滑

项目中的 `ReferenceLine` 用 cubic spline 表示：

$$
x=x(s),\qquad y=y(s)
$$

这样可以从 spline 的导数连续得到：

- position；
- yaw；
- curvature；
- curvature derivative。

Yaw：

$$
\psi_r = \mathrm{atan2}\left(\frac{dy}{ds},\frac{dx}{ds}\right)
$$

Curvature：

$$
\kappa_r = \frac{x'y''-y'x''}{(x'^2+y'^2)^{3/2}}
$$

如果只保留离散 waypoint 而没有平滑 Reference Line，后面的 Frenet geometry、curvature 和 MPC reference 都会更容易受到离散噪声影响。

## 3. Cartesian → Frenet

先在 Reference Line 上找到距离当前点最近的 $s$：

$$
s^*=\arg\min_s \|P-P_r(s)\|^2
$$

项目里先用 raw waypoint 做 coarse search，再在局部 spline 区间内细化 `nearest_s()`。

Reference Line 的单位法向量：

$$
n_r=[-\sin\psi_r,\ \cos\psi_r]
$$

横向偏移：

$$
l=(P-P_r)\cdot n_r
$$

所以 `xy_to_sl()` 的本质就是：

> 找最近参考点，然后把位置误差投影到 Reference Line 的法向方向。

## 4. Frenet → Cartesian

给定 $(s,l)$：

$$
P = P_r(s)+l\,n_r(s)
$$

对应：

$$
x=x_r-l\sin\psi_r
$$

$$
y=y_r+l\cos\psi_r
$$

这就是 `sl_to_xy()`。

## 5. `l'`、`l''` 的含义

项目中的导数是对空间弧长 $s$ 求导，不是对时间：

$$
l'=\frac{dl}{ds},\qquad l''=\frac{d^2l}{ds^2}
$$

直觉：

- $l$：车离 Reference Line 多远；
- $l'$：路径相对 Reference Line 有多“斜”；
- $l''$：这个斜率改变得多快。

候选路径相对 Reference Line 的 heading 满足：

$$
\Delta\psi = \mathrm{atan2}(l',1-\kappa_r l)
$$

所以：

$$
\psi=\psi_r+\Delta\psi
$$

在直路、小偏移情况下 $\kappa_r\approx0$：

$$
\tan(\Delta\psi)\approx l'
$$

这个关系非常重要，因为后面的 rotated vehicle footprint clearance 会直接受 $l'$ 影响。

## 6. Frenet 曲率为什么比想象中复杂

Cartesian path curvature 不等于简单的 $l''$。它还受到：

- Reference Line curvature $\kappa_r$；
- curvature derivative $\kappa_r'$；
- lateral offset $l$；
- slope $l'$；
- second derivative $l''$。

项目在 `frenet_to_cartesian_geometry()` 中完整重建 Cartesian yaw 和 curvature，再用物理 validator 判断最终是否可行。

因此：

> planner 内部用 Frenet quantity 做优化，不代表最终可以只用 Frenet proxy 判断车辆物理可行性。

## 7. 代码对应

主要文件：

- [`src/planning/reference_line.py`](../../src/planning/reference_line.py)
  - `compute_cumulative_s()`
  - `ReferenceLine.position()`
  - `ReferenceLine.yaw()`
  - `ReferenceLine.curvature()`
  - `ReferenceLine.curvature_derivative()`
  - `ReferenceLine.nearest_s()`
  - `ReferenceLine.query()`
- [`src/planning/frenet.py`](../../src/planning/frenet.py)
  - `xy_to_sl()`
  - `sl_to_xy()`
  - `frenet_to_cartesian_geometry()`
  - `cartesian_to_frenet_geometry()`
  - `frenet_trajectory_to_path()`
  - `vehicle_state_to_frenet()`

## 8. 面试 30 秒讲法

> 我先用平滑 Reference Line 把道路表示成弧长参数 $s$，然后把二维路径规划问题转到 Frenet 坐标。这样 planner 主要优化横向 profile $l(s)$，对道路弯曲更自然。最终我不会直接把 Frenet proxy 当作真实车辆几何，而是重新构造 Cartesian position、heading 和 curvature，再做统一物理验证。

## 9. 常见追问

**Q：为什么不直接在 Cartesian 中优化 $x,y$？**  
A：可以，但 Frenet 把道路约束和 lateral maneuver 表达得更直接，特别适合 lane-level path planning。

**Q：`l'` 是横向速度吗？**  
A：不是。这里是 $dl/ds$，是空间 profile 的 slope，不是 $dl/dt$。

**Q：Frenet 有什么风险？**  
A：Reference Line 投影需要局部唯一；当 $1-\kappa_r l$ 接近零时几何会接近奇异；真实曲率和 footprint 仍需要回到 Cartesian 检查。
