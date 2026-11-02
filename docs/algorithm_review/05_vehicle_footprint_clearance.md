# 05 — Vehicle Footprint 与 Obstacle Clearance

## 1. 为什么不能只看车中心点

自动驾驶路径是否安全，不取决于 vehicle center 是否避开障碍，而取决于整车 footprint 是否与障碍保持足够距离。

项目用矩形车辆模型，参考点位于 rear axle。局部坐标中的四个角由：

- wheelbase；
- front overhang；
- rear overhang；
- vehicle width；

共同决定。

## 2. 车辆矩形如何旋转到世界坐标

设车辆姿态为：

$$
(x,y,\psi)
$$

局部角点 $(x_b,y_b)$ 旋转到世界坐标：

$$
\begin{bmatrix}
x_g\\y_g
\end{bmatrix}
=
\begin{bmatrix}
x\\y
\end{bmatrix}
+
\begin{bmatrix}
\cos\psi & -\sin\psi\\
\sin\psi & \cos\psi
\end{bmatrix}
\begin{bmatrix}
x_b\\y_b
\end{bmatrix}
$$

项目里的 `compute_vehicle_footprint()` 就是在做这件事。

## 3. 圆形障碍与矩形车辆的 clearance

障碍物在世界坐标中是圆：

$$
O=(x_o,y_o,r_o)
$$

为了求距离，项目先把 obstacle center 变换到 vehicle-local frame。

然后找这个点到 axis-aligned vehicle rectangle 的最近点：

$$
x_c=\operatorname{clamp}(x_o^{local},x_{min},x_{max})
$$

$$
y_c=\operatorname{clamp}(y_o^{local},y_{min},y_{max})
$$

点到矩形的距离：

$$
d_{rect}=\sqrt{(x_o^{local}-x_c)^2+(y_o^{local}-y_c)^2}
$$

再减去障碍物半径：

$$
clearance=d_{rect}-r_o
$$

最终要求：

$$
clearance>d_{min}
$$

当前 benchmark 的 minimum clearance 是 0.5 m。

## 4. 为什么旧 rectangular Frenet corridor 不够

旧 corridor 会根据 obstacle、vehicle width、margin 等在 Frenet 中构造一个近似禁区。

问题是车辆真实 footprint 会随 yaw 旋转。

在直路近似下：

$$
\Delta\psi\approx\arctan(l')
$$

一个半长为 $a$、半宽为 $b$ 的矩形旋转后，其横向投影大约为：

$$
b|\cos\Delta\psi|+a|\sin\Delta\psi|
$$

所以当 $l'\neq0$ 时，车辆 front/rear corner 可能比简单的 half-width 更向外伸。

于是可能出现：

> centerline 满足旧 Frenet corridor，但真实 rotated vehicle corner 仍然违反 obstacle clearance。

## 5. Sampling 为什么在这里更“自然”

Sampling 的流程是：

1. 生成完整 candidate；
2. 重建真实 $(x,y,\psi)$；
3. 对每个 sampled pose 计算 exact rectangle-circle clearance；
4. 不安全就直接丢掉该 candidate。

因为它有很多候选，一个 candidate 失败后可能还有别的候选可用。

Optimization 旧版本通常只有某个 branch 下的一条 QP solution，如果这条 path 通过 coarse corridor 但 exact validator 失败，就没有天然的 alternate candidate。

这也是后来需要 sequential refinement 的原因之一。

## 6. “Exact” 应该怎么说才准确

项目目前的 obstacle geometry 对每一个 sampled pose 的 rectangle-circle distance 是 exact 的。

但它不是 continuous swept-volume collision checking。

也就是说当前实现是：

> exact rotated-footprint clearance at sampled poses

而不是：

> exact continuous collision checking between all knots

这一区分在面试中要说清楚。

## 7. Road boundary 也不是同样精度

Obstacle clearance 使用 rotated footprint geometry。

但 production road boundary 仍然使用：

$$
l-\frac{w}{2}\ge l_{min}
$$

$$
l+\frac{w}{2}\le l_{max}
$$

也就是 Frenet centerline ± half vehicle width。

因此当前项目的正确描述是：

> exact sampled-pose obstacle clearance and Cartesian curvature validation, with Frenet-based road-boundary validation.

## 8. 代码对应

主要文件：

- [`src/planning/hard_constraints.py`](../../src/planning/hard_constraints.py)
  - `compute_vehicle_footprint()`
  - `obstacle_clearance()`
  - `has_safe_clearance()`
  - `is_obstacle_clearance_feasible()`
  - `is_road_boundary_feasible()`
  - `is_physical_path_feasible()`
- [`src/planning/frenet.py`](../../src/planning/frenet.py)
  - path yaw / curvature reconstruction

## 9. 面试 30 秒讲法

> 我没有把障碍避让只做成 center-point distance。最终 validator 会根据 path pose 构造 rotated vehicle rectangle，再把 circular obstacle 转到 vehicle frame 中，计算圆心到矩形最近点的距离，从而得到真实 footprint clearance。旧 optimizer 的 rectangular Frenet corridor 没有完全表达 heading-dependent footprint，这也是后续引入 exact-clearance SCP refinement 的主要原因。

## 10. 常见追问

**Q：为什么 obstacle 用圆？**  
A：这是当前 portfolio 的简化场景模型，重点验证 planner formulation 和 vehicle footprint handling；扩展到 polygon obstacle 不改变整体思路，但最近距离计算会更复杂。

**Q：为什么不用每个角点到障碍物的距离？**  
A：障碍中心到矩形最近点更完整，因为最近点可能落在边上，不一定是 corner。

**Q：sampled-pose checking 会漏碰撞吗？**  
A：理论上会，如果采样过稀且两点之间发生碰撞。因此 swept-volume 或 adaptive spatial sampling 是更严格的后续方向。