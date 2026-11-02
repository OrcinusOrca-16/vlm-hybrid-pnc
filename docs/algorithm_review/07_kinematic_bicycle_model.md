# 07 — Kinematic Bicycle Model

## 1. 直觉

Kinematic Bicycle Model 用一个“等效前轮 + 等效后轮”描述车辆平面运动，不建模轮胎侧偏力、悬架、横摆惯量等动力学。

它适合：

- 中低速几何运动；
- 路径跟踪算法原型；
- Planning 与 Control 的几何一致性检查。

它不适合直接代表极限操稳、高速轮胎动力学或执行器延迟。

## 2. 连续状态方程

项目使用 rear-axle reference：

$$
\dot x=v\cos\psi
$$

$$
\dot y=v\sin\psi
$$

$$
\dot\psi=\frac{v}{L}\tan\delta
$$

$$
\dot v=a
$$

其中：

- $(x,y)$：rear axle reference point；
- $\psi$：yaw；
- $v$：speed；
- $L$：wheelbase；
- $\delta$：front steering angle；
- $a$：longitudinal acceleration。

## 3. Steering 与 curvature 的关系

几何关系：

$$
\tan\delta=\frac{L}{R}
$$

而：

$$
\kappa=\frac1R
$$

所以：

$$
\kappa=\frac{\tan\delta}{L}
$$

反过来：

$$
\delta=\arctan(L\kappa)
$$

这是整个项目 Planning → Control consistency 的关键桥梁。

如果车辆 steering limit 是：

$$
|\delta|\le\delta_{max}
$$

则对应 curvature capability：

$$
|\kappa|\le\frac{\tan\delta_{max}}{L}
$$

项目用：

$$
L=2.7\text{ m},\qquad \delta_{max}=0.5\text{ rad}
$$

得到：

$$
\kappa_{max}\approx0.202334\ \mathrm{m}^{-1}
$$

## 4. 为什么这个 limit 触发了 Optimization redesign

早期 benchmark 的 curvature acceptance 过宽。

当改成 vehicle-derived curvature limit 后，一些旧 Optimization path 无法通过真实 vehicle capability。

这件事很重要，因为 redesign 的出发点不是“为了让 benchmark 分数更好”，而是：

> planner 的 hard constraint 应该和 downstream vehicle / controller capability 一致。

## 5. 离散化

项目用 Forward Euler：

$$
x_{k+1}=x_k+\dot x_k\Delta t
$$

同理：

$$
y_{k+1}=y_k+v_k\sin\psi_k\Delta t
$$

$$
\psi_{k+1}=\psi_k+\frac{v_k}{L}\tan\delta_k\Delta t
$$

$$
v_{k+1}=v_k+a_k\Delta t
$$

并保证 speed 不低于 0。

## 6. Kinematic Model 的主要假设

这个模型隐含：

- no tire slip；
- steering command 立即实现；
- 没有 actuator dynamics；
- 没有 lateral tire force saturation；
- road flat；
- vehicle rigid body。

所以最终 benchmark 能说明当前算法在这个 model 下的 closed-loop behavior，但不能直接等价于真实量产车辆表现。

## 7. 代码对应

主要文件：

- [`src/vehicle/kinematic_bicycle.py`](../../src/vehicle/kinematic_bicycle.py)
  - `update_vehicle_state()`
- [`src/control/path_tracking.py`](../../src/control/path_tracking.py)
  - curvature ↔ steering helper
- [`src/config/`](../../src/config/)
  - vehicle / planning capability parameters

## 8. 面试 30 秒讲法

> 我用 rear-axle kinematic bicycle model 做 closed-loop simulation。它的核心几何关系是 $\kappa=\tan\delta/L$。我也用同一个关系把 steering limit 转成 planner curvature limit，所以 Planning 的 path feasibility 和 Control 的 steering capability 是一致的。这个一致性检查后来直接暴露了旧 Optimization formulation 的结构问题。

## 9. 常见追问

**Q：为什么不用 dynamic bicycle model？**  
A：当前项目重点是 path planning、QP/SCP 和 lateral MPC integration。Kinematic model 足够验证几何层 Planning/Control consistency，同时保持模型和参数可解释。

**Q：高速下会有什么问题？**  
A：轮胎侧偏、yaw dynamics、lat acceleration、actuator delay 等都会变重要，kinematic model 会变得不够准确。

**Q：Forward Euler 是否太简单？**  
A：它适合作为当前 fixed-step prototype。更严格时可以改更高阶 integration，但这不是当前 portfolio 的主要误差来源。