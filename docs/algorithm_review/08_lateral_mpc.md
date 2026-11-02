# 08 — Lateral MPC

## 1. MPC 在这里解决什么问题

Planner 给出的是 reference path，MPC 的任务是让车辆沿这条 path 走。

项目中的 lateral state：

$$
x=[e_y,e_{yaw}]^T
$$

其中：

- $e_y$：lateral tracking error；
- $e_{yaw}$：heading error。

control input 是 steering angle 或 steering correction。

MPC 的核心思想：

> 不只看当前误差，而是预测未来一段 horizon 内的误差，并提前选择一串 steering commands，使未来总代价最小。

## 2. Baseline linear error model

在 small-angle / local linearization 下，项目基础模型：

$$
e_{y,k+1}=e_{y,k}+v\Delta t\,e_{yaw,k}
$$

$$
e_{yaw,k+1}=e_{yaw,k}+\frac{v\Delta t}{L}\delta_k
$$

写成：

$$
x_{k+1}=Ax_k+Bu_k
$$

其中：

$$
A=
\begin{bmatrix}
1 & v\Delta t\\
0 & 1
\end{bmatrix}
$$

$$
B=
\begin{bmatrix}
0\\
\frac{v\Delta t}{L}
\end{bmatrix}
$$

## 3. Prediction matrices

为了预测 horizon $N$：

$$
X=\bar A x_0+\bar B U
$$

其中：

$$
X=[x_1^T,x_2^T,\dots,x_N^T]^T
$$

$$
U=[u_0,u_1,\dots,u_{N-1}]^T
$$

$\bar A$ 由 $A,A^2,\dots,A^N$ 堆叠，$\bar B$ 是由 $A^iB$ 构成的 lower-triangular block matrix。

这一步把未来所有状态都改写成“当前 state + future input sequence”的函数。

## 4. Cost function

典型 cost：

$$
J=X^T\bar QX+U^T\bar RU
$$

代入 prediction equation：

$$
X=\bar Ax+\bar BU
$$

展开后得到 QP：

$$
\min_U \frac12U^THU+f^TU
$$

其中：

$$
H=2(\bar B^T\bar Q\bar B+\bar R)
$$

$$
f=2\bar B^T\bar Q\bar A x
$$

所以 lateral MPC 本质上也是一个 QP。

## 5. Steering angle constraint

车辆不能无限打方向：

$$
\delta_{min}\le\delta_k\le\delta_{max}
$$

这可以直接写成 identity matrix 上的 bound。

## 6. Steering rate constraint

除了 steering angle，本项目还限制相邻 command 的变化：

$$
|\delta_k-\delta_{k-1}|\le\dot\delta_{max}\Delta t
$$

用 difference matrix：

$$
D U=
\begin{bmatrix}
\delta_0\\
\delta_1-\delta_0\\
\delta_2-\delta_1\\
\vdots
\end{bmatrix}
$$

第一个 input 需要和上一时刻实际执行的 steering 比较。

这就是为什么 controller 保存 `previous_steering_angle`。

## 7. Receding Horizon

MPC 每次会求一整串：

$$
U^*=[\delta_0^*,\delta_1^*,\dots]
$$

但只真正执行第一个：

$$
\delta=\delta_0^*
$$

下一控制周期重新测量 state，再重新 solve。

这就是 receding horizon control。

## 8. 为什么后来加 curvature feedforward

基础 MPC 如果 reference path 本身有 curvature，即使 tracking error 为 0，车辆仍需要持续 steering 才能沿弯道走。

因此先计算：

$$
\delta_{ff}=\arctan(L\kappa_{ref})
$$

然后 MPC 优化 correction：

$$
\delta=\delta_{ff}+\delta_{fb}
$$

这比让 feedback controller 独自“制造”整个弯道 steering 更合理。

## 9. Curvature-aware error model

项目在 curved reference 附近线性化：

$$
\dot e_y\approx v e_{yaw}
$$

$$
\dot e_{yaw}\approx -v\kappa_{ref}^2 e_y+rac{v}{L}\sec^2(\delta_{ff})\delta_{correction}
$$

然后离散化成新的 $A,B$。

注意当前 implementation 在一个 prediction horizon 内把 reference curvature 视作局部常数。

## 10. 代码对应

主要文件：

- [`src/control/lateral_mpc.py`](../../src/control/lateral_mpc.py)
  - `build_discrete_model()`
  - `build_prediction_matrices()`
  - `build_cost_matrices()`
  - `build_qp_cost()`
  - `build_input_constraints()`
  - `solve_qp()`
  - `MPCController`
- [`src/control/curvature_aware_mpc.py`](../../src/control/curvature_aware_mpc.py)
  - `build_curvature_aware_discrete_model()`
  - `build_curvature_aware_input_constraints()`
  - `CurvatureAwareMPCController`

## 11. 面试 30 秒讲法

> 我的 lateral MPC 状态是 lateral error 和 heading error。先用 linearized kinematic model 建立 $x_{k+1}=Ax_k+Bu_k$，再堆叠成 horizon prediction，形成 quadratic cost，同时加入 steering-angle 和 steering-rate constraints，用 OSQP 求解。对于 curved path，我额外加入 $\delta_{ff}=\arctan(L\kappa_{ref})$ 的 curvature feedforward，MPC 只优化 feedback correction。

## 12. 常见追问

**Q：为什么 MPC 比普通 feedback 更合适？**  
A：它能显式预测未来误差并同时处理 input magnitude / rate constraints。

**Q：Q、R 怎么理解？**  
A：Q 控制 tracking error 惩罚，R 控制 steering effort。权重不是“越大越好”，而是在 tracking、smoothness 和 constraint usage 之间做 trade-off。

**Q：为什么 feedforward 很重要？**  
A：弯道上即使 error=0，也需要非零 steering。feedforward 负责 nominal curvature demand，feedback 负责模型误差和 tracking correction。