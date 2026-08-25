# Lateral MPC

## Purpose

Use predicted future tracking errors to choose steering.

## State and Input

Current baseline error state:

$$
x=
\begin{bmatrix}
e_y\\
e_{yaw}
\end{bmatrix}
$$

Control input:

$$
u=\delta
$$

## 1. Predict One Step

Linearized straight-reference model:

$$
e_{y,k+1}=e_{y,k}+v\Delta t\,e_{yaw,k}
$$

$$
e_{yaw,k+1}=e_{yaw,k}+\frac{v\Delta t}{L}\delta_k
$$

Write as:

$$
x_{k+1}=Ax_k+Bu_k
$$

## 2. Predict Many Steps

Stack the prediction over horizon $N$:

$$
X=\bar A x+\bar B U
$$

This answers:

> If I apply this sequence of steering commands, how will future errors change?

## 3. Define What "Good" Means

Penalize tracking error + steering effort:

$$
J=X^T\bar QX+U^T\bar RU
$$

Substitute the prediction model:

$$
\min_U\frac12U^THU+f^TU
$$

So MPC becomes a QP (Quadratic Program).

## 4. Add Constraints

Current baseline constrains:

```text
steering angle
steering rate
```

## 5. Solve and Execute

```text
current errors
    ↓
predict future errors
    ↓
minimize cost under constraints
    ↓
OSQP
    ↓
optimal steering sequence U*
    ↓
execute only first δ
    ↓
repeat next time step
```

This is Receding Horizon Control（滚动时域控制）.

## Output

```text
(e_y, e_yaw, v)
      ↓
     MPC
      ↓
steering angle δ
```

## Baseline Limitation

Phase 1 assumes a straight reference and has no curvature feedforward.

That is intentional for the frozen baseline; curved-path tracking is upgraded later.
