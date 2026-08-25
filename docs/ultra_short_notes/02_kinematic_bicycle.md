# Kinematic Bicycle Model

## Purpose

Convert the current vehicle state + control command into the next vehicle state.

## Input

```text
State:   x, y, yaw, v
Control: acceleration a, steering δ
```

## Core Model

Rear-axle kinematic bicycle model:

$$
\dot{x}=v\cos\psi
$$

$$
\dot{y}=v\sin\psi
$$

$$
\dot{\psi}=\frac{v}{L}\tan\delta
$$

$$
\dot{v}=a
$$

Why steering changes yaw:

$$
\tan\delta=\frac{L}{R}
$$

so

$$
\dot{\psi}=\frac{v}{R}=\frac{v}{L}\tan\delta
$$

## Discretization

Forward Euler:

$$
\text{state}_{k+1}=\text{state}_k+\dot{\text{state}}_k\Delta t
$$

## Output

```text
(x, y, yaw, v)_k
      +
(a, δ)_k
      ↓
(x, y, yaw, v)_(k+1)
```

## Why Next Step Is Needed

The vehicle model only says **what the car will do for a given command**.

We still need controllers to decide:

```text
How much acceleration?
How much steering?
```
