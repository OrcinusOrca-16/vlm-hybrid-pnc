# Closed-loop Simulation

## Purpose

Connect planning, control, and vehicle dynamics into one feedback loop.

## Core Loop

At time step $k$:

```text
reference point
     +
current vehicle state
     ↓
compute tracking errors
     ↓
Longitudinal Controller → acceleration
Lateral MPC            → steering
     ↓
ControlCommand
     ↓
Vehicle Model
     ↓
new VehicleState
     ↓
next time step
```

Mathematically:

$$
x_{k+1}=f(x_k,u_k)
$$

and the new state changes the next control error.

That feedback is what makes it **closed loop**.

## Current Errors

Phase 1 straight-reference baseline uses:

$$
e_v=v_{ref}-v
$$

$$
e_y=y-y_{ref}
$$

$$
e_{yaw}=\psi-\psi_{ref}
$$

## Metrics

Tracking quality:

```text
Speed:   MAE / RMSE / MAX
Lateral: MAE / RMSE / MAX
Heading: RMSE / MAX
```

Steering smoothness:

$$
\dot\delta_k\approx\frac{\delta_{k+1}-\delta_k}{\Delta t}
$$

then evaluate mean/max absolute steering rate.

## Why This Matters

A controller cannot be judged only by one command.

The real question is:

> After the controller changes the vehicle, does the next error become better or worse?

## Output

A complete Phase 1 baseline that can be quantitatively evaluated.

## Next

The baseline trajectory is straight.

To build a real path planner, we next need road geometry and a road-centered coordinate system.
