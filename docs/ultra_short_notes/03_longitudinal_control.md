# Longitudinal Control

## Purpose

Track the reference speed by converting speed error into acceleration command.

## Input

```text
current speed v
reference speed v_ref
next reference speed v_ref_next (optional)
```

## Core Relation

Speed error:

$$
e_v=v_{ref}-v
$$

PID feedback:

$$
a_{fb}=K_p e_v+K_i\int e_vdt+K_d\frac{de_v}{dt}
$$

Current Phase 1 baseline uses:

```text
Kp = 1
Ki = 0
Kd = 0
```

so it is effectively P control.

Reference acceleration feedforward:

$$
a_{ref}=\frac{v_{ref,k+1}-v_{ref,k}}{\Delta t}
$$

Final command:

$$
a_{cmd}=a_{ref}+a_{fb}
$$

then clamp to acceleration/deceleration limits.

## Why P Is Enough Here

The Phase 1 vehicle model is ideal:

$$
\dot v=a
$$

No drag, slope, actuator delay, or external disturbance is modeled, so integral action is not necessary for the baseline.

## Output

```text
speed error
   ↓
feedback + feedforward
   ↓
acceleration command
```

## Next

Longitudinal control handles speed only.

We still need lateral control to track:

```text
position + heading
```
