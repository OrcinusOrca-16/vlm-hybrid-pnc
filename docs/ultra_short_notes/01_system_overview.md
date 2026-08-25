# System Overview

## Purpose

Build a minimal Planning & Control closed loop and make every module's input/output explicit.

## Main Flow

```text
ScenarioState
   ↓
Rule-based Behavior
   ↓
BehaviorCommand
(target speed / behavior)
   ↓
Straight Trajectory Generator
   ↓
TrajectoryPoint[]
(x, y, yaw, speed, time)
   ↓
┌──────────────────────────────┐
│ Longitudinal: speed error    │ → acceleration
│ Lateral MPC: tracking error  │ → steering
└──────────────────────────────┘
   ↓
ControlCommand
   ↓
Kinematic Bicycle Model
   ↓
New VehicleState
   └──────────────→ next control step
```

## Core Data

```text
VehicleState = (x, y, yaw, speed)
ControlCommand = (acceleration, steering_angle)
TrajectoryPoint = (x, y, yaw, speed, time)
```

## Why This Baseline Exists

First verify that:

```text
Behavior → Reference → Control → Vehicle → Feedback
```

works end-to-end before adding complex planning.

## Output

A runnable closed-loop baseline with tracking and smoothness metrics.

## Next

To plan non-straight paths, we need a road-centered geometric representation:

```text
Reference Line → Frenet / SL
```
