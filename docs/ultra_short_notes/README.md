# P&C Ultra-short Notes

> Goal: use the shortest possible notes to recover **where I am, why this step exists, what it outputs, and why the next step is needed**.

These notes describe the **implemented baseline before Chapter 10**. They do not rewrite Phase 1 as an ideal future system.

## Main Flow

```text
Scenario
  ↓
Rule-based Behavior
  ↓
Straight Reference Trajectory
  ↓
Longitudinal Control + Lateral MPC
  ↓
Vehicle Model
  ↓
New Vehicle State
  └────────────→ feedback to controllers
```

Phase 2 then adds the planning geometry foundation:

```text
Discrete Reference Line
  ↓
Cartesian ↔ Frenet / SL
  ↓
Chapter 10: Smooth Reference Line
  ↓
Sampling Planner
```

## Notes

1. [System Overview](01_system_overview.md)
2. [Kinematic Bicycle Model](02_kinematic_bicycle.md)
3. [Longitudinal Control](03_longitudinal_control.md)
4. [Lateral MPC](04_lateral_mpc.md)
5. [Closed-loop Simulation](05_closed_loop_simulation.md)
6. [Reference Line Basics](06_reference_line_basics.md)
7. [Cartesian ↔ Frenet Basics](07_cartesian_frenet_basics.md)

## One-sentence Memory

- **Vehicle Model**: control command → next vehicle state.
- **Longitudinal Control**: speed error → acceleration.
- **Lateral MPC**: lateral/heading error → steering.
- **Closed Loop**: controller output changes the vehicle, then the new state creates the next error.
- **Reference Line**: gives road geometry a longitudinal coordinate `s`.
- **Frenet / SL**: describes a point as “how far along the road + how far sideways”.
