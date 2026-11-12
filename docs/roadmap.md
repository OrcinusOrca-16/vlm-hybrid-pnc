> Chinese version: [roadmap_cn.md](roadmap_cn.md). Both versions retain the same sections and checklist states.

# VLM-Guided Hybrid Autonomous Driving Planner — Development Roadmap

## Phase 0 — Project Initialization

- [x] Create GitHub repository
- [x] Set up Python environment
- [x] Configure Git / SSH
- [x] Create project directories

---

## Phase 1 — Planning & Control Baseline System

### 1. Common Data Structures

Establish shared data interfaces between Planning and Control.

- [x] `VehicleState`
- [x] `ScenarioState`
- [x] `BehaviorType`
- [x] `BehaviorCommand`
- [x] `TrajectoryPoint`
- [x] `ControlCommand`

### 2. Basic Scenario Modeling

- [x] Normal straight driving
- [x] Lead-vehicle slowdown
- [x] Pedestrian crossing

### 3. Rule-based Behavior Planner

- [x] `KEEP_LANE`
- [x] `SLOW_DOWN`
- [x] `STOP`

### 4. Basic Reference Trajectory Generation

- [x] Fixed-`dt` straight-line reference trajectory
- [x] KEEP_LANE / SLOW_DOWN / STOP validation
- [x] Acceleration/deceleration limits
- [x] Position integration
- [x] Stopping-distance check
- [x] Reference speed and acceleration outputs

### 5. Kinematic Bicycle Model

- [x] Derive `x / y / yaw / speed` state equations
- [x] Front-wheel steering and longitudinal acceleration inputs
- [x] Fixed-`dt` discretization
- [x] Constant-speed straight-driving validation
- [x] Acceleration/deceleration validation
- [x] Constant-steering turning validation

### 6. Longitudinal PID

- [x] Closed-loop speed-error regulation
- [x] PID controller
- [x] Acceleration clamping
- [x] anti-windup
- [x] Closed loop with vehicle model
- [x] Add reference-acceleration feedforward
- [x] Acceleration/deceleration/target-change validation

### 7. Lateral MPC

- [x] Define lateral / heading error
- [x] Derive linearized kinematic-bicycle tracking-error model
- [x] Discrete state-space model
- [x] Multi-step prediction model
- [x] Q / R quadratic cost
- [x] steering-angle constraint
- [x] steering-rate constraint
- [x] QP formulation
- [x] OSQP solve
- [x] receding-horizon first-control application
- [x] Initial lateral-error recovery
- [x] Initial heading-error recovery
- [x] baseline MPC calibration

### 8. Planning & Control Closed-loop Simulation

- [x] Reference Trajectory → PID / MPC
- [x] PID → acceleration
- [x] MPC → steering
- [x] ControlCommand → vehicle model
- [x] VehicleState feedback
- [x] KEEP_LANE closed loop
- [x] SLOW_DOWN closed loop
- [x] STOP closed loop

### 9. Basic Evaluation and Visualization

- [x] speed MAE / RMSE / max error
- [x] lateral MAE / RMSE / max error
- [x] heading RMSE / max error
- [x] steering-rate / smoothness
- [x] Unified P&C summary figure

---

## Phase 2 — Classical Trajectory Planning

### 10. Reference Line

Build a common smooth Reference Line.

- [x] Discrete Reference Points
- [x] cumulative arc length `s`
- [x] yaw
- [x] curvature
- [x] cubic spline `x(s) / y(s)`
- [x] curvature derivative `κ'`
- [x] `ReferenceLine.query(s)`
- [x] straight / circular / varying-curvature validation

### 11. Cartesian ↔ Frenet / SL

- [x] XY → SL
- [x] SL → XY
- [x] raw-waypoint coarse localization
- [x] spline projection
- [x] smooth Reference Line geometry
- [x] straight / endpoint / circular / varying-curvature round-trip validation

### 12. Sampling-based Planner

Generate spatial candidate paths with Frenet Sampling and screen using shared physical constraints.

#### Frenet geometry

- [x] `s / l / l' / l''`
- [x] `FrenetTrajectoryPoint / FrenetTrajectory`
- [x] Candidate yaw
- [x] `1 - κ_r l` geometry
- [x] Candidate curvature
- [x] `κ_r'`
- [x] Frenet → Cartesian geometry
- [x] `PathPoint`

#### Candidate generation

- [x] target lateral positions
- [x] target maneuver lengths
- [x] quintic polynomial
- [x] ego initial `l / l' / l''`
- [x] bypass-and-return candidate family
- [x] common planning horizon
- [x] terminal state sampling

#### Physical constraints

- [x] vehicle footprint
- [x] static circular obstacle
- [x] exact sampled-pose obstacle clearance
- [x] exact sampled Cartesian curvature check
- [x] Frenet-based road-boundary pruning
- [x] infeasible candidate removal

#### Objective / selection

- [x] lateral deviation
- [x] `l' / l'' / l'''` smoothness terms
- [x] terminal `l / l'`
- [x] canonical matched objective
- [x] minimum-cost feasible candidate selection

#### Validation

- [x] straight nominal
- [x] curved Reference Line
- [x] lateral recovery
- [x] static obstacle avoidance
- [x] candidate visualization
- [x] planning time
- [x] minimum clearance
- [x] maximum curvature
- [x] path length
- [x] planning success rate

---

### 13. Optimization-based Path Planner — Initial Version

The first Optimization Planner used an `l`-only convex QP, finite differences and Frenet obstacle corridors.

This historical baseline is retained to explain the subsequent redesign.

#### Initial QP state / objective

- [x] optimization variable `l`
- [x] `D1` finite-difference approximation of `l'`
- [x] `D2` finite-difference approximation of `l''`
- [x] `D3` finite-difference approximation of `l'''`
- [x] quadratic lateral / smoothness objective
- [x] terminal objective
- [x] OSQP `P / q`

#### Initial constraints

- [x] road lateral bounds
- [x] initial `l` constraint
- [x] initial `l'` finite-difference constraint
- [x] initial `l''` finite-difference constraint
- [x] `D2` curvature-related proxy bound
- [x] OSQP `A / lower / upper`

#### Initial obstacle treatment

- [x] obstacle Frenet projection
- [x] obstacle inflation
- [x] negative / positive lateral corridor branches
- [x] corridor feasibility check
- [x] multi-corridor QP solve
- [x] final physical post-validation

#### Initial validation

- [x] nominal solve
- [x] curved Reference Line
- [x] lateral maneuver
- [x] static obstacle avoidance
- [x] corridor / optimized-path visualization

---

### 14. Sampling vs Optimization Unified Benchmark

Establish a fair comparison under identical Behavior intent.

#### 14.1 Centralized Configuration

- [x] `src/config/` centralized configuration
- [x] vehicle geometry
- [x] road / planning constraints
- [x] Sampling parameters
- [x] Optimization parameters
- [x] OSQP / numerical tolerance
- [x] benchmark runtime settings

#### 14.2 Matched Objective

Common geometric objective:

`J = w_l J_l + w_dl J_dl + w_d2l J_d2l + w_d3l J_d3l + w_terminal_l J_terminal_l + w_terminal_dl J_terminal_dl`

- [x] `l`
- [x] `l'`
- [x] `l''`
- [x] `l'''`
- [x] terminal `l`
- [x] terminal `l'`
- [x] mean-square normalization
- [x] planner-independent canonical objective evaluator

#### 14.3 Shared physical acceptance

Both planners use the same final acceptance contract:

- [x] exact sampled Cartesian curvature check
- [x] exact sampled-pose footprint-to-obstacle clearance
- [x] Frenet-based road-boundary validation
- [x] initial-state consistency

> Note: production road validation uses the Frenet centerline ± half-width contract rather than exact rotated full-footprint containment. The subsequent rotated-corner diagnostic does not change production PASS/FAIL.

#### 14.4 Unified benchmark interface

- [x] `PlannerBenchmarkResult`
- [x] success / failure
- [x] Frenet trajectory
- [x] Cartesian path
- [x] planning time
- [x] failure reason
- [x] canonical matched objective
- [x] Sampling adapter
- [x] Optimization adapter

#### 14.5 Common metrics

- [x] planning success
- [x] canonical objective
- [x] terminal lateral error
- [x] terminal slope error
- [x] RMS lateral deviation
- [x] Cartesian path length
- [x] minimum exact obstacle clearance
- [x] maximum absolute Cartesian curvature
- [x] RMS curvature
- [x] RMS curvature rate
- [x] planning computation time

#### 14.6 Benchmark visualization

- [x] Cartesian overlay
- [x] Frenet `s-l` overlay
- [x] road boundaries
- [x] original obstacle
- [x] Sampling best path
- [x] Optimization path

#### 14.7 Runtime / stress benchmark

- [x] warm-up / repeated timing
- [x] full Sampling pipeline timing
- [x] full Optimization pipeline timing
- [x] nominal / curved / obstacle / narrow-road scenarios
- [x] obstacle lateral sweep
- [x] obstacle longitudinal sweep
- [x] road-width sweep
- [x] failure recording

#### 14.8 Tuning methodology

- [x] independent tuning scenario set
- [x] freeze Sampling candidate/config search space
- [x] freeze Optimization tuning parameters
- [x] avoid tuning on final evaluation scenarios

#### 14.9 Original frozen evaluation

Original 9-scenario evaluation result before physical-consistency redesign:

- Sampling: `8 / 9`
- Optimization: `8 / 9`
- shared failure: `evaluation_road_half_width_1.85`

The `1.85 m` case was already part of the original evaluation geometry; it was not added later.

---

### 15. Optimization Planner Physical-Consistency Redesign

This redesign was triggered by **Planning → MPC / vehicle consistency**, not by benchmark score optimization.

#### 15.1 Vehicle-derived curvature limit

The earlier benchmark used an overly permissive limit:

`κ_max = 1.0 1/m`

Derived from vehicle parameters:

- wheelbase `L = 2.7 m`
- max steering `δ_max = 0.5 rad`

Replaced with:

`κ_max = tan(δ_max) / L ≈ 0.202334 1/m`

- [x] Derive curvature limit from vehicle config
- [x] Align Planning acceptance with vehicle/MPC physical capability
- [x] exact Cartesian curvature post-validation

#### 15.2 Root-cause diagnosis

The physically consistent curvature limit exposed structural weaknesses in the original Optimization formulation.

- [x] Compare Sampling and Optimization feasibility under identical physical constraints
- [x] Confirm the close-obstacle scene is not inherently infeasible
- [x] Identify early-path coupling/start-locking caused by `l`-only finite-difference initial constraints
- [x] Identify the conservative heading freedom of rectangular Frenet corridors
- [x] Identify a formulation limitation rather than an objective-weight tuning problem

#### 15.3 Explicit Frenet state

New Optimization state:

`x = [l, l', l'']`

- [x] explicit `l`
- [x] explicit `l'`
- [x] explicit `l''`
- [x] direct initial-state constraints

#### 15.4 Piecewise-constant-jerk dynamics

- [x] `j_i = (l''_{i+1} - l''_i) / Δs`
- [x] `l'_{i+1} = l'_i + Δs/2 (l''_i + l''_{i+1})`
- [x] `l_{i+1} = l_i + Δs l'_i + Δs²/3 l''_i + Δs²/6 l''_{i+1}`
- [x] linear equality constraints for QP

#### 15.5 Heading-aware obstacle geometry

- [x] Frenet heading relation `tan(Δψ) = l' / (1 - κ_r l)`
- [x] heading-aware nominal obstacle constraints
- [x] exact rotated vehicle rectangle vs circular obstacle clearance
- [x] sampled-pose exact clearance validation

#### 15.6 Sequential convex QP refinement

- [x] numerical clearance gradient w.r.t. `l`
- [x] numerical clearance gradient w.r.t. `l'`
- [x] local linearized clearance constraint
- [x] trust region in `l`
- [x] trust region in `l'`
- [x] iterative QP refinement
- [x] exact revalidation after each solve

Frozen redesign settings include:

- `sequential_max_iterations = 6`
- `clearance_numerical_buffer_m = 0.001`
- `trust_region_l_m = 0.15`
- `trust_region_dl = 0.08`
- clearance gradient epsilon `= 1e-4`

#### 15.7 Development-only regression

- [x] dedicated `validate_planner_redesign_development.py`
- [x] development regression does not build or run final evaluation scenarios
- [x] required development/tuning scenarios: `7 / 7 PASS`
- [x] stress envelope: `19` cases
- [x] production-valid stress successes: `16`
- [x] capability-boundary / infeasible stress cases: `3`
- [x] near-boundary diagnostics
- [x] independent rotated-footprint road diagnostic

#### 15.8 Road-boundary limitation documented

Production road acceptance remains:

`Frenet centerline ± vehicle half-width`

- [x] keep shared production road contract unchanged for both planners
- [x] add rotated-corner diagnostic warning
- [x] document cases where rotated corners can exceed the simplified road contract
- [ ] future: exact rotated-footprint production road-boundary constraint

Correct project wording:

> Exact sampled-pose obstacle clearance and Cartesian curvature validation, with Frenet-based road-boundary validation.

#### 15.9 Post-redesign frozen evaluation

Same 9 scenario geometries rerun after redesign:

- Sampling: `8 / 9`
- Optimization: `9 / 9`
- Sampling failure: `evaluation_road_half_width_1.85`
- Optimization: no evaluation failure

Methodological note:

Because some evaluation cases were inspected during diagnosis, this set is better described as a **frozen evaluation suite** rather than a pristine unseen held-out set.

- [x] do not expand/tune Sampling candidate set after observing `1.85 m` result
- [x] compare path quality only on common-success scenarios
- [x] archive previous figures
- [x] save source provenance
- [x] update benchmark documentation

---

### 16. Curvature-aware MPC + Planning Integration

The Phase 1 MPC remains the baseline. Spatial planned paths feed the common curvature-aware controller; longitudinal speed-transition dynamics remain out of scope.

- [x] Obtain reference curvature from the planned path
- [x] steering curvature feedforward
- [x] MPC feedback correction
- [x] spatial path → constant safe reference speed
- [x] curved-path tracking validation
- [x] Sampling path + common MPC
- [x] Optimization path + common MPC
- [x] Check consistency between planner curvature and MPC steering constraints
- [x] Assess need for horizon/Q/R tuning: current tests do not justify a systematic search
- [ ] Future: planner-level steering-rate / curvature-rate feasibility at design speed

---

### 17. Final Planning & Control Benchmark

Compare both planners in closed-loop operation using the same controller.

- [x] Sampling Planner + common MPC
- [x] Optimization Planner + common MPC
- [x] lateral tracking error
- [x] heading error
- [x] steering / steering rate
- [x] trajectory quality
- [x] planning computation time
- [x] controller feasibility
- [x] failure cases

Final frozen 9-scenario result:

- Sampling Planner: `8 / 9` planning success; all `8 / 8` successful plans tracked successfully.
- Optimization Planner: `9 / 9` planning success and closed-loop tracking success.
- Common-success comparisons use the same controller and an auxiliary common-speed diagnostic.
- Final interpretation and limitations are documented in `docs/reports/planning_control_benchmark.md`.

---

## Potential Future Extensions (Outside Current Scope)

These are optional research directions, not required phases of the completed Classical Planning & Control implementation.

- **Dynamic scenarios and system evaluation:** dynamic vehicles, pedestrians, lead-vehicle slowdown, and cut-ins; evaluate collision rate, task completion, and minimum separation.
- **Hybrid Sampling–Optimization Planner:** use Sampling for a feasible path or initial guess, refine it with Optimization, and compare all three approaches under common metrics.
- **Learning-based Trajectory Planner:** explore learned trajectory generation or refinement (e.g., imitation learning or diffusion-based planning), then apply the same physical feasibility checks and downstream MPC evaluation used for classical planners.
- **Safety monitoring and fallback:** independent deterministic safety checks, fallback after planning failure, and safety-intervention metrics.
- **VLM-based behavior planning and evaluation:** use VLM only for high-level `BehaviorCommand` selection (`KEEP_LANE / SLOW_DOWN / STOP / CAUTIOUS`), never direct control; compare rule-based and VLM decisions using the same scenarios and downstream Planning & Control for accuracy, latency, interventions, and failure cases.
