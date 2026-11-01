# VLM-Guided Hybrid Autonomous Driving Planner

A modular autonomous-driving **Planning & Control** portfolio focused on classical path planning, optimization, vehicle constraints, controller integration, and reproducible evaluation.

> **Current focus:** classical Planning & Control. The VLM behavior layer in the repository name is a planned optional extension and is not part of the current planner benchmark.

The repository currently contains two completed and connected tracks:

1. a Frenet path-planning benchmark comparing a Sampling Planner with a redesigned QP Optimization Planner under shared vehicle and safety constraints;
2. a common curvature-aware MPC closed loop used to evaluate how each planner path behaves after control integration.

---

## Current Status

### Planning

- Smooth spline Reference Line with yaw, curvature, and curvature derivative
- Cartesian ↔ Frenet conversion
- Sampling-based Frenet path planner
- Optimization-based Frenet path planner
- Explicit Optimization state `[l, l', l'']`
- Piecewise-constant-jerk dynamics
- Static-obstacle avoidance
- Heading-aware / exact sampled-pose vehicle-footprint clearance
- Vehicle-derived curvature limit
- Sequential convex QP refinement
- Common physical acceptance checks
- Development regression and stress-envelope validation
- Frozen nine-scenario evaluation suite

### Control

- Longitudinal feedforward + PID baseline
- Curvature-aware lateral MPC based on a linearized kinematic bicycle model
- Steering-angle and steering-rate constraints
- OSQP-based QP solve
- Constant-safe-speed interface derived from path steering demand
- Closed-loop Sampling-vs-Optimization comparison through the same controller

### Figure color convention

Planner identity is fixed across portfolio figures:

- **Sampling = red**
- **Optimization = purple**

Colors are assigned by planner name, not by plotting order. This prevents a remaining Optimization curve from appearing red when Sampling has no valid result.

---

## Headline Results

### Planning-only benchmark

| Planner | Evaluation success | Mean runtime |
| --- | ---: | ---: |
| Sampling | 8 / 9 | 190.7 ms |
| Redesigned Optimization | **9 / 9** | **46.3 ms** |

On the 8 scenarios solved by both planners:

| Metric | Sampling | Optimization |
| --- | ---: | ---: |
| Canonical objective | 0.5125 | **0.2029** |
| RMS lateral deviation [m] | 0.6156 | **0.3772** |
| Worst obstacle clearance [m] | **0.5544** | 0.500096 |
| Mean max curvature [1/m] | **0.0772** | 0.1152 |
| RMS curvature rate [1/m²] | 0.0263 | **0.0224** |

The planning-only conclusion is:

> **The current Optimization Planner is stronger on planning coverage, canonical objective, and computation time, while Sampling retains advantages in peak-curvature and clearance margin in the current suite.**

### Planning + Control benchmark

All paths that were successfully planned were also tracked successfully by the common curvature-aware MPC.

For the 8 common-success scenarios:

| Metric | Sampling | Optimization |
| --- | ---: | ---: |
| Mean constant safe speed | **3.473 m/s** | 3.058 m/s |
| Mean system RMS lateral error | **0.00989 m** | 0.01317 m |
| Mean system RMS heading error | **0.00615 rad** | 0.00913 rad |
| Mean same-speed RMS lateral error | **0.00655 m** | 0.01109 m |
| Mean same-speed RMS heading error | **0.00451 rad** | 0.00793 rad |
| Mean same-speed max feedback magnitude | **0.0273 rad** | 0.0710 rad |

This does **not** reverse the planning conclusion. It shows that planner quality and controller trackability are related but not identical objectives.

> **Optimization is the stronger planner overall in the current frozen suite, but Sampling can produce paths that are easier for the current controller to track.**

See [`docs/final_planning_control_findings.md`](docs/final_planning_control_findings.md) for the detailed interpretation.

---

## System Architecture

```text
Scenario / Fixed Behavior Intent
            ↓
      Reference Line
            ↓
     Cartesian ↔ Frenet
            ↓
   ┌────────┴─────────┐
   │                  │
Sampling Planner   Optimization Planner
   │                  │
   └────────┬─────────┘
            ↓
 FrenetTrajectory + Cartesian Path
            ↓
 Common Physical Validation
            ↓
 Constant Safe-Speed Interface
            ↓
      Curvature-aware MPC
            ↓
      Steering command
            ↓
    Kinematic Bicycle Model
            ↓
 Closed-loop tracking metrics
```

The same controller and vehicle assumptions are used for both planner outputs.

---

## The Main Engineering Story: Planning–Control Consistency

The key planner redesign began when the planner was prepared for downstream MPC tracking.

The earlier benchmark used a permissive curvature acceptance limit near:

```text
kappa_max = 1.0 1/m
```

But the configured vehicle has:

```text
wheelbase L        = 2.7 m
max steering angle = 0.5 rad
```

Using bicycle-model steering geometry:

```text
kappa_max = tan(delta_max) / L
          = tan(0.5) / 2.7
          ≈ 0.202334 1/m
```

Once planning acceptance was aligned with this vehicle-derived limit, the old Optimization Planner exposed structural weaknesses in close-obstacle cases.

The original formulation used:

- an `l`-only optimization vector;
- finite-difference approximations for `l'`, `l''`, and `l'''`;
- initial derivative constraints imposed indirectly through early `l` samples;
- inflated rectangular Frenet obstacle corridors.

Two limitations became important:

1. finite-difference initial-state constraints could over-couple early lateral samples and delay useful motion;
2. a rectangular Frenet obstacle corridor could not fully exploit heading-dependent vehicle-footprint clearance.

The planner was therefore redesigned instead of tuned around those symptoms.

---

## Sampling Planner

The Sampling Planner generates a discrete family of smooth Frenet candidates using quintic polynomials.

The frozen obstacle-bypass search uses:

```text
9 lateral positions × 5 bypass lengths = 45 candidates
```

### Strengths

- simple and interpretable candidate generation;
- smooth quintic candidate family;
- strong terminal-return behavior;
- low peak curvature in several benchmark cases;
- easy failure diagnosis because candidates can be inspected individually.

### Structural limitation

Sampling only searches the predefined candidate family. A continuous feasible region may exist between candidates.

That limitation is visible in:

```text
evaluation_road_half_width_1.85
```

where Sampling fails but the Optimization Planner succeeds.

---

## Redesigned Optimization Planner

The current Optimization Planner uses the explicit Frenet state:

```text
x = [l, l', l'']
```

instead of optimizing only lateral position and reconstructing every derivative afterward.

### Piecewise-constant-jerk dynamics

Within each spatial interval:

```text
j_i = (l''_{i+1} - l''_i) / delta_s
```

This gives linear state-transition constraints while allowing the true initial Frenet state to be imposed directly.

### Vehicle-derived curvature handling

The QP uses a local convex curvature proxy internally. Final acceptance uses reconstructed Cartesian curvature against:

```text
kappa_max ≈ 0.202334 1/m
```

A successful OSQP solve alone is therefore not treated as physical acceptance.

### Exact clearance and sequential convex refinement

Exact rotated-footprint obstacle clearance is nonlinear, so the planner uses sequential convexification:

```text
nominal QP
   ↓
Cartesian reconstruction
   ↓
exact sampled-pose clearance
   ↓
numerical clearance gradient
   ↓
linearized clearance constraint
   ↓
trust-region QP
   ↓
exact revalidation
```

Current refinement settings include:

- maximum iterations: `6`
- clearance numerical buffer: `0.001 m`
- trust region in `l`: `0.15 m`
- trust region in `l'`: `0.08`
- numerical clearance-gradient epsilon: `1e-4`

The physical clearance requirement remains `> 0.5 m`; the `1 mm` buffer is only numerical margin for linearization / solver tolerance.

### Computational cost

| Optimization version | Main formulation | Mean planner runtime |
| --- | --- | ---: |
| Original | `l`-only + finite differences + convex corridor | ~7.6 ms |
| Redesigned | explicit `[l, l', l'']` + exact footprint evaluation + sequential QP refinement | ~46.3 ms |

The redesign is substantially more expensive than the original optimizer, but still about **4.1× faster** than the frozen Sampling benchmark on mean runtime.

---

## Why Planning Cost and Control Metrics Can Rank Paths Differently

The canonical planner objective penalizes Frenet-domain quantities such as:

```text
l, l', l'', l'''
```

The controller is directly affected by quantities such as:

```text
Cartesian curvature
steering angle
steering rate
lateral tracking error
heading tracking error
feedback correction
```

On curved reference lines these are nonlinear functions of the reference geometry and Frenet state. Therefore:

> **A lower Frenet planning objective does not necessarily imply lower steering demand or easier closed-loop tracking.**

The objective is still meaningful; it is simply not a complete controller-trackability objective.

Representative examples:

- `evaluation_obstacle_longitudinal_19.00`: Optimization is both planning-efficient and more controller-friendly.
- `evaluation_curved_obstacle`: Sampling has higher safe speed and lower tracking error despite a higher canonical objective.

This planning–control objective-alignment gap is one of the main engineering findings of the project.

---

## Common Physical Acceptance Contract

Both planners are judged using the same final acceptance definitions.

### Cartesian curvature

Returned Cartesian path curvature must remain inside the vehicle-derived curvature limit.

### Obstacle clearance

At each returned sampled pose, the rotated vehicle rectangle is checked against circular obstacle geometry.

```text
clearance > 0.5 m
```

This is exact for the implemented rectangle-circle geometry **at returned poses**. It is not continuous swept-volume collision checking between knots.

### Road boundary

Production road-boundary validation currently uses the shared Frenet width contract:

```text
l - vehicle_half_width >= road_min_l
l + vehicle_half_width <= road_max_l
```

It does not yet enforce exact rotated-corner road containment.

---

## Development Regression

Run the development-only Optimization regression:

```bash
python -m scripts.planning.validation.validate_planner_redesign_development
```

Current result:

- required development/tuning scenarios: **7 / 7 PASS**
- stress scenarios: **19 total**
- production-validation successes: **16**
- explicit infeasible / capability-boundary cases: **3**
- ordinary validation failures: **0**

The development suite is kept separate from the frozen evaluation suite.

---

## Frozen Evaluation Artifacts

Planner-only results:

```text
outputs/final_evaluation/
├── figures/
├── final_planner_evaluation.csv
└── final_planner_evaluation_summary.json
```

Planning & Control results are generated under:

```text
outputs/final_planning_control/
```

Useful figure commands:

```bash
python -m scripts.planning.evaluation.build_planner_summary_figure
python -m scripts.planning.evaluation.build_planner_scenario_figures
python -m scripts.mpc.build_final_planning_control_figures
```

`build_final_planner_summary_figure` reads the frozen planner CSV and does not rerun the benchmark.

`build_final_planner_figures` rebuilds scenario figures using the current frozen suite specifically for presentation/color consistency; it does not overwrite benchmark CSV/JSON or tune parameters.

---

## Representative Planning Cases

### Close obstacle at 19 m

```text
evaluation_obstacle_longitudinal_19.00
```

Sampling:

```text
max curvature     ≈ 0.1522 1/m
minimum clearance ≈ 0.5991 m
canonical objective ≈ 0.4522
```

Optimization:

```text
max curvature     ≈ 0.1263 1/m
minimum clearance ≈ 0.50085 m
canonical objective ≈ 0.1711
```

![19 m close-obstacle comparison](outputs/final_evaluation/figures/evaluation_obstacle_longitudinal_19_00.png)

### Narrow road

```text
evaluation_road_half_width_1.85
```

- Sampling: **FAIL**
- Optimization: **PASS**

The Sampling candidate set is intentionally left unchanged rather than tuned against this frozen evaluation result.

![1.85 m half-width comparison](outputs/final_evaluation/figures/evaluation_road_half_width_1_85.png)

---

## Environment

Primary implementation:

- Python 3.10
- NumPy
- SciPy
- OSQP
- Matplotlib

Install dependencies:

```bash
python -m venv .venv
pip install -r requirements.txt
```

Activate on Windows PowerShell:

```powershell
.venv\Scripts\Activate.ps1
```

---

## Useful Commands

Development Optimization regression:

```bash
python -m scripts.planning.validation.validate_planner_redesign_development
```

Planner-only summary figure from frozen CSV:

```bash
python -m scripts.planning.evaluation.build_planner_summary_figure
```

Rebuild fixed-color planner scenario figures:

```bash
python -m scripts.planning.evaluation.build_planner_scenario_figures
```

Run final Planning & Control benchmark:

```bash
python -m scripts.mpc.run_final_planning_control_benchmark
```

Build final Planning & Control figures:

```bash
python -m scripts.mpc.build_final_planning_control_figures
```

The frozen evaluation suite should be treated as evaluation, not as a tuning loop.

---

## Known Limitations and Improvement Directions

| Observation | Interpretation | Improvement direction |
| --- | --- | --- |
| Sampling fails the `1.85 m` half-width case while Optimization passes | The frozen 45-candidate library does not cover this narrow feasible region | Adaptive / denser / coarse-to-fine Sampling, evaluated on new development cases |
| Optimization often runs close to `0.5 m` clearance | The constrained optimizer uses available feasible space aggressively | Evaluate robustness buffers or clearance soft costs in a new tuning cycle |
| Optimization runtime rose from ~`7.6 ms` to ~`46.3 ms` | Exact geometry and sequential refinement improve consistency at computational cost | Warm starts, analytical gradients, active-constraint screening, adaptive refinement |
| Obstacle clearance uses rotated geometry but road acceptance uses Frenet width | Physical geometry is not equally exact for obstacle and road constraints | Promote rotated-footprint road containment after a new regression cycle |
| Collision checking is sampled-pose only | Safe knots do not mathematically guarantee safe swept volume | Inter-knot / swept-volume checking or adaptive spatial sampling |
| Current planner objective is not fully controller-aligned | Low Frenet cost may still produce higher steering-rate demand or tracking effort | Add steering-rate / curvature-rate / controller-oriented terms in a future planner study |
| Vehicle/control model is kinematic | Actuator delay, tire dynamics, slope, and model mismatch are omitted | Higher-fidelity vehicle/control validation |

Longer-term extensions include dynamic obstacles, prediction, C++ migration, ROS 2 integration, deterministic fallback logic, and an optional VLM-based high-level behavior layer.
