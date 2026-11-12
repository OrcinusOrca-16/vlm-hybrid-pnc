# Sampling Trackability — DEVELOPMENT Study (2026-11-01)

> **Historical status:** This is a development-only parameter study, not the frozen final evaluation. The **108-candidate proposal was not adopted** in the final published 45-candidate Sampling baseline. The original working-tree and review status below describes 2026-11-01, not the present repository.

## Scope and baseline

- Original base commit: `159d72baccf124e85f5642cdff0ca52fd1201d25`, initially clean working tree.
- Separate **26-scenario DEVELOPMENT** suite; no invalid-start cases removed; no frozen final evaluation data were used for tuning.
- The initial **45-candidate** baseline solved **22/26** scenes. Mean safe speed **4.312302 m/s**, worst/early-obstacle speed **0.897230 m/s**, mean canonical geometric cost **0.5275265**, minimum clearance **0.5363775 m**.
- All candidate selection comparisons with 22/26 coverage use the same 22 successful cases. A 23/26 success-set mean is not directly comparable.
- Production checks sampled at **0.5 m**; the **0.05 m** inspection was DEVELOPMENT-only and is not proof of continuous collision avoidance. Shared physical thresholds, vehicle parameters, MPC settings and Optimization production code were not changed in this study.

## Experiment design and candidate search

The study reproduced earlier searches, then cached candidate geometry and independent constraint-rejection reasons. It varied 0.5/0.4/0.25 m lateral grids, bypass-length sets, zero and ±0.05/±0.10/±0.15 junction slopes and selection weights 0–25. Follow-up cases retained important ±1.0/±1.5 lateral samples in nonuniform grids, extended lengths up to 26 m, refined slopes at 0.025 and diagnosed ±0.3. Targeted difficult-scene scans tested **14,040** zero-second-derivative candidates (bypass lengths 4–27 m); separate junction-second-derivative tests examined **2,744** late-obstacle and **3,773** candidates for each narrow-road scene. Weight refinement tested 3/3.5/4/4.5/5/6/7.5/10, and a clearance preference `r*(0.5/clearance)^2` at r=0/0.25/0.5/1. The terminal state stayed (0,0,0).

| DEVELOPMENT variant | Candidates | Coarse success | Mean safe speed m/s | Early speed m/s | Mean geometric cost | Min clearance m |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `r2.00_d0.50/dense/strict/w0.0` | 45 | 22/26 | 4.3123 | 0.8972 | 0.527527 | 0.536377 |
| `d05/tiny/s005/w4.0/r0.0` | 81 | 22/26 | 4.6139 | 1.0068 | 0.607591 | 0.536378 |
| `d05/pruned/s005/w4.0/r0.0` | 108 | 22/26 | 4.5631 | 1.0068 | 0.553479 | 0.536377 |
| `d05/base/s005/w4.0/r0.0` | 135 | 22/26 | 4.5631 | 1.0068 | 0.553479 | 0.536377 |
| `nonuniform12/tiny/s0075/w4.0/r0.0` | 117 | 22/26 | 4.6584 | 1.0732 | 0.471691 | 0.508526 |
| `r2.00_d0.40/dense/slope_015/w7.5` | 385 | 23/26 | 4.4679 | 1.0383 | 0.554311 | 0.500642 |

## Fine-grid rejection and aliasing evidence

- Late obstacle `[-1.6,22,0.15,0]`: coarse clearance **0.500642 m** fell to **0.485458 m** with 0.05 m sampling; curvature margin just **0.002266 1/m**. All 108 coarse-feasible diagnostic candidates for that scene failed the fine check, including two with nonzero junction second derivatives.
- A 27 m bypass with just a 1 m return segment produced a misleading coarse-grid 5 m/s safe speed.
- Early obstacles `[-1.5,8,0,0]` and `[-1,6,-0.1,0]` saw clearance drop **0.514243 → 0.475886 m** and **0.527376 → 0.498691 m**. Such apparent improvements were rejected.
- A nonuniform ±0.075 slope variant passed but left only about **0.5085 m** of clearance, a weaker margin. Increasing search density did not automatically improve reliable feasibility.

## Proposed 108-candidate configuration (not frozen baseline)

- Lateral positions (m): `[-2,-1.5,-1,-0.5,0,0.5,1,1.5,2]`.
- Bypass lengths (m): `[6,10,14,18]`.
- Junction slopes: `[-0.05,0,0.05]`; junction second derivative: `0`; final state: `(0,0,0)`.
- Candidate selection: `J_select = J_geo + 4 * J_speed`, preserving geometric weights and hard constraints.
- Same **22/26** development coverage/failure set. Coarse mean safe speed **+5.8%**, early-obstacle speed **+12.2%**, geometric cost **+4.9%** (worse), unchanged minimum clearance, improved curvature/steering margin.
- Selection weight 3.5–5 was on the same plateau; weight 4 was selected. Removing unselected 22 m bypasses reduced 135 to 108 with no selection change. Keeping 10 m lengths prevented the geometry-cost penalty of the 81-candidate option. A separate clearance soft term gave insufficient benefit.

The separate **0.05 m diagnostic** yielded:

| Metric | Baseline 45 | Proposed 108 |
| --- | ---: | ---: |
| Successful paths | 22 | 22 |
| Mean safe speed, m/s | 4.216808 | 4.473303 |
| Minimum safe speed, m/s | 0.683968 | 0.776710 |
| Minimum clearance, m | 0.535674 | 0.535674 |
| Minimum curvature margin, 1/m | 0.046187 | 0.060919 |
| Minimum steering-angle margin, rad | 0.101016 | 0.135263 |

The early-obstacle fine-grid minimum safe speed remains low even with the proposal. The downstream interface still computes steering-gradient speed limits from the selected discrete path; **5 m/s steering-rate feasibility is not a planner hard constraint**.

## Additional quality and failure analysis

- Mean RMS curvature **0.027275 → 0.025219 1/m**; mean RMS curvature spatial derivative **0.016917 → 0.013577 1/m²**.
- Maximum terminal errors in `l`, `l'`, `l''`: **1.78e−15**, **8.88e−16**, **2.22e−16** (roundoff scale).
- Across 26 cases, baseline 1,170 candidates: curvature rejected **207**, clearance rejected **762**, road rejected **37**, feasible **164 (14.02%)**. Proposed 2,808: **310**, **1,947**, **116**, feasible **435 (15.49%)**. Rejections are exclusive and ordered curvature → clearance → road.
- `obstacle_longitudinal_16.00`: initial exact clearance **−0.087689 m**, already violating the 0.5 m margin.
- `road_half_width_1.75`: the tested path family could achieve at most **0.45 m** clearance while meeting road and curvature requirements; clearance-feasible alternatives crossed the road acceptance boundary.
- `road_half_width_1.50`: analogous, maximum diagnosed clearance around **0.20 m**.
- `obstacle_longitudinal_36.00`: fixed endpoint and short return segments in the two-part quintic family coupled curvature and clearance; coarse solutions failed fine validation.

These are limits **of the tested candidate family**, not proofs that every continuous vehicle trajectory is infeasible.

## Runtime and verification

Full production-adapter benchmark (5 warmups, 50 measurements, no diagnostic caching):

| Metric | Baseline 45 | Proposed 108 |
| --- | ---: | ---: |
| Mean ms | 188.3557 | 441.9098 |
| Median ms | 188.3082 | 440.7716 |
| P95 ms | 195.0896 | 453.5068 |

Candidate count **2.4×**; runtime about **2.35×**. Twenty-six-case regression single-run average **188.997 → 448.330 ms**. This is not evidence of high-frequency real-time suitability.

Historical validations passed: Sampling generator/adapter, matched costs, objective separation, physical validator, canonical/discrete/stress/runtime checks, new junction and optional soft-cost integration. Two historical test fixtures explicitly fixed zero junction slope; a changed candidate selection caused a matched-cost fixture failure that was corrected, and a NumPy-boolean JSON serialization bug was repaired before final results.

Development-only curvature-aware MPC tests passed:

- Early obstacle at `1.006821 m/s`: max lateral error **0.0175 m**, final ≈0, max steering **0.3963 rad**, max executed steering rate **0.5055 rad/s**.
- Curved obstacle at `5 m/s`: max lateral error **0.0285 m**, final **−0.0137 m**, max steering **0.1619 rad**, max rate **0.5001 rad/s**.

A 0.0055 rad/s peak rate excess remained within the existing **0.02 rad/s** numerical validation tolerance; the configured 0.5 rad/s threshold was not changed. These simulations started at the chosen safe reference speed and did not simulate braking from 5 m/s or demonstrate full continuous collision safety.

## Implementation notes and reproducibility

The historical study discussed changes to `src/config/planning.py`, `src/planning/sampling_planner.py`, `src/planning/soft_cost.py`, and `src/benchmark/sampling_adapter.py`. The present default `src/config/planning.py` still uses a 45-candidate Sampling baseline, zero junction slope and zero trackability selection weight; **do not mistake the experiment snapshot for active defaults**.

- [Historical PowerShell command record](sampling_trackability_commands.md), not a one-click current script.
- [Historical candidate configuration JSON](sampling_trackability_config.json).
- Experiment caches `outputs/sampling_development_study/` and logs `outputs/sampling_*.log` were gitignored and not committed; published repository alone cannot reproduce every cached diagnostic artifact verbatim.
- Published results: [Planning benchmark](../reports/planning_benchmark.md) and [Planning & Control benchmark](../reports/planning_control_benchmark.md).

The original report ended by noting uncommitted DEVELOPMENT edits and review pending as of 2026-11-01; these are historical statements, not current working-tree assertions.
