"""Sampling/control integration invariants, using development inputs only."""

import math
from dataclasses import replace

from scripts.planning.validation.validate_sampling_trackability_development import arguments
from src.benchmark.sampling_adapter import run_sampling_benchmark_adapter
from src.benchmark.tuning_scenario_suite import build_development_tuning_scenarios
from src.config.vehicle import DEFAULT_VEHICLE_CONFIG as VEHICLE
from src.control.path_tracking import compute_constant_safe_speed
from src.planning.frenet import vehicle_state_to_frenet
from src.planning.sampling_planner import generate_returning_frenet_candidate, generate_returning_frenet_candidates
from src.planning.soft_cost import normalized_speed_deficit_cost


def safe(path, s):
    return compute_constant_safe_speed(
        path,
        s.vehicle_state.speed,
        s.vehicle_params.wheelbase_m,
        VEHICLE.max_steering_angle_rad,
        VEHICLE.max_steering_rate_radps,
    )


def main():
    s = next(s for s in build_development_tuning_scenarios() if s.name == "tuning_obstacle_longitudinal_18.00")
    state = vehicle_state_to_frenet(s.vehicle_state, s.reference_line)
    trajectory = generate_returning_frenet_candidate(
        state.s, state.l, state.dl_ds, state.d2l_ds2, -1.0, 6.0, 22.0, junction_dl_ds=-0.05
    )
    junction = next(p for p in trajectory.points if math.isclose(p.s, state.s + 6.0))
    assert math.isclose(junction.dl_ds, -0.05, abs_tol=1e-10)
    assert abs(junction.d2l_ds2) < 1e-10
    assert len({p.s for p in trajectory.points}) == len(trajectory.points)
    assert abs(trajectory.points[-1].l) < 1e-10
    assert abs(trajectory.points[-1].dl_ds) < 1e-10
    assert abs(trajectory.points[-1].d2l_ds2) < 1e-10

    # Production defaults remain the lightweight 45-candidate geometric planner.
    assert len(generate_returning_frenet_candidates(state.s, state.l, state.dl_ds, state.d2l_ds2)) == 45

    # The larger trackability-aware design remains available as an explicit experiment.
    experimental = generate_returning_frenet_candidates(
        state.s,
        state.l,
        state.dl_ds,
        state.d2l_ds2,
        bypass_lengths=(6.0, 10.0, 14.0, 18.0),
        junction_dl_ds_values=(-0.05, 0.0, 0.05),
    )
    assert len(experimental) == 108

    kwargs = arguments(s)
    baseline = run_sampling_benchmark_adapter(
        **kwargs, trackability_weight=0.0, junction_dl_ds_values=(0.0,), bypass_lengths=(6.0, 10.0, 14.0, 18.0, 22.0)
    )
    tuned = run_sampling_benchmark_adapter(
        **kwargs,
        trackability_weight=4.0,
        junction_dl_ds_values=(-0.05, 0.0, 0.05),
        bypass_lengths=(6.0, 10.0, 14.0, 18.0),
    )
    assert baseline.success and tuned.success
    assert math.isclose(safe(baseline.path, s), 0.8972302340401858, abs_tol=1e-8)
    assert safe(tuned.path, s) > safe(baseline.path, s)

    # The canonical metric must exclude the speed preference.
    target = next(p for p in tuned.trajectory.points if math.isclose(p.s, state.s + 6.0))
    assert math.isclose(target.dl_ds, -0.05, abs_tol=1e-9)
    assert tuned.matched_objective_value < 1.0
    assert normalized_speed_deficit_cost(0.0, 0.0) == 0.0
    zero = run_sampling_benchmark_adapter(**arguments(replace(s, vehicle_state=replace(s.vehicle_state, speed=0.0))))
    assert zero.success
    for slopes in ((), (float("nan"),)):
        try:
            generate_returning_frenet_candidates(
                state.s, state.l, state.dl_ds, state.d2l_ds2, junction_dl_ds_values=slopes
            )
        except ValueError:
            pass
        else:
            raise AssertionError("Invalid junction sampling accepted")
    print("Production baseline plus explicit trackability experiment invariants passed.")


if __name__ == "__main__":
    main()
