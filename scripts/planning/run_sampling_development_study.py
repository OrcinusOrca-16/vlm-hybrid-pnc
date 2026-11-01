"""Reproducible DEVELOPMENT-only cached sampling study. Never loads final cases.

Run --phase broad, then --phase refine, then --phase diagnose. Cached rows
include independent feasibility flags; timing of cached searches is not planner
runtime. Production runtime must be measured separately without this cache.
"""
import argparse
from collections import Counter
from dataclasses import asdict, replace
from functools import lru_cache
from itertools import product
import json
import math
from pathlib import Path
from statistics import mean
from time import perf_counter

from scripts.planning.run_sampling_trackability_tuning import (
    LATERAL_PROFILES, LENGTH_PROFILES, JUNCTION_PROFILES, geometric_cost,
    minimum_clearance, speed_cost,
)
from src.benchmark.tuning_scenario_suite import build_development_tuning_scenarios
from src.common.types import FrenetTrajectory
from src.config.vehicle import DEFAULT_VEHICLE_CONFIG as VEHICLE
from src.control.path_tracking import compute_constant_safe_speed
from src.planning.frenet import vehicle_state_to_frenet, frenet_trajectory_to_path
from src.planning.hard_constraints import is_curvature_feasible, is_road_boundary_feasible
from src.planning.quintic_polynomial import QuinticPolynomial
from src.planning.sampling_planner import sample_quintic_candidate
from src.planning.soft_cost import matched_total_cost

OUT = Path('outputs/sampling_development_study')


def build(s, lateral, length, slope, second=0., resolution=None):
    initial = s._study_initial
    resolution = s.resolution_m if resolution is None else resolution
    first = QuinticPolynomial(initial.l, initial.dl_ds, initial.d2l_ds2,
                              lateral, slope, second, length)
    a = sample_quintic_candidate(first, initial.s, resolution)
    final = QuinticPolynomial(lateral, slope, second, s.terminal_lateral_target_m,
                             s.terminal_first_derivative_target, 0., s.planning_horizon_m-length)
    b = sample_quintic_candidate(final, initial.s+length, resolution)
    trajectory = FrenetTrajectory(a.points+b.points[1:])
    return trajectory, frenet_trajectory_to_path(trajectory, s.reference_line)


def scenarios():
    result = build_development_tuning_scenarios()
    for s in result:
        # Diagnostic-only memoization: same exact queries, unchanged geometry.
        object.__setattr__(s, '_study_initial', vehicle_state_to_frenet(s.vehicle_state, s.reference_line))
        s.reference_line.query = lru_cache(None)(s.reference_line.query)
    return result


def measure(s, params, resolution=None):
    tr, path = build(s, *params, resolution=resolution)
    clear = minimum_clearance(path, s)
    k = max(abs(p.curvature) for p in path)
    flags = [k <= s.max_curvature, clear > s.minimum_clearance_m,
             is_road_boundary_feasible(tr, s.vehicle_params, s.road_min_l, s.road_max_l)]
    row = dict(params=list(params), flags=[bool(f) for f in flags], clearance=clear, max_k=k,
               curvature_margin=s.max_curvature-k,
               steering_margin=VEHICLE.max_steering_angle_rad-math.atan(s.vehicle_params.wheelbase_m*k))
    if all(flags):
        safe = compute_constant_safe_speed(path, s.vehicle_state.speed,
                    s.vehicle_params.wheelbase_m, VEHICLE.max_steering_angle_rad,
                    VEHICLE.max_steering_rate_radps)
        row.update(geo=geometric_cost(tr, s), speed=safe, deficit=speed_cost(safe, s.vehicle_state.speed),
                   rms_k=math.sqrt(mean(p.curvature**2 for p in path)),
                   rms_krate=math.sqrt(mean(((b.curvature-a.curvature)/(b.s-a.s))**2 for a,b in zip(path,path[1:]))),
                   terminal_l=tr.points[-1].l, terminal_dl=tr.points[-1].dl_ds,
                   terminal_d2=tr.points[-1].d2l_ds2)
    return row


def pools_for(ss, laterals, lengths, slopes, seconds=(0.,)):
    pools = {}
    for s in ss:
        file = OUT / (s.name+'.json')
        rows = json.loads(file.read_text()) if file.exists() else []
        cache = {tuple(r['params']): r for r in rows}
        wanted = list(product(laterals,lengths,slopes,seconds))
        start = perf_counter()
        for params in wanted:
            if params not in cache:
                cache[params] = measure(s, params)
        file.write_text(json.dumps(list(cache.values())))
        pools[s.name] = [cache[p] for p in wanted]
        print(s.name, 'rows',len(wanted),'feasible',sum(all(r['flags']) for r in pools[s.name]),
              'new_seconds',round(perf_counter()-start,2), flush=True)
    return pools


def summarize(ss, pools, name, lat, lengths, slopes, weight, seconds=(0.,), robust_weight=0.):
    selected, pruning = {}, Counter()
    for s in ss:
        rows = [r for r in pools[s.name] if r['params'][0] in lat and r['params'][1] in lengths
                and r['params'][2] in slopes and r['params'][3] in seconds]
        for r in rows:
            pruning['generated'] += 1
            reject = next((label for label,flag in zip(('curvature','clearance','road'),r['flags']) if not flag),'feasible')
            pruning[reject] += 1
        good = [r for r in rows if all(r['flags'])]
        if good:
            # Continuous soft preference only; does not alter the hard threshold.
            def cost(r):
                margin_cost = 0. if not math.isfinite(r['clearance']) else (s.minimum_clearance_m/r['clearance'])**2
                return r['geo']+weight*r['deficit']+robust_weight*margin_cost
            selected[s.name] = min(good, key=cost)
    values = list(selected.values())
    early = selected.get('tuning_obstacle_longitudinal_18.00',{})
    return dict(name=name, lat=lat, lengths=lengths, slopes=slopes, seconds=seconds,
        weight=weight, robust_weight=robust_weight, count=len(lat)*len(lengths)*len(slopes)*len(seconds),
        success=len(values), failures=[s.name for s in ss if s.name not in selected],
        mean_speed=mean(r['speed'] for r in values), min_speed=min(r['speed'] for r in values),
        early_speed=early.get('speed'), geo=mean(r['geo'] for r in values),
        deficit=mean(r['deficit'] for r in values),
        clearance=min(r['clearance'] for r in values),
        curvature_margin=min(r['curvature_margin'] for r in values),
        steering_margin=min(r['steering_margin'] for r in values),
        pruning=dict(pruning), selected=selected)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--phase', choices=['broad','refine','diagnose','junction'], required=True)
    args = parser.parse_args()
    OUT.mkdir(parents=True, exist_ok=True)
    ss = scenarios()
    lateral = dict(LATERAL_PROFILES)
    lengths = dict(LENGTH_PROFILES)
    slopes = dict(JUNCTION_PROFILES)
    weights = (0., .5, 1., 2., 3., 4., 5., 7.5, 10., 15., 25.)
    if args.phase != 'broad':
        lateral.update(nonuniform=tuple(sorted(set(lateral['r2.00_d0.40']+(-1.,1.)))),
                       focused=(-2.,-1.6,-1.2,-1.,-.8,-.6,0.,.6,.8,1.,1.2,1.6,2.))
        lengths.update(even=(6.,8.,10.,12.,14.,16.,18.,20.,22.),
                       long=(6.,10.,14.,18.,22.,24.,26.))
        slopes.update(refined=tuple(i*.025 for i in range(-8,9)), wide=(-.3,-.2,-.1,0.,.1,.2,.3))
        # Round every value to keep exact tuple membership deterministic.
        slopes = {k:tuple(round(v,6) for v in vals) for k,vals in slopes.items()}
    if args.phase in ('diagnose','junction'):
        # s16 is already infeasible at the fixed initial pose; densifying it
        # cannot fix that condition. Keep its failure in aggregate coverage.
        ss = tuple(s for s in ss if s.name in ('road_half_width_1.75','road_half_width_1.50','obstacle_longitudinal_36.00'))
        p={}
        for s in ss:
            if args.phase=='diagnose':
                lat=tuple(sorted(set(tuple(i/10 for i in range(-20,21))+(-.85,-.84,.84,.85))))
                lens=tuple(float(i) for i in range(4,28))
                sl=tuple(i/20 for i in range(-6,7))
                sec=(0.,)
            else:
                lat=(-.85,-.84,-.8,-.75,-.7,-.6,0.) if 'road_' in s.name else (-2.,-1.8,-1.6,-1.5,-1.4,-1.2,-1.,-.8)
                lens=tuple(float(i) for i in range(6,27,2)) if 'road_' in s.name else (18.,20.,22.,23.,24.,25.,26.)
                sl=(-.3,-.2,-.1,0.,.1,.2,.3)
                sec=(-.1,-.05,-.025,0.,.025,.05,.1)
            p.update(pools_for((s,),lat,lens,sl,sec))
        diagnostics = {}
        for s in ss:
            rows = p[s.name]
            road_curve = [r for r in rows if r['flags'][0] and r['flags'][2]]
            best = max(road_curve,key=lambda r:r['clearance']) if road_curve else None
            diagnostics[s.name] = dict(total=len(rows),feasible=sum(all(r['flags']) for r in rows),
                best_road_curvature_clearance=best,
                flags={str(flags):sum(tuple(r['flags'])==flags for r in rows)
                       for flags in product((False,True),repeat=3)})
            print(s.name,diagnostics[s.name],flush=True)
        (OUT/(args.phase+'.json')).write_text(json.dumps(diagnostics,indent=2))
        return
    all_lat=tuple(sorted({v for vals in lateral.values() for v in vals}))
    all_len=tuple(sorted({v for vals in lengths.values() for v in vals}))
    all_slope=tuple(sorted({v for vals in slopes.values() for v in vals}))
    pools=pools_for(ss,all_lat,all_len,all_slope)
    results=[]
    for (ln,lat),(bn,lens),(jn,sl) in product(lateral.items(),lengths.items(),slopes.items()):
        for w in weights:
            results.append(summarize(ss,pools,f'{ln}/{bn}/{jn}/w{w}',lat,lens,sl,w))
    results.sort(key=lambda r:(-r['success'],r['deficit'],r['geo'],r['count']))
    (OUT/(args.phase+'.json')).write_text(json.dumps(results,indent=2))
    for r in results[:15]:
        print({k:v for k,v in r.items() if k not in ('selected','pruning')},flush=True)


if __name__ == '__main__':
    main()

