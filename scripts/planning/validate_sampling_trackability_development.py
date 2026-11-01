"""Development-only production integration regression and uncached timings."""
import argparse
from dataclasses import asdict
import json
from pathlib import Path
from statistics import mean

from src.benchmark.sampling_adapter import run_sampling_benchmark_adapter
from src.benchmark.tuning_scenario_suite import build_development_tuning_scenarios
from src.config.vehicle import DEFAULT_VEHICLE_CONFIG as VEHICLE
from src.control.path_tracking import compute_constant_safe_speed
from src.planning.hard_constraints import is_physical_path_feasible, obstacle_clearance


def arguments(s):
    names = ('reference_line','vehicle_state','vehicle_params','road_min_l','road_max_l',
             'max_curvature','minimum_clearance_m','lateral_reference_m','terminal_lateral_target_m',
             'terminal_first_derivative_target','lateral_weight','first_derivative_weight',
             'second_derivative_weight','third_derivative_weight','terminal_lateral_weight',
             'terminal_first_derivative_weight')
    return dict({k:getattr(s,k) for k in names}, obstacles=list(s.obstacles),
                total_length=s.planning_horizon_m, resolution_m=s.resolution_m,
                final_lateral_positions=(s.terminal_lateral_target_m,),
                final_dl_ds_values=(s.terminal_first_derivative_target,))


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument('--label',required=True)
    parser.add_argument('--expected', type=int)
    parser.add_argument('--verify-recommendation', action='store_true')
    args=parser.parse_args()
    recommendation=json.loads(Path('docs/sampling_trackability_development_config.json').read_text()) if args.verify_recommendation else None
    rows=[]
    for s in build_development_tuning_scenarios():
        result=run_sampling_benchmark_adapter(**arguments(s))
        row=dict(name=s.name, success=result.success, ms=result.planning_time_ms,
                 failure=result.failure_reason, objective=result.matched_objective_value)
        if result.success:
            tr,path=result.trajectory,result.path
            assert is_physical_path_feasible(tr,path,s.vehicle_params,list(s.obstacles),s.max_curvature,
                                            s.minimum_clearance_m,s.road_min_l,s.road_max_l)
            assert abs(tr.points[-1].l-s.terminal_lateral_target_m)<1e-8
            assert abs(tr.points[-1].dl_ds-s.terminal_first_derivative_target)<1e-8
            assert abs(tr.points[-1].d2l_ds2)<1e-8
            row['speed']=compute_constant_safe_speed(path,s.vehicle_state.speed,s.vehicle_params.wheelbase_m,
                                  VEHICLE.max_steering_angle_rad,VEHICLE.max_steering_rate_radps)
            row['clearance']=min((obstacle_clearance(p,s.vehicle_params,o) for p in path for o in s.obstacles),default=float('inf'))
            if recommendation is not None:
                expected=recommendation['selected'][s.name]
                assert abs(row['speed']-expected['speed'])<1e-8
                if expected['clearance'] is None:
                    assert row['clearance']==float('inf')
                else:
                    assert abs(row['clearance']-expected['clearance'])<1e-8
                from src.planning.frenet import vehicle_state_to_frenet
                from src.planning.sampling_planner import generate_returning_frenet_candidate
                initial=vehicle_state_to_frenet(s.vehicle_state,s.reference_line)
                lateral,length,slope,second=expected['params']
                assert second==0.
                reproduced=generate_returning_frenet_candidate(initial.s,initial.l,initial.dl_ds,initial.d2l_ds2,
                    lateral,length,s.planning_horizon_m-length,resolution=s.resolution_m,junction_dl_ds=slope)
                assert len(reproduced.points)==len(tr.points)
                for a,b in zip(reproduced.points,tr.points):
                    assert max(abs(a.s-b.s),abs(a.l-b.l),abs(a.dl_ds-b.dl_ds),abs(a.d2l_ds2-b.d2l_ds2))<1e-8
        if recommendation is not None:
            assert result.success==(s.name in recommendation['selected'])
        rows.append(row)
        print(row,flush=True)
    successes=[r for r in rows if r['success']]
    summary=dict(label=args.label, success=len(successes), total=len(rows),
                 failures=[r['name'] for r in rows if not r['success']],
                 mean_ms=mean(r['ms'] for r in rows), mean_speed=mean(r['speed'] for r in successes),
                 min_speed=min(r['speed'] for r in successes),rows=rows)
    out=Path('outputs/sampling_development_study')
    out.mkdir(parents=True,exist_ok=True)
    (out/(args.label+'_regression.json')).write_text(json.dumps(summary,indent=2))
    print({k:v for k,v in summary.items() if k!='rows'},flush=True)
    if args.expected is not None:
        assert len(successes)==args.expected


if __name__=='__main__':
    main()
