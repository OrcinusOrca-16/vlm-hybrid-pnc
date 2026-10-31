"""Development-only tuning for Sampling junction slope and trackability cost.

This script uses only build_development_tuning_scenarios(). It never imports
or runs the frozen final-evaluation suite.
"""

from dataclasses import dataclass
import math
from statistics import mean

from src.benchmark.scenario import BenchmarkScenario
from src.benchmark.tuning_scenario_suite import build_development_tuning_scenarios
from src.common.types import FrenetTrajectory
from src.config.planning import DEFAULT_SAMPLING_PLANNER_CONFIG
from src.control.mpc_config import MPC_MAX_STEERING_ANGLE, MPC_MAX_STEERING_RATE, MPC_WHEELBASE_M
from src.control.path_tracking import compute_constant_safe_speed
from src.planning.frenet import frenet_trajectory_to_path, vehicle_state_to_frenet
from src.planning.hard_constraints import (
    is_curvature_feasible,
    is_obstacle_clearance_feasible,
    is_road_boundary_feasible,
    obstacle_clearance,
)
from src.planning.quintic_polynomial import QuinticPolynomial
from src.planning.sampling_planner import sample_quintic_candidate
from src.planning.soft_cost import matched_total_cost


JUNCTION_PROFILES = {
    "strict": (0.0,),
    "slope_005": (-0.05, 0.0, 0.05),
    "slope_010": (-0.10, -0.05, 0.0, 0.05, 0.10),
    "slope_015": (-0.15, -0.10, -0.05, 0.0, 0.05, 0.10, 0.15),
}
TRACKABILITY_WEIGHTS = (0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 7.5, 10.0)
ALL_JUNCTION_SLOPES = tuple(sorted({value for values in JUNCTION_PROFILES.values() for value in values}))


@dataclass(frozen=True)
class CandidateRecord:
    junction_dl_ds: float
    bypass_l: float
    bypass_length: float
    geometric_cost: float
    safe_speed_mps: float
    speed_cost: float
    minimum_clearance_m: float


@dataclass(frozen=True)
class ScenarioSelection:
    scenario_name: str
    success: bool
    selected: CandidateRecord | None


@dataclass(frozen=True)
class ConfigSummary:
    profile_name: str
    trackability_weight: float
    success_count: int
    total_count: int
    mean_safe_speed_mps: float | None
    minimum_safe_speed_mps: float | None
    mean_speed_cost: float | None
    mean_geometric_cost: float | None
    minimum_clearance_m: float | None
    early_obstacle_speed_mps: float | None
    narrow_190_success: bool
    narrow_175_success: bool
    selections: tuple[ScenarioSelection, ...]


def geometric_cost(trajectory: FrenetTrajectory, scenario: BenchmarkScenario) -> float:
    weights = DEFAULT_SAMPLING_PLANNER_CONFIG.matched_objective_weights
    return matched_total_cost(
        trajectory=trajectory,
        resolution_m=scenario.resolution_m,
        lateral_reference_m=scenario.lateral_reference_m,
        terminal_lateral_target_m=scenario.terminal_lateral_target_m,
        terminal_first_derivative_target=scenario.terminal_first_derivative_target,
        lateral_weight=weights.lateral_weight,
        first_derivative_weight=weights.first_derivative_weight,
        second_derivative_weight=weights.second_derivative_weight,
        third_derivative_weight=weights.third_derivative_weight,
        terminal_lateral_weight=weights.terminal_lateral_weight,
        terminal_first_derivative_weight=weights.terminal_first_derivative_weight,
    )


def speed_cost(safe_speed_mps: float, target_speed_mps: float) -> float:
    if target_speed_mps <= 0.0:
        return 0.0
    deficit = max(0.0, target_speed_mps - safe_speed_mps) / target_speed_mps
    return deficit**2


def minimum_clearance(path, scenario: BenchmarkScenario) -> float:
    if not scenario.obstacles:
        return math.inf
    return min(
        obstacle_clearance(path_point=point, vehicle_params=scenario.vehicle_params, obstacle=obstacle)
        for point in path
        for obstacle in scenario.obstacles
    )


def build_candidate(
    scenario: BenchmarkScenario,
    bypass_l: float,
    bypass_length: float,
    junction_dl_ds: float,
) -> tuple[FrenetTrajectory, list] | None:
    return_length = scenario.planning_horizon_m - bypass_length
    if return_length <= 0.0:
        return None

    initial = vehicle_state_to_frenet(vehicle_state=scenario.vehicle_state, reference_line=scenario.reference_line)
    config = DEFAULT_SAMPLING_PLANNER_CONFIG

    bypass_curve = QuinticPolynomial(
        start_l=initial.l,
        start_dl_ds=initial.dl_ds,
        start_d2l_ds2=initial.d2l_ds2,
        end_l=bypass_l,
        end_dl_ds=junction_dl_ds,
        end_d2l_ds2=0.0,
        length=bypass_length,
    )
    bypass_trajectory = sample_quintic_candidate(curve=bypass_curve, start_s=initial.s, resolution=scenario.resolution_m)
    junction = bypass_trajectory.points[-1]

    return_curve = QuinticPolynomial(
        start_l=junction.l,
        start_dl_ds=junction.dl_ds,
        start_d2l_ds2=junction.d2l_ds2,
        end_l=scenario.terminal_lateral_target_m,
        end_dl_ds=scenario.terminal_first_derivative_target,
        end_d2l_ds2=config.returning_final_d2l_ds2,
        length=return_length,
    )
    return_trajectory = sample_quintic_candidate(curve=return_curve, start_s=junction.s, resolution=scenario.resolution_m)
    trajectory = FrenetTrajectory(points=bypass_trajectory.points + return_trajectory.points[1:])
    path = frenet_trajectory_to_path(trajectory=trajectory, reference_line=scenario.reference_line)
    return trajectory, path


def build_candidate_pool(scenario: BenchmarkScenario) -> tuple[CandidateRecord, ...]:
    config = DEFAULT_SAMPLING_PLANNER_CONFIG
    records = []

    for bypass_l in config.returning_bypass_lateral_positions_m:
        for bypass_length in config.returning_bypass_lengths_m:
            for junction_dl_ds in ALL_JUNCTION_SLOPES:
                candidate = build_candidate(
                    scenario=scenario,
                    bypass_l=bypass_l,
                    bypass_length=bypass_length,
                    junction_dl_ds=junction_dl_ds,
                )
                if candidate is None:
                    continue

                trajectory, path = candidate
                curvature_ok = is_curvature_feasible(path=path, max_curvature=scenario.max_curvature)
                clearance_ok = is_obstacle_clearance_feasible(
                    path=path,
                    vehicle_params=scenario.vehicle_params,
                    obstacles=list(scenario.obstacles),
                    minimum_clearance_m=scenario.minimum_clearance_m,
                )
                road_ok = is_road_boundary_feasible(
                    trajectory=trajectory,
                    vehicle_params=scenario.vehicle_params,
                    min_l=scenario.road_min_l,
                    max_l=scenario.road_max_l,
                )
                if not (curvature_ok and clearance_ok and road_ok):
                    continue

                try:
                    safe_speed = compute_constant_safe_speed(
                        path=path,
                        target_speed_mps=scenario.vehicle_state.speed,
                        wheelbase_m=MPC_WHEELBASE_M,
                        max_steering_angle=MPC_MAX_STEERING_ANGLE,
                        max_steering_rate=MPC_MAX_STEERING_RATE,
                    )
                except ValueError:
                    continue

                records.append(CandidateRecord(
                    junction_dl_ds=junction_dl_ds,
                    bypass_l=bypass_l,
                    bypass_length=bypass_length,
                    geometric_cost=geometric_cost(trajectory, scenario),
                    safe_speed_mps=safe_speed,
                    speed_cost=speed_cost(safe_speed, scenario.vehicle_state.speed),
                    minimum_clearance_m=minimum_clearance(path, scenario),
                ))

    return tuple(records)


def select_for_config(
    scenario: BenchmarkScenario,
    pool: tuple[CandidateRecord, ...],
    allowed_slopes: tuple[float, ...],
    trackability_weight: float,
) -> ScenarioSelection:
    allowed = set(allowed_slopes)
    candidates = [record for record in pool if record.junction_dl_ds in allowed]
    if not candidates:
        return ScenarioSelection(scenario_name=scenario.name, success=False, selected=None)

    winner = min(candidates, key=lambda record: record.geometric_cost + trackability_weight * record.speed_cost)
    return ScenarioSelection(scenario_name=scenario.name, success=True, selected=winner)


def optional_mean(values: list[float]) -> float | None:
    return mean(values) if values else None


def optional_min(values: list[float]) -> float | None:
    return min(values) if values else None


def summarize_config(
    scenarios: tuple[BenchmarkScenario, ...],
    pools: dict[str, tuple[CandidateRecord, ...]],
    profile_name: str,
    allowed_slopes: tuple[float, ...],
    trackability_weight: float,
) -> ConfigSummary:
    selections = tuple(
        select_for_config(
            scenario=scenario,
            pool=pools[scenario.name],
            allowed_slopes=allowed_slopes,
            trackability_weight=trackability_weight,
        )
        for scenario in scenarios
    )
    selected = [item.selected for item in selections if item.success and item.selected is not None]
    success_names = {item.scenario_name for item in selections if item.success}

    speed_values = [item.safe_speed_mps for item in selected]
    speed_costs = [item.speed_cost for item in selected]
    geometric_costs = [item.geometric_cost for item in selected]
    clearances = [item.minimum_clearance_m for item in selected if math.isfinite(item.minimum_clearance_m)]

    early = next((item.selected for item in selections if item.scenario_name == "tuning_obstacle_longitudinal_18.00"), None)

    return ConfigSummary(
        profile_name=profile_name,
        trackability_weight=trackability_weight,
        success_count=len(success_names),
        total_count=len(scenarios),
        mean_safe_speed_mps=optional_mean(speed_values),
        minimum_safe_speed_mps=optional_min(speed_values),
        mean_speed_cost=optional_mean(speed_costs),
        mean_geometric_cost=optional_mean(geometric_costs),
        minimum_clearance_m=optional_min(clearances),
        early_obstacle_speed_mps=None if early is None else early.safe_speed_mps,
        narrow_190_success="tuning_road_half_width_1.90" in success_names,
        narrow_175_success="road_half_width_1.75" in success_names,
        selections=selections,
    )


def value(value: float | None, digits: int = 3) -> str:
    return "n/a" if value is None else f"{value:.{digits}f}"


def print_summary(summary: ConfigSummary) -> None:
    print(
        f"{summary.profile_name:10s}  w={summary.trackability_weight:>4.1f}  "
        f"success={summary.success_count:>2d}/{summary.total_count:<2d}  "
        f"mean_v={value(summary.mean_safe_speed_mps)}  min_v={value(summary.minimum_safe_speed_mps)}  "
        f"early18_v={value(summary.early_obstacle_speed_mps)}  "
        f"mean_Jgeo={value(summary.mean_geometric_cost, 4)}  "
        f"min_clear={value(summary.minimum_clearance_m)}  "
        f"road1.90={summary.narrow_190_success} road1.75={summary.narrow_175_success}"
    )


def pareto_dominates(a: ConfigSummary, b: ConfigSummary) -> bool:
    if a.success_count != b.success_count:
        return a.success_count > b.success_count
    if None in (a.mean_speed_cost, a.mean_geometric_cost, b.mean_speed_cost, b.mean_geometric_cost):
        return False

    a_values = (a.mean_speed_cost, a.mean_geometric_cost)
    b_values = (b.mean_speed_cost, b.mean_geometric_cost)
    return all(x <= y for x, y in zip(a_values, b_values)) and any(x < y for x, y in zip(a_values, b_values))


def print_pareto(summaries: list[ConfigSummary]) -> None:
    max_success = max(summary.success_count for summary in summaries)
    max_coverage = [summary for summary in summaries if summary.success_count == max_success]
    frontier = [
        reference
        for reference in max_coverage
        if not any(pareto_dominates(candidate, reference) for candidate in max_coverage if candidate is not reference)
    ]
    frontier.sort(key=lambda item: (item.mean_speed_cost if item.mean_speed_cost is not None else math.inf, item.mean_geometric_cost if item.mean_geometric_cost is not None else math.inf))

    print("\n=== Max-coverage Pareto frontier: speed deficit vs geometric objective ===")
    for summary in frontier:
        print_summary(summary)


def print_selected_cases(summary: ConfigSummary) -> None:
    print(f"\n=== Selected cases for {summary.profile_name}, w={summary.trackability_weight:.1f} ===")
    for item in summary.selections:
        if item.selected is None:
            print(f"{item.scenario_name:38s} FAIL")
            continue
        selected = item.selected
        print(
            f"{item.scenario_name:38s} v_safe={selected.safe_speed_mps:6.3f}  "
            f"junction_dl={selected.junction_dl_ds:+.3f}  bypass_l={selected.bypass_l:+.2f}  "
            f"bypass_len={selected.bypass_length:4.1f}  J_geo={selected.geometric_cost:.5f}  "
            f"clear={selected.minimum_clearance_m if math.isfinite(selected.minimum_clearance_m) else math.inf:.3f}"
        )


def main() -> None:
    scenarios = build_development_tuning_scenarios()
    print(f"Building candidate pools for {len(scenarios)} development scenarios...")
    pools = {}
    for scenario in scenarios:
        pool = build_candidate_pool(scenario)
        pools[scenario.name] = pool
        print(f"  {scenario.name:38s} feasible pooled candidates={len(pool)}")

    summaries = []
    print("\n=== Config sweep ===")
    for profile_name, slopes in JUNCTION_PROFILES.items():
        for weight in TRACKABILITY_WEIGHTS:
            summary = summarize_config(
                scenarios=scenarios,
                pools=pools,
                profile_name=profile_name,
                allowed_slopes=slopes,
                trackability_weight=weight,
            )
            summaries.append(summary)
            print_summary(summary)

    print_pareto(summaries)

    baseline = next(summary for summary in summaries if summary.profile_name == "strict" and summary.trackability_weight == 0.0)
    max_success = max(summary.success_count for summary in summaries)
    max_coverage = [summary for summary in summaries if summary.success_count == max_success]
    trackability_best = min(
        max_coverage,
        key=lambda item: (
            item.mean_speed_cost if item.mean_speed_cost is not None else math.inf,
            item.mean_geometric_cost if item.mean_geometric_cost is not None else math.inf,
            item.trackability_weight,
        ),
    )

    print_selected_cases(baseline)
    if trackability_best != baseline:
        print_selected_cases(trackability_best)


if __name__ == "__main__":
    main()
