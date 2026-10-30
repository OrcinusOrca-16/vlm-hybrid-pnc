import math

from src.benchmark.optimization_adapter import (
    run_optimization_benchmark_adapter,
)
from src.benchmark.tuning_scenario_suite import (
    build_tuning_scenarios,
)
from src.config.planning import (
    DEFAULT_OPTIMIZATION_PLANNER_CONFIG,
)
from src.planning.hard_constraints import (
    obstacle_clearance,
)


def run_scenario(
    scenario,
) -> bool:
    cfg = DEFAULT_OPTIMIZATION_PLANNER_CONFIG

    result = run_optimization_benchmark_adapter(
        reference_line=scenario.reference_line,
        vehicle_state=scenario.vehicle_state,
        vehicle_params=scenario.vehicle_params,
        obstacles=list(scenario.obstacles),
        road_min_l=scenario.road_min_l,
        road_max_l=scenario.road_max_l,
        max_curvature=scenario.max_curvature,
        minimum_clearance_m=scenario.minimum_clearance_m,
        planning_horizon_m=scenario.planning_horizon_m,
        resolution_m=scenario.resolution_m,
        lateral_reference_m=scenario.lateral_reference_m,
        terminal_lateral_target_m=scenario.terminal_lateral_target_m,
        terminal_first_derivative_target=(
            scenario.terminal_first_derivative_target
        ),
        lateral_weight=scenario.lateral_weight,
        first_derivative_weight=scenario.first_derivative_weight,
        second_derivative_weight=scenario.second_derivative_weight,
        third_derivative_weight=scenario.third_derivative_weight,
        terminal_lateral_weight=scenario.terminal_lateral_weight,
        terminal_first_derivative_weight=(
            scenario.terminal_first_derivative_weight
        ),
        max_second_derivative_abs=cfg.max_second_derivative_abs,
    )

    if not result.success:
        print(f"FAIL  {scenario.name}")
        print(f"      reason = {result.failure_reason}")
        return False

    max_curvature = max(
        abs(point.curvature)
        for point in result.path
    )

    if scenario.obstacles:
        min_clearance = min(
            obstacle_clearance(
                path_point=path_point,
                vehicle_params=scenario.vehicle_params,
                obstacle=obstacle,
            )
            for path_point in result.path
            for obstacle in scenario.obstacles
        )
    else:
        min_clearance = math.inf

    print(f"PASS  {scenario.name}")
    print(f"      objective = {result.matched_objective_value:.9f}")
    print(f"      planning  = {result.planning_time_ms:.3f} ms")
    print(
        f"      max |k|   = {max_curvature:.6f} "
        f"(limit {scenario.max_curvature:.6f})"
    )
    if scenario.obstacles:
        print(
            f"      clearance = {min_clearance:.6f} m "
            f"(limit > {scenario.minimum_clearance_m:.3f} m)"
        )

    return True


def main() -> None:
    scenarios = build_tuning_scenarios()

    print("Development-only Optimization Planner validation")
    print("Held-out evaluation scenarios are NOT used here.")
    print()

    pass_count = 0

    for scenario in scenarios:
        if run_scenario(scenario):
            pass_count += 1
        print()

    print("=" * 72)
    print(f"Summary: {pass_count}/{len(scenarios)} PASS")
    print("=" * 72)

    if pass_count != len(scenarios):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
