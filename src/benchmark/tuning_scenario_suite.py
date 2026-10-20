from dataclasses import replace

from src.benchmark.scenario import (
    BenchmarkScenario,
)
from src.benchmark.scenario_suite import (
    build_curved_reference_scenario,
    build_nominal_straight_scenario,
    build_static_obstacle_scenario,
    build_stress_sweep_scenarios,
)
from src.common.types import (
    StaticObstacle,
)
from src.planning.frenet import (
    sl_to_xy,
)


def _replace_single_obstacle_sl(
    base: BenchmarkScenario,
    name: str,
    obstacle_s: float,
    obstacle_l: float,
    obstacle_radius_m: float = 0.5,
) -> BenchmarkScenario:
    obstacle_x, obstacle_y = sl_to_xy(
        s=obstacle_s,
        l=obstacle_l,
        reference_line=base.reference_line,
    )

    obstacle = StaticObstacle(
        x=obstacle_x,
        y=obstacle_y,
        radius_m=obstacle_radius_m,
    )

    return replace(
        base,
        name=name,
        obstacles=(obstacle,),
    )


def build_tuning_lateral_recovery_scenario(
) -> BenchmarkScenario:
    base = build_nominal_straight_scenario()

    return replace(
        base,
        name="tuning_lateral_recovery",
        vehicle_state=replace(
            base.vehicle_state,
            y=0.8,
        ),
    )


def build_tuning_obstacle_lateral_negative_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return _replace_single_obstacle_sl(
        base=base,
        name="tuning_obstacle_lateral_-0.75",
        obstacle_s=24.0,
        obstacle_l=-0.75,
    )


def build_tuning_obstacle_center_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return _replace_single_obstacle_sl(
        base=base,
        name="tuning_obstacle_lateral_+0.00",
        obstacle_s=24.0,
        obstacle_l=0.0,
    )


def build_tuning_obstacle_lateral_positive_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return _replace_single_obstacle_sl(
        base=base,
        name="tuning_obstacle_lateral_+0.75",
        obstacle_s=24.0,
        obstacle_l=0.75,
    )


def build_tuning_obstacle_early_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return _replace_single_obstacle_sl(
        base=base,
        name="tuning_obstacle_longitudinal_18.00",
        obstacle_s=18.0,
        obstacle_l=1.0,
    )


def build_tuning_obstacle_late_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return _replace_single_obstacle_sl(
        base=base,
        name="tuning_obstacle_longitudinal_34.00",
        obstacle_s=34.0,
        obstacle_l=1.0,
    )


def build_tuning_narrow_road_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return replace(
        base,
        name="tuning_road_half_width_1.90",
        road_min_l=-1.9,
        road_max_l=1.9,
    )


def build_tuning_curved_obstacle_scenario(
) -> BenchmarkScenario:
    base = build_curved_reference_scenario()

    return _replace_single_obstacle_sl(
        base=base,
        name="tuning_curved_obstacle",
        obstacle_s=26.0,
        obstacle_l=-0.75,
    )


def build_tuning_scenarios(
) -> tuple[BenchmarkScenario, ...]:
    """Build the fixed scenario set used only for planner tuning."""

    return (
        build_tuning_lateral_recovery_scenario(),
        build_tuning_obstacle_lateral_negative_scenario(),
        build_tuning_obstacle_center_scenario(),
        build_tuning_obstacle_lateral_positive_scenario(),
        build_tuning_obstacle_early_scenario(),
        build_tuning_obstacle_late_scenario(),
        build_tuning_narrow_road_scenario(),
        build_tuning_curved_obstacle_scenario(),
    )


def build_development_tuning_scenarios(
) -> tuple[BenchmarkScenario, ...]:
    """Build all scenarios allowed for planner tuning."""

    return (
        build_tuning_scenarios()
        + build_stress_sweep_scenarios()
    )