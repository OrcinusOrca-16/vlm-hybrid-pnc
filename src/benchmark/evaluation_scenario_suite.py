from dataclasses import replace

from src.benchmark.scenario import (
    BenchmarkScenario,
)
from src.benchmark.scenario_suite import (
    build_nominal_straight_scenario,
    build_static_obstacle_scenario,
)
from src.common.types import (
    StaticObstacle,
    VehicleState,
)
from src.planning.frenet import (
    sl_to_xy,
)
from src.planning.reference_line import (
    ReferenceLine,
)


_EVALUATION_CURVED_START_S = 12.0


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


def build_evaluation_lateral_recovery_scenario(
) -> BenchmarkScenario:
    base = build_nominal_straight_scenario()

    return replace(
        base,
        name="evaluation_lateral_recovery",
        vehicle_state=replace(
            base.vehicle_state,
            y=-0.9,
        ),
    )


def build_evaluation_obstacle_lateral_negative_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return _replace_single_obstacle_sl(
        base=base,
        name="evaluation_obstacle_lateral_-0.60",
        obstacle_s=24.0,
        obstacle_l=-0.60,
    )


def build_evaluation_obstacle_lateral_positive_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return _replace_single_obstacle_sl(
        base=base,
        name="evaluation_obstacle_lateral_+0.60",
        obstacle_s=24.0,
        obstacle_l=0.60,
    )


def build_evaluation_obstacle_early_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return _replace_single_obstacle_sl(
        base=base,
        name="evaluation_obstacle_longitudinal_19.00",
        obstacle_s=19.0,
        obstacle_l=1.0,
    )


def build_evaluation_obstacle_late_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return _replace_single_obstacle_sl(
        base=base,
        name="evaluation_obstacle_longitudinal_35.00",
        obstacle_s=35.0,
        obstacle_l=1.0,
    )


def build_evaluation_moderately_narrow_road_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return replace(
        base,
        name="evaluation_road_half_width_2.10",
        road_min_l=-2.1,
        road_max_l=2.1,
    )


def build_evaluation_hard_narrow_road_scenario(
) -> BenchmarkScenario:
    base = build_static_obstacle_scenario()

    return replace(
        base,
        name="evaluation_road_half_width_1.85",
        road_min_l=-1.85,
        road_max_l=1.85,
    )


def _build_evaluation_curved_nominal_scenario(
) -> BenchmarkScenario:
    base = build_nominal_straight_scenario()

    reference_line = ReferenceLine([
        (0.0, 0.0),
        (10.0, -0.4),
        (20.0, -1.6),
        (30.0, -3.6),
        (40.0, -6.4),
        (50.0, -10.0),
        (60.0, -14.4),
    ])

    initial_reference_point = reference_line.query(
        _EVALUATION_CURVED_START_S
    )

    vehicle_state = VehicleState(
        x=initial_reference_point.x,
        y=initial_reference_point.y,
        yaw=initial_reference_point.yaw,
        speed=base.vehicle_state.speed,
        curvature=initial_reference_point.curvature,
    )

    return replace(
        base,
        name="evaluation_curved_nominal",
        reference_line=reference_line,
        vehicle_state=vehicle_state,
    )


def build_evaluation_curved_obstacle_scenario(
) -> BenchmarkScenario:
    base = _build_evaluation_curved_nominal_scenario()

    return _replace_single_obstacle_sl(
        base=base,
        name="evaluation_curved_obstacle",
        obstacle_s=27.0,
        obstacle_l=0.8,
    )


def build_final_evaluation_scenarios(
) -> tuple[BenchmarkScenario, ...]:
    """Build the frozen held-out final evaluation scenario set."""

    return (
        build_evaluation_lateral_recovery_scenario(),
        build_evaluation_obstacle_lateral_negative_scenario(),
        build_evaluation_obstacle_lateral_positive_scenario(),
        build_evaluation_obstacle_early_scenario(),
        build_evaluation_obstacle_late_scenario(),
        build_evaluation_moderately_narrow_road_scenario(),
        build_evaluation_hard_narrow_road_scenario(),
        _build_evaluation_curved_nominal_scenario(),
        build_evaluation_curved_obstacle_scenario(),
    )