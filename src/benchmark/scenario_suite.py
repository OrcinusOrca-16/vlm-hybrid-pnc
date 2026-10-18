from dataclasses import replace

from src.benchmark.canonical_scenario import (
    build_canonical_static_obstacle_scenario,
)
from src.benchmark.scenario import (
    BenchmarkScenario,
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


_CURVED_START_S = 12.0
_CURVED_OBSTACLE_S = 24.0
_CURVED_OBSTACLE_L = 1.0


def build_nominal_straight_scenario(
) -> BenchmarkScenario:
    """Straight-road nominal case with no obstacle."""

    base = (
        build_canonical_static_obstacle_scenario()
    )

    return replace(
        base,
        name="nominal_straight",
        obstacles=(),
    )


def build_lateral_recovery_scenario(
) -> BenchmarkScenario:
    """Straight-road recovery from an initial lateral offset."""

    base = build_nominal_straight_scenario()

    return replace(
        base,
        name="lateral_recovery",
        vehicle_state=replace(
            base.vehicle_state,
            y=1.0,
        ),
    )


def build_static_obstacle_scenario(
) -> BenchmarkScenario:
    """Canonical straight-road static-obstacle case."""

    return (
        build_canonical_static_obstacle_scenario()
    )


def build_mirrored_obstacle_scenario(
) -> BenchmarkScenario:
    """Mirror the canonical obstacle across the reference line."""

    base = build_static_obstacle_scenario()

    obstacle = base.obstacles[0]

    mirrored_obstacle = replace(
        obstacle,
        y=-obstacle.y,
    )

    return replace(
        base,
        name="mirrored_obstacle",
        obstacles=(
            mirrored_obstacle,
        ),
    )


def _build_curved_reference_line(
) -> ReferenceLine:
    """Build a gentle curved reference line."""

    return ReferenceLine([
        (0.0, 0.0),
        (10.0, 0.5),
        (20.0, 2.0),
        (30.0, 4.5),
        (40.0, 8.0),
        (50.0, 12.5),
    ])


def build_curved_reference_scenario(
) -> BenchmarkScenario:
    """Curved-road nominal case with no obstacle."""

    base = build_nominal_straight_scenario()

    reference_line = (
        _build_curved_reference_line()
    )

    initial_reference_point = (
        reference_line.query(
            _CURVED_START_S
        )
    )

    vehicle_state = VehicleState(
        x=initial_reference_point.x,
        y=initial_reference_point.y,
        yaw=initial_reference_point.yaw,
        speed=base.vehicle_state.speed,
        curvature=(
            initial_reference_point.curvature
        ),
    )

    return replace(
        base,
        name="curved_reference",
        reference_line=reference_line,
        vehicle_state=vehicle_state,
    )


def build_curved_reference_obstacle_scenario(
) -> BenchmarkScenario:
    """Curved-road case with one static obstacle."""

    base = build_curved_reference_scenario()

    obstacle_x, obstacle_y = sl_to_xy(
        s=_CURVED_OBSTACLE_S,
        l=_CURVED_OBSTACLE_L,
        reference_line=base.reference_line,
    )

    obstacle = StaticObstacle(
        x=obstacle_x,
        y=obstacle_y,
        radius_m=0.5,
    )

    return replace(
        base,
        name="curved_reference_obstacle",
        obstacles=(
            obstacle,
        ),
    )


def build_narrower_road_scenario(
) -> BenchmarkScenario:
    """Canonical obstacle case with a narrower road."""

    base = build_static_obstacle_scenario()

    return replace(
        base,
        name="narrower_road",
        road_min_l=-2.5,
        road_max_l=2.5,
    )


def build_discrete_benchmark_scenarios(
) -> tuple[BenchmarkScenario, ...]:
    """Build the fixed discrete scenario benchmark suite."""

    return (
        build_nominal_straight_scenario(),
        build_lateral_recovery_scenario(),
        build_static_obstacle_scenario(),
        build_mirrored_obstacle_scenario(),
        build_curved_reference_scenario(),
        build_curved_reference_obstacle_scenario(),
        build_narrower_road_scenario(),
    )


def build_obstacle_lateral_sweep_scenarios(
) -> tuple[BenchmarkScenario, ...]:
    """Sweep obstacle lateral position on a straight road."""

    base = build_static_obstacle_scenario()

    lateral_positions = (
        -1.5,
        -1.0,
        -0.5,
        0.0,
        0.5,
        1.0,
        1.5,
    )

    scenarios = []

    for lateral_position in lateral_positions:
        obstacle_x, obstacle_y = sl_to_xy(
            s=24.0,
            l=lateral_position,
            reference_line=base.reference_line,
        )

        obstacle = replace(
            base.obstacles[0],
            x=obstacle_x,
            y=obstacle_y,
        )

        scenarios.append(
            replace(
                base,
                name=(
                    "obstacle_lateral_"
                    f"{lateral_position:+.2f}"
                ),
                obstacles=(
                    obstacle,
                ),
            )
        )

    return tuple(
        scenarios
    )


def build_obstacle_longitudinal_sweep_scenarios(
) -> tuple[BenchmarkScenario, ...]:
    """Sweep obstacle longitudinal position."""

    base = build_static_obstacle_scenario()

    longitudinal_positions = (
        16.0,
        20.0,
        24.0,
        28.0,
        32.0,
        36.0,
    )

    scenarios = []

    for longitudinal_position in (
        longitudinal_positions
    ):
        obstacle_x, obstacle_y = sl_to_xy(
            s=longitudinal_position,
            l=1.0,
            reference_line=base.reference_line,
        )

        obstacle = replace(
            base.obstacles[0],
            x=obstacle_x,
            y=obstacle_y,
        )

        scenarios.append(
            replace(
                base,
                name=(
                    "obstacle_longitudinal_"
                    f"{longitudinal_position:.2f}"
                ),
                obstacles=(
                    obstacle,
                ),
            )
        )

    return tuple(
        scenarios
    )


def build_road_width_sweep_scenarios(
) -> tuple[BenchmarkScenario, ...]:
    """Sweep symmetric road half-width."""

    base = build_static_obstacle_scenario()

    road_half_widths = (
        3.0,
        2.5,
        2.25,
        2.0,
        1.75,
        1.5,
    )

    scenarios = []

    for half_width in road_half_widths:
        scenarios.append(
            replace(
                base,
                name=(
                    "road_half_width_"
                    f"{half_width:.2f}"
                ),
                road_min_l=-half_width,
                road_max_l=half_width,
            )
        )

    return tuple(
        scenarios
    )


def build_stress_sweep_scenarios(
) -> tuple[BenchmarkScenario, ...]:
    """Build all stress-sweep benchmark scenarios."""

    return (
        build_obstacle_lateral_sweep_scenarios()
        + build_obstacle_longitudinal_sweep_scenarios()
        + build_road_width_sweep_scenarios()
    )