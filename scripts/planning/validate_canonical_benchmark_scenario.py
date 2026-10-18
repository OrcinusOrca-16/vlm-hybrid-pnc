import math

from src.benchmark.canonical_scenario import (
    build_canonical_static_obstacle_scenario,
)


def main() -> None:
    scenario = (
        build_canonical_static_obstacle_scenario()
    )

    assert math.isclose(
        scenario.reference_line.length,
        50.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        scenario.vehicle_state.x,
        12.0,
        abs_tol=1e-12,
    )

    assert len(scenario.obstacles) == 1

    obstacle = scenario.obstacles[0]

    assert math.isclose(
        obstacle.x,
        24.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        obstacle.y,
        1.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        obstacle.radius_m,
        0.5,
        abs_tol=1e-12,
    )

    assert math.isclose(
        scenario.planning_horizon_m,
        28.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        scenario.resolution_m,
        0.5,
        abs_tol=1e-12,
    )

    assert (
        scenario.vehicle_state.x
        + scenario.planning_horizon_m
        < scenario.reference_line.length
    )

    assert (
        scenario.road_min_l
        < scenario.road_max_l
    )

    assert scenario.minimum_clearance_m >= 0.0
    assert scenario.max_curvature > 0.0

    assert math.isclose(
        scenario.terminal_lateral_target_m,
        0.0,
        abs_tol=1e-12,
    )

    assert math.isclose(
        scenario.terminal_first_derivative_target,
        0.0,
        abs_tol=1e-12,
    )

    print(
        "Canonical benchmark scenario passed."
    )
    print(
        "  reference_length="
        f"{scenario.reference_line.length:.2f}"
    )
    print(
        "  start_x="
        f"{scenario.vehicle_state.x:.2f}"
    )
    print(
        "  obstacle="
        f"({obstacle.x:.2f}, {obstacle.y:.2f})"
    )
    print(
        "  horizon="
        f"{scenario.planning_horizon_m:.2f}"
    )
    print(
        "  resolution="
        f"{scenario.resolution_m:.2f}"
    )


if __name__ == "__main__":
    main()