import numpy as np

from src.common.types import (
    StaticObstacle,
)
from src.config.vehicle import (
    DEFAULT_VEHICLE_CONFIG,
)
from src.planning.lateral_corridor import (
    LateralCorridor,
)
from src.planning.optimization_planner import (
    plan_matched_multi_corridor_path,
    plan_matched_optimized_path,
)
from src.planning.reference_line import (
    ReferenceLine,
)


def build_reference_line() -> ReferenceLine:
    return ReferenceLine([
        (0.0, 0.0),
        (20.0, 0.0),
    ])


def build_s_values() -> np.ndarray:
    return np.arange(
        0.0,
        10.0 + 0.5,
        0.5,
    )


def common_objective_arguments() -> dict:
    return {
        "resolution_m": 0.5,
        "lateral_reference_m": 0.0,
        "terminal_lateral_target_m": 0.0,
        "terminal_first_derivative_target": 0.0,
        "lateral_weight": 1.0,
        "first_derivative_weight": 1.0,
        "second_derivative_weight": 5.0,
        "third_derivative_weight": 20.0,
        "terminal_lateral_weight": 5.0,
        "terminal_first_derivative_weight": 5.0,
        "max_second_derivative_abs": 1.0,
    }


def validate_safe_solution() -> None:
    reference_line = build_reference_line()
    s_values = build_s_values()

    lower_bounds = np.full(
        len(s_values),
        -2.0,
    )
    upper_bounds = np.full(
        len(s_values),
        2.0,
    )

    result = plan_matched_optimized_path(
        reference_line=reference_line,
        s_values=s_values,
        corridor_lower_bounds=lower_bounds,
        corridor_upper_bounds=upper_bounds,
        initial_l=0.0,
        initial_dl_ds=0.0,
        initial_d2l_ds2=0.0,
        vehicle_params=(
            DEFAULT_VEHICLE_CONFIG
            .to_vehicle_params()
        ),
        obstacles=[],
        max_curvature=0.1,
        minimum_clearance_m=0.5,
        road_min_l=-2.0,
        road_max_l=2.0,
        **common_objective_arguments(),
    )

    assert result is not None

    print(
        "Safe matched Optimization result passed."
    )


def validate_curvature_rejection() -> None:
    reference_line = build_reference_line()
    s_values = build_s_values()

    lower_bounds = np.full(
        len(s_values),
        -2.0,
    )
    upper_bounds = np.full(
        len(s_values),
        2.0,
    )

    try:
        plan_matched_optimized_path(
            reference_line=reference_line,
            s_values=s_values,
            corridor_lower_bounds=lower_bounds,
            corridor_upper_bounds=upper_bounds,
            initial_l=1.0,
            initial_dl_ds=0.0,
            initial_d2l_ds2=0.0,
            vehicle_params=(
                DEFAULT_VEHICLE_CONFIG
                .to_vehicle_params()
            ),
            obstacles=[],
            max_curvature=1e-6,
            minimum_clearance_m=0.0,
            road_min_l=-2.5,
            road_max_l=2.5,
            **common_objective_arguments(),
        )
    except RuntimeError:
        print(
            "Exact Cartesian curvature rejection passed."
        )
        return

    raise AssertionError(
        "Curvature-violating Optimization result was not rejected."
    )


def validate_clearance_rejection() -> None:
    reference_line = build_reference_line()
    s_values = build_s_values()

    lower_bounds = np.full(
        len(s_values),
        -2.0,
    )
    upper_bounds = np.full(
        len(s_values),
        2.0,
    )

    obstacle = StaticObstacle(
        x=5.0,
        y=0.0,
        radius_m=0.3,
    )

    try:
        plan_matched_optimized_path(
            reference_line=reference_line,
            s_values=s_values,
            corridor_lower_bounds=lower_bounds,
            corridor_upper_bounds=upper_bounds,
            initial_l=0.0,
            initial_dl_ds=0.0,
            initial_d2l_ds2=0.0,
            vehicle_params=(
                DEFAULT_VEHICLE_CONFIG
                .to_vehicle_params()
            ),
            obstacles=[obstacle],
            max_curvature=0.1,
            minimum_clearance_m=0.5,
            road_min_l=-2.0,
            road_max_l=2.0,
            **common_objective_arguments(),
        )
    except RuntimeError:
        print(
            "Exact obstacle-clearance rejection passed."
        )
        return

    raise AssertionError(
        "Clearance-violating Optimization result was not rejected."
    )


def validate_road_boundary_rejection() -> None:
    reference_line = build_reference_line()
    s_values = build_s_values()

    lower_bounds = np.full(
        len(s_values),
        -2.0,
    )
    upper_bounds = np.full(
        len(s_values),
        2.0,
    )

    try:
        plan_matched_optimized_path(
            reference_line=reference_line,
            s_values=s_values,
            corridor_lower_bounds=lower_bounds,
            corridor_upper_bounds=upper_bounds,
            initial_l=1.2,
            initial_dl_ds=0.0,
            initial_d2l_ds2=0.0,
            vehicle_params=(
                DEFAULT_VEHICLE_CONFIG
                .to_vehicle_params()
            ),
            obstacles=[],
            max_curvature=1.0,
            minimum_clearance_m=0.0,
            road_min_l=-2.0,
            road_max_l=2.0,
            **common_objective_arguments(),
        )
    except RuntimeError:
        print(
            "Exact road-boundary rejection passed."
        )
        return

    raise AssertionError(
        "Road-boundary-violating Optimization result was not rejected."
    )


def validate_multi_corridor_rejection() -> None:
    reference_line = build_reference_line()
    s_values = build_s_values()

    negative_lower = np.full(
        len(s_values),
        -2.1,
    )
    negative_upper = np.full(
        len(s_values),
        2.1,
    )

    positive_lower = np.full(
        len(s_values),
        -2.1,
    )
    positive_upper = np.full(
        len(s_values),
        2.1,
    )

    negative_upper[8:13] = -1.2
    positive_lower[8:13] = 1.2

    negative_corridor = LateralCorridor(
        s_values=s_values,
        lower_bounds=negative_lower,
        upper_bounds=negative_upper,
    )

    positive_corridor = LateralCorridor(
        s_values=s_values,
        lower_bounds=positive_lower,
        upper_bounds=positive_upper,
    )

    obstacle = StaticObstacle(
        x=5.0,
        y=-1.5,
        radius_m=0.2,
    )

    result = plan_matched_multi_corridor_path(
        reference_line=reference_line,
        corridors=[
            negative_corridor,
            positive_corridor,
        ],
        initial_l=0.0,
        initial_dl_ds=0.0,
        initial_d2l_ds2=0.0,
        vehicle_params=(
            DEFAULT_VEHICLE_CONFIG
            .to_vehicle_params()
        ),
        obstacles=[obstacle],
        max_curvature=1.0,
        minimum_clearance_m=0.2,
        road_min_l=-3.0,
        road_max_l=3.0,
        **common_objective_arguments(),
    )

    middle_lateral = (
        result.lateral_solution[8:13]
    )

    assert np.all(
        middle_lateral > 0.0
    )

    print(
        "Matched multi-corridor physical rejection passed."
    )

    print(
        "  selected_branch=positive"
    )

    print(
        "  min_middle_l="
        f"{np.min(middle_lateral):.6f}"
    )


def main() -> None:
    validate_safe_solution()
    validate_curvature_rejection()
    validate_clearance_rejection()
    validate_road_boundary_rejection()
    validate_multi_corridor_rejection()

    print()
    print(
        "All matched physical acceptance cases passed."
    )


if __name__ == "__main__":
    main()