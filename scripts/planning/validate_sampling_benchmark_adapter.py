from src.benchmark.sampling_adapter import (
    run_sampling_benchmark_adapter,
)
from src.common.types import (
    StaticObstacle,
    VehicleState,
)
from src.config.vehicle import (
    DEFAULT_VEHICLE_CONFIG,
)
from src.planning.reference_line import (
    ReferenceLine,
)


def build_reference_line() -> ReferenceLine:
    return ReferenceLine([
        (0.0, 0.0),
        (40.0, 0.0),
    ])


def build_vehicle_state() -> VehicleState:
    return VehicleState(
        x=0.0,
        y=0.0,
        yaw=0.0,
        speed=5.0,
    )


def common_arguments() -> dict:
    return {
        "road_min_l": -3.0,
        "road_max_l": 3.0,
        "max_curvature": 1.0,
        "minimum_clearance_m": 0.5,
        "lateral_reference_m": 0.0,
        "terminal_lateral_target_m": 0.0,
        "terminal_first_derivative_target": 0.0,
        "lateral_weight": 1.0,
        "first_derivative_weight": 1.0,
        "second_derivative_weight": 1.0,
        "third_derivative_weight": 1.0,
        "terminal_lateral_weight": 1.0,
        "terminal_first_derivative_weight": 1.0,
        "bypass_lateral_positions": (
            -1.0,
            0.0,
            1.0,
        ),
        "bypass_lengths": (
            4.0,
        ),
        "total_length": 10.0,
        "final_lateral_positions": (
            0.0,
        ),
        "final_dl_ds_values": (
            0.0,
        ),
        "final_d2l_ds2": 0.0,
        "resolution_m": 0.5,
    }


def validate_success_case() -> None:
    result = run_sampling_benchmark_adapter(
        reference_line=build_reference_line(),
        vehicle_state=build_vehicle_state(),
        vehicle_params=(
            DEFAULT_VEHICLE_CONFIG
            .to_vehicle_params()
        ),
        obstacles=[],
        **common_arguments(),
    )

    assert result.success
    assert result.planner_name == "sampling"
    assert result.trajectory is not None
    assert result.path is not None
    assert result.failure_reason is None
    assert result.planning_time_ms >= 0.0
    assert result.matched_objective_value is not None

    assert abs(
        result.matched_objective_value
    ) < 1e-10

    assert abs(
        result.trajectory.points[-1].l
    ) < 1e-10

    print(
        "Sampling benchmark adapter success case passed."
    )
    print(
        "  planning_time_ms="
        f"{result.planning_time_ms:.4f}"
    )
    print(
        "  matched_objective="
        f"{result.matched_objective_value:.6f}"
    )


def validate_failure_case() -> None:
    obstacle = StaticObstacle(
        x=5.0,
        y=0.0,
        radius_m=5.0,
    )

    result = run_sampling_benchmark_adapter(
        reference_line=build_reference_line(),
        vehicle_state=build_vehicle_state(),
        vehicle_params=(
            DEFAULT_VEHICLE_CONFIG
            .to_vehicle_params()
        ),
        obstacles=[
            obstacle,
        ],
        **common_arguments(),
    )

    assert not result.success
    assert result.planner_name == "sampling"
    assert result.trajectory is None
    assert result.path is None
    assert result.matched_objective_value is None
    assert result.failure_reason is not None
    assert result.planning_time_ms >= 0.0

    print(
        "Sampling benchmark adapter failure case passed."
    )
    print(
        "  failure_reason="
        f"{result.failure_reason}"
    )
    print(
        "  planning_time_ms="
        f"{result.planning_time_ms:.4f}"
    )


def main() -> None:
    validate_success_case()
    validate_failure_case()

    print()
    print(
        "All Sampling benchmark adapter cases passed."
    )


if __name__ == "__main__":
    main()