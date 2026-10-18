from time import perf_counter

from src.benchmark.planner_benchmark import (
    PlannerBenchmarkResult,
)
from src.common.types import (
    StaticObstacle,
    VehicleParams,
    VehicleState,
)
from src.config.planning import (
    DEFAULT_SAMPLING_PLANNER_CONFIG,
)
from src.planning.frenet import (
    frenet_trajectory_to_path,
)
from src.planning.hard_constraints import (
    filter_feasible_candidates,
)
from src.planning.reference_line import (
    ReferenceLine,
)
from src.planning.sampling_planner import (
    generate_returning_frenet_candidates_from_vehicle_state,
)
from src.planning.soft_cost import (
    select_best_matched_candidate,
)


def run_sampling_benchmark_adapter(
    reference_line: ReferenceLine,
    vehicle_state: VehicleState,
    vehicle_params: VehicleParams,
    obstacles: list[StaticObstacle],
    road_min_l: float,
    road_max_l: float,
    max_curvature: float,
    minimum_clearance_m: float,
    lateral_reference_m: float,
    terminal_lateral_target_m: float,
    terminal_first_derivative_target: float,
    lateral_weight: float,
    first_derivative_weight: float,
    second_derivative_weight: float,
    third_derivative_weight: float,
    terminal_lateral_weight: float,
    terminal_first_derivative_weight: float,
    bypass_lateral_positions: tuple[
        float,
        ...,
    ] = (
        DEFAULT_SAMPLING_PLANNER_CONFIG
        .returning_bypass_lateral_positions_m
    ),
    bypass_lengths: tuple[
        float,
        ...,
    ] = (
        DEFAULT_SAMPLING_PLANNER_CONFIG
        .returning_bypass_lengths_m
    ),
    total_length: float = (
        DEFAULT_SAMPLING_PLANNER_CONFIG
        .returning_total_length_m
    ),
    final_lateral_positions: tuple[
        float,
        ...,
    ] = (
        DEFAULT_SAMPLING_PLANNER_CONFIG
        .returning_final_lateral_positions_m
    ),
    final_dl_ds_values: tuple[
        float,
        ...,
    ] = (
        DEFAULT_SAMPLING_PLANNER_CONFIG
        .returning_final_dl_ds_values
    ),
    final_d2l_ds2: float = (
        DEFAULT_SAMPLING_PLANNER_CONFIG
        .returning_final_d2l_ds2
    ),
    resolution_m: float = (
        DEFAULT_SAMPLING_PLANNER_CONFIG
        .resolution_m
    ),
) -> PlannerBenchmarkResult:
    """Run the matched Sampling Planner through the benchmark interface."""

    start_time = perf_counter()

    trajectories = (
        generate_returning_frenet_candidates_from_vehicle_state(
            vehicle_state=vehicle_state,
            reference_line=reference_line,
            bypass_lateral_positions=bypass_lateral_positions,
            bypass_lengths=bypass_lengths,
            total_length=total_length,
            final_lateral_positions=final_lateral_positions,
            final_dl_ds_values=final_dl_ds_values,
            final_d2l_ds2=final_d2l_ds2,
            resolution=resolution_m,
        )
    )

    paths = [
        frenet_trajectory_to_path(
            trajectory=trajectory,
            reference_line=reference_line,
        )
        for trajectory in trajectories
    ]

    feasible_candidates = (
        filter_feasible_candidates(
            trajectories=trajectories,
            paths=paths,
            vehicle_params=vehicle_params,
            obstacles=obstacles,
            max_curvature=max_curvature,
            minimum_clearance_m=minimum_clearance_m,
            min_l=road_min_l,
            max_l=road_max_l,
        )
    )

    if not feasible_candidates:
        planning_time_ms = (
            perf_counter()
            - start_time
        ) * 1000.0

        return PlannerBenchmarkResult.failed(
            planner_name="sampling",
            planning_time_ms=planning_time_ms,
            failure_reason=(
                "No physically feasible Sampling candidate."
            ),
        )

    best_candidate = (
        select_best_matched_candidate(
            feasible_candidates=feasible_candidates,
            resolution_m=resolution_m,
            lateral_reference_m=lateral_reference_m,
            terminal_lateral_target_m=terminal_lateral_target_m,
            terminal_first_derivative_target=(
                terminal_first_derivative_target
            ),
            lateral_weight=lateral_weight,
            first_derivative_weight=first_derivative_weight,
            second_derivative_weight=second_derivative_weight,
            third_derivative_weight=third_derivative_weight,
            terminal_lateral_weight=terminal_lateral_weight,
            terminal_first_derivative_weight=(
                terminal_first_derivative_weight
            ),
        )
    )

    if best_candidate is None:
        raise RuntimeError(
            "Matched Sampling selection unexpectedly returned None."
        )

    (
        best_trajectory,
        best_path,
        matched_objective_value,
    ) = best_candidate

    planning_time_ms = (
        perf_counter()
        - start_time
    ) * 1000.0

    return PlannerBenchmarkResult.succeeded(
        planner_name="sampling",
        trajectory=best_trajectory,
        path=best_path,
        planning_time_ms=planning_time_ms,
        matched_objective_value=(
            matched_objective_value
        ),
    )