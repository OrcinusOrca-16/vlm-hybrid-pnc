import math

from src.common.types import (
    StaticObstacle,
    VehicleParams,
    VehicleState,
)
from src.planning.frenet import frenet_trajectories_to_paths
from src.planning.hard_constraints import (
    filter_feasible_candidates,
    obstacle_clearance,
)
from src.planning.reference_line import ReferenceLine
from src.planning.sampling_planner import (
    generate_frenet_candidates_from_vehicle_state,
)
from src.planning.soft_cost import (
    select_best_candidate,
    total_cost,
)


LATERAL_WEIGHT = 1.0
CURVATURE_WEIGHT = 10.0
LENGTH_WEIGHT = 0.05


def build_reference_line() -> ReferenceLine:
    return ReferenceLine([
        (0.0, 0.0),
        (20.0, 0.0),
        (40.0, 0.0),
    ])


def build_ego_state() -> VehicleState:
    return VehicleState(
        x=12.0,
        y=0.0,
        yaw=0.0,
        speed=5.0,
        curvature=0.0,
    )


def build_vehicle_params() -> VehicleParams:
    return VehicleParams(
        wheelbase_m=2.7,
        front_overhang_m=0.9,
        rear_overhang_m=1.0,
        width_m=1.8,
    )


def run_nominal_case() -> None:
    reference_line = build_reference_line()
    ego_state = build_ego_state()
    vehicle_params = build_vehicle_params()

    candidates = generate_frenet_candidates_from_vehicle_state(
        vehicle_state=ego_state,
        reference_line=reference_line,
    )

    paths = frenet_trajectories_to_paths(
        trajectories=candidates,
        reference_line=reference_line,
    )

    feasible_candidates = filter_feasible_candidates(
        trajectories=candidates,
        paths=paths,
        vehicle_params=vehicle_params,
        obstacles=[],
        max_curvature=1.0,
        minimum_clearance_m=0.5,
        min_l=-2.0,
        max_l=2.0,
    )

    best_candidate = select_best_candidate(
        feasible_candidates=feasible_candidates,
        lateral_weight=LATERAL_WEIGHT,
        curvature_weight=CURVATURE_WEIGHT,
        length_weight=LENGTH_WEIGHT,
    )

    assert best_candidate is not None

    best_trajectory, best_path = best_candidate
    best_end_l = best_trajectory.points[-1].l
    best_length = best_path[-1].s

    assert math.isclose(
        best_end_l,
        0.0,
        abs_tol=1e-9,
    )

    assert math.isclose(
        best_length,
        20.0,
        abs_tol=1e-6,
    )

    print("Nominal case passed.")
    print(
        f"  candidates={len(candidates)}, "
        f"feasible={len(feasible_candidates)}, "
        f"best_l={best_end_l:.2f}, "
        f"length={best_length:.2f}"
    )


def run_obstacle_case() -> None:
    reference_line = build_reference_line()
    ego_state = build_ego_state()
    vehicle_params = build_vehicle_params()

    obstacles = [
        StaticObstacle(
            x=24.0,
            y=1.0,
            radius_m=0.5,
        )
    ]

    candidates = generate_frenet_candidates_from_vehicle_state(
        vehicle_state=ego_state,
        reference_line=reference_line,
        target_lateral_positions=(
            -2.0,
            -1.0,
            0.0,
            1.0,
            2.0,
        ),
    )

    paths = frenet_trajectories_to_paths(
        trajectories=candidates,
        reference_line=reference_line,
    )

    feasible_candidates = filter_feasible_candidates(
        trajectories=candidates,
        paths=paths,
        vehicle_params=vehicle_params,
        obstacles=obstacles,
        max_curvature=1.0,
        minimum_clearance_m=0.5,
        min_l=-3.0,
        max_l=3.0,
    )

    best_candidate = select_best_candidate(
        feasible_candidates=feasible_candidates,
        lateral_weight=LATERAL_WEIGHT,
        curvature_weight=CURVATURE_WEIGHT,
        length_weight=LENGTH_WEIGHT,
    )

    assert best_candidate is not None

    best_trajectory, best_path = best_candidate
    best_end_l = best_trajectory.points[-1].l

    min_clearance = min(
        obstacle_clearance(
            path_point=point,
            vehicle_params=vehicle_params,
            obstacle=obstacle,
        )
        for point in best_path
        for obstacle in obstacles
    )

    best_cost = total_cost(
        trajectory=best_trajectory,
        path=best_path,
        lateral_weight=LATERAL_WEIGHT,
        curvature_weight=CURVATURE_WEIGHT,
        length_weight=LENGTH_WEIGHT,
    )

    assert math.isclose(
        best_end_l,
        -1.0,
        abs_tol=1e-9,
    )

    assert min_clearance > 0.5

    print("Obstacle case passed.")
    print(
        f"  candidates={len(candidates)}, "
        f"feasible={len(feasible_candidates)}, "
        f"best_l={best_end_l:.2f}, "
        f"length={best_path[-1].s:.2f}, "
        f"clearance={min_clearance:.2f}, "
        f"cost={best_cost:.4f}"
    )


def run_curved_reference_line_case() -> None:
    reference_line = ReferenceLine([
        (0.0, 0.0),
        (10.0, 0.5),
        (20.0, 2.0),
        (30.0, 4.5),
        (40.0, 8.0),
        (50.0, 12.5),
    ])

    ego_reference = reference_line.query(
        12.0
    )

    ego_state = VehicleState(
        x=ego_reference.x,
        y=ego_reference.y,
        yaw=ego_reference.yaw,
        speed=5.0,
        curvature=ego_reference.curvature,
    )

    vehicle_params = build_vehicle_params()

    candidates = generate_frenet_candidates_from_vehicle_state(
        vehicle_state=ego_state,
        reference_line=reference_line,
    )

    paths = frenet_trajectories_to_paths(
        trajectories=candidates,
        reference_line=reference_line,
    )

    feasible_candidates = filter_feasible_candidates(
        trajectories=candidates,
        paths=paths,
        vehicle_params=vehicle_params,
        obstacles=[],
        max_curvature=1.0,
        minimum_clearance_m=0.5,
        min_l=-2.0,
        max_l=2.0,
    )

    best_candidate = select_best_candidate(
        feasible_candidates=feasible_candidates,
        lateral_weight=LATERAL_WEIGHT,
        curvature_weight=CURVATURE_WEIGHT,
        length_weight=LENGTH_WEIGHT,
    )

    assert best_candidate is not None

    best_trajectory, best_path = best_candidate

    best_end_l = best_trajectory.points[-1].l

    best_horizon = (
        best_trajectory.points[-1].s
        - best_trajectory.points[0].s
    )

    max_path_curvature = max(
        abs(point.curvature)
        for point in best_path
    )

    end_reference_x, end_reference_y = (
        reference_line.position(
            best_trajectory.points[-1].s
        )
    )

    endpoint_error = math.hypot(
        best_path[-1].x - end_reference_x,
        best_path[-1].y - end_reference_y,
    )

    assert len(candidates) == 10
    assert len(feasible_candidates) == 6

    assert math.isclose(
        best_end_l,
        0.0,
        abs_tol=1e-9,
    )

    assert math.isclose(
        best_horizon,
        20.0,
        abs_tol=1e-9,
    )

    assert max_path_curvature > 1e-3

    assert endpoint_error < 1e-6

    print("Curved Reference Line case passed.")
    print(
        f"  candidates={len(candidates)}, "
        f"feasible={len(feasible_candidates)}, "
        f"best_l={best_end_l:.2f}, "
        f"horizon={best_horizon:.2f}, "
        f"path_length={best_path[-1].s:.2f}, "
        f"max_curvature={max_path_curvature:.4f}"
    )


def run_lateral_maneuver_case() -> None:
    reference_line = build_reference_line()

    ego_state = VehicleState(
        x=12.0,
        y=1.0,
        yaw=-0.05,
        speed=5.0,
        curvature=0.0,
    )

    vehicle_params = build_vehicle_params()

    candidates = generate_frenet_candidates_from_vehicle_state(
        vehicle_state=ego_state,
        reference_line=reference_line,
    )

    paths = frenet_trajectories_to_paths(
        trajectories=candidates,
        reference_line=reference_line,
    )

    feasible_candidates = filter_feasible_candidates(
        trajectories=candidates,
        paths=paths,
        vehicle_params=vehicle_params,
        obstacles=[],
        max_curvature=1.0,
        minimum_clearance_m=0.5,
        min_l=-3.0,
        max_l=3.0,
    )

    best_candidate = select_best_candidate(
        feasible_candidates=feasible_candidates,
        lateral_weight=LATERAL_WEIGHT,
        curvature_weight=CURVATURE_WEIGHT,
        length_weight=LENGTH_WEIGHT,
    )

    assert best_candidate is not None

    best_trajectory, best_path = best_candidate

    start_point = best_trajectory.points[0]
    end_point = best_trajectory.points[-1]

    expected_start_dl_ds = math.tan(
        ego_state.yaw
    )

    best_horizon = (
        end_point.s
        - start_point.s
    )

    assert math.isclose(
        start_point.l,
        1.0,
        abs_tol=1e-6,
    )

    assert math.isclose(
        start_point.dl_ds,
        expected_start_dl_ds,
        abs_tol=1e-6,
    )

    assert math.isclose(
        end_point.l,
        0.0,
        abs_tol=1e-9,
    )

    assert math.isclose(
        end_point.dl_ds,
        0.0,
        abs_tol=1e-9,
    )

    assert math.isclose(
        best_horizon,
        20.0,
        abs_tol=1e-9,
    )

    print("Lateral maneuver case passed.")
    print(
        f"  candidates={len(candidates)}, "
        f"feasible={len(feasible_candidates)}, "
        f"start_l={start_point.l:.2f}, "
        f"start_dl_ds={start_point.dl_ds:.4f}, "
        f"best_l={end_point.l:.2f}, "
        f"horizon={best_horizon:.2f}, "
        f"path_length={best_path[-1].s:.2f}"
    )


def main() -> None:
    run_nominal_case()
    run_obstacle_case()
    run_curved_reference_line_case()
    run_lateral_maneuver_case()

    print()
    print("All Sampling Planner validation cases passed.")


if __name__ == "__main__":
    main()