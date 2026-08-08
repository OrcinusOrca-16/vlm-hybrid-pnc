from src.common.types import (
    BehaviorCommand,
    BehaviorType,
    ScenarioState,
    VehicleState,
)


def decide_behavior(
    ego_state: VehicleState,
    scenario: ScenarioState,
) -> BehaviorCommand:
    """Generate a high-level driving command using deterministic rules."""

    # Rule 1: Stop for a pedestrian crossing nearby.
    if (
        scenario.pedestrian_crossing
        and scenario.pedestrian_distance_m is not None
        and scenario.pedestrian_distance_m <= 15.0
    ):
        return BehaviorCommand(
            behavior=BehaviorType.STOP,
            target_speed_mps=0.0,
            reason="Pedestrian is crossing ahead.",
        )

    # Rule 2: Slow down for a slower lead vehicle.
    if (
        scenario.lead_vehicle_distance_m is not None
        and scenario.lead_vehicle_speed_mps is not None
        and scenario.lead_vehicle_distance_m < 30.0
        and scenario.lead_vehicle_speed_mps < ego_state.speed
    ):
        return BehaviorCommand(
            behavior=BehaviorType.SLOW_DOWN,
            target_speed_mps=scenario.lead_vehicle_speed_mps,
            reason="Slower lead vehicle is close ahead.",
        )

    # Rule 3: Keep lane when the road ahead is clear.
    if (
        scenario.lead_vehicle_distance_m is None
        and not scenario.pedestrian_crossing
    ):
        return BehaviorCommand(
            behavior=BehaviorType.KEEP_LANE,
            target_speed_mps=scenario.speed_limit_mps,
            reason="Road ahead is clear.",
        )

    raise NotImplementedError("This scenario is not supported yet.")