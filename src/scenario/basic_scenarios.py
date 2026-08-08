from src.common.types import ScenarioState, VehicleState


def create_normal_driving_scenario() -> tuple[VehicleState, ScenarioState]:
    """Create a basic straight-road scenario with no obstacles."""

    ego_state = VehicleState(
        x=0.0,
        y=0.0,
        yaw=0.0,
        speed=8.0,
    )

    scenario_state = ScenarioState(
        speed_limit_mps=10.0,
    )

    return ego_state, scenario_state


def create_lead_vehicle_slowdown_scenario() -> tuple[VehicleState, ScenarioState]:
    """Create a scenario where a slower vehicle is driving ahead."""

    ego_state = VehicleState(
        x=0.0,
        y=0.0,
        yaw=0.0,
        speed=10.0,
    )

    scenario_state = ScenarioState(
        speed_limit_mps=10.0,
        lead_vehicle_distance_m=20.0,
        lead_vehicle_speed_mps=5.0,
    )

    return ego_state, scenario_state


def create_pedestrian_crossing_scenario() -> tuple[VehicleState, ScenarioState]:
    """Create a scenario where a pedestrian is crossing ahead."""

    ego_state = VehicleState(
        x=0.0,
        y=0.0,
        yaw=0.0,
        speed=8.0,
    )

    scenario_state = ScenarioState(
        speed_limit_mps=10.0,
        pedestrian_distance_m=12.0,
        pedestrian_crossing=True,
    )

    return ego_state, scenario_state