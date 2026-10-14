from dataclasses import dataclass

from src.common.types import VehicleParams


@dataclass(frozen=True)
class VehicleConfig:
    """Centralized ego-vehicle geometry configuration."""

    wheelbase_m: float
    front_overhang_m: float
    rear_overhang_m: float
    width_m: float

    def to_vehicle_params(self) -> VehicleParams:
        return VehicleParams(
            wheelbase_m=self.wheelbase_m,
            front_overhang_m=self.front_overhang_m,
            rear_overhang_m=self.rear_overhang_m,
            width_m=self.width_m,
        )


DEFAULT_VEHICLE_CONFIG = VehicleConfig(
    wheelbase_m=2.7,
    front_overhang_m=0.9,
    rear_overhang_m=1.0,
    width_m=1.8,
)