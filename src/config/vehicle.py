from dataclasses import dataclass

from src.common.types import (
    VehicleParams,
)
from src.common.vehicle_geometry import (
    curvature_from_steering_angle,
)


@dataclass(frozen=True)
class VehicleConfig:
    """Centralized ego-vehicle geometry and steering configuration."""

    wheelbase_m: float
    front_overhang_m: float
    rear_overhang_m: float
    width_m: float
    max_steering_angle_rad: float
    max_steering_rate_radps: float

    @property
    def max_curvature_per_m(
        self,
    ) -> float:
        return curvature_from_steering_angle(
            steering_angle_rad=(
                self.max_steering_angle_rad
            ),
            wheelbase_m=self.wheelbase_m,
        )

    def to_vehicle_params(
        self,
    ) -> VehicleParams:
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
    max_steering_angle_rad=0.5,
    max_steering_rate_radps=0.5,
)
