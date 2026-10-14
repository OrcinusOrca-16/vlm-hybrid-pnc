from dataclasses import dataclass


@dataclass(frozen=True)
class RoadConfig:
    """Simple Frenet road-boundary configuration."""

    min_l_m: float
    max_l_m: float


DEFAULT_ROAD_CONFIG = RoadConfig(
    min_l_m=-3.0,
    max_l_m=3.0,
)


NARROW_ROAD_CONFIG = RoadConfig(
    min_l_m=-2.0,
    max_l_m=2.0,
)