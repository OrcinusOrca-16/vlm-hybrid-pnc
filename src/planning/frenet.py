import math

from src.common.types import FrenetPoint
from src.planning.reference_line import ReferenceLine


def xy_to_sl(
    x: float,
    y: float,
    reference_line: ReferenceLine,
) -> FrenetPoint:
    """Convert one Cartesian point to Frenet / SL coordinates."""

    s = reference_line.nearest_s(
        x=x,
        y=y,
    )
    reference_x, reference_y = reference_line.position(s)
    reference_yaw = reference_line.yaw(s)

    normal_x = -math.sin(reference_yaw)
    normal_y = math.cos(reference_yaw)

    error_x = x - reference_x
    error_y = y - reference_y

    l = (
        error_x * normal_x
        + error_y * normal_y
    )

    return FrenetPoint(
        s=s,
        l=l,
    )


def sl_to_xy(
    s: float,
    l: float,
    reference_line: ReferenceLine,
) -> tuple[float, float]:
    """Convert one Frenet / SL point to Cartesian coordinates."""

    reference_x, reference_y = reference_line.position(s)
    reference_yaw = reference_line.yaw(s)

    normal_x = -math.sin(reference_yaw)
    normal_y = math.cos(reference_yaw)

    x = reference_x + l * normal_x
    y = reference_y + l * normal_y

    return x, y
