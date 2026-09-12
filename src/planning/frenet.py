import math

from src.common.types import (
    FrenetPoint,
    FrenetTrajectoryPoint,
    FrenetTrajectory,
    PathPoint,
)
from src.planning.reference_line import ReferenceLine

_FRENET_GEOMETRY_EPSILON = 1e-8

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


def frenet_to_cartesian_geometry(
    frenet_point: FrenetTrajectoryPoint,
    reference_line: ReferenceLine,
) -> tuple[float, float, float, float]:
    """Convert one Frenet trajectory point to Cartesian path geometry."""

    reference_point = reference_line.query(
        frenet_point.s
    )

    l = frenet_point.l
    dl_ds = frenet_point.dl_ds
    d2l_ds2 = frenet_point.d2l_ds2

    kappa_r = reference_point.curvature
    dkappa_r_ds = reference_point.curvature_derivative

    # Chapter 11:
    # (s, l) -> (x, y)
    x, y = sl_to_xy(
        s=frenet_point.s,
        l=l,
        reference_line=reference_line,
    )

    # Tangential component of P'.
    tangent_scale = 1.0 - kappa_r * l

    # Candidate yaw.
    yaw = (
        reference_point.yaw
        + math.atan2(
            dl_ds,
            tangent_scale,
        )
    )

    # Normalize yaw to [-pi, pi].
    yaw = math.atan2(
        math.sin(yaw),
        math.cos(yaw),
    )

    # ||P'||^2
    p_prime_norm_squared = (
        tangent_scale ** 2
        + dl_ds ** 2
    )

    if p_prime_norm_squared < _FRENET_GEOMETRY_EPSILON:
        raise ValueError(
            "Frenet geometry is singular."
        )

    # Numerator of:
    #
    # kappa = (P' x P'') / ||P'||^3
    curvature_numerator = (
        tangent_scale
        * (
            kappa_r * tangent_scale
            + d2l_ds2
        )
        + dl_ds
        * (
            dkappa_r_ds * l
            + 2.0 * kappa_r * dl_ds
        )
    )

    curvature = (
        curvature_numerator
        / p_prime_norm_squared ** 1.5
    )

    return x, y, yaw, curvature

def frenet_trajectory_to_path(
    trajectory: FrenetTrajectory,
    reference_line: ReferenceLine,
) -> list[PathPoint]:
    """Convert one Frenet candidate trajectory to a Cartesian spatial path."""

    if not trajectory.points:
        raise ValueError(
            "Frenet trajectory must contain at least one point."
        )

    path_points = []
    path_s = 0.0

    for frenet_point in trajectory.points:
        x, y, yaw, curvature = frenet_to_cartesian_geometry(
            frenet_point=frenet_point,
            reference_line=reference_line,
        )

        if path_points:
            previous_point = path_points[-1]

            path_s += math.hypot(
                x - previous_point.x,
                y - previous_point.y,
            )

        path_points.append(
            PathPoint(
                x=x,
                y=y,
                s=path_s,
                yaw=yaw,
                curvature=curvature,
            )
        )

    return path_points