import math

from src.common.types import (
    FrenetPoint,
    FrenetTrajectoryPoint,
    FrenetTrajectory,
    PathPoint,
    VehicleState,
)
from src.planning.reference_line import ReferenceLine

_FRENET_GEOMETRY_EPSILON = 1e-8
_FRENET_HEADING_EPSILON = 1e-8

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


def cartesian_to_frenet_geometry(
    x: float,
    y: float,
    yaw: float,
    curvature: float,
    reference_line: ReferenceLine,
) -> FrenetTrajectoryPoint:
    """Convert Cartesian path geometry to Frenet path geometry."""

    frenet_point = xy_to_sl(
        x=x,
        y=y,
        reference_line=reference_line,
    )

    reference_point = reference_line.query(
        frenet_point.s
    )

    l = frenet_point.l

    heading_error = (
        yaw
        - reference_point.yaw
    )

    heading_error = math.atan2(
        math.sin(heading_error),
        math.cos(heading_error),
    )

    cos_heading_error = math.cos(
        heading_error
    )

    if abs(cos_heading_error) < _FRENET_HEADING_EPSILON:
        raise ValueError(
            "Cartesian heading is singular in the Frenet frame."
        )

    tangent_scale = (
        1.0
        - reference_point.curvature * l
    )

    dl_ds = (
        tangent_scale
        * math.tan(heading_error)
    )

    d2l_ds2 = (
        -(
            reference_point.curvature_derivative * l
            + reference_point.curvature * dl_ds
        )
        * math.tan(heading_error)
        + (
            tangent_scale
            / cos_heading_error ** 2
        )
        * (
            curvature
            * tangent_scale
            / cos_heading_error
            - reference_point.curvature
        )
    )

    return FrenetTrajectoryPoint(
        s=frenet_point.s,
        l=l,
        dl_ds=dl_ds,
        d2l_ds2=d2l_ds2,
    )


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


def frenet_trajectories_to_paths(
    trajectories: list[FrenetTrajectory],
    reference_line: ReferenceLine,
) -> list[list[PathPoint]]:
    """Convert multiple Frenet candidate trajectories to Cartesian paths."""

    return [
        frenet_trajectory_to_path(
            trajectory=trajectory,
            reference_line=reference_line,
        )
        for trajectory in trajectories
    ]


def vehicle_state_to_frenet(
    vehicle_state: VehicleState,
    reference_line: ReferenceLine,
) -> FrenetTrajectoryPoint:
    """Convert ego vehicle state to the initial Frenet path state."""

    return cartesian_to_frenet_geometry(
        x=vehicle_state.x,
        y=vehicle_state.y,
        yaw=vehicle_state.yaw,
        curvature=vehicle_state.curvature,
        reference_line=reference_line,
    )