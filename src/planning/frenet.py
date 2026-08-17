from src.common.types import FrenetPoint, ReferencePoint
import math

def project_point_to_segment(
    x: float,
    y: float,
    point0: ReferencePoint,
    point1: ReferencePoint,
) -> tuple[float, float, float, float]:
    """Project a Cartesian point onto one reference-line segment."""

    segment_x = point1.x - point0.x
    segment_y = point1.y - point0.y

    segment_length_squared = (
        segment_x ** 2
        + segment_y ** 2
    )

    if segment_length_squared <= 0.0:
        raise ValueError(
            "Reference-line segment must have non-zero length."
        )

    relative_x = x - point0.x
    relative_y = y - point0.y
 
    # Projection ratio along the infinite line:
    #
    # t = (P - A) · (B - A) / |B - A|^2
    ratio = (
        relative_x * segment_x
        + relative_y * segment_y
    ) / segment_length_squared

    # Restrict the projection to the finite segment.
    ratio = max(
        0.0,
        min(ratio, 1.0),
    )

    projection_x = (
        point0.x
        + ratio * segment_x
    )

    projection_y = (
        point0.y
        + ratio * segment_y
    )

    error_x = x - projection_x
    error_y = y - projection_y

    distance_squared = (
        error_x ** 2
        + error_y ** 2
    )

    return (
        projection_x,
        projection_y,
        ratio,
        distance_squared,
    )


def find_closest_projection(
    x: float,
    y: float,
    reference_line: list[ReferencePoint],
) -> tuple[int, float, float, float, float]:
    """Find the closest projection on the whole reference line."""

    if len(reference_line) < 2:
        raise ValueError(
            "Reference line must contain at least two points."
        )

    best_segment_index = -1
    best_projection_x = 0.0
    best_projection_y = 0.0
    best_ratio = 0.0
    best_distance_squared = float("inf")

    for i in range(len(reference_line) - 1):
        point0 = reference_line[i]
        point1 = reference_line[i + 1]

        (
            projection_x,
            projection_y,
            ratio,
            distance_squared,
        ) = project_point_to_segment(
            x=x,
            y=y,
            point0=point0,
            point1=point1,
        )

        if distance_squared < best_distance_squared:
            best_segment_index = i
            best_projection_x = projection_x
            best_projection_y = projection_y
            best_ratio = ratio
            best_distance_squared = distance_squared

    return (
        best_segment_index,
        best_projection_x,
        best_projection_y,
        best_ratio,
        best_distance_squared,
    )


def compute_projected_s(
    point0: ReferencePoint,
    point1: ReferencePoint,
    ratio: float,
) -> float:
    """Compute Frenet longitudinal coordinate s on a segment."""

    if not 0.0 <= ratio <= 1.0:
        raise ValueError(
            "ratio must be within [0, 1]."
        )

    return (
        point0.s
        + ratio * (point1.s - point0.s)
    )


def compute_lateral_offset(
    x: float,
    y: float,
    projection_x: float,
    projection_y: float,
    point0: ReferencePoint,
    point1: ReferencePoint,
) -> float:
    """Compute signed Frenet lateral coordinate l."""

    segment_x = point1.x - point0.x
    segment_y = point1.y - point0.y

    if segment_x == 0.0 and segment_y == 0.0:
        raise ValueError(
            "Reference-line segment must have non-zero length."
        )

    segment_yaw = math.atan2(
        segment_y,
        segment_x,
    )

    # Left normal:
    # n = [-sin(yaw), cos(yaw)]
    normal_x = -math.sin(segment_yaw)
    normal_y = math.cos(segment_yaw)

    error_x = x - projection_x
    error_y = y - projection_y

    l = (
        error_x * normal_x
        + error_y * normal_y
    )

    return l


def xy_to_sl(
    x: float,
    y: float,
    reference_line: list[ReferencePoint],
) -> FrenetPoint:
    """Convert one Cartesian point to Frenet / SL coordinates."""

    (
        segment_index,
        projection_x,
        projection_y,
        ratio,
        _,
    ) = find_closest_projection(
        x=x,
        y=y,
        reference_line=reference_line,
    )

    point0 = reference_line[segment_index]
    point1 = reference_line[segment_index + 1]

    s = compute_projected_s(
        point0=point0,
        point1=point1,
        ratio=ratio,
    )

    l = compute_lateral_offset(
        x=x,
        y=y,
        projection_x=projection_x,
        projection_y=projection_y,
        point0=point0,
        point1=point1,
    )

    return FrenetPoint(
        s=s,
        l=l,
    )