import math
from src.common.types import ReferencePoint

def compute_cumulative_s(
    points: list[tuple[float, float]],
) -> list[float]:
    """Compute cumulative arc length along discrete 2D points."""

    if len(points) < 2:
        raise ValueError("At least two points are required.")

    cumulative_s = [0.0]

    for i in range(1, len(points)):
        x_previous, y_previous = points[i - 1]
        x_current, y_current = points[i]

        dx = x_current - x_previous
        dy = y_current - y_previous

        ds = math.sqrt(
            dx ** 2 + dy ** 2
        )

        if ds <= 0.0:
            raise ValueError(
                "Adjacent reference-line points must be different."
            )

        cumulative_s.append(
            cumulative_s[-1] + ds
        )

    return cumulative_s


def compute_yaws(
    points: list[tuple[float, float]],
) -> list[float]:
    """Compute heading angles along discrete 2D points."""

    if len(points) < 2:
        raise ValueError("At least two points are required.")

    yaws = []

    for i in range(len(points) - 1):
        x_current, y_current = points[i]
        x_next, y_next = points[i + 1]

        dx = x_next - x_current
        dy = y_next - y_current

        if dx == 0.0 and dy == 0.0:
            raise ValueError(
                "Adjacent reference-line points must be different."
            )

        yaw = math.atan2(dy, dx)
        yaws.append(yaw)

    # The last point uses the direction of the last segment.
    yaws.append(yaws[-1])

    return yaws


def normalize_angle(angle: float) -> float:
    """Normalize an angle to [-pi, pi]."""

    return math.atan2(
        math.sin(angle),
        math.cos(angle),
    )


def compute_curvatures(
    cumulative_s: list[float],
    yaws: list[float],
) -> list[float]:
    """Compute curvature along a discrete reference line."""

    if len(cumulative_s) != len(yaws):
        raise ValueError(
            "cumulative_s and yaws must have the same length."
        )

    if len(yaws) < 3:
        raise ValueError(
            "At least three points are required to compute curvature."
        )

    curvatures = [0.0] * len(yaws)

    for i in range(1, len(yaws) - 1):
        delta_yaw = normalize_angle(
            yaws[i + 1] - yaws[i - 1]
        )

        delta_s = (
            cumulative_s[i + 1]
            - cumulative_s[i - 1]
        )

        if delta_s <= 0.0:
            raise ValueError(
                "cumulative_s must be strictly increasing."
            )

        curvatures[i] = delta_yaw / delta_s

    # Baseline treatment for endpoints:
    # use the nearest interior curvature.
    curvatures[0] = curvatures[1]
    curvatures[-1] = curvatures[-2]

    return curvatures


def build_reference_line(
    points: list[tuple[float, float]],
) -> list[ReferencePoint]:
    """Build a reference line from discrete Cartesian points."""

    cumulative_s = compute_cumulative_s(points)
    yaws = compute_yaws(points)
    curvatures = compute_curvatures(
        cumulative_s,
        yaws,
    )

    reference_line = []

    for i, (x, y) in enumerate(points):
        reference_point = ReferencePoint(
            x=x,
            y=y,
            s=cumulative_s[i],
            yaw=yaws[i],
            curvature=curvatures[i],
        )

        reference_line.append(reference_point)

    return reference_line


def interpolate_reference_point(
    point0: ReferencePoint,
    point1: ReferencePoint,
    s_query: float,
) -> ReferencePoint:
    """Interpolate a reference point between two adjacent points."""

    if point1.s <= point0.s:
        raise ValueError(
            "point1.s must be greater than point0.s."
        )

    if not point0.s <= s_query <= point1.s:
        raise ValueError(
            "s_query must lie between point0.s and point1.s."
        )

    ratio = (
        (s_query - point0.s)
        / (point1.s - point0.s)
    )

    x = point0.x + ratio * (
        point1.x - point0.x
    )

    y = point0.y + ratio * (
        point1.y - point0.y
    )

    delta_yaw = normalize_angle(
        point1.yaw - point0.yaw
    )

    yaw = normalize_angle(
        point0.yaw
        + ratio * delta_yaw
    )

    curvature = point0.curvature + ratio * (
        point1.curvature
        - point0.curvature
    )

    return ReferencePoint(
        x=x,
        y=y,
        s=s_query,
        yaw=yaw,
        curvature=curvature,
    )


def get_reference_point_by_s(
    reference_line: list[ReferencePoint],
    s_query: float,
) -> ReferencePoint:
    """Query an interpolated reference point by arc length s."""

    if len(reference_line) < 2:
        raise ValueError(
            "Reference line must contain at least two points."
        )

    if (
        s_query < reference_line[0].s
        or s_query > reference_line[-1].s
    ):
        raise ValueError(
            "s_query is outside the reference line range."
        )

    for i in range(1, len(reference_line)):
        point0 = reference_line[i - 1]
        point1 = reference_line[i]

        if s_query <= point1.s:
            return interpolate_reference_point(
                point0=point0,
                point1=point1,
                s_query=s_query,
            )

    raise RuntimeError(
        "Failed to find reference point."
    )