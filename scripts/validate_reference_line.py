"""Validate reference-line construction and interpolation."""

import math

from src.planning.reference_line import (
    build_reference_line,
    get_reference_point_by_s,
)


def validate_straight_line() -> None:
    """Validate a straight reference line."""

    points = [
        (0.0, 0.0),
        (2.0, 0.0),
        (4.0, 0.0),
        (6.0, 0.0),
    ]

    reference_line = build_reference_line(points)

    point = get_reference_point_by_s(
        reference_line,
        s_query=3.2,
    )

    print("Straight line:")
    print(point)

    assert abs(point.x - 3.2) < 1e-6
    assert abs(point.y) < 1e-6
    assert abs(point.yaw) < 1e-6
    assert abs(point.curvature) < 1e-6


def validate_circular_arc() -> None:
    """Validate a circular reference line."""

    radius = 10.0

    angles = [
        0.0,
        0.1,
        0.2,
        0.3,
        0.4,
        0.5,
        0.6,
        0.7,
        0.8,
    ]

    points = [
        (
            radius * math.sin(theta),
            radius * (1.0 - math.cos(theta)),
        )
        for theta in angles
    ]

    reference_line = build_reference_line(points)

    # Query an interior point to avoid endpoint
    # curvature approximation effects.
    s_query = reference_line[4].s

    point = get_reference_point_by_s(
        reference_line,
        s_query=s_query,
    )

    theoretical_curvature = 1.0 / radius

    print("\nCircular arc:")
    print(point)
    print(
        "Theoretical curvature:",
        theoretical_curvature,
    )

    assert abs(
        point.curvature - theoretical_curvature
    ) < 0.01


def main() -> None:
    validate_straight_line()
    validate_circular_arc()

    print("\nReference line validation passed.")


if __name__ == "__main__":
    main()