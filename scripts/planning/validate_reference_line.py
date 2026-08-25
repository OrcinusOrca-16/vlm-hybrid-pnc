"""Validate reference-line construction and interpolation."""

import math

from src.planning.reference_line import (
    build_reference_line,
    get_reference_point_by_s,
)

from src.planning.reference_line import (
    ReferenceLine,
    build_reference_line,
    get_reference_point_by_s,
    compute_cumulative_s,
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

def validate_smooth_reference_line() -> None:
    """Validate spline-based reference-line position query."""

    points = [
        (0.0, 0.0),
        (2.0, 0.0),
        (4.0, 0.0),
        (6.0, 0.0),
    ]

    reference_line = ReferenceLine(points)
    x, y = reference_line.position(3.2)
    yaw = reference_line.yaw(3.2)
    curvature = reference_line.curvature(3.2)
    curvature_derivative = (reference_line.curvature_derivative(3.2))
    point = reference_line.query(3.2)

    print("\nSmooth reference line:")
    print("x:", x)
    print("y:", y)
    print("yaw:", yaw)
    print("curvature:", curvature)
    print("curvature derivative:", curvature_derivative)
    print("length:", reference_line.length)
    print("query point:", point)

    assert abs(x - 3.2) < 1e-6
    assert abs(y) < 1e-6
    assert abs(reference_line.length - 6.0) < 1e-6
    assert abs(yaw) < 1e-6
    assert abs(curvature) < 1e-6
    assert abs(curvature_derivative) < 1e-6
    assert abs(point.x - 3.2) < 1e-6
    assert abs(point.y) < 1e-6
    assert abs(point.yaw) < 1e-6
    assert abs(point.curvature) < 1e-6
    assert abs(point.curvature_derivative) < 1e-6

def validate_smooth_circular_arc() -> None:
    """Validate spline position query on a circular arc."""

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

    reference_line = ReferenceLine(points)
    cumulative_s = compute_cumulative_s(points)
    s_query = 0.5 * (
        cumulative_s[3]
        + cumulative_s[4]
    )
    
    x, y = reference_line.position(s_query)
    yaw = reference_line.yaw(s_query)
    curvature = reference_line.curvature(s_query)
    curvature_derivative = (reference_line.curvature_derivative(s_query))

    theta_query = 0.35

    expected_x = radius * math.sin(theta_query)
    expected_y = radius * (1.0 - math.cos(theta_query))
    expected_yaw = theta_query
    expected_curvature = 1.0 / radius

    print("\nSmooth circular arc:")
    print("x:", x)
    print("y:", y)
    print("expected x:", expected_x)
    print("expected y:", expected_y)
    print("yaw:", yaw)
    print("expected yaw:", expected_yaw)
    print("curvature:", curvature)
    print("expected curvature:", expected_curvature)
    print("curvature derivative:",curvature_derivative,)

    assert abs(x - expected_x) < 1e-3
    assert abs(y - expected_y) < 1e-3
    assert abs(yaw - expected_yaw) < 1e-3
    assert abs(curvature - expected_curvature) < 1e-3
    assert abs(curvature_derivative) < 1e-3

def validate_smooth_curved_path() -> None:
    """Validate a smooth reference line with varying curvature."""

    points = [
        (0.0, 0.0),
        (5.0, 0.5),
        (10.0, 2.0),
        (15.0, 3.0),
        (20.0, 2.0),
        (25.0, 0.5),
        (30.0, 0.0),
    ]

    reference_line = ReferenceLine(points)

    sample_s = [
        reference_line.length * 0.25,
        reference_line.length * 0.50,
        reference_line.length * 0.75,
    ]

    curvatures = []
    curvature_derivatives = []

    for s in sample_s:
        point = reference_line.query(s)

        print("\nSmooth curved path:")
        print(point)

        assert math.isfinite(point.x)
        assert math.isfinite(point.y)
        assert math.isfinite(point.yaw)
        assert math.isfinite(point.curvature)
        assert math.isfinite(point.curvature_derivative)

        curvatures.append(point.curvature)
        curvature_derivatives.append(
            point.curvature_derivative
        )

    # Unlike a straight line or circular arc,
    # curvature should vary along this path.
    assert max(curvatures) - min(curvatures) > 1e-3

    # A varying-curvature path should have
    # non-zero curvature derivative somewhere.
    assert any(
        abs(value) > 1e-4
        for value in curvature_derivatives
    )

def main() -> None:
    validate_straight_line()
    validate_circular_arc()
    validate_smooth_reference_line()
    validate_smooth_circular_arc()
    validate_smooth_curved_path()

    print("\nReference line validation passed.")


if __name__ == "__main__":
    main()