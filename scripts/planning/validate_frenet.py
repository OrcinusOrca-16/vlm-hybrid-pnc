"""Validate Cartesian <-> Frenet / SL coordinate conversion."""

import math

from src.planning.frenet import (
    sl_to_xy,
    xy_to_sl,
)
from src.planning.reference_line import ReferenceLine


TOLERANCE = 5e-3


def assert_close(
    actual: float,
    expected: float,
    name: str,
) -> None:
    if abs(actual - expected) > TOLERANCE:
        raise AssertionError(
            f"{name}: expected {expected}, got {actual}"
        )


def validate_straight_line() -> None:
    """Validate spline-based Frenet conversion on a straight line."""

    reference_line = ReferenceLine([
        (0.0, 0.0),
        (10.0, 0.0),
        (20.0, 0.0),
    ])

    test_cases = [
        (14.0, 3.0, 14.0, 3.0),
        (14.0, -3.0, 14.0, -3.0),
        (5.0, 0.0, 5.0, 0.0),
    ]

    for x, y, expected_s, expected_l in test_cases:
        frenet_point = xy_to_sl(
            x=x,
            y=y,
            reference_line=reference_line,
        )

        assert_close(
            frenet_point.s,
            expected_s,
            "straight s",
        )
        assert_close(
            frenet_point.l,
            expected_l,
            "straight l",
        )

        recovered_x, recovered_y = sl_to_xy(
            s=frenet_point.s,
            l=frenet_point.l,
            reference_line=reference_line,
        )

        assert_close(
            recovered_x,
            x,
            "straight round-trip x",
        )
        assert_close(
            recovered_y,
            y,
            "straight round-trip y",
        )

    print("Straight-line spline Frenet validation passed.")


def validate_endpoint_projection() -> None:
    """Validate projection when the nearest point is a spline endpoint."""

    reference_line = ReferenceLine([
        (0.0, 0.0),
        (5.0, 0.0),
        (10.0, 0.0),
    ])

    frenet_point = xy_to_sl(
        x=-2.0,
        y=1.5,
        reference_line=reference_line,
    )

    assert_close(
        frenet_point.s,
        0.0,
        "endpoint s",
    )
    assert_close(
        frenet_point.l,
        1.5,
        "endpoint l",
    )

    print("Endpoint spline projection validation passed.")


def validate_circular_arc() -> None:
    """Validate spline-based Frenet round-trip on a circular arc."""

    radius = 10.0

    angles = [
        0.0,
        math.pi / 12,
        math.pi / 6,
        math.pi / 4,
        math.pi / 3,
    ]

    points = [
        (
            radius * math.cos(angle),
            radius * math.sin(angle),
        )
        for angle in angles
    ]

    reference_line = ReferenceLine(points)
    test_s = reference_line.length * 0.42

    for test_l in (1.0, -1.0):
        x, y = sl_to_xy(
            s=test_s,
            l=test_l,
            reference_line=reference_line,
        )

        recovered = xy_to_sl(
            x=x,
            y=y,
            reference_line=reference_line,
        )

        assert_close(
            recovered.s,
            test_s,
            "circular round-trip s",
        )
        assert_close(
            recovered.l,
            test_l,
            "circular round-trip l",
        )

    print("Circular-arc spline Frenet validation passed.")


def validate_varying_curvature_path() -> None:
    """Validate spline Frenet round-trip on a varying-curvature path."""

    reference_line = ReferenceLine([
        (0.0, 0.0),
        (5.0, 0.5),
        (10.0, 2.0),
        (15.0, 3.0),
        (20.0, 2.0),
        (25.0, 0.5),
        (30.0, 0.0),
    ])

    test_cases = [
        (0.22, 0.8),
        (0.48, -1.2),
        (0.73, 1.5),
    ]

    for fraction, test_l in test_cases:
        test_s = reference_line.length * fraction
        x, y = sl_to_xy(
            s=test_s,
            l=test_l,
            reference_line=reference_line,
        )

        recovered = xy_to_sl(
            x=x,
            y=y,
            reference_line=reference_line,
        )

        assert_close(
            recovered.s,
            test_s,
            "varying-curvature round-trip s",
        )
        assert_close(
            recovered.l,
            test_l,
            "varying-curvature round-trip l",
        )

    print("Varying-curvature spline Frenet validation passed.")


def main() -> None:
    validate_straight_line()
    validate_endpoint_projection()
    validate_circular_arc()
    validate_varying_curvature_path()

    print(
        "\nAll spline Frenet coordinate validation cases passed."
    )


if __name__ == "__main__":
    main()
