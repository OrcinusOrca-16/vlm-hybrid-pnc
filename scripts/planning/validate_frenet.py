"""Validate Cartesian <-> Frenet / SL coordinate conversion."""

import math

from src.planning.frenet import (
    sl_to_xy,
    xy_to_sl,
)
from src.planning.reference_line import build_reference_line


TOLERANCE = 1e-9


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
    """Validate Frenet conversion on a straight reference line."""

    reference_line = build_reference_line([
        (0.0, 0.0),
        (10.0, 0.0),
        (20.0, 0.0),
    ])

    test_cases = [
        (14.0, 3.0),
        (14.0, -3.0),
        (5.0, 0.0),
    ]

    for x, y in test_cases:
        frenet_point = xy_to_sl(
            x=x,
            y=y,
            reference_line=reference_line,
        )

        recovered_x, recovered_y = sl_to_xy(
            s=frenet_point.s,
            l=frenet_point.l,
            reference_line=reference_line,
        )

        assert_close(
            recovered_x,
            x,
            "straight x",
        )

        assert_close(
            recovered_y,
            y,
            "straight y",
        )

    print("Straight-line Frenet validation passed.")


def validate_curved_line() -> None:
    """Validate Frenet conversion on a curved reference line."""

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

    reference_line = build_reference_line(
        points
    )

    test_s = (
        reference_line[1].s
        + 0.4
        * (
            reference_line[2].s
            - reference_line[1].s
        )
    )

    test_l = 1.0

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
        "curved s",
    )

    assert_close(
        recovered.l,
        test_l,
        "curved l",
    )

    print("Curved-line Frenet validation passed.")


def main() -> None:
    validate_straight_line()
    validate_curved_line()

    print(
        "\nAll Frenet coordinate validation cases passed."
    )


if __name__ == "__main__":
    main()