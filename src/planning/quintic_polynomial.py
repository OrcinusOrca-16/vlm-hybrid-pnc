import numpy as np


class QuinticPolynomial:
    """Quintic polynomial defined by start and end boundary conditions."""

    def __init__(
        self,
        start_l: float,
        start_dl_ds: float,
        start_d2l_ds2: float,
        end_l: float,
        end_dl_ds: float,
        end_d2l_ds2: float,
        length: float,
    ) -> None:
        if length <= 0.0:
            raise ValueError(
                "Polynomial length must be positive."
            )

        self.length = length

        self.a0 = start_l
        self.a1 = start_dl_ds
        self.a2 = 0.5 * start_d2l_ds2

        s = length

        matrix = np.array([
            [s ** 3,      s ** 4,       s ** 5],
            [3.0 * s ** 2, 4.0 * s ** 3, 5.0 * s ** 4],
            [6.0 * s,    12.0 * s ** 2, 20.0 * s ** 3],
        ])

        target = np.array([
            end_l
            - (
                self.a0
                + self.a1 * s
                + self.a2 * s ** 2
            ),
            end_dl_ds
            - (
                self.a1
                + 2.0 * self.a2 * s
            ),
            end_d2l_ds2
            - 2.0 * self.a2,
        ])

        self.a3, self.a4, self.a5 = np.linalg.solve(
            matrix,
            target,
        )

    def position(
        self,
        s: float,
    ) -> float:
        """Return lateral position l(s)."""

        return (
            self.a0
            + self.a1 * s
            + self.a2 * s ** 2
            + self.a3 * s ** 3
            + self.a4 * s ** 4
            + self.a5 * s ** 5
        )

    def first_derivative(
        self,
        s: float,
    ) -> float:
        """Return dl/ds."""

        return (
            self.a1
            + 2.0 * self.a2 * s
            + 3.0 * self.a3 * s ** 2
            + 4.0 * self.a4 * s ** 3
            + 5.0 * self.a5 * s ** 4
        )

    def second_derivative(
        self,
        s: float,
    ) -> float:
        """Return d2l/ds2."""

        return (
            2.0 * self.a2
            + 6.0 * self.a3 * s
            + 12.0 * self.a4 * s ** 2
            + 20.0 * self.a5 * s ** 3
        )