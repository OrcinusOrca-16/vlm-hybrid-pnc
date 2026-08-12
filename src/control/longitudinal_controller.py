"""Longitudinal vehicle controller."""

from src.control.pid import PIDController


class LongitudinalController:
    """Longitudinal controller with feedforward and PID feedback."""

    def __init__(
        self,
        pid: PIDController,
        dt: float,
        max_acceleration: float = 2.0,
        max_deceleration: float = 3.0,
    ):
        if dt <= 0.0:
            raise ValueError("dt must be positive.")

        if max_acceleration <= 0.0:
            raise ValueError("max_acceleration must be positive.")

        if max_deceleration <= 0.0:
            raise ValueError("max_deceleration must be positive.")

        self.pid = pid
        self.dt = dt
        self.max_acceleration = max_acceleration
        self.max_deceleration = max_deceleration

    def compute_acceleration(
        self,
        current_speed: float,
        reference_speed: float,
        next_reference_speed: float | None = None,
    ) -> float:
        """Compute longitudinal acceleration command."""

        # Feedback:
        #
        #   e_v = v_ref - v
        speed_error = reference_speed - current_speed

        feedback_acceleration = self.pid.update(
            error=speed_error,
            dt=self.dt,
        )

        # Feedforward:
        #
        #   a_ref = (v_ref[k+1] - v_ref[k]) / dt
        if next_reference_speed is None:
            reference_acceleration = 0.0
        else:
            reference_acceleration = (
                next_reference_speed
                - reference_speed
            ) / self.dt

        # Feedforward + feedback:
        #
        #   a_cmd = a_ref + a_fb
        raw_acceleration = (
            reference_acceleration
            + feedback_acceleration
        )

        return max(
            -self.max_deceleration,
            min(
                raw_acceleration,
                self.max_acceleration,
            ),
        )