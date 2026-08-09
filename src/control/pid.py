class PIDController:
    """Discrete PID controller."""

    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        min_output: float = -3.0,
        max_output: float = 2.0,
    ):
        if min_output >= max_output:
            raise ValueError("min_output must be smaller than max_output.")

        self.kp = kp
        self.ki = ki
        self.kd = kd

        self.min_output = min_output
        self.max_output = max_output

        self.integral = 0.0
        self.previous_error: float | None = None

    def update(
        self,
        error: float,
        dt: float,
    ) -> float:
        """Calculate the control output for one time step."""

        if dt <= 0.0:
            raise ValueError("dt must be positive.")

        # PID:
        #   u = Kp * e + Ki * integral(e dt) + Kd * de/dt
        #
        # Discrete approximation:
        #   integral_k = integral_(k-1) + e_k * dt
        #   derivative = (e_k - e_(k-1)) / dt

        candidate_integral = self.integral + error * dt

        if self.previous_error is None:
            derivative = 0.0
        else:
            derivative = (error - self.previous_error) / dt

        candidate_output = (
            self.kp * error
            + self.ki * candidate_integral
            + self.kd * derivative
        )

        upper_saturated = candidate_output > self.max_output
        lower_saturated = candidate_output < self.min_output

        if not (
            (upper_saturated and error > 0.0)
            or (lower_saturated and error < 0.0)
        ):
            self.integral = candidate_integral

        raw_output = (
            self.kp * error
            + self.ki * self.integral
            + self.kd * derivative
        )

        output = max(
            self.min_output,
            min(raw_output, self.max_output),
        )

        self.previous_error = error

        return output