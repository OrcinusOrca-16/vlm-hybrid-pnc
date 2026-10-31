import math
from dataclasses import dataclass

import numpy as np

from src.control.lateral_mpc import (
    MPCController,
    build_cost_matrices,
    build_prediction_matrices,
    build_qp_cost,
    solve_qp,
)
from src.control.path_tracking import curvature_to_steering


@dataclass(frozen=True)
class CurvatureAwareSteeringResult:
    """Executed total steering and its feedforward/feedback components."""

    steering_angle: float
    feedforward_angle: float
    feedback_angle: float


def build_curvature_aware_discrete_model(
    speed_mps: float,
    reference_curvature: float,
    wheelbase_m: float,
    dt: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Linearize Frenet tracking-error dynamics around curved-path feedforward.

    State:
        x = [e_y, e_yaw]^T

    QP input:
        delta_correction

    Total steering:
        delta = delta_ff + delta_correction

    with:
        delta_ff = atan(L * kappa_ref)

    The local continuous error dynamics are linearized as:

        e_y_dot   ~= v * e_yaw
        e_yaw_dot ~= -v * kappa_ref^2 * e_y
                     + v/L * sec(delta_ff)^2 * delta_correction
    """

    if speed_mps < 0.0:
        raise ValueError("speed_mps must not be negative.")
    if wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive.")
    if dt <= 0.0:
        raise ValueError("dt must be positive.")
    if not math.isfinite(reference_curvature):
        raise ValueError("reference_curvature must be finite.")

    steering_feedforward = curvature_to_steering(curvature_per_m=reference_curvature, wheelbase_m=wheelbase_m)
    cos_feedforward = math.cos(steering_feedforward)
    if abs(cos_feedforward) < 1e-9:
        raise ValueError("Feedforward steering is too close to pi/2.")

    A = np.array([
        [1.0, speed_mps * dt],
        [-speed_mps * reference_curvature**2 * dt, 1.0],
    ])
    B = np.array([
        [0.0],
        [speed_mps * dt / wheelbase_m / cos_feedforward**2],
    ])
    return A, B


def build_curvature_aware_input_constraints(
    horizon: int,
    previous_steering_angle: float,
    steering_feedforward: float,
    min_steering_angle: float,
    max_steering_angle: float,
    max_steering_rate: float,
    dt: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Constrain total steering while optimizing steering correction.

    Reference curvature is treated as constant over the current prediction
    horizon. Therefore feedforward cancels from predicted steering-rate
    differences after the first input. The first input is constrained against
    the previously executed total steering angle.
    """

    if not isinstance(horizon, int) or horizon <= 0:
        raise ValueError("horizon must be a positive integer.")
    if min_steering_angle >= max_steering_angle:
        raise ValueError("min_steering_angle must be smaller than max_steering_angle.")
    if max_steering_rate <= 0.0:
        raise ValueError("max_steering_rate must be positive.")
    if dt <= 0.0:
        raise ValueError("dt must be positive.")
    if not math.isfinite(steering_feedforward):
        raise ValueError("steering_feedforward must be finite.")
    if not min_steering_angle <= previous_steering_angle <= max_steering_angle:
        raise ValueError("previous_steering_angle must be within steering limits.")

    angle_matrix = np.eye(horizon)
    angle_lower = np.full(horizon, min_steering_angle - steering_feedforward)
    angle_upper = np.full(horizon, max_steering_angle - steering_feedforward)

    rate_matrix = np.eye(horizon)
    for index in range(1, horizon):
        rate_matrix[index, index - 1] = -1.0

    max_delta_change = max_steering_rate * dt
    rate_lower = np.full(horizon, -max_delta_change)
    rate_upper = np.full(horizon, max_delta_change)

    previous_correction = previous_steering_angle - steering_feedforward
    rate_lower[0] = previous_correction - max_delta_change
    rate_upper[0] = previous_correction + max_delta_change

    constraint_matrix = np.vstack([angle_matrix, rate_matrix])
    lower_bound = np.concatenate([angle_lower, rate_lower])
    upper_bound = np.concatenate([angle_upper, rate_upper])
    return constraint_matrix, lower_bound, upper_bound


class CurvatureAwareMPCController(MPCController):
    """Lateral MPC with kinematic steering feedforward for curved references."""

    def compute_steering_components(
        self,
        lateral_error: float,
        heading_error: float,
        speed_mps: float,
        reference_curvature: float,
    ) -> CurvatureAwareSteeringResult:
        """Calculate constrained total steering and expose FF/FB components."""

        if speed_mps < 0.0:
            raise ValueError("speed_mps must not be negative.")
        if not math.isfinite(reference_curvature):
            raise ValueError("reference_curvature must be finite.")

        steering_feedforward = curvature_to_steering(curvature_per_m=reference_curvature, wheelbase_m=self.wheelbase_m)
        if not self.min_steering_angle <= steering_feedforward <= self.max_steering_angle:
            raise ValueError("Reference curvature requires feedforward steering outside the configured steering limit.")

        state = np.array([lateral_error, heading_error])
        A, B = build_curvature_aware_discrete_model(
            speed_mps=speed_mps,
            reference_curvature=reference_curvature,
            wheelbase_m=self.wheelbase_m,
            dt=self.dt,
        )
        A_bar, B_bar = build_prediction_matrices(A=A, B=B, horizon=self.horizon)
        Q_bar, R_bar = build_cost_matrices(Q=self.Q, R=self.R, horizon=self.horizon)
        H, f = build_qp_cost(A_bar=A_bar, B_bar=B_bar, Q_bar=Q_bar, R_bar=R_bar, state=state)

        constraint_matrix, lower_bound, upper_bound = build_curvature_aware_input_constraints(
            horizon=self.horizon,
            previous_steering_angle=self.previous_steering_angle,
            steering_feedforward=steering_feedforward,
            min_steering_angle=self.min_steering_angle,
            max_steering_angle=self.max_steering_angle,
            max_steering_rate=self.max_steering_rate,
            dt=self.dt,
        )
        optimal_correction = solve_qp(
            H=H,
            f=f,
            constraint_matrix=constraint_matrix,
            lower_bound=lower_bound,
            upper_bound=upper_bound,
        )

        steering = steering_feedforward + float(optimal_correction[0])
        steering = max(self.min_steering_angle, min(steering, self.max_steering_angle))
        feedback = steering - steering_feedforward
        self.previous_steering_angle = steering

        return CurvatureAwareSteeringResult(
            steering_angle=steering,
            feedforward_angle=steering_feedforward,
            feedback_angle=feedback,
        )

    def compute_steering(
        self,
        lateral_error: float,
        heading_error: float,
        speed_mps: float,
        reference_curvature: float,
    ) -> float:
        """Calculate the constrained total steering command."""

        result = self.compute_steering_components(
            lateral_error=lateral_error,
            heading_error=heading_error,
            speed_mps=speed_mps,
            reference_curvature=reference_curvature,
        )
        return result.steering_angle
