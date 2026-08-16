import numpy as np
import osqp
from scipy import sparse

from src.control.mpc_config import (
    MPC_DT,
    MPC_HORIZON,
    MPC_MAX_STEERING_ANGLE,
    MPC_MAX_STEERING_RATE,
    MPC_Q_HEADING,
    MPC_Q_LATERAL,
    MPC_R_STEERING,
    MPC_WHEELBASE_M,
)


def build_discrete_model(
    speed_mps: float,
    wheelbase_m: float = MPC_WHEELBASE_M,
    dt: float = MPC_DT,
) -> tuple[np.ndarray, np.ndarray]:
    """Build the discrete linear lateral-error model."""

    if speed_mps < 0.0:
        raise ValueError("speed_mps must not be negative.")
    if wheelbase_m <= 0.0:
        raise ValueError("wheelbase_m must be positive.")
    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    # State:
    #   x = [e_y, e_yaw]^T
    #
    # Discrete model:
    #   x_(k+1) = A x_k + B u_k
    #
    #   e_y_(k+1)   = e_y_k + v * dt * e_yaw_k
    #   e_yaw_(k+1) = e_yaw_k + v * dt / L * delta_k

    A = np.array(
        [
            [1.0, speed_mps * dt],
            [0.0, 1.0],
        ]
    )

    B = np.array(
        [
            [0.0],
            [speed_mps * dt / wheelbase_m],
        ]
    )

    return A, B


def build_prediction_matrices(
    A: np.ndarray,
    B: np.ndarray,
    horizon: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Build the stacked MPC prediction matrices."""

    if A.ndim != 2 or A.shape[0] != A.shape[1]:
        raise ValueError("A must be a square matrix.")
    if B.ndim != 2 or B.shape[0] != A.shape[0]:
        raise ValueError("B must have the same number of rows as A.")
    if not isinstance(horizon, int) or horizon <= 0:
        raise ValueError("horizon must be a positive integer.")

    state_dim = A.shape[0]
    input_dim = B.shape[1]

    A_bar = np.zeros(
        (state_dim * horizon, state_dim)
    )

    B_bar = np.zeros(
        (state_dim * horizon, input_dim * horizon)
    )

    for i in range(horizon):
        A_bar[
            i * state_dim : (i + 1) * state_dim,
            :
        ] = np.linalg.matrix_power(A, i + 1)

        for j in range(i + 1):
            B_bar[
                i * state_dim : (i + 1) * state_dim,
                j * input_dim : (j + 1) * input_dim,
            ] = (
                np.linalg.matrix_power(A, i - j)
                @ B
            )

    return A_bar, B_bar


def build_cost_matrices(
    Q: np.ndarray,
    R: np.ndarray,
    horizon: int,
) -> tuple[np.ndarray, np.ndarray]:
    """Build stacked MPC state and input cost matrices."""

    if Q.ndim != 2 or Q.shape[0] != Q.shape[1]:
        raise ValueError("Q must be a square matrix.")
    if R.ndim != 2 or R.shape[0] != R.shape[1]:
        raise ValueError("R must be a square matrix.")
    if not isinstance(horizon, int) or horizon <= 0:
        raise ValueError("horizon must be a positive integer.")

    Q_bar = np.kron(
        np.eye(horizon),
        Q,
    )

    R_bar = np.kron(
        np.eye(horizon),
        R,
    )

    return Q_bar, R_bar


def build_qp_cost(
    A_bar: np.ndarray,
    B_bar: np.ndarray,
    Q_bar: np.ndarray,
    R_bar: np.ndarray,
    state: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """Build the quadratic and linear terms of the MPC QP."""

    if state.ndim != 1:
        raise ValueError("state must be a 1D vector.")
    if A_bar.shape[1] != state.shape[0]:
        raise ValueError("A_bar and state dimensions do not match.")
    if Q_bar.shape[0] != A_bar.shape[0]:
        raise ValueError("Q_bar and A_bar dimensions do not match.")
    if B_bar.shape[0] != A_bar.shape[0]:
        raise ValueError("B_bar and A_bar dimensions do not match.")
    if R_bar.shape[0] != B_bar.shape[1]:
        raise ValueError("R_bar and B_bar dimensions do not match.")

    # Cost:
    #   J = X^T Q_bar X + U^T R_bar U
    #
    # Prediction:
    #   X = A_bar x + B_bar U
    #
    # QP:
    #   min 1/2 U^T H U + f^T U

    H = 2.0 * (
        B_bar.T @ Q_bar @ B_bar
        + R_bar
    )

    f = 2.0 * (
        B_bar.T
        @ Q_bar
        @ A_bar
        @ state
    )

    return H, f


def build_input_constraints(
    horizon: int,
    previous_steering_angle: float = 0.0,
    min_steering_angle: float = -MPC_MAX_STEERING_ANGLE,
    max_steering_angle: float = MPC_MAX_STEERING_ANGLE,
    max_steering_rate: float = MPC_MAX_STEERING_RATE,
    dt: float = MPC_DT,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Build steering-angle and steering-rate constraints."""

    if not isinstance(horizon, int) or horizon <= 0:
        raise ValueError("horizon must be a positive integer.")

    if min_steering_angle >= max_steering_angle:
        raise ValueError(
            "min_steering_angle must be smaller than max_steering_angle."
        )

    if max_steering_rate <= 0.0:
        raise ValueError("max_steering_rate must be positive.")

    if dt <= 0.0:
        raise ValueError("dt must be positive.")

    if not (
        min_steering_angle
        <= previous_steering_angle
        <= max_steering_angle
    ):
        raise ValueError(
            "previous_steering_angle must be within steering limits."
        )

    # Steering-angle constraint:
    #   min_delta <= U <= max_delta
    angle_matrix = np.eye(horizon)

    angle_lower = np.full(
        horizon,
        min_steering_angle,
    )

    angle_upper = np.full(
        horizon,
        max_steering_angle,
    )

    # Steering-rate constraint:
    #
    #   delta_0 - previous_delta
    #   delta_1 - delta_0
    #   delta_2 - delta_1
    #
    # D @ U gives:
    #   [delta_0,
    #    delta_1 - delta_0,
    #    delta_2 - delta_1]

    rate_matrix = np.eye(horizon)

    for i in range(1, horizon):
        rate_matrix[i, i - 1] = -1.0

    max_delta_change = max_steering_rate * dt

    rate_lower = np.full(
        horizon,
        -max_delta_change,
    )

    rate_upper = np.full(
        horizon,
        max_delta_change,
    )

    # First predicted steering angle is compared
    # with the steering angle executed previously.
    rate_lower[0] = (
        previous_steering_angle - max_delta_change
    )

    rate_upper[0] = (
        previous_steering_angle + max_delta_change
    )

    constraint_matrix = np.vstack([
        angle_matrix,
        rate_matrix,
    ])

    lower_bound = np.concatenate([
        angle_lower,
        rate_lower,
    ])

    upper_bound = np.concatenate([
        angle_upper,
        rate_upper,
    ])

    return constraint_matrix, lower_bound, upper_bound


def solve_qp(
    H: np.ndarray,
    f: np.ndarray,
    constraint_matrix: np.ndarray,
    lower_bound: np.ndarray,
    upper_bound: np.ndarray,
) -> np.ndarray:
    """Solve the MPC quadratic program."""

    if H.ndim != 2 or H.shape[0] != H.shape[1]:
        raise ValueError("H must be a square matrix.")

    if f.ndim != 1 or f.shape[0] != H.shape[0]:
        raise ValueError("f dimension does not match H.")

    if constraint_matrix.ndim != 2:
        raise ValueError("constraint_matrix must be 2D.")

    if constraint_matrix.shape[1] != H.shape[0]:
        raise ValueError(
            "constraint_matrix and H dimensions do not match."
        )

    if lower_bound.shape != upper_bound.shape:
        raise ValueError(
            "lower_bound and upper_bound dimensions do not match."
        )

    if lower_bound.shape[0] != constraint_matrix.shape[0]:
        raise ValueError(
            "constraint bounds and matrix dimensions do not match."
        )

    P = sparse.csc_matrix(H)
    A_constraint = sparse.csc_matrix(constraint_matrix)

    solver = osqp.OSQP()

    solver.setup(
        P=P,
        q=f,
        A=A_constraint,
        l=lower_bound,
        u=upper_bound,
        verbose=False,
    )

    result = solver.solve()

    if result.x is None:
        raise RuntimeError(
            f"OSQP failed to find a solution: {result.info.status}"
        )

    return result.x


class MPCController:
    """Linear MPC controller for lateral path tracking."""

    def __init__(
        self,
        horizon: int = MPC_HORIZON,
        wheelbase_m: float = MPC_WHEELBASE_M,
        dt: float = MPC_DT,
        q_lateral: float = MPC_Q_LATERAL,
        q_heading: float = MPC_Q_HEADING,
        r_steering: float = MPC_R_STEERING,
        min_steering_angle: float = -MPC_MAX_STEERING_ANGLE,
        max_steering_angle: float = MPC_MAX_STEERING_ANGLE,
        max_steering_rate: float = MPC_MAX_STEERING_RATE,
    ):
        if not isinstance(horizon, int) or horizon <= 0:
            raise ValueError("horizon must be a positive integer.")
        if wheelbase_m <= 0.0:
            raise ValueError("wheelbase_m must be positive.")
        if dt <= 0.0:
            raise ValueError("dt must be positive.")
        if q_lateral < 0.0:
            raise ValueError("q_lateral must not be negative.")
        if q_heading < 0.0:
            raise ValueError("q_heading must not be negative.")
        if r_steering <= 0.0:
            raise ValueError("r_steering must be positive.")
        if min_steering_angle >= max_steering_angle:
            raise ValueError(
                "min_steering_angle must be smaller than max_steering_angle."
            )
        if max_steering_rate <= 0.0:
            raise ValueError("max_steering_rate must be positive.")

        self.horizon = horizon
        self.wheelbase_m = wheelbase_m
        self.dt = dt

        self.Q = np.array([
            [q_lateral, 0.0],
            [0.0, q_heading],
        ])

        self.R = np.array([
            [r_steering],
        ])

        self.min_steering_angle = min_steering_angle
        self.max_steering_angle = max_steering_angle
        self.max_steering_rate = max_steering_rate
        self.previous_steering_angle = 0.0

    def compute_steering(
        self,
        lateral_error: float,
        heading_error: float,
        speed_mps: float,
    ) -> float:
        """Calculate the steering command for the current vehicle state."""

        if speed_mps < 0.0:
            raise ValueError("speed_mps must not be negative.")

        state = np.array([
            lateral_error,
            heading_error,
        ])

        A, B = build_discrete_model(
            speed_mps=speed_mps,
            wheelbase_m=self.wheelbase_m,
            dt=self.dt,
        )

        A_bar, B_bar = build_prediction_matrices(
            A=A,
            B=B,
            horizon=self.horizon,
        )

        Q_bar, R_bar = build_cost_matrices(
            Q=self.Q,
            R=self.R,
            horizon=self.horizon,
        )

        H, f = build_qp_cost(
            A_bar=A_bar,
            B_bar=B_bar,
            Q_bar=Q_bar,
            R_bar=R_bar,
            state=state,
        )

        C, lower, upper = build_input_constraints(
            horizon=self.horizon,
            previous_steering_angle=self.previous_steering_angle,
            min_steering_angle=self.min_steering_angle,
            max_steering_angle=self.max_steering_angle,
            max_steering_rate=self.max_steering_rate,
            dt=self.dt,
        )

        optimal_U = solve_qp(
            H=H,
            f=f,
            constraint_matrix=C,
            lower_bound=lower,
            upper_bound=upper,
        )

        steering = float(optimal_U[0])

        steering = max(
            self.min_steering_angle,
            min(steering, self.max_steering_angle),
        )

        self.previous_steering_angle = steering

        return steering
