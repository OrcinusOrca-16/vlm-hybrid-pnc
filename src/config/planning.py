from dataclasses import dataclass

from src.planning.path_objective import (
    PathObjectiveWeights,
)


@dataclass(frozen=True)
class PlanningConstraintConfig:
    """Physical constraints shared by the planning algorithms."""

    # Legacy fallback retained for code outside the canonical
    # benchmark. Canonical scenarios use vehicle-derived max curvature.
    max_curvature: float = 1.0
    minimum_clearance_m: float = 0.5


@dataclass(frozen=True)
class SamplingPlannerConfig:
    """Configuration for the Frenet Sampling Planner."""

    target_lateral_positions_m: tuple[float, ...] = (
        -1.5,
        -0.75,
        0.0,
        0.75,
        1.5,
    )

    target_lengths_m: tuple[float, ...] = (
        10.0,
        20.0,
    )

    resolution_m: float = 0.5

    sampling_epsilon_m: float = 1e-9

    returning_bypass_lateral_positions_m: tuple[float, ...] = (
        -2.0,
        -1.5,
        -1.0,
        -0.5,
        0.0,
        0.5,
        1.0,
        1.5,
        2.0,
    )

    returning_bypass_lengths_m: tuple[float, ...] = (
        6.0,
        10.0,
        14.0,
        18.0,
        22.0,
    )

    # Keep the production Sampling Planner lightweight. Development studies
    # may opt into wider junction-slope sampling and trackability preference.
    returning_junction_dl_ds_values: tuple[float, ...] = (0.0,)
    trackability_weight: float = 0.0

    returning_total_length_m: float = 28.0

    returning_final_lateral_positions_m: tuple[float, ...] = (
        0.0,
    )
    returning_final_dl_ds_values: tuple[float, ...] = (
        0.0,
    )
    returning_final_d2l_ds2: float = 0.0

    lateral_weight: float = 1.0
    curvature_weight: float = 10.0
    length_weight: float = 0.05

    matched_objective_weights: PathObjectiveWeights = (
        PathObjectiveWeights(
            lateral_weight=1.0,
            first_derivative_weight=1.0,
            second_derivative_weight=5.0,
            third_derivative_weight=20.0,
            terminal_lateral_weight=5.0,
            terminal_first_derivative_weight=5.0,
        )
    )


@dataclass(frozen=True)
class OptimizationPlannerConfig:
    """Configuration for the Frenet QP Optimization Planner."""

    resolution_m: float = 0.5
    planning_horizon_m: float = 28.0

    lateral_reference_m: float = 0.0

    lateral_weight: float = 0.2
    first_derivative_weight: float = 1.0
    second_derivative_weight: float = 5.0
    third_derivative_weight: float = 20.0

    # Temporarily retained for benchmark-call compatibility.
    # Production explicit-state planning now uses vehicle max curvature.
    max_second_derivative_abs: float = 1.0

    osqp_verbose: bool = False
    osqp_eps_abs: float = 1e-6
    osqp_eps_rel: float = 1e-6

    horizon_discretization_epsilon_m: float = 1e-9

    # Legacy obstacle-corridor knobs retained for diagnostics only.
    corridor_affected_epsilon_m: float = 1e-2
    obstacle_inflation_safety_margin_m: float = 5e-2

    # Sequential exact-footprint refinement.
    sequential_max_iterations: int = 6
    clearance_numerical_buffer_m: float = 1e-3

    trust_region_l_m: float = 0.15
    trust_region_dl: float = 0.08

    clearance_gradient_eps_l: float = 1e-4
    clearance_gradient_eps_dl: float = 1e-4

    heading_constraint_body_sample_count: int = 9

    matched_objective_weights: PathObjectiveWeights = (
        PathObjectiveWeights(
            lateral_weight=0.5,
            first_derivative_weight=1.0,
            second_derivative_weight=5.0,
            third_derivative_weight=20.0,
            terminal_lateral_weight=5.0,
            terminal_first_derivative_weight=5.0,
        )
    )


DEFAULT_PLANNING_CONSTRAINT_CONFIG = (
    PlanningConstraintConfig()
)

DEFAULT_SAMPLING_PLANNER_CONFIG = (
    SamplingPlannerConfig()
)

DEFAULT_OPTIMIZATION_PLANNER_CONFIG = (
    OptimizationPlannerConfig()
)
