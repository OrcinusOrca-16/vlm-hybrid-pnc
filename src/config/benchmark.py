from dataclasses import dataclass

from src.planning.path_objective import (
    PathObjectiveWeights,
)


@dataclass(frozen=True)
class BenchmarkRuntimeConfig:
    """Configuration for repeated planner runtime benchmarks."""

    warmup_runs: int = 5
    repeated_runs: int = 50

    def __post_init__(self) -> None:
        if self.warmup_runs < 0:
            raise ValueError(
                "warmup_runs must not be negative."
            )

        if self.repeated_runs <= 0:
            raise ValueError(
                "repeated_runs must be positive."
            )


@dataclass(frozen=True)
class CanonicalBenchmarkConfig:
    """Planner-independent configuration for canonical benchmark evaluation."""

    planning_horizon_m: float = 28.0
    resolution_m: float = 0.5

    lateral_reference_m: float = 0.0
    terminal_lateral_target_m: float = 0.0
    terminal_first_derivative_target: float = 0.0

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

    def __post_init__(self) -> None:
        if self.planning_horizon_m <= 0.0:
            raise ValueError(
                "planning_horizon_m must be positive."
            )

        if self.resolution_m <= 0.0:
            raise ValueError(
                "resolution_m must be positive."
            )


DEFAULT_BENCHMARK_RUNTIME_CONFIG = (
    BenchmarkRuntimeConfig()
)

DEFAULT_CANONICAL_BENCHMARK_CONFIG = (
    CanonicalBenchmarkConfig()
)