from dataclasses import dataclass


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


DEFAULT_BENCHMARK_RUNTIME_CONFIG = (
    BenchmarkRuntimeConfig()
)