from collections.abc import Callable
from dataclasses import dataclass
import statistics

import numpy as np

from src.benchmark.planner_benchmark import (
    PlannerBenchmarkResult,
)
from src.config.benchmark import (
    DEFAULT_BENCHMARK_RUNTIME_CONFIG,
)


@dataclass(frozen=True)
class PlannerRuntimeStatistics:
    """Repeated runtime statistics for one planner."""

    planner_name: str

    warmup_runs: int
    repeated_runs: int

    mean_ms: float
    median_ms: float
    p95_ms: float
    min_ms: float
    max_ms: float

    samples_ms: tuple[float, ...]


def summarize_planning_times(
    planner_name: str,
    samples_ms: list[float],
    warmup_runs: int,
) -> PlannerRuntimeStatistics:
    """Summarize already measured planner runtimes."""

    if not planner_name:
        raise ValueError(
            "planner_name must not be empty."
        )

    if warmup_runs < 0:
        raise ValueError(
            "warmup_runs must not be negative."
        )

    if not samples_ms:
        raise ValueError(
            "samples_ms must not be empty."
        )

    if any(
        sample < 0.0
        for sample in samples_ms
    ):
        raise ValueError(
            "Runtime samples must not be negative."
        )

    return PlannerRuntimeStatistics(
        planner_name=planner_name,
        warmup_runs=warmup_runs,
        repeated_runs=len(samples_ms),
        mean_ms=statistics.mean(
            samples_ms
        ),
        median_ms=statistics.median(
            samples_ms
        ),
        p95_ms=float(
            np.percentile(
                samples_ms,
                95.0,
            )
        ),
        min_ms=min(
            samples_ms
        ),
        max_ms=max(
            samples_ms
        ),
        samples_ms=tuple(
            samples_ms
        ),
    )


def benchmark_planner_runtime(
    run_once: Callable[
        [],
        PlannerBenchmarkResult,
    ],
    warmup_runs: int = (
        DEFAULT_BENCHMARK_RUNTIME_CONFIG
        .warmup_runs
    ),
    repeated_runs: int = (
        DEFAULT_BENCHMARK_RUNTIME_CONFIG
        .repeated_runs
    ),
) -> PlannerRuntimeStatistics:
    """Run one benchmark adapter repeatedly and summarize its runtime."""

    if warmup_runs < 0:
        raise ValueError(
            "warmup_runs must not be negative."
        )

    if repeated_runs <= 0:
        raise ValueError(
            "repeated_runs must be positive."
        )

    for _ in range(
        warmup_runs
    ):
        result = run_once()

        if not result.success:
            raise RuntimeError(
                "Planner failed during runtime warm-up: "
                f"{result.failure_reason}"
            )

    samples_ms = []
    planner_name = None

    for _ in range(
        repeated_runs
    ):
        result = run_once()

        if not result.success:
            raise RuntimeError(
                "Planner failed during runtime measurement: "
                f"{result.failure_reason}"
            )

        if planner_name is None:
            planner_name = (
                result.planner_name
            )
        elif (
            result.planner_name
            != planner_name
        ):
            raise RuntimeError(
                "Planner name changed during runtime benchmark."
            )

        samples_ms.append(
            result.planning_time_ms
        )

    if planner_name is None:
        raise RuntimeError(
            "Runtime benchmark produced no result."
        )

    return summarize_planning_times(
        planner_name=planner_name,
        samples_ms=samples_ms,
        warmup_runs=warmup_runs,
    )