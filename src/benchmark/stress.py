from dataclasses import dataclass

from src.benchmark.planner_benchmark import (
    PlannerBenchmarkResult,
)


@dataclass(frozen=True)
class PlannerStressSummary:
    """Aggregated success information for stress cases."""

    planner_name: str

    total_cases: int
    successful_cases: int
    failed_cases: int

    success_rate: float

    failure_cases: tuple[
        tuple[str, str],
        ...,
    ]


def summarize_planner_stress(
    planner_name: str,
    results: list[
        tuple[
            str,
            PlannerBenchmarkResult,
        ]
    ],
) -> PlannerStressSummary:
    if not planner_name:
        raise ValueError(
            "planner_name must not be empty."
        )

    if not results:
        raise ValueError(
            "Stress benchmark results must not be empty."
        )

    successful_cases = 0
    failure_cases = []

    for scenario_name, result in results:
        if result.success:
            successful_cases += 1
            continue

        failure_reason = (
            result.failure_reason
            or "Unknown failure"
        )

        failure_cases.append(
            (
                scenario_name,
                failure_reason,
            )
        )

    total_cases = len(
        results
    )

    failed_cases = (
        total_cases
        - successful_cases
    )

    return PlannerStressSummary(
        planner_name=planner_name,
        total_cases=total_cases,
        successful_cases=successful_cases,
        failed_cases=failed_cases,
        success_rate=(
            successful_cases
            / total_cases
        ),
        failure_cases=tuple(
            failure_cases
        ),
    )