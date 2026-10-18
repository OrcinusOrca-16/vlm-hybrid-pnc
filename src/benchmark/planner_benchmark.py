from dataclasses import dataclass

from src.common.types import (
    FrenetTrajectory,
    PathPoint,
)


@dataclass(frozen=True)
class PlannerBenchmarkResult:
    """Planner-independent output used by the benchmark layer."""

    planner_name: str
    success: bool

    trajectory: FrenetTrajectory | None
    path: list[PathPoint] | None

    planning_time_ms: float
    failure_reason: str | None

    matched_objective_value: float | None

    def __post_init__(self) -> None:
        if not self.planner_name.strip():
            raise ValueError(
                "planner_name must not be empty."
            )

        if self.planning_time_ms < 0.0:
            raise ValueError(
                "planning_time_ms must not be negative."
            )

        if self.success:
            if self.trajectory is None:
                raise ValueError(
                    "Successful result must contain a Frenet trajectory."
                )

            if self.path is None:
                raise ValueError(
                    "Successful result must contain a Cartesian path."
                )

            if self.failure_reason is not None:
                raise ValueError(
                    "Successful result must not contain a failure reason."
                )

            if self.matched_objective_value is None:
                raise ValueError(
                    "Successful result must contain the matched objective."
                )

        else:
            if (
                self.failure_reason is None
                or not self.failure_reason.strip()
            ):
                raise ValueError(
                    "Failed result must contain a failure reason."
                )

    @classmethod
    def succeeded(
        cls,
        planner_name: str,
        trajectory: FrenetTrajectory,
        path: list[PathPoint],
        planning_time_ms: float,
        matched_objective_value: float,
    ) -> "PlannerBenchmarkResult":
        return cls(
            planner_name=planner_name,
            success=True,
            trajectory=trajectory,
            path=path,
            planning_time_ms=planning_time_ms,
            failure_reason=None,
            matched_objective_value=(
                matched_objective_value
            ),
        )

    @classmethod
    def failed(
        cls,
        planner_name: str,
        planning_time_ms: float,
        failure_reason: str,
        trajectory: FrenetTrajectory | None = None,
        path: list[PathPoint] | None = None,
        matched_objective_value: float | None = None,
    ) -> "PlannerBenchmarkResult":
        return cls(
            planner_name=planner_name,
            success=False,
            trajectory=trajectory,
            path=path,
            planning_time_ms=planning_time_ms,
            failure_reason=failure_reason,
            matched_objective_value=(
                matched_objective_value
            ),
        )