from src.benchmark.canonical_runner import (
    run_canonical_benchmark,
)
from src.benchmark.comparison import (
    format_benchmark_comparison_table,
)


def main() -> None:
    benchmark = run_canonical_benchmark()

    print(
        format_benchmark_comparison_table(
            sampling=benchmark.sampling_metrics,
            optimization=(
                benchmark.optimization_metrics
            ),
        )
    )


if __name__ == "__main__":
    main()