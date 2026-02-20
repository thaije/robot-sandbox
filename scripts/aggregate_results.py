"""
Aggregate metrics across multiple run result JSON files.
Outputs mean, std, min, max for each numeric metric.

Usage: python scripts/aggregate_results.py results/
"""
from __future__ import annotations

import json
import statistics
import sys
from pathlib import Path


def load_results(results_dir: Path) -> list[dict]:
    files = sorted(results_dir.glob("*.json"))
    if not files:
        print(f"No result files found in {results_dir}")
        sys.exit(1)
    return [json.loads(f.read_text()) for f in files]


def aggregate(results: list[dict]) -> dict:
    numeric_keys: set[str] = set()
    for r in results:
        for k, v in r.get("raw_metrics", {}).items():
            if isinstance(v, (int, float)):
                numeric_keys.add(k)

    summary = {}
    for key in sorted(numeric_keys):
        values = [r["raw_metrics"][key] for r in results if key in r.get("raw_metrics", {})]
        summary[key] = {
            "mean": round(statistics.mean(values), 4),
            "std":  round(statistics.stdev(values), 4) if len(values) > 1 else 0.0,
            "min":  round(min(values), 4),
            "max":  round(max(values), 4),
            "n":    len(values),
        }
    return summary


def main() -> None:
    if len(sys.argv) < 2:
        print("Usage: aggregate_results.py <results_dir>")
        sys.exit(1)

    results_dir = Path(sys.argv[1])
    results = load_results(results_dir)
    summary = aggregate(results)

    print(f"\nAggregate summary ({len(results)} runs):\n")
    for metric, stats in summary.items():
        print(f"  {metric}:")
        for k, v in stats.items():
            print(f"    {k}: {v}")
    print()

    out = results_dir / "aggregate_summary.json"
    out.write_text(json.dumps(summary, indent=2))
    print(f"Written: {out}")


if __name__ == "__main__":
    main()
