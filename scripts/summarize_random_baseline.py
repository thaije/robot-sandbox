#!/usr/bin/env python3.12
"""Generate summary.json for the random-baseline submission.

Reads all per-run JSONs from results/submissions/random-baseline/ and
produces mean ± std for each sub-score per difficulty tier.

Usage:
    python3.12 scripts/summarize_random_baseline.py [--results-dir PATH]
"""
from __future__ import annotations

import argparse
import json
import math
import re
import statistics
import sys
from datetime import datetime, timezone
from pathlib import Path

DIFFICULTIES = ["easy", "medium", "hard", "brutal", "perception_stress"]
SUB_SCORES = ["speed", "accuracy", "safety", "efficiency", "effectiveness"]
# filename pattern: <difficulty>_seed<N>_run<K>.json
FILENAME_RE = re.compile(
    r"^(?P<difficulty>[a-z_]+)_seed(?P<seed>\d+)_run(?P<run>\d+)\.json$"
)
# CategoryScore string: "CategoryScore(name='speed', score=21.2, ...)"
CAT_RE = re.compile(r"name='(?P<name>\w+)',\s*score=(?P<score>[\d.]+)")


def parse_category_scores(result: dict) -> dict[str, float]:
    """Extract sub-scores from the categories list in a result JSON."""
    scores: dict[str, float] = {}
    for entry in result.get("categories", []):
        m = CAT_RE.search(str(entry))
        if m:
            scores[m.group("name")] = float(m.group("score"))
    return scores


def mean_std(values: list[float]) -> dict:
    if not values:
        return {"mean": None, "std": None, "n": 0}
    n = len(values)
    mu = statistics.mean(values)
    sd = statistics.stdev(values) if n > 1 else 0.0
    return {"mean": round(mu, 2), "std": round(sd, 2), "n": n}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--results-dir",
        default=Path(__file__).resolve().parent.parent
        / "results/submissions/random-baseline",
        type=Path,
    )
    args = parser.parse_args()

    results_dir: Path = args.results_dir
    if not results_dir.is_dir():
        print(f"Error: {results_dir} does not exist", file=sys.stderr)
        sys.exit(1)

    # Collect per-difficulty buckets
    buckets: dict[str, dict[str, list[float]]] = {
        d: {s: [] for s in SUB_SCORES + ["overall"]} for d in DIFFICULTIES
    }
    status_counts: dict[str, dict[str, int]] = {
        d: {"SUCCESS": 0, "TIME_LIMIT": 0, "other": 0} for d in DIFFICULTIES
    }
    total = 0
    missing: list[str] = []

    for difficulty in DIFFICULTIES:
        for seed in range(1, 6):
            for run in range(1, 4):
                fname = f"{difficulty}_seed{seed}_run{run}.json"
                fpath = results_dir / fname
                if not fpath.exists():
                    missing.append(fname)
                    continue

                data = json.loads(fpath.read_text())
                total += 1

                sub = parse_category_scores(data)
                for score_name in SUB_SCORES:
                    if score_name in sub:
                        buckets[difficulty][score_name].append(sub[score_name])

                overall = data.get("overall_score")
                if overall is not None:
                    buckets[difficulty]["overall"].append(float(overall))

                status = data.get("status", "other")
                if status in ("SUCCESS", "TIME_LIMIT"):
                    status_counts[difficulty][status] += 1
                else:
                    status_counts[difficulty]["other"] += 1

    # Build summary
    summary: dict = {
        "stack_name": "random-baseline",
        "generated_at": datetime.now(timezone.utc).isoformat(),
        "total_runs": total,
        "expected_runs": 75,
        "missing_runs": missing,
        "results": {},
    }

    for difficulty in DIFFICULTIES:
        n_runs = len(buckets[difficulty]["overall"])
        if n_runs == 0:
            continue
        summary["results"][difficulty] = {
            "n_runs": n_runs,
            "status_counts": status_counts[difficulty],
        }
        for score_name in SUB_SCORES + ["overall"]:
            vals = buckets[difficulty][score_name]
            summary["results"][difficulty][score_name] = mean_std(vals)

    out_path = results_dir / "summary.json"
    out_path.write_text(json.dumps(summary, indent=2))
    print(f"Written {total} runs → {out_path}")

    if missing:
        print(f"Missing {len(missing)} runs:", file=sys.stderr)
        for m in missing[:10]:
            print(f"  {m}", file=sys.stderr)
        if len(missing) > 10:
            print(f"  ... and {len(missing) - 10} more", file=sys.stderr)


if __name__ == "__main__":
    main()
