#!/usr/bin/env python3.12
"""Convert baseline result JSONs to a flat CSV for analysis."""

import csv
import json
import re
import sys
from pathlib import Path

RESULTS_DIR = Path(__file__).parent.parent / "baseline_results"
OUTPUT_CSV = RESULTS_DIR / "baseline_results.csv"

CAT_RE = re.compile(
    r"CategoryScore\(name='(\w+)', score=([\d.]+), grade='(\w+)', grade_label='[^']+', weight=([\d.]+)\)"
)

CATEGORIES = ["speed", "accuracy", "safety", "efficiency", "effectiveness"]
OBJECT_TYPES = ["fire_extinguisher", "first_aid_kit", "hazard_sign"]

TIER_ORDER = {"easy": 0, "medium": 1, "hard": 2, "brutal": 3, "perception_stress": 4}


def parse_tier(scenario_name: str) -> str:
    """Extract tier from scenario name like 'office_easy_001'."""
    for tier in TIER_ORDER:
        if f"_{tier}_" in scenario_name or scenario_name.endswith(f"_{tier}"):
            return tier
    return "unknown"


def parse_categories(cat_strings: list[str]) -> dict:
    out = {}
    for s in cat_strings:
        m = CAT_RE.search(s)
        if m:
            name, score, grade, weight = m.groups()
            out[f"{name}_score"] = float(score)
            out[f"{name}_grade"] = grade
            out[f"{name}_weight"] = float(weight)
    return out


def load_result(path: Path) -> dict:
    data = json.loads(path.read_text())
    rm = data["raw_metrics"]

    row: dict = {}

    # Run info
    row["filename"] = path.name
    ts = path.stem.rsplit("_", 1)[-1]  # e.g. 20260303T205652
    row["timestamp"] = ts
    row["scenario_name"] = data["scenario_name"]
    row["tier"] = parse_tier(data["scenario_name"])
    row["status"] = data["status"]
    row["elapsed_seconds"] = data["elapsed_seconds"]
    row["timeout_seconds"] = data["timeout_seconds"]
    row["random_seed"] = data["random_seed"]

    # Raw metrics (flat, no events)
    for key in [
        "meters_traveled", "collision_count", "near_miss_count",
        "exploration_coverage", "revisit_ratio",
        "found_ratio", "precision", "duplicate_rate",
        "time_to_all_detections", "average_time_per_detection",
        "false_positive_count", "duplicate_count", "detection_count",
        "task_completion_time",
    ]:
        row[key] = rm.get(key, "")

    # Detection by type
    dbt = rm.get("detection_by_type", {})
    for obj_type in OBJECT_TYPES:
        info = dbt.get(obj_type, {})
        row[f"{obj_type}_detected"] = info.get("detected", "")
        row[f"{obj_type}_total"] = info.get("total", "")

    # Category scores
    row.update(parse_categories(data.get("categories", [])))
    # Ensure all expected category columns exist
    for cat in CATEGORIES:
        for suffix in ("_score", "_grade", "_weight"):
            row.setdefault(f"{cat}{suffix}", "")

    # Overall
    row["overall_score"] = data["overall_score"]
    row["overall_grade"] = data["overall_grade"]
    row["overall_grade_label"] = data["overall_grade_label"]

    return row


def main():
    files = sorted(RESULTS_DIR.glob("*.json"))
    if not files:
        print(f"No JSON files found in {RESULTS_DIR}", file=sys.stderr)
        sys.exit(1)

    rows = [load_result(f) for f in files]

    # Sort by tier order then timestamp
    rows.sort(key=lambda r: (TIER_ORDER.get(r["tier"], 99), r["timestamp"]))

    # Add run_idx per tier
    tier_counter: dict[str, int] = {}
    for row in rows:
        tier = row["tier"]
        tier_counter[tier] = tier_counter.get(tier, 0) + 1
        row["run_idx"] = tier_counter[tier]

    # Build ordered column list
    columns = [
        "filename", "timestamp", "scenario_name", "tier", "run_idx",
        "status", "elapsed_seconds", "timeout_seconds", "random_seed",
        # raw metrics
        "meters_traveled", "collision_count", "near_miss_count",
        "exploration_coverage", "revisit_ratio",
        "found_ratio", "precision", "duplicate_rate",
        "time_to_all_detections", "average_time_per_detection",
        "false_positive_count", "duplicate_count", "detection_count",
        "task_completion_time",
        # detection by type
        *[f"{t}_detected" for t in OBJECT_TYPES],
        *[f"{t}_total" for t in OBJECT_TYPES],
        # category scores
        *[f"{c}{s}" for c in CATEGORIES for s in ("_score", "_grade", "_weight")],
        # overall
        "overall_score", "overall_grade", "overall_grade_label",
    ]

    with OUTPUT_CSV.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        writer.writerows(rows)

    print(f"Written {len(rows)} rows → {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
