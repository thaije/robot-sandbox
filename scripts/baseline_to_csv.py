#!/usr/bin/env python3.12
"""Convert baseline result JSONs to a flat CSV for analysis.

Category scores are recomputed from raw metrics using the current scoring
formulas and par_values from each tier's YAML, so the CSV reflects the
updated scoring system rather than the archived JSON scores.
"""

import csv
import json
import sys
from pathlib import Path

import yaml

RESULTS_DIR  = Path(__file__).parent.parent / "baseline_results"
SCENARIOS_DIR = Path(__file__).parent.parent / "config/scenarios/office_explore_detect"
OUTPUT_CSV   = RESULTS_DIR / "baseline_results.csv"

CATEGORIES   = ["speed", "accuracy", "safety", "efficiency", "effectiveness"]
OBJECT_TYPES = ["fire_extinguisher", "first_aid_kit", "hazard_sign"]
TIER_ORDER   = {"easy": 0, "medium": 1, "hard": 2, "brutal": 3, "perception_stress": 4}

GRADE_MAP = [(95, "S"), (85, "A"), (70, "B"), (55, "C"), (40, "D"), (0, "F")]


# ── Scoring helpers (mirrors src/metrics/scoring.py) ──────────────────────────

def _grade(score: float) -> str:
    for threshold, letter in GRADE_MAP:
        if score >= threshold:
            return letter
    return "F"


def _stepwise(count: int, thresholds: list, scores: list) -> float:
    for i, t in enumerate(thresholds):
        if count <= t:
            return float(scores[i])
    return float(scores[-1])


def compute_scores(rm: dict, scoring_cfg: dict, timeout: float) -> dict:
    """Recompute all category scores from raw metrics + scoring config."""
    par     = scoring_cfg.get("par_values", {})
    weights = scoring_cfg.get("category_weights", {})

    # Speed: elapsed vs par
    elapsed   = float(rm.get("task_completion_time", timeout))
    par_time  = float(par.get("completion_time_par", timeout * 0.5))
    speed     = round(min(100.0, par_time / elapsed * 70), 2) if elapsed > 0 else 100.0

    # Accuracy: 0.60 × found_ratio + 0.40 × precision
    recall    = min(100.0, float(rm.get("found_ratio", 0.0)) * 100)
    prec      = min(100.0, float(rm.get("precision", 0.0)) * 100)
    accuracy  = round(0.60 * recall + 0.40 * prec, 2)

    # Safety: collisions (0.70) + near-misses (0.30) — stepwise
    thresholds = scoring_cfg.get("collision_thresholds", [0, 2, 5, 10])
    tier_scores = scoring_cfg.get("collision_scores", [100, 80, 60, 40, 20])
    col_score  = _stepwise(int(rm.get("collision_count", 0)), thresholds, tier_scores)
    nm_score   = _stepwise(int(rm.get("near_miss_count", 0)), thresholds, tier_scores)
    safety     = round(0.70 * col_score + 0.30 * nm_score, 2)

    # Efficiency: 0.60 × cpm_score + 0.40 × path_score
    coverage  = float(rm.get("exploration_coverage", 0.0))
    meters    = float(rm.get("meters_traveled", 0.0))
    par_cpm   = float(par.get("coverage_per_meter_par", 2.0))
    par_path  = float(par.get("path_length_par", 50.0))
    cpm_score  = min(100.0, (coverage / meters) / par_cpm * 70) if meters > 0 and par_cpm > 0 else 0.0
    path_score = min(100.0, par_path / meters * 70)             if meters > 0 and par_path > 0 else 0.0
    efficiency = round(0.60 * cpm_score + 0.40 * path_score, 2)

    # Effectiveness: 0.65 × detection_completeness + 0.35 × coverage%
    eff_weights = scoring_cfg.get("effectiveness_weights", {})
    by_type     = rm.get("detection_by_type", {})
    if eff_weights and by_type:
        total_w = sum(float(w) for w in eff_weights.values()) or 1.0
        det_score = sum(
            (float(w) / total_w) * (int(by_type.get(t, {}).get("detected", 0)) /
                                     max(1, int(by_type.get(t, {}).get("total", 1)))) * 100
            for t, w in eff_weights.items()
        )
    else:
        det_score = float(rm.get("found_ratio", 0.0)) * 100
    effectiveness = round(0.65 * det_score + 0.35 * min(100.0, coverage), 2)

    # Overall weighted sum
    cat_vals = dict(speed=speed, accuracy=accuracy, safety=safety,
                    efficiency=efficiency, effectiveness=effectiveness)
    total_w  = sum(float(weights.get(c, 0)) for c in CATEGORIES) or 1.0
    overall  = round(sum(cat_vals[c] * float(weights.get(c, 0)) for c in CATEGORIES) / total_w, 1)

    return {
        **{f"{c}_score": cat_vals[c] for c in CATEGORIES},
        **{f"{c}_grade": _grade(cat_vals[c]) for c in CATEGORIES},
        **{f"{c}_weight": float(weights.get(c, 0)) for c in CATEGORIES},
        "overall_score": overall,
        "overall_grade": _grade(overall),
    }


# ── Tier YAML loader ───────────────────────────────────────────────────────────

_yaml_cache: dict[str, dict] = {}

def load_tier_cfg(tier: str) -> dict:
    if tier not in _yaml_cache:
        path = SCENARIOS_DIR / f"{tier}.yaml"
        _yaml_cache[tier] = yaml.safe_load(path.read_text()) if path.exists() else {}
    return _yaml_cache[tier]


def parse_tier(scenario_name: str) -> str:
    for tier in TIER_ORDER:
        if f"_{tier}_" in scenario_name or scenario_name.endswith(f"_{tier}"):
            return tier
    return "unknown"


# ── Row builder ───────────────────────────────────────────────────────────────

def load_result(path: Path) -> dict:
    data   = json.loads(path.read_text())
    rm     = data["raw_metrics"]
    tier   = parse_tier(data["scenario_name"])
    cfg    = load_tier_cfg(tier)
    timeout = float(data["timeout_seconds"])

    row: dict = {}

    # Run info
    row["filename"]      = path.name
    row["timestamp"]     = path.stem.rsplit("_", 1)[-1]
    row["scenario_name"] = data["scenario_name"]
    row["tier"]          = tier
    row["status"]        = data["status"]
    row["elapsed_seconds"]  = data["elapsed_seconds"]
    row["timeout_seconds"]  = data["timeout_seconds"]
    row["random_seed"]   = data["random_seed"]

    # Raw metrics
    for key in [
        "meters_traveled", "collision_count", "near_miss_count",
        "exploration_coverage",
        "found_ratio", "precision", "duplicate_rate",
        "false_positive_count", "duplicate_count", "detection_count",
        "task_completion_time",
    ]:
        row[key] = rm.get(key, "")

    # Derived raw metrics
    elapsed = float(rm.get("task_completion_time") or data["elapsed_seconds"])
    meters  = float(rm.get("meters_traveled", 0.0))
    row["avg_speed_kmh"]      = round(meters / elapsed * 3.6, 3) if elapsed > 0 else 0.0
    row["coverage_per_meter"] = round(float(rm.get("exploration_coverage", 0.0)) / meters, 3) if meters > 0 else 0.0

    # Detection by type
    dbt = rm.get("detection_by_type", {})
    for obj_type in OBJECT_TYPES:
        info = dbt.get(obj_type, {})
        row[f"{obj_type}_detected"] = info.get("detected", "")
        row[f"{obj_type}_total"]    = info.get("total", "")

    # Category scores — recomputed with new formulas
    scoring_cfg = cfg.get("scoring", {})
    scores = compute_scores(rm, scoring_cfg, timeout)
    row.update(scores)
    for cat in CATEGORIES:
        for suffix in ("_score", "_grade", "_weight"):
            row.setdefault(f"{cat}{suffix}", "")
    row.setdefault("overall_grade", "")

    return row


def main():
    files = sorted(RESULTS_DIR.glob("*.json"))
    if not files:
        print(f"No JSON files found in {RESULTS_DIR}", file=sys.stderr)
        sys.exit(1)

    rows = [load_result(f) for f in files]
    rows.sort(key=lambda r: (TIER_ORDER.get(r["tier"], 99), r["timestamp"]))

    tier_counter: dict[str, int] = {}
    for row in rows:
        tier = row["tier"]
        tier_counter[tier] = tier_counter.get(tier, 0) + 1
        row["run_idx"] = tier_counter[tier]

    columns = [
        "filename", "timestamp", "scenario_name", "tier", "run_idx",
        "status", "elapsed_seconds", "timeout_seconds", "random_seed",
        # raw metrics
        "meters_traveled", "collision_count", "near_miss_count",
        "exploration_coverage",
        "found_ratio", "precision", "duplicate_rate",
        "false_positive_count", "duplicate_count", "detection_count",
        "task_completion_time",
        # derived raw metrics
        "avg_speed_kmh", "coverage_per_meter",
        # detection by type
        *[f"{t}_detected" for t in OBJECT_TYPES],
        *[f"{t}_total"    for t in OBJECT_TYPES],
        # category scores (recomputed with new formulas)
        *[f"{c}{s}" for c in CATEGORIES for s in ("_score", "_grade", "_weight")],
        # overall
        "overall_score", "overall_grade",
    ]

    with OUTPUT_CSV.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)

    print(f"Written {len(rows)} rows → {OUTPUT_CSV}")


if __name__ == "__main__":
    main()
