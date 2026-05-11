#!/usr/bin/env python3.12
"""Rescore submission results with current par values from scenario YAMLs.

Reads each result JSON, recomputes Speed, Efficiency, and Overall scores
using the current par_values from the matching scenario YAML, and overwrites
the score-related fields in-place. Raw metrics are preserved unchanged.

Usage:
    python3.12 scripts/rescore_submissions.py [SUBMISSION_DIRS...]

    Defaults to all directories under results/submissions/ that contain
    a benchmark_submission.yaml. Pass specific directories to rescore
    only those.
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
SCENARIOS_DIR = REPO_ROOT / "config" / "scenarios" / "office_explore_detect"
SUBMISSIONS_DIR = REPO_ROOT / "results" / "submissions"
TIER_ORDER = {"easy": 0, "medium": 1, "hard": 2, "brutal": 3, "perception_stress": 4}

sys.path.insert(0, str(REPO_ROOT / "src"))
from metrics.scoring import ScoringEngine, CategoryScore, numeric_to_grade


def parse_tier(scenario_name: str) -> str:
    for tier in TIER_ORDER:
        if f"_{tier}_" in scenario_name or scenario_name.endswith(f"_{tier}"):
            return tier
    return "unknown"


_yaml_cache: dict[str, dict] = {}


def load_tier_cfg(tier: str) -> dict:
    if tier not in _yaml_cache:
        path = SCENARIOS_DIR / f"{tier}.yaml"
        _yaml_cache[tier] = yaml.safe_load(path.read_text()) if path.exists() else {}
    return _yaml_cache[tier]


def rescore_file(path: Path) -> bool:
    data = json.loads(path.read_text())
    tier = parse_tier(data.get("scenario_name", ""))
    if tier not in TIER_ORDER:
        print(f"  SKIP {path.name}: unknown tier '{tier}'")
        return False

    cfg = load_tier_cfg(tier)
    scoring_cfg = cfg.get("scoring", {})
    if not scoring_cfg.get("par_values"):
        print(f"  SKIP {path.name}: no par_values in {tier}.yaml")
        return False

    timeout = float(data.get("timeout_seconds", cfg.get("scenario", {}).get("timeout_seconds", 600)))
    scenario_config = {
        "scenario": {
            "name": data.get("scenario_name", f"office_{tier}_001"),
            "timeout_seconds": timeout,
        },
    }

    raw_metrics = data.get("raw_metrics", {})
    if not raw_metrics:
        print(f"  SKIP {path.name}: no raw_metrics")
        return False

    effectiveness_weights = scoring_cfg.get("effectiveness_weights")
    world_objects = cfg.get("world", {}).get("objects", [])
    uses_mission_target = any("mission_target" in obj for obj in world_objects)
    if "effectiveness_weights" not in scoring_cfg and uses_mission_target:
        mission_types = list(dict.fromkeys(
            obj["type"] for obj in world_objects if obj.get("mission_target", False)
        ))
        if mission_types:
            weight = 100.0 / len(mission_types)
            effectiveness_weights = {t: weight for t in mission_types}
    if effectiveness_weights:
        scoring_cfg = {**scoring_cfg, "effectiveness_weights": effectiveness_weights}

    engine = ScoringEngine(scoring_cfg)
    scorecard = engine.compute(raw_metrics, scenario_config)

    scorecard.status = data.get("status", "")
    scorecard.elapsed_seconds = data.get("elapsed_seconds", timeout)
    scorecard.random_seed = data.get("random_seed", 0)

    data["categories"] = [str(c) for c in scorecard.categories]
    data["overall_score"] = scorecard.overall_score
    data["overall_grade"] = scorecard.overall_grade
    data["overall_grade_label"] = scorecard.overall_grade_label

    path.write_text(json.dumps(data, indent=2, default=str) + "\n")
    print(f"  OK   {path.name}: overall {scorecard.overall_score} ({scorecard.overall_grade})")
    return True


def find_submission_dirs(args: list[str]) -> list[Path]:
    if args:
        return [Path(a) for a in args]
    dirs = []
    for d in sorted(SUBMISSIONS_DIR.iterdir()):
        if d.is_dir() and (d / "benchmark_submission.yaml").exists():
            dirs.append(d)
    return dirs


def main() -> None:
    dirs = find_submission_dirs(sys.argv[1:])
    if not dirs:
        print("No submission directories found.")
        sys.exit(1)

    total = 0
    updated = 0
    for d in dirs:
        jsons = sorted(d.glob("*.json"))
        if not jsons:
            continue
        print(f"\n{d.name}/ ({len(jsons)} result files)")
        for jf in jsons:
            total += 1
            if rescore_file(jf):
                updated += 1

    print(f"\nRescored {updated}/{total} files.")


if __name__ == "__main__":
    main()