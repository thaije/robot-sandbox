#!/usr/bin/env python3.12
"""
Validate a benchmark submission YAML before leaderboard inclusion.

Usage:
    python3.12 scripts/validate_submission.py <benchmark_submission.yaml>

Exit codes:
    0  all checks passed
    1  one or more checks failed
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import yaml

REQUIRED_SUBMISSION_KEYS = {
    "stack_name",
    "sandbox_version",
    "scenario",
    "difficulties",
    "seeds",
    "runs_per_seed",
    "results_dir",
}

REQUIRED_RESULT_KEYS = {
    "scenario_name",
    "status",
    "elapsed_seconds",
    "overall_score",
    "overall_grade",
    "raw_metrics",
}

VALID_DIFFICULTIES = {"easy", "medium", "hard", "brutal", "perception_stress"}
VALID_STATUSES = {"SUCCESS", "TIME_LIMIT"}


def check(errors: list[str], warnings: list[str], condition: bool, msg: str) -> bool:
    if not condition:
        errors.append(msg)
    return condition


def warn(warnings: list[str], condition: bool, msg: str) -> bool:
    if not condition:
        warnings.append(msg)
    return condition


def validate(submission_path: Path) -> tuple[list[str], list[str]]:
    errors: list[str] = []
    warnings: list[str] = []

    # --- load submission YAML ---
    try:
        sub = yaml.safe_load(submission_path.read_text())
    except Exception as exc:
        return [f"Cannot parse submission YAML: {exc}"], []

    if not isinstance(sub, dict):
        return ["Submission YAML must be a mapping"], []

    # --- required keys ---
    missing = REQUIRED_SUBMISSION_KEYS - sub.keys()
    if missing:
        errors.append(f"Missing required keys: {sorted(missing)}")

    if errors:
        return errors, warnings

    # --- field types / values ---
    difficulties = sub["difficulties"]
    seeds = sub["seeds"]
    runs_per_seed = sub["runs_per_seed"]

    check(errors, warnings, isinstance(difficulties, list) and len(difficulties) > 0,
          "difficulties must be a non-empty list")
    check(errors, warnings, isinstance(seeds, list) and len(seeds) > 0,
          "seeds must be a non-empty list")
    check(errors, warnings, isinstance(runs_per_seed, int) and runs_per_seed >= 1,
          "runs_per_seed must be a positive integer")

    unknown_diff = set(difficulties) - VALID_DIFFICULTIES
    if unknown_diff:
        errors.append(f"Unknown difficulties: {sorted(unknown_diff)}. "
                      f"Valid: {sorted(VALID_DIFFICULTIES)}")

    if errors:
        return errors, warnings

    # --- sandbox_version tag exists in this repo ---
    version_tag = sub["sandbox_version"]
    repo_root = Path(__file__).parent.parent
    try:
        result = subprocess.run(
            ["git", "tag", "--list", version_tag],
            capture_output=True, text=True, cwd=repo_root, check=True,
        )
        warn(warnings, result.stdout.strip() == version_tag,
             f"sandbox_version tag '{version_tag}' not found in this repo "
             f"(tag must exist before submission is final)")
    except subprocess.CalledProcessError:
        warnings.append(f"Could not check git tags for version '{version_tag}'")

    # --- results_dir exists ---
    results_dir = Path(sub["results_dir"])
    if not results_dir.is_absolute():
        results_dir = repo_root / results_dir

    check(errors, warnings, results_dir.exists(),
          f"results_dir does not exist: {results_dir}")

    if errors:
        return errors, warnings

    # --- per-run JSON files ---
    total_runs = len(difficulties) * len(seeds) * runs_per_seed
    found = 0
    for diff in difficulties:
        for seed in seeds:
            for run in range(1, runs_per_seed + 1):
                fname = f"{diff}_seed{seed}_run{run}.json"
                fpath = results_dir / fname
                if not check(errors, warnings, fpath.exists(),
                             f"Missing result file: {fname}"):
                    continue
                found += 1
                validate_result_json(fpath, errors, warnings)

    if found < total_runs:
        errors.append(f"Found {found}/{total_runs} expected result files")

    return errors, warnings


def validate_result_json(path: Path, errors: list[str], warnings: list[str]) -> None:
    try:
        data = json.loads(path.read_text())
    except Exception as exc:
        errors.append(f"{path.name}: JSON parse error: {exc}")
        return

    missing = REQUIRED_RESULT_KEYS - data.keys()
    if missing:
        errors.append(f"{path.name}: missing keys {sorted(missing)}")
        return

    status = data.get("status")
    if status not in VALID_STATUSES:
        errors.append(f"{path.name}: invalid status '{status}'. "
                      f"Expected one of {sorted(VALID_STATUSES)}")

    score = data.get("overall_score")
    if not isinstance(score, (int, float)) or not (0 <= score <= 100):
        errors.append(f"{path.name}: overall_score must be a number in [0, 100], got {score!r}")

    raw = data.get("raw_metrics")
    if not isinstance(raw, dict):
        errors.append(f"{path.name}: raw_metrics must be a dict")


def main() -> None:
    parser = argparse.ArgumentParser(description="Validate a benchmark submission YAML")
    parser.add_argument("submission", type=Path, help="Path to benchmark_submission.yaml")
    args = parser.parse_args()

    if not args.submission.exists():
        print(f"ERROR: file not found: {args.submission}", file=sys.stderr)
        sys.exit(1)

    errors, warnings = validate(args.submission)

    for w in warnings:
        print(f"WARNING: {w}")

    if errors:
        print(f"\nFAIL — {len(errors)} error(s):")
        for e in errors:
            print(f"  - {e}")
        sys.exit(1)
    else:
        print("PASS — submission is valid")
        if warnings:
            print(f"  ({len(warnings)} warning(s) above; address before final submission)")


if __name__ == "__main__":
    main()
