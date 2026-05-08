#!/usr/bin/env python3.12
"""
Generate results/leaderboard.json and docs/leaderboard.html from benchmark submissions.

Discovers all results/submissions/*/benchmark_submission.yaml, aggregates raw_metrics
from per-run JSONs per difficulty, and writes the leaderboard.

Total score = found_ratio (objects found / total objects). Other metrics shown on hover.

Usage:
    python3.12 scripts/generate_leaderboard.py
"""
from __future__ import annotations

import json
import statistics
import sys
from collections import defaultdict
from pathlib import Path

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
SUBMISSIONS_ROOT = REPO_ROOT / "results" / "submissions"
LEADERBOARD_JSON = REPO_ROOT / "docs" / "leaderboard.json"
LEADERBOARD_HTML = REPO_ROOT / "docs" / "leaderboard.html"

DIFFICULTIES_ORDER = ["easy", "medium", "hard", "brutal", "perception_stress"]
DIFFICULTY_LABELS = {
    "easy": "Easy",
    "medium": "Medium",
    "hard": "Hard",
    "brutal": "Brutal",
    "perception_stress": "Perception Stress",
}
SCENARIO_LABELS = {
    "office_explore_detect": "Office: Explore &amp; Detect",
}


def _ms(values: list[float]) -> dict:
    if not values:
        return {"mean": None, "std": None}
    mu = statistics.mean(values)
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return {"mean": round(mu, 4), "std": round(sd, 4)}


def aggregate_difficulty(sub: dict, results_dir: Path, difficulty: str) -> dict | None:
    seeds = sub["seeds"]
    runs_per_seed = sub["runs_per_seed"]

    found_ratios: list[float] = []
    objects_found_list: list[float] = []
    total_objects_list: list[float] = []
    speeds: list[float] = []
    collisions: list[float] = []
    paths: list[float] = []
    coverages: list[float] = []
    elapsed: list[float] = []

    found_files = 0
    for seed in seeds:
        for run in range(1, runs_per_seed + 1):
            fpath = results_dir / f"{difficulty}_seed{seed}_run{run}.json"
            if not fpath.exists():
                continue
            found_files += 1
            data = json.loads(fpath.read_text())
            raw = data.get("raw_metrics", {})

            found_ratios.append(float(raw.get("found_ratio", 0.0)))
            elapsed.append(float(data.get("elapsed_seconds", 0.0)))

            dt = raw.get("detection_by_type", {})
            if dt:
                objects_found_list.append(float(sum(v["detected"] for v in dt.values())))
                total_objects_list.append(float(sum(v["total"] for v in dt.values())))
            if "avg_speed_kmh" in raw:
                speeds.append(float(raw["avg_speed_kmh"]))
            if "collision_count" in raw:
                collisions.append(float(raw["collision_count"]))
            if "meters_traveled" in raw:
                paths.append(float(raw["meters_traveled"]))
            if "exploration_coverage" in raw:
                coverages.append(float(raw["exploration_coverage"]))

    if found_files == 0:
        return None

    total_obj = round(statistics.mean(total_objects_list)) if total_objects_list else None

    return {
        "stack_name": sub["stack_name"],
        "repo_url": sub.get("repo_url", ""),
        "sandbox_version": sub["sandbox_version"],
        "scenario": sub["scenario"],
        "difficulty": difficulty,
        "is_baseline": sub.get("is_baseline", False),
        "n_runs": found_files,
        "seeds": sub["seeds"],
        "submitted": sub.get("submitted", ""),
        "total_score": _ms(found_ratios),
        "raw_metrics": {
            "objects_found": {**_ms(objects_found_list), "of_total": total_obj},
            "avg_speed_kmh": _ms(speeds),
            "collision_count": _ms(collisions),
            "path_length_m": _ms(paths),
            "exploration_coverage_pct": _ms(coverages),
            "elapsed_s": _ms(elapsed),
        },
    }


def build_leaderboard() -> list[dict]:
    entries = []
    for yaml_path in sorted(SUBMISSIONS_ROOT.glob("*/benchmark_submission.yaml")):
        try:
            sub = yaml.safe_load(yaml_path.read_text())
        except Exception as exc:
            print(f"SKIP {yaml_path.parent.name}: {exc}", file=sys.stderr)
            continue

        results_dir = Path(sub["results_dir"])
        if not results_dir.is_absolute():
            results_dir = REPO_ROOT / results_dir

        for diff in sub.get("difficulties", []):
            if diff not in DIFFICULTIES_ORDER:
                continue
            entry = aggregate_difficulty(sub, results_dir, diff)
            if entry:
                entries.append(entry)

    return entries


def _fmt(val: float | None, decimals: int = 2) -> str:
    return f"{val:.{decimals}f}" if val is not None else "—"


def _tooltip(raw: dict) -> str:
    parts = []
    of = raw["objects_found"]
    if of["mean"] is not None:
        parts.append(f"found {_fmt(of['mean'], 1)}/{of['of_total']}")
    spd = raw["avg_speed_kmh"]
    if spd["mean"] is not None:
        parts.append(f"{_fmt(spd['mean'], 2)} km/h")
    col = raw["collision_count"]
    if col["mean"] is not None:
        parts.append(f"{_fmt(col['mean'], 1)} collisions")
    pth = raw["path_length_m"]
    if pth["mean"] is not None:
        parts.append(f"{_fmt(pth['mean'], 0)} m path")
    cov = raw["exploration_coverage_pct"]
    if cov["mean"] is not None:
        parts.append(f"{_fmt(cov['mean'], 1)}% coverage")
    ela = raw["elapsed_s"]
    if ela["mean"] is not None:
        parts.append(f"{_fmt(ela['mean'], 0)}s")
    return " · ".join(parts)


def render_html(entries: list[dict]) -> str:
    groups: dict[str, dict[str, list[dict]]] = defaultdict(lambda: defaultdict(list))
    scenarios: list[str] = []
    for e in entries:
        s = e["scenario"]
        if s not in scenarios:
            scenarios.append(s)
        groups[s][e["difficulty"]].append(e)

    rows_html = ""
    for scenario in scenarios:
        label = SCENARIO_LABELS.get(scenario, scenario)
        rows_html += f'<tr class="scenario-header"><td colspan="6">{label}</td></tr>\n'
        for diff in DIFFICULTIES_ORDER:
            if diff not in groups[scenario]:
                continue
            diff_entries = groups[scenario][diff]
            non_baselines = sorted(
                [e for e in diff_entries if not e["is_baseline"]],
                key=lambda e: (e["total_score"]["mean"] or 0),
                reverse=True,
            )
            baselines = [e for e in diff_entries if e["is_baseline"]]
            ordered = non_baselines + baselines

            rows_html += (
                f'<tr class="diff-header"><td colspan="6">'
                f'{DIFFICULTY_LABELS.get(diff, diff)}</td></tr>\n'
            )
            rank = 1
            for e in ordered:
                is_bl = e["is_baseline"]
                row_cls = ' class="baseline-row"' if is_bl else ""
                rank_str = "—" if is_bl else str(rank)
                if not is_bl:
                    rank += 1

                ts = e["total_score"]
                score_str = f"{_fmt(ts['mean'], 3)} ± {_fmt(ts['std'], 3)}"
                tip = _tooltip(e["raw_metrics"])

                repo_url = e.get("repo_url", "")
                stack = e["stack_name"]
                stack_cell = (
                    f'<a href="{repo_url}" target="_blank">{stack}</a>'
                    if repo_url else stack
                )
                submitted = e.get("submitted") or ""

                rows_html += (
                    f'<tr{row_cls}>'
                    f'<td>{rank_str}</td>'
                    f'<td>{stack_cell}</td>'
                    f'<td class="score-cell" data-tip="{tip}">{score_str}</td>'
                    f'<td>{e["n_runs"]}</td>'
                    f'<td>{submitted}</td>'
                    f'<td>{e["sandbox_version"]}</td>'
                    f'</tr>\n'
                )

    return f"""<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>robot-sandbox Leaderboard</title>
<style>
  body {{
    font-family: system-ui, -apple-system, sans-serif;
    max-width: 900px;
    margin: 2rem auto;
    padding: 0 1.5rem;
    color: #1a1a2e;
    line-height: 1.5;
  }}
  h1 {{ font-size: 1.6rem; margin-bottom: 0.2rem; }}
  .subtitle {{ color: #555; font-size: 0.9rem; margin-bottom: 2rem; }}
  .subtitle a {{ color: #0969da; text-decoration: none; }}
  .subtitle a:hover {{ text-decoration: underline; }}
  table {{ width: 100%; border-collapse: collapse; font-size: 0.88rem; }}
  th {{
    background: #1a1a2e;
    color: #e8e8f4;
    text-align: left;
    padding: 9px 12px;
    font-weight: 600;
  }}
  td {{ padding: 7px 12px; border-bottom: 1px solid #e5e7eb; }}
  tbody tr:not(.scenario-header):not(.diff-header):hover td {{ background: #f0f4ff; }}
  tr.scenario-header td {{
    background: #1a1a2e;
    color: #e8e8f4;
    font-weight: 700;
    font-size: 0.95rem;
    padding: 10px 12px;
  }}
  tr.diff-header td {{
    background: #eef0f6;
    color: #555;
    font-weight: 600;
    font-size: 0.78rem;
    letter-spacing: 0.06em;
    text-transform: uppercase;
    padding: 5px 12px;
  }}
  tr.baseline-row td {{ color: #888; font-style: italic; }}
  a {{ color: #0969da; text-decoration: none; }}
  a:hover {{ text-decoration: underline; }}
  .score-cell {{
    cursor: help;
    position: relative;
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
  }}
  .score-cell::after {{
    content: attr(data-tip);
    display: none;
    position: absolute;
    background: #1a1a2e;
    color: #dde;
    padding: 6px 12px;
    border-radius: 6px;
    font-size: 0.8rem;
    font-style: normal;
    white-space: nowrap;
    z-index: 100;
    bottom: calc(100% + 5px);
    left: 50%;
    transform: translateX(-50%);
    pointer-events: none;
    box-shadow: 0 2px 10px rgba(0,0,0,0.35);
  }}
  .score-cell:hover::after {{ display: block; }}
  .legend {{
    margin-top: 1.5rem;
    font-size: 0.78rem;
    color: #888;
    line-height: 1.8;
  }}
</style>
</head>
<body>
<h1>robot-sandbox Leaderboard</h1>
<p class="subtitle">
  Scenario: <strong>office_explore_detect</strong>&nbsp;&nbsp;·&nbsp;&nbsp;
  Score&nbsp;=&nbsp;objects found / total objects (0–1)&nbsp;&nbsp;·&nbsp;&nbsp;
  Hover score cell for raw metrics&nbsp;&nbsp;·&nbsp;&nbsp;
  <a href="https://github.com/thaije/robot-sandbox" target="_blank">GitHub ↗</a>
</p>
<table>
<thead>
<tr>
  <th>#</th>
  <th>Stack</th>
  <th>Score (mean ± std)</th>
  <th>Runs</th>
  <th>Submitted</th>
  <th>Version</th>
</tr>
</thead>
<tbody>
{rows_html}</tbody>
</table>
<p class="legend">
  Italic rows = baselines (not ranked — reference floor/ceiling only).<br>
  Score = <code>found_ratio</code>: objects found / total objects in scenario.<br>
  Hover the score cell to see: objects found, avg speed, collisions, path length, exploration coverage, elapsed time.
</p>
</body>
</html>"""


def main() -> None:
    entries = build_leaderboard()
    if not entries:
        print("No entries found — check that run JSONs exist in submissions subdirs.", file=sys.stderr)
        sys.exit(1)

    LEADERBOARD_JSON.write_text(json.dumps(entries, indent=2))
    print(f"Written {len(entries)} entries → {LEADERBOARD_JSON.relative_to(REPO_ROOT)}")

    html = render_html(entries)
    LEADERBOARD_HTML.write_text(html)
    print(f"Written → {LEADERBOARD_HTML.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
