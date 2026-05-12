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
SCENARIOS_DIR = REPO_ROOT / "config" / "scenarios" / "office_explore_detect"
SCENARIO_GOAL = (
    "Explore an unknown indoor environment and detect all mission-target objects "
    "before the time limit. Scored on Speed, Accuracy, Safety, Efficiency, "
    "and Effectiveness."
)


def _load_mission_targets(scenario: str, difficulty: str) -> set[str] | None:
    """Load mission-target object types from the scenario YAML.

    Returns a set of type strings marked mission_target, or None if the
    config file cannot be found (caller should fall back to all types).
    """
    cfg_path = SCENARIOS_DIR / f"{difficulty}.yaml"
    if not cfg_path.exists():
        return None
    try:
        cfg = yaml.safe_load(cfg_path.read_text())
    except Exception:
        return None
    objs = cfg.get("world", {}).get("objects", [])
    uses_mt = any("mission_target" in obj for obj in objs)
    if not uses_mt:
        return None  # all objects are mission targets
    return {obj["type"] for obj in objs if obj.get("mission_target", False)}


def _load_scenario_info(scenario: str, difficulty: str) -> dict:
    """Load timeout_seconds and description from the scenario YAML."""
    cfg_path = SCENARIOS_DIR / f"{difficulty}.yaml"
    info: dict = {"timeout_seconds": None, "description": ""}
    if not cfg_path.exists():
        return info
    try:
        cfg = yaml.safe_load(cfg_path.read_text())
    except Exception:
        return info
    sc = cfg.get("scenario", {})
    info["timeout_seconds"] = sc.get("timeout_seconds")
    info["description"] = sc.get("description", "")
    return info


def _ms(values: list[float]) -> dict:
    if not values:
        return {"mean": None, "std": None}
    mu = statistics.mean(values)
    sd = statistics.stdev(values) if len(values) > 1 else 0.0
    return {"mean": round(mu, 4), "std": round(sd, 4)}


def aggregate_difficulty(sub: dict, results_dir: Path, difficulty: str) -> dict | None:
    seeds = sub["seeds"]
    runs_per_seed = sub["runs_per_seed"]
    scenario = sub["scenario"]
    mission_types = _load_mission_targets(scenario, difficulty)

    found_ratios: list[float] = []
    objects_found_list: list[float] = []
    total_objects_list: list[float] = []
    speeds: list[float] = []
    collisions: list[float] = []
    paths: list[float] = []
    coverages: list[float] = []
    elapsed: list[float] = []
    timeout_seconds: float | None = None

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
            if timeout_seconds is None and data.get("timeout_seconds"):
                timeout_seconds = float(data["timeout_seconds"])

            dt = raw.get("detection_by_type", {})
            if dt:
                has_mt_field = any("mission_target" in v for v in dt.values())
                if has_mt_field:
                    mt = {k for k, v in dt.items() if v.get("mission_target", False)}
                elif mission_types is not None:
                    mt = mission_types
                else:
                    mt = set(dt.keys())
                objects_found_list.append(float(sum(v["detected"] for k, v in dt.items() if k in mt)))
                total_objects_list.append(float(sum(v["total"] for k, v in dt.items() if k in mt)))
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
    scenario_info = _load_scenario_info(scenario, difficulty)
    if timeout_seconds is None:
        timeout_seconds = scenario_info.get("timeout_seconds")

    return {
        "stack_name": sub["stack_name"],
        "repo_url": sub.get("repo_url", ""),
        "code_url": sub.get("code_url", ""),
        "sandbox_version": sub["sandbox_version"],
        "scenario": sub["scenario"],
        "difficulty": difficulty,
        "is_baseline": sub.get("is_baseline", False),
        "n_runs": found_files,
        "seeds": sub["seeds"],
        "runs_per_seed": runs_per_seed,
        "submitted": sub.get("submitted", ""),
        "timeout_seconds": timeout_seconds,
        "difficulty_description": scenario_info.get("description", ""),
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


def _detail_cells(raw: dict) -> str:
    """Build the inner HTML for the expandable detail row (mean ± std per metric)."""
    of = raw["objects_found"]
    found_line = (
        f'{_fmt(of["mean"], 1)} <span class="d-std">± {_fmt(of["std"], 1)}</span> / {of["of_total"]}'
        if of["mean"] is not None else "—"
    )
    parts = [f'<span class="d-pair"><span class="d-label">Found</span><span class="d-val">{found_line}</span></span>']

    spd = raw["avg_speed_kmh"]
    if spd["mean"] is not None:
        parts.append(
            f'<span class="d-pair"><span class="d-label">Speed</span>'
            f'<span class="d-val">{_fmt(spd["mean"], 2)} <span class="d-std">± {_fmt(spd["std"], 2)}</span> km/h</span></span>'
        )
    col = raw["collision_count"]
    if col["mean"] is not None:
        parts.append(
            f'<span class="d-pair"><span class="d-label">Collisions</span>'
            f'<span class="d-val">{_fmt(col["mean"], 1)} <span class="d-std">± {_fmt(col["std"], 1)}</span></span></span>'
        )
    pth = raw["path_length_m"]
    if pth["mean"] is not None:
        parts.append(
            f'<span class="d-pair"><span class="d-label">Path</span>'
            f'<span class="d-val">{_fmt(pth["mean"], 0)} <span class="d-std">± {_fmt(pth["std"], 0)}</span> m</span></span>'
        )
    cov = raw["exploration_coverage_pct"]
    if cov["mean"] is not None:
        parts.append(
            f'<span class="d-pair"><span class="d-label">Coverage</span>'
            f'<span class="d-val">{_fmt(cov["mean"], 1)} <span class="d-std">± {_fmt(cov["std"], 1)}</span>%</span></span>'
        )
    ela = raw["elapsed_s"]
    if ela["mean"] is not None:
        parts.append(
            f'<span class="d-pair"><span class="d-label">Time</span>'
            f'<span class="d-val">{_fmt(ela["mean"], 0)} <span class="d-std">± {_fmt(ela["std"], 0)}</span>s</span></span>'
        )
    return "".join(parts)


def render_html(entries: list[dict]) -> str:
    groups: dict[str, dict[str, list[dict]]] = defaultdict(lambda: defaultdict(list))
    scenarios: list[str] = []
    for e in entries:
        s = e["scenario"]
        if s not in scenarios:
            scenarios.append(s)
        groups[s][e["difficulty"]].append(e)

    # Build per-scenario info panels
    info_panels = ""
    rows_html = ""
    for scenario in scenarios:
        label = SCENARIO_LABELS.get(scenario, scenario)
        # Collect timeout per difficulty from entries
        timeouts: dict[str, int] = {}
        for diff in DIFFICULTIES_ORDER:
            if diff in groups[scenario]:
                e0 = groups[scenario][diff][0]
                t = e0.get("timeout_seconds")
                if t is not None:
                    timeouts[diff] = int(t)
                desc = e0.get("difficulty_description", "")

        timeout_parts = " · ".join(
            f'{DIFFICULTY_LABELS.get(d, d)}: {timeouts[d]}s'
            for d in DIFFICULTIES_ORDER if d in timeouts
        )
        info_panels += f"""<div class="scenario-info">
  <img class="scenario-img" src="leaderboard_scenario.png" alt="Scenario preview">
  <div class="scenario-desc">
    <strong>{label}</strong><br>
    {SCENARIO_GOAL}<br>
    <span class="timeouts">Time limits — {timeout_parts}</span>
  </div>
</div>
"""

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

            diff_label = DIFFICULTY_LABELS.get(diff, diff)
            timeout_str = f" · {timeouts[diff]}s" if diff in timeouts else ""
            rows_html += (
                f'<tr class="diff-header"><td colspan="6">'
                f'{diff_label}{timeout_str}</td></tr>\n'
            )
            rank = 1
            for e in ordered:
                is_bl = e["is_baseline"]
                row_cls = ' class="baseline-row"' if is_bl else ""
                rank_str = "—" if is_bl else str(rank)
                if not is_bl:
                    rank += 1

                ts = e["total_score"]
                score_str = f"{_fmt(ts['mean'], 3)} <span class=\"score-std\">± {_fmt(ts['std'], 3)}</span>"

                repo_url = e.get("repo_url", "")
                code_url = e.get("code_url", "")
                stack = e["stack_name"]
                stack_cell = (
                    f'<a href="{repo_url}" target="_blank">{stack}</a>'
                    if repo_url else stack
                )
                if code_url:
                    stack_cell += f' <a href="{code_url}" target="_blank" class="code-link">code</a>'
                submitted = e.get("submitted") or ""

                rows_html += (
                    f'<tr{row_cls}>'
                    f'<td>{rank_str}</td>'
                    f'<td>{stack_cell}</td>'
                    f'<td class="score-cell">{score_str}</td>'
                    f'<td>{len(e["seeds"])}×{e["runs_per_seed"]}</td>'
                    f'<td>{submitted}</td>'
                    f'<td>{e["sandbox_version"]}</td>'
                    f'</tr>\n'
                    f'<tr class="detail-row"><td colspan="6"><div class="detail-inner">'
                    f'{_detail_cells(e["raw_metrics"])}'
                    f'</div></td></tr>\n'
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
    cursor: pointer;
    font-variant-numeric: tabular-nums;
    white-space: nowrap;
  }}
  .detail-row {{
    display: none;
  }}
  .detail-row.open {{
    display: table-row;
  }}
  .detail-inner {{
    display: flex;
    flex-wrap: wrap;
    gap: 0.3rem 1.5rem;
    padding: 6px 12px 8px;
    font-size: 0.82rem;
    color: #555;
  }}
  .detail-inner .d-pair {{
    display: inline-flex;
    gap: 0.35rem;
  }}
  .detail-inner .d-label {{
    color: #888;
  }}
  .detail-inner .d-val {{
    font-variant-numeric: tabular-nums;
  }}
  .detail-inner .d-std {{
    font-size: 0.88em;
    opacity: 0.6;
  }}
  .legend {{
    margin-top: 1.5rem;
    font-size: 0.78rem;
    color: #888;
    line-height: 1.8;
  }}
  .scenario-info {{
    display: flex;
    align-items: center;
    gap: 1rem;
    background: #f5f6fa;
    border-radius: 8px;
    padding: 1rem;
    margin-bottom: 1.5rem;
  }}
  .scenario-img {{
    width: 160px;
    height: auto;
    border-radius: 6px;
    background: #e0e2ea;
    flex-shrink: 0;
  }}
  .scenario-desc {{
    font-size: 0.88rem;
    color: #333;
    line-height: 1.6;
  }}
  .scenario-desc strong {{
    font-size: 1rem;
    color: #1a1a2e;
  }}
  .timeouts {{
    display: inline-block;
    margin-top: 0.3rem;
    font-size: 0.82rem;
    color: #666;
    background: #e8eaf0;
    border-radius: 4px;
    padding: 0.15rem 0.5rem;
  }}
  .score-std {{
    font-size: 0.85em;
    opacity: 0.7;
  }}
  .code-link {{
    font-size: 0.78em;
    opacity: 0.55;
    margin-left: 0.15em;
  }}
  .code-link:hover {{ opacity: 1; }}
  @media (max-width: 640px) {{
    body {{
      margin: 0;
      padding: 0.5rem 0;
    }}
    .scenario-info {{
      flex-direction: column;
      padding: 0.5rem;
      margin: 0 0.25rem 0.75rem;
      border-radius: 0;
    }}
    .scenario-img {{
      width: 100%;
      height: auto;
    }}
    h1 {{ font-size: 1.2rem; padding: 0 0.25rem; margin-bottom: 0.1rem; }}
    .subtitle {{ font-size: 0.78rem; margin-bottom: 1rem; padding: 0 0.25rem; }}
    .legend {{ padding: 0 0.25rem; }}
    table {{ font-size: 0.72rem; table-layout: fixed; }}
    th {{ padding: 6px 3px; font-size: 0.68rem; }}
    td {{ padding: 4px 3px; overflow: hidden; text-overflow: ellipsis; }}
    th:nth-child(1), td:nth-child(1) {{ width: 1.5rem; }}
    th:nth-child(2), td:nth-child(2) {{ width: auto; }}
    th:nth-child(3), td:nth-child(3) {{ width: 4.5rem; }}
    th:nth-child(4), td:nth-child(4) {{ width: 3rem; }}
    th:nth-child(5), td:nth-child(5) {{ width: 4.5rem; }}
    th:nth-child(6), td:nth-child(6) {{ width: 3.5rem; }}
    tr.scenario-header td {{ padding: 7px 4px; font-size: 0.82rem; }}
    tr.diff-header td {{ padding: 4px 4px; font-size: 0.7rem; }}
    .score-cell {{ white-space: normal; }}
    .score-std {{ display: block; }}
    .detail-inner {{ font-size: 0.76rem; gap: 0.2rem 1rem; }}
    .detail-inner .d-pair {{ gap: 0.2rem; }}
  }}
</style>
</head>
<body>
<h1>robot-sandbox Leaderboard</h1>
<p class="subtitle">
  Score&nbsp;=&nbsp;mission targets found / total mission targets (0–1)&nbsp;&nbsp;·&nbsp;&nbsp;
  Click a score to show raw metrics&nbsp;&nbsp;·&nbsp;&nbsp;
  <a href="https://github.com/thaije/robot-sandbox" target="_blank">GitHub ↗</a>
</p>
{info_panels}
<table>
<thead>
<tr>
  <th>#</th>
  <th>Stack</th>
  <th>Score (mean <span class="score-std">± std</span>)</th>
  <th>Seeds × Runs</th>
  <th>Submitted</th>
  <th>Version</th>
</tr>
</thead>
<tbody>
{rows_html}</tbody>
</table>
<p class="legend">
  Italic rows = baselines (not ranked — reference floor/ceiling only).<br>
  Seeds × Runs = number of seeds × runs per seed (total runs = seeds × runs_per_seed).<br>
  Score = <code>found_ratio</code>: mission targets found / total mission targets.<br>
  Click a score cell to expand raw metrics (mean ± std across runs).
</p>
<script>
document.querySelectorAll('.score-cell').forEach(function(cell) {{
  cell.addEventListener('click', function() {{
    var row = this.closest('tr');
    var detail = row ? row.nextElementSibling : null;
    if (detail && detail.classList.contains('detail-row')) {{
      detail.classList.toggle('open');
    }}
  }});
}});
</script>
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
