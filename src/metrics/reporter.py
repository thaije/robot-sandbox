"""
Metrics reporter — Steps 3.9 + 3.12.
Collects results, writes JSON/CSV, displays ASCII scorecard.
"""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from metrics.scoring import Scorecard, numeric_to_grade


SCORECARD_WIDTH = 60


def _bar(score: float, width: int = 10) -> str:
    filled = round(score / 100 * width)
    return "█" * filled + "░" * (width - filled)


def _format_event(item: Any) -> str:
    """Format a single event dict as a compact human-readable string."""
    if not isinstance(item, dict):
        return str(item)
    # detection_events: {class_id, class_name, timestamp}
    if "class_name" in item:
        return f"t={item.get('timestamp', '?'):6.1f}s  {item['class_name']} [label {item.get('class_id', '?')}]"
    # collision_events: {t, count}  (no min_range key)
    if "count" in item and "min_range" not in item:
        return f"t={item.get('t', '?'):6.1f}s  collision #{item['count']}"
    # near_miss_events: {t, count, min_range}
    if "min_range" in item:
        return f"t={item.get('t', '?'):6.1f}s  near-miss #{item['count']}  ({item['min_range']:.2f} m)"
    # fallback
    return str(item)


def render_scorecard(sc: Scorecard) -> str:
    lines = []
    w = SCORECARD_WIDTH
    lines.append("╔" + "═" * (w - 2) + "╗")
    lines.append(f"║  SCENARIO: {sc.scenario_name:<{w - 15}}║")
    lines.append("╠" + "═" * (w - 2) + "╣")
    lines.append(f"║  Status: {sc.status:<10}  Time: {sc.elapsed_seconds:.0f}s / {sc.timeout_seconds:.0f}s{'':<6}║")
    lines.append("╠" + "═" * (w - 2) + "╣")
    lines.append("║" + " " * (w - 2) + "║")

    icons = {"speed": "🏎 ", "accuracy": "🎯", "safety": "🛡 ", "efficiency": "⚡"}
    for cat in sc.categories:
        if cat.weight == 0.0:
            continue  # skip disabled categories (e.g. efficiency without LiDAR)
        icon = icons.get(cat.name, "  ")
        bar = _bar(cat.score)
        line = f"║  {icon} {cat.name.capitalize():<12}{cat.score:5.0f}/100  {bar}  {cat.grade}  {cat.grade_label:<13}║"
        lines.append(line)

    lines.append("║" + " " * (w - 2) + "║")
    lines.append("║  " + "─" * (w - 6) + "  ║")
    bar = _bar(sc.overall_score)
    lines.append(f"║  ⭐ Overall      {sc.overall_score:5.0f}/100  {bar}  {sc.overall_grade}  {sc.overall_grade_label:<13}║")
    lines.append("║" + " " * (w - 2) + "║")
    lines.append("╠" + "═" * (w - 2) + "╣")
    lines.append("║  Raw Metrics:" + " " * (w - 16) + "║")

    # ── Metric display order (grouped by scoring category) ──────────────────
    # Keys not listed here appear at the end in their original order.
    _ORDER = [
        # Movement / efficiency
        "meters_traveled",
        "exploration_coverage",
        "revisit_ratio",
        # Timing / speed
        "task_completion_time",
        "time_to_all_detections",
        "average_time_per_detection",
        # Detection / accuracy
        "found_ratio",
        "precision",
        "duplicate_rate",
        "mean_localization_error",
        "false_positive_count",
        "duplicate_count",
        "detection_count",
        "detection_by_type",
        "detection_events",
        # Safety
        "collision_count",
        "collision_events",
        "near_miss_count",
        "near_miss_events",
    ]
    # Suffix to append for specific metrics (exploration_coverage is stored as %, others as 0–1)
    _SUFFIX: dict[str, str] = {
        "exploration_coverage": "%",
    }

    ordered_keys = [k for k in _ORDER if k in sc.raw_metrics]
    remaining = [k for k in sc.raw_metrics if k not in set(_ORDER)]
    all_keys = ordered_keys + remaining

    for k in all_keys:
        v = sc.raw_metrics[k]
        suffix = _SUFFIX.get(k, "")
        if isinstance(v, list):
            header = f"║    {k} ({len(v)}):"
            lines.append(header + " " * (w - 2 - len(header) + 2) + "║")
            for item in v:
                sub = _format_event(item)
                sub_line = f"║      {sub}"
                lines.append(sub_line + " " * (w - 2 - len(sub_line) + 2) + "║")
        elif isinstance(v, dict):
            header = f"║    {k}:"
            lines.append(header + " " * (w - 2 - len(header) + 2) + "║")
            for dk, dv in v.items():
                if isinstance(dv, dict) and "detected" in dv and "total" in dv:
                    sub_line = f"║      {dk}: {dv['detected']}/{dv['total']}"
                else:
                    sub_line = f"║      {dk}: {dv}"
                lines.append(sub_line + " " * (w - 2 - len(sub_line) + 2) + "║")
        else:
            # Show integer-valued floats without decimal (e.g. collision_count: 3 not 3.0)
            display_v = int(v) if isinstance(v, float) and v == int(v) else v
            line = f"║    {k}: {display_v}{suffix}"
            lines.append(line + " " * (w - 2 - len(line) + 2) + "║")
    lines.append("╚" + "═" * (w - 2) + "╝")
    return "\n".join(lines)


def write_results(scorecard: Scorecard, output_dir: Path) -> Path:
    from datetime import datetime  # noqa: PLC0415
    output_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.now().strftime("%Y%m%dT%H%M%S")
    out_file = output_dir / f"{scorecard.scenario_name}_{ts}.json"
    with open(out_file, "w") as f:
        json.dump(scorecard.__dict__, f, indent=2, default=str)
    return out_file
