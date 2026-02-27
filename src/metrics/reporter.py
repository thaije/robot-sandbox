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
    # collision_events: {t, count}
    if "count" in item:
        return f"t={item.get('t', '?'):6.1f}s  collision #{item['count']}"
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
    for k, v in sc.raw_metrics.items():
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
            line = f"║    {k}: {display_v}"
            lines.append(line + " " * (w - 2 - len(line) + 2) + "║")
    lines.append("╚" + "═" * (w - 2) + "╝")
    return "\n".join(lines)


def write_results(scorecard: Scorecard, output_dir: Path) -> Path:
    output_dir.mkdir(parents=True, exist_ok=True)
    out_file = output_dir / f"{scorecard.scenario_name}.json"
    with open(out_file, "w") as f:
        json.dump(scorecard.__dict__, f, indent=2, default=str)
    return out_file
