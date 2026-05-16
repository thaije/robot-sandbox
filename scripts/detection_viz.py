#!/usr/bin/env python3.12
"""detection_viz.py — Overlay detection submissions on the ground-truth floor plan.

Reads a results JSON (with submission_log + ground_truth_objects) and renders
a PNG showing every submitted detection on the occupancy-grid map.

Map legend
----------
- Coloured filled circles = ground-truth mission-target objects
    fire_extinguisher = red, first_aid_kit = green, hazard_sign = cyan,
    person = yellow, exit_sign = purple, other = magenta
- Grey filled circles = non-mission environmental objects
- Green ring + line to GT = TP (true positive)
- Yellow ring = DP (duplicate positive)
- Red ✕ + distance annotation = FP (false positive, outside threshold)
- Orange ✕ + line to GT = FP_LOS (within threshold but no line of sight)
- Grey ✕ (faint) = IGN (unknown type, not penalised)

Companion to world_state.py: that script renders robot + live view; this one
renders completed-run detections for post-hoc analysis.

Usage
-----
    python3.12 scripts/detection_viz.py --results results/run.json
    python3.12 scripts/detection_viz.py --results results/run.json --png /tmp/viz.png
    python3.12 scripts/detection_viz.py --results results/run.json --state /tmp/arst_worlds/world_state.json

Default PNG output: detections.png in the repo root (next to arst_world_map.png).
If --state is omitted, defaults to /tmp/arst_worlds/world_state.json (must exist
or be explicitly provided for completed runs where the sim has been torn down).
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

WORLD_STATE = Path("/tmp/arst_worlds/world_state.json")
REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PNG = REPO_ROOT / "detections.png"

_SCALE = 40  # px per metre (matches world_state.py)

TYPE_COLOR = {
    "fire_extinguisher": (50, 50, 220),
    "first_aid_kit": (50, 180, 50),
    "hazard_sign": (30, 200, 230),
    "person": (60, 220, 220),
    "exit_sign": (180, 60, 180),
}
NON_MISSION_COLOR = (180, 180, 180)

OUTCOME_COLOR = {
    "TP": (0, 200, 0),
    "DP": (200, 200, 50),
    "FP": (0, 0, 220),
    "FP_LOS": (0, 140, 255),
    "IGN": (120, 120, 120),
}


def _read_pgm(path: Path) -> tuple[list[list[int]], int, int]:
    data = path.read_bytes()
    tokens: list[str] = []
    i = 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i+1] in (b" ", b"\t", b"\n", b"\r"):
            i += 1
        if i >= len(data):
            break
        if data[i:i+1] == b"#":
            while i < len(data) and data[i:i+1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and data[j:j+1] not in (b" ", b"\t", b"\n", b"\r"):
            j += 1
        tokens.append(data[i:j].decode())
        i = j
    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])
    if magic == "P5":
        raw = data[i + 1:]
        if maxval <= 255:
            pixels = [[raw[r * w + c] for c in range(w)] for r in range(h)]
        else:
            pixels = [[(raw[(r*w+c)*2] << 8 | raw[(r*w+c)*2+1]) for c in range(w)] for r in range(h)]
    else:
        vals = list(map(int, data[i:].split()))
        pixels = [[vals[r * w + c] for c in range(w)] for r in range(h)]
    if maxval != 255:
        pixels = [[int(v * 255 / maxval) for v in row] for row in pixels]
    return pixels, w, h


def render(
    pixels: list[list[int]], W: int, H: int, res: float,
    gt_objects: dict, obstacles: list[dict],
    submission_log: list[dict],
    match_threshold: float,
    out_path: str,
) -> None:
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("ERROR: opencv-python (cv2) required. Install with: pip install opencv-python-headless", file=sys.stderr)
        sys.exit(1)

    SCALE = _SCALE
    ppc = max(1, int(res * SCALE))
    iw, ih = int(W * res * SCALE), int(H * res * SCALE)

    grid_gray = np.where(np.array(pixels, dtype=np.uint8) >= 128, 255, 50)
    grid_up = cv2.resize(grid_gray.astype(np.uint8), (W * ppc, H * ppc),
                         interpolation=cv2.INTER_NEAREST)
    img = cv2.cvtColor(grid_up, cv2.COLOR_GRAY2BGR)

    for obs in obstacles:
        cx, cy = obs["x"], obs["y"]
        hw, hh = obs["w"] / 2, obs["h"] / 2
        x1 = int((cx - hw) * SCALE)
        x2 = int((cx + hw) * SCALE)
        y1 = ih - int((cy + hh) * SCALE)
        y2 = ih - int((cy - hh) * SCALE)
        name = obs.get("name", "")
        if "desk" in name or "table" in name:
            color = (100, 70, 40)
        elif "chair" in name:
            color = (80, 80, 80)
        else:
            color = (130, 130, 150)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), 1)

    drawn_gt_positions: dict[str, tuple[int, int]] = {}
    for key, obj in gt_objects.items():
        ix = int(obj["x"] * SCALE)
        iy = ih - int(obj["y"] * SCALE)
        drawn_gt_positions[key] = (ix, iy)
        is_mission = obj.get("mission_target", True)
        color = TYPE_COLOR.get(obj["type"], (180, 0, 180)) if is_mission else NON_MISSION_COLOR
        cv2.circle(img, (ix, iy), 9, color, -1)
        cv2.circle(img, (ix, iy), 9, (0, 0, 0), 1)
        label = f"{key}"
        cv2.putText(img, label, (ix - 6, iy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    for entry in submission_log:
        outcome = entry.get("outcome", "")
        sx = entry["submitted_x"]
        sy = entry["submitted_y"]
        six = int(sx * SCALE)
        siy = ih - int(sy * SCALE)
        col = OUTCOME_COLOR.get(outcome, (120, 120, 120))

        gt_key = entry.get("nearest_gt_key")
        gt_pos = drawn_gt_positions.get(gt_key) if gt_key else None

        if outcome == "TP":
            cv2.circle(img, (six, siy), 7, col, 2)
            if gt_pos:
                cv2.line(img, (six, siy), gt_pos, col, 1, cv2.LINE_AA)
        elif outcome == "DP":
            cv2.circle(img, (six, siy), 7, col, 2)
        elif outcome in ("FP", "FP_LOS"):
            d = int(six - 4), int(siy - 4)
            d2 = (int(six + 4), int(siy + 4))
            d3 = (int(six + 4), int(siy - 4))
            d4 = (int(six - 4), int(siy + 4))
            cv2.line(img, d, d2, col, 2)
            cv2.line(img, d3, d4, col, 2)
            if gt_pos and outcome == "FP_LOS":
                cv2.line(img, (six, siy), gt_pos, col, 1, cv2.LINE_AA)
            dist = entry.get("distance_to_nearest")
            if dist is not None and dist <= match_threshold * 2:
                ann = f"{dist:.1f}m"
                cv2.putText(img, ann, (six + 6, siy - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, col, 1)
        elif outcome == "IGN":
            d = int(six - 3), int(siy - 3)
            d2 = (int(six + 3), int(siy + 3))
            d3 = (int(six + 3), int(siy - 3)
                  )
            d4 = (int(six - 3), int(siy + 3))
            cv2.line(img, d, d2, col, 1)
            cv2.line(img, d3, d4, col, 1)

    legend_y = 20
    legend_items = [
        ("TP", OUTCOME_COLOR["TP"], "ring"),
        ("DP", OUTCOME_COLOR["DP"], "ring"),
        ("FP", OUTCOME_COLOR["FP"], "X"),
        ("FP_LOS", OUTCOME_COLOR["FP_LOS"], "X"),
        ("IGN", OUTCOME_COLOR["IGN"], "X"),
    ]
    for label, col, style in legend_items:
        cv2.putText(img, label, (10, legend_y),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, col, 1)
        legend_y += 16

    count_by_outcome: dict[str, int] = {}
    for entry in submission_log:
        o = entry.get("outcome", "?")
        count_by_outcome[o] = count_by_outcome.get(o, 0) + 1
    summary = "  ".join(f"{o}:{n}" for o, n in sorted(count_by_outcome.items()))
    cv2.putText(img, summary, (10, ih - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

    cv2.imwrite(out_path, img)
    print(out_path)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Overlay detection submissions on the ground-truth floor plan"
    )
    parser.add_argument("--results", required=True, help="Results JSON with submission_log")
    parser.add_argument("--state", default=str(WORLD_STATE), help="world_state.json path (for PGM map)")
    parser.add_argument("--png", default=None, help="Output PNG path (default: repo-root/detections.png)")
    parser.add_argument("--match-threshold", type=float, default=1.5, help="Near-miss annotation threshold (m)")
    args = parser.parse_args()

    results_path = Path(args.results)
    if not results_path.exists():
        print(f"ERROR: {results_path} not found", file=sys.stderr)
        sys.exit(1)

    data = json.loads(results_path.read_text())
    raw = data.get("raw_metrics", data)
    submission_log = raw.get("submission_log", [])
    gt_objects = raw.get("ground_truth_objects", {})

    if not gt_objects:
        print("WARNING: no ground_truth_objects in results JSON; drawing submissions without GT markers", file=sys.stderr)

    state_path = Path(args.state)
    if not state_path.exists():
        print(
            f"ERROR: {state_path} not found.\n"
            "Provide --state path to a world_state.json (or run the sim first).",
            file=sys.stderr,
        )
        sys.exit(1)

    state = json.loads(state_path.read_text())
    pgm_path = Path(state["map_pgm"])
    res = float(state.get("map_resolution", 0.5))
    obstacles = state.get("obstacles", [])

    if not pgm_path.exists():
        print(f"ERROR: PGM not found at {pgm_path}", file=sys.stderr)
        sys.exit(1)

    pixels, W, H = _read_pgm(pgm_path)

    out_path = args.png or str(DEFAULT_PNG)

    render(pixels, W, H, res, gt_objects, obstacles, submission_log,
           args.match_threshold, out_path)


if __name__ == "__main__":
    main()