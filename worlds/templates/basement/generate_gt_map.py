#!/usr/bin/env python3
"""
Generate ground truth PGM + YAML map for basement template.

Run from repo root:
    python3 worlds/templates/basement/generate_gt_map.py

Output:
    worlds/templates/basement/ground_truth_map.pgm
    worlds/templates/basement/ground_truth_map.yaml

Coordinate conventions:
    World:  x right, y up,  origin bottom-left (0,0)
    PGM:    col right, row down, row 0 = top = world y=WORLD_H

The PGM is the authoritative source for coverage calculation.
Furniture is excluded — it is placed per run by the object placer.
"""
import math
from pathlib import Path

import numpy as np

# ─── World parameters ────────────────────────────────────────────────────────
RESOLUTION  = 0.05         # meters per pixel (higher res for 80cm ceiling + tight passages)
WORLD_W     = 8.0          # meters  (x axis)
WORLD_H     = 6.0          # meters  (y axis)
COLS        = int(WORLD_W / RESOLUTION)   # 160
ROWS        = int(WORLD_H / RESOLUTION)   # 120

FREE        = 254   # ROS convention: >free_thresh → free
OCCUPIED    = 0     # ROS convention: <occ_thresh  → occupied

WALL_T      = 0.1   # wall thickness (m)

OUT_DIR = Path(__file__).parent


def _mark(grid: np.ndarray, cx: float, cy: float, sx: float, sy: float) -> None:
    """Mark the rectangle [cx±sx/2, cy±sy/2] as OCCUPIED in the grid."""
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2

    c0 = max(0,      math.floor(x0 / RESOLUTION))
    c1 = min(COLS,   math.ceil (x1 / RESOLUTION))

    r0 = max(0,      math.floor((WORLD_H - y1) / RESOLUTION))
    r1 = min(ROWS,   math.ceil ((WORLD_H - y0) / RESOLUTION))

    grid[r0:r1, c0:c1] = OCCUPIED


def _clear(grid: np.ndarray, cx: float, cy: float, sx: float, sy: float) -> None:
    """Clear the rectangle [cx±sx/2, cy±sy/2] as FREE in the grid (for passages)."""
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2

    c0 = max(0,      math.floor(x0 / RESOLUTION))
    c1 = min(COLS,   math.ceil (x1 / RESOLUTION))

    r0 = max(0,      math.floor((WORLD_H - y1) / RESOLUTION))
    r1 = min(ROWS,   math.ceil ((WORLD_H - y0) / RESOLUTION))

    grid[r0:r1, c0:c1] = FREE


def generate() -> np.ndarray:
    grid = np.full((ROWS, COLS), FREE, dtype=np.uint8)

    # ── Outer walls ──────────────────────────────────────────────────────────
    _mark(grid, 4.0, 0.0, 8.2, WALL_T)      # south (y=0)
    _mark(grid, 4.0, 6.0, 8.2, WALL_T)      # north (y=6)
    _mark(grid, 0.0, 3.0, WALL_T, 6.2)       # west (x=0)
    _mark(grid, 8.0, 3.0, WALL_T, 6.2)       # east (x=8)

    # ── Internal walls ───────────────────────────────────────────────────────
    # Room 3 west wall: x=5.5, y=[0, 2.1] (below passage)
    _mark(grid, 5.5, 1.05, WALL_T, 2.1)
    # Room 3 west wall: x=5.5, y=[2.1, 2.9] is PASSAGE (0.8m wide) — leave open
    # Passage arch above: box at (5.5, 2.5, 0.1, 0.8) z=[0.55, 0.8] — not in 2D map
    # Room 3 west wall: x=5.5, y=[2.9, 4.0] (above passage to ceiling area)
    _mark(grid, 5.5, 3.45, WALL_T, 1.1)

    # Room 3 north wall: y=3, x=[5.5, 8]
    _mark(grid, 6.75, 3.0, 2.5, WALL_T)

    # Room 1 north wall: y=4, x=[0, 5.5]
    _mark(grid, 2.75, 4.0, 5.5, WALL_T)

    # Corridor wall: y=[3,4] x=[5.5,6] — south side of corridor
    _mark(grid, 5.75, 3.5, 0.5, WALL_T)

    # Room 2 west wall above passage: x=6, y=[4.8, 6]
    _mark(grid, 6.0, 5.4, WALL_T, 1.2)
    # Passage arch at y=[4.0, 4.8] — leave open
    # (arch is above, at z level, not in 2D map)

    return grid


def save_pgm(grid: np.ndarray, path: Path) -> None:
    """Write P5 (binary) PGM."""
    rows, cols = grid.shape
    header = f"P5\n{cols} {rows}\n255\n".encode()
    path.write_bytes(header + grid.tobytes())
    print(f"Wrote {path}  ({cols}×{rows} px, {RESOLUTION} m/px)")


def save_yaml(path: Path) -> None:
    yaml = (
        f"image: ground_truth_map.pgm\n"
        f"resolution: {RESOLUTION}\n"
        f"origin: [0.0, 0.0, 0.0]\n"
        f"negate: 0\n"
        f"occupied_thresh: 0.65\n"
        f"free_thresh: 0.196\n"
    )
    path.write_text(yaml)
    print(f"Wrote {path}")


def print_ascii(grid: np.ndarray) -> None:
    """Print a compact ASCII preview (# = wall, . = free)."""
    print(f"\nGround truth map ({COLS}×{ROWS}, N↑):\n")
    step = max(1, ROWS // 40)
    for r in range(0, ROWS, step):
        row_str = "".join("#" if grid[r, c] == OCCUPIED else "." for c in range(0, COLS, step))
        y_world = WORLD_H - (r + 0.5) * RESOLUTION
        print(f"  y≈{y_world:4.1f}  {row_str}")
    print()
    free_cells = int(np.sum(grid == FREE))
    total_cells = ROWS * COLS
    print(f"  Free: {free_cells}/{total_cells} cells = "
          f"{free_cells * RESOLUTION**2:.1f} m² explorable area\n")


if __name__ == "__main__":
    grid = generate()
    print_ascii(grid)
    save_pgm(grid, OUT_DIR / "ground_truth_map.pgm")
    save_yaml(OUT_DIR / "ground_truth_map.yaml")