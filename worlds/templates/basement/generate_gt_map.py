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
    ROS map YAML origin = bottom-left corner of the map in world frame

The PGM is the authoritative source for coverage calculation.
Furniture is excluded — it is placed per run by the object placer.
"""
import math
from pathlib import Path

import numpy as np

# ─── World parameters ────────────────────────────────────────────────────────
RESOLUTION  = 0.5          # meters per pixel
WORLD_W     = 12.0         # meters  (x axis)
WORLD_H     = 8.0          # meters  (y axis)
COLS        = int(WORLD_W / RESOLUTION)   # 24
ROWS        = int(WORLD_H / RESOLUTION)   # 16

FREE        = 254   # ROS convention: >free_thresh → free
OCCUPIED    = 0     # ROS convention: <occ_thresh  → occupied

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


def generate() -> np.ndarray:
    grid = np.full((ROWS, COLS), FREE, dtype=np.uint8)

    # ── Outer walls ──────────────────────────────────────────────────────────
    _mark(grid, 6.0,  0.0, 12.4, 0.2)    # south
    _mark(grid, 6.0,  8.0, 12.4, 0.2)    # north
    _mark(grid,  0.0,  4.0, 0.2, 8.4)     # west
    _mark(grid, 12.0,  4.0, 0.2, 8.4)     # east

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
    for r in range(ROWS):
        row_str = "".join("#" if grid[r, c] == OCCUPIED else "." for c in range(COLS))
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