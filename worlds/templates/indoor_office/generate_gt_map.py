#!/usr/bin/env python3
"""
Generate ground truth PGM + YAML map for indoor_office template.

Run from repo root:
    python3 worlds/templates/indoor_office/generate_gt_map.py

Output:
    worlds/templates/indoor_office/ground_truth_map.pgm
    worlds/templates/indoor_office/ground_truth_map.yaml

Coordinate conventions:
    World:  x right, y up,  origin bottom-left (0,0)
    PGM:    col right, row down, row 0 = top = world y=WORLD_H
    ROS map YAML origin = bottom-left corner of the map in world frame

The PGM is the authoritative source for coverage calculation.
Furniture is excluded — it is placed randomly per run by the object placer.
"""
import math
from pathlib import Path

import numpy as np

# ─── World parameters ────────────────────────────────────────────────────────
RESOLUTION  = 0.5          # meters per pixel
WORLD_W     = 20.0         # meters  (x axis)
WORLD_H     = 15.0         # meters  (y axis)
COLS        = int(WORLD_W / RESOLUTION)   # 40
ROWS        = int(WORLD_H / RESOLUTION)   # 30

FREE        = 254   # ROS convention: >free_thresh → free
OCCUPIED    = 0     # ROS convention: <occ_thresh  → occupied

OUT_DIR = Path(__file__).parent


def _mark(grid: np.ndarray, cx: float, cy: float, sx: float, sy: float) -> None:
    """Mark the rectangle [cx±sx/2, cy±sy/2] as OCCUPIED in the grid."""
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2

    # Column indices (x axis — same direction in world and image)
    c0 = max(0,      math.floor(x0 / RESOLUTION))
    c1 = min(COLS,   math.ceil (x1 / RESOLUTION))

    # Row indices (y axis — inverted: row 0 = world y=WORLD_H)
    r0 = max(0,      math.floor((WORLD_H - y1) / RESOLUTION))
    r1 = min(ROWS,   math.ceil ((WORLD_H - y0) / RESOLUTION))

    grid[r0:r1, c0:c1] = OCCUPIED


def generate() -> np.ndarray:
    grid = np.full((ROWS, COLS), FREE, dtype=np.uint8)

    # ── Outer walls ──────────────────────────────────────────────────────────
    _mark(grid, 10.0,  0.0, 20.4, 0.2)   # south
    _mark(grid, 10.0, 15.0, 20.4, 0.2)   # north
    _mark(grid,  0.0,  7.5, 0.2, 15.0)   # west
    _mark(grid, 20.0,  7.5, 0.2, 15.0)   # east

    # ── Interior walls (door openings are gaps — not marked) ─────────────────
    # Office A / B divider at x=9  (door y=[2.5, 3.5])
    _mark(grid, 9.0, 1.25, 0.15, 2.5)    # south segment  y=[0.0, 2.5]
    _mark(grid, 9.0, 5.0,  0.15, 3.0)    # north segment  y=[3.5, 6.5]

    # Office → Corridor at y=6.5  (doors x=[2,3] and x=[14,15])
    _mark(grid,  1.0, 6.5, 2.0,  0.15)   # west  x=[0,  2]
    _mark(grid,  8.5, 6.5, 11.0, 0.15)   # mid   x=[3, 14]
    _mark(grid, 17.5, 6.5, 5.0,  0.15)   # east  x=[15,20]

    # Corridor → Meeting Room at y=9.5  (door x=[9,10])
    _mark(grid,  4.5, 9.5, 9.0,  0.15)   # west  x=[0,  9]
    _mark(grid, 15.0, 9.5, 10.0, 0.15)   # east  x=[10,20]

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
        f"origin: [0.0, 0.0, 0.0]\n"   # bottom-left corner in world frame
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
