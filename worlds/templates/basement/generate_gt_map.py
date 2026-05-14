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
RESOLUTION  = 0.05         # meters per pixel
WORLD_W     = 6.0          # meters  (x axis, E-W)
WORLD_H     = 8.0          # meters  (y axis, N-S)
COLS        = int(WORLD_W / RESOLUTION)   # 120
ROWS        = int(WORLD_H / RESOLUTION)   # 160

FREE        = 254
OCCUPIED    = 0

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
    _mark(grid, 3.0, 0.0, 6.2, WALL_T)       # south (y=0)
    _mark(grid, 3.0, 8.0, 6.2, WALL_T)       # north (y=8)
    _mark(grid, 0.0, 4.0, WALL_T, 8.2)       # west (x=0)
    _mark(grid, 6.0, 4.0, WALL_T, 8.2)       # east (x=6)

    # ── Internal wall at x=4 (divides big room from side rooms) ─────────────
    # R1↔R2 passage at y=[1.1, 1.9], R1↔R3 passage at y=[5.1, 5.9]

    # Below R2 passage: x=4, y=[0, 1.1]
    _mark(grid, 4.0, 0.55, WALL_T, 1.1)
    # Between passages: x=4, y=[1.9, 5.1]
    _mark(grid, 4.0, 3.5, WALL_T, 3.2)
    # Above R3 passage: x=4, y=[5.9, 8]
    _mark(grid, 4.0, 6.95, WALL_T, 2.1)

    # ── Wall between R2 and R3: y=3, x=[4, 6] ──────────────────────────────
    _mark(grid, 5.0, 3.0, 2.0, WALL_T)

    # ── Support pillars in Room 1 ──────────────────────────────────────────
    PILLAR_R = 0.1   # pillar radius (m)
    _mark(grid, 2.0, 3.0, PILLAR_R * 2, PILLAR_R * 2)
    _mark(grid, 2.0, 5.5, PILLAR_R * 2, PILLAR_R * 2)

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