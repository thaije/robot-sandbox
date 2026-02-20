"""Randomised object placement engine — Step 1.4.

Placement guarantees
--------------------
- Zone-constrained : candidates are drawn only from ``placement_zones`` bounding boxes
  defined in the template ``config.yaml``.
- Wall-aware       : the ground-truth PGM is used to reject any candidate whose
  surrounding footprint (radius = clearance) touches an occupied cell.
- Clearance-checked: placed objects must be at least ``clearance`` metres apart.
- Deterministic    : the same (objects, seed) pair always produces the same layout.
"""
from __future__ import annotations

import math
import random
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import numpy as np


@dataclass
class PlacedObject:
    model_type: str
    x: float
    y: float
    yaw: float = 0.0


class PlacementError(RuntimeError):
    """Raised when a valid position cannot be found within the attempt budget."""


class ObjectPlacer:
    """Place target objects inside a world template."""

    #: Maximum random samples per object before giving up.
    MAX_ATTEMPTS: int = 2000

    def __init__(self, template_config: dict, clearance: float | None = None) -> None:
        """
        Parameters
        ----------
        template_config:
            Parsed ``worlds/templates/<name>/config.yaml`` dict.
        clearance:
            Override the template's ``placement_clearance`` (metres).
        """
        self._zones: list[dict] = template_config.get("placement_zones", [])
        self._clearance: float = clearance if clearance is not None else float(
            template_config.get("placement_clearance", 0.5)
        )
        self._gt_mask, self._resolution, self._world_h = _load_gt_mask(template_config)
        self._zone_weights: list[float] = _zone_area_weights(self._zones)

    # ── Public API ────────────────────────────────────────────────────────────

    def place(self, objects: list[dict[str, Any]], seed: int) -> list[PlacedObject]:
        """Return a deterministic, collision-free placement list.

        Parameters
        ----------
        objects:
            List of ``{"type": str, "count": int}`` dicts from the scenario config.
        seed:
            Integer random seed for reproducibility.

        Returns
        -------
        list[PlacedObject]
            One entry per individual object instance.

        Raises
        ------
        PlacementError
            If any object cannot be placed after ``MAX_ATTEMPTS`` tries.
        """
        rng = random.Random(seed)
        placed: list[PlacedObject] = []

        for spec in objects:
            model_type: str = spec["type"]
            count: int = int(spec.get("count", 1))
            for _ in range(count):
                obj = self._place_one(model_type, rng, placed)
                placed.append(obj)

        return placed

    # ── Private helpers ───────────────────────────────────────────────────────

    def _place_one(
        self,
        model_type: str,
        rng: random.Random,
        already: list[PlacedObject],
    ) -> PlacedObject:
        for _ in range(self.MAX_ATTEMPTS):
            zone = rng.choices(self._zones, weights=self._zone_weights, k=1)[0]
            x = rng.uniform(zone["x_min"], zone["x_max"])
            y = rng.uniform(zone["y_min"], zone["y_max"])
            if self._is_valid(x, y, already):
                # Cardinal yaws look neater in a rectilinear world
                yaw = rng.choice([0.0, math.pi / 2, math.pi, 3 * math.pi / 2])
                return PlacedObject(model_type=model_type, x=x, y=y, yaw=yaw)

        raise PlacementError(
            f"Could not place {model_type!r} after {self.MAX_ATTEMPTS} attempts. "
            "Consider reducing object count or placement_clearance."
        )

    def _is_valid(self, x: float, y: float, placed: list[PlacedObject]) -> bool:
        """True iff (x, y) is free, wall-clear, and inter-object-clear."""
        # ── Wall / boundary clearance ─────────────────────────────────────────
        # Convert clearance to pixel radius; expand by 1 to be safe
        c_px = int(math.ceil(self._clearance / self._resolution))
        col_c = int(x / self._resolution)
        row_c = int((self._world_h - y) / self._resolution)  # y-axis inverted in PGM

        rows, cols = self._gt_mask.shape
        radius_sq = (c_px + 0.5) ** 2
        for dr in range(-c_px - 1, c_px + 2):
            for dc in range(-c_px - 1, c_px + 2):
                if dr * dr + dc * dc > radius_sq:
                    continue  # outside the disk
                r, c = row_c + dr, col_c + dc
                if r < 0 or r >= rows or c < 0 or c >= cols:
                    return False  # treat out-of-bounds as occupied
                if not self._gt_mask[r, c]:
                    return False  # occupied cell → too close to a wall

        # ── Inter-object clearance ────────────────────────────────────────────
        for p in placed:
            if math.hypot(x - p.x, y - p.y) < self._clearance:
                return False

        return True


# ── Module-level helpers ──────────────────────────────────────────────────────

def _load_gt_mask(template_config: dict) -> tuple[np.ndarray, float, float]:
    """Read the ground-truth PGM and return (mask, resolution, world_height).

    The mask is a boolean array (ROWS × COLS) where True = navigable / free.
    World height is derived from ``rows × resolution`` and is needed to convert
    world-frame y coordinates to PGM row indices (the y-axis is inverted).
    """
    gt_cfg = template_config.get("ground_truth_map", {})
    pgm_path = Path(gt_cfg.get("pgm", ""))
    resolution = float(gt_cfg.get("resolution", 0.5))

    if not pgm_path.exists():
        raise FileNotFoundError(
            f"Ground-truth PGM not found: {pgm_path}. "
            "Run worlds/templates/<template>/generate_gt_map.py first."
        )

    data = pgm_path.read_bytes()
    # P5 binary PGM: header lines separated by '\n', then raw uint8 pixels
    lines = data.split(b"\n")
    if lines[0].strip() != b"P5":
        raise ValueError(f"Expected P5 PGM, got magic: {lines[0]!r}")
    map_cols, map_rows = map(int, lines[1].split())
    # header = "P5\n{cols} {rows}\n{maxval}\n"
    header_len = sum(len(lines[i]) + 1 for i in range(3))
    pixels = np.frombuffer(data[header_len:], dtype=np.uint8).reshape(map_rows, map_cols)

    world_h = map_rows * resolution
    mask = pixels >= 128  # 254 = free, 0 = wall in our convention
    return mask, resolution, world_h


def _zone_area_weights(zones: list[dict]) -> list[float]:
    """Return area-proportional sampling weights for the placement zones."""
    areas = [
        max(0.0, (z["x_max"] - z["x_min"]) * (z["y_max"] - z["y_min"]))
        for z in zones
    ]
    total = sum(areas) or 1.0
    return [a / total for a in areas]
