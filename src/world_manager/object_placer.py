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
    z: float = 0.0          # 0.0 = floor; >0 = elevated (on furniture surface)


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
        self._obstacles: list[dict] = template_config.get("obstacles", [])

    # ── Public API ────────────────────────────────────────────────────────────

    def place(self, objects: list[dict[str, Any]], seed: int) -> list[PlacedObject]:
        """Return a deterministic, collision-free placement list.

        Parameters
        ----------
        objects:
            List of object spec dicts from the scenario config.  Recognised keys:
            ``type`` (str), ``count`` (int), ``placement`` (str, default
            ``"random"``), ``z_offset`` (float, default 0.0),
            ``surface`` (str — surface_class to place on, e.g. ``"desk"``).
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
                obj = self._place_one(model_type, rng, placed, spec)
                placed.append(obj)

        return placed

    # ── Private helpers ───────────────────────────────────────────────────────

    def _place_one(
        self,
        model_type: str,
        rng: random.Random,
        already: list[PlacedObject],
        spec: dict | None = None,
    ) -> PlacedObject:
        spec = spec or {}
        z_offset = float(spec.get("z_offset", 0.0))
        surface_class = spec.get("surface", "")

        # ── Surface placement ──────────────────────────────────────────────────
        if surface_class:
            candidates = [
                obs for obs in self._obstacles
                if obs.get("surface_class", "") == surface_class
                   and obs.get("surface_height", 0.0) > 0.0
            ]
            if not candidates:
                raise PlacementError(
                    f"No obstacles with surface_class={surface_class!r} found in template."
                )
            rng.shuffle(candidates)
            for surf in candidates:
                # Place anywhere on the surface footprint (with small margin)
                margin = 0.1
                hw, hh = surf["w"] / 2 - margin, surf["h"] / 2 - margin
                if hw <= 0 or hh <= 0:
                    continue
                for _ in range(self.MAX_ATTEMPTS):
                    x = rng.uniform(surf["x"] - hw, surf["x"] + hw)
                    y = rng.uniform(surf["y"] - hh, surf["y"] + hh)
                    # Clearance check against other placed objects only
                    # (PGM check skipped — surface is inside occupied footprint)
                    if self._is_valid(x, y, already, skip_pgm=True):
                        yaw = rng.choice([0.0, math.pi / 2, math.pi, 3 * math.pi / 2])
                        z = float(surf["surface_height"]) + z_offset
                        return PlacedObject(model_type=model_type, x=x, y=y, yaw=yaw, z=z)
            raise PlacementError(
                f"Could not place {model_type!r} on surface_class={surface_class!r} "
                f"after {self.MAX_ATTEMPTS} attempts per candidate."
            )

        # ── Standard zone-based floor placement ───────────────────────────────
        for _ in range(self.MAX_ATTEMPTS):
            zone = rng.choices(self._zones, weights=self._zone_weights, k=1)[0]
            x = rng.uniform(zone["x_min"], zone["x_max"])
            y = rng.uniform(zone["y_min"], zone["y_max"])
            if self._is_valid(x, y, already):
                yaw = rng.choice([0.0, math.pi / 2, math.pi, 3 * math.pi / 2])
                return PlacedObject(model_type=model_type, x=x, y=y, yaw=yaw, z=z_offset)

        raise PlacementError(
            f"Could not place {model_type!r} after {self.MAX_ATTEMPTS} attempts. "
            "Consider reducing object count or placement_clearance."
        )

    def _is_valid(
        self, x: float, y: float, placed: list[PlacedObject], skip_pgm: bool = False
    ) -> bool:
        """True iff (x, y) is free, wall-clear (unless *skip_pgm*), and inter-object-clear."""
        if not skip_pgm:
            # ── Wall / boundary clearance ─────────────────────────────────────
            c_px = int(math.ceil(self._clearance / self._resolution))
            col_c = int(x / self._resolution)
            row_c = int((self._world_h - y) / self._resolution)  # y-axis inverted in PGM

            rows, cols = self._gt_mask.shape
            radius_sq = (c_px + 0.5) ** 2
            for dr in range(-c_px - 1, c_px + 2):
                for dc in range(-c_px - 1, c_px + 2):
                    if dr * dr + dc * dc > radius_sq:
                        continue
                    r, c = row_c + dr, col_c + dc
                    if r < 0 or r >= rows or c < 0 or c >= cols:
                        return False
                    if not self._gt_mask[r, c]:
                        return False

        # ── Inter-object clearance (always applied) ───────────────────────────
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
