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
            strategy: str = spec.get("placement", "random")
            if strategy == "clustered":
                # Place all instances of this type as a cluster together
                cluster = self._place_clustered(model_type, rng, placed, spec, count)
                placed.extend(cluster)
            else:
                for _ in range(count):
                    obj = self._place_one_strategy(model_type, rng, placed, spec)
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

    # ── Strategy dispatcher ───────────────────────────────────────────────────

    def _place_one_strategy(
        self,
        model_type: str,
        rng: random.Random,
        already: list[PlacedObject],
        spec: dict,
    ) -> PlacedObject:
        """Dispatch to the placement strategy named in *spec['placement']*."""
        strategy = spec.get("placement", "random")
        if strategy == "random":
            return self._place_one(model_type, rng, already, spec)
        elif strategy == "spread":
            return self._place_spread(model_type, rng, already, spec)
        elif strategy == "cornered":
            return self._place_cornered(model_type, rng, already, spec)
        elif strategy == "elevated":
            return self._place_elevated(model_type, rng, already, spec)
        else:
            # Unknown strategy: fall back to random
            return self._place_one(model_type, rng, already, spec)

    # ── Strategy implementations ──────────────────────────────────────────────

    def _place_clustered(
        self,
        model_type: str,
        rng: random.Random,
        already: list[PlacedObject],
        spec: dict,
        count: int,
    ) -> list[PlacedObject]:
        """Place *count* instances within 2.5 m of a randomly chosen anchor.

        Retries the whole cluster up to MAX_ATTEMPTS times.  Falls back to
        placing each instance independently with the ``random`` strategy if no
        cluster fits.
        """
        CLUSTER_RADIUS = 2.5
        z_offset = float(spec.get("z_offset", 0.0))

        for _ in range(self.MAX_ATTEMPTS):
            # Pick a random zone and a random anchor within it
            zone = rng.choices(self._zones, weights=self._zone_weights, k=1)[0]
            ax = rng.uniform(zone["x_min"], zone["x_max"])
            ay = rng.uniform(zone["y_min"], zone["y_max"])

            # Try to place all instances within cluster radius of anchor
            cluster: list[PlacedObject] = []
            cluster_valid = True
            for _ in range(count):
                placed_ok = False
                for _ in range(self.MAX_ATTEMPTS // count + 1):
                    # Sample uniformly within a circle using rejection sampling
                    dx = rng.uniform(-CLUSTER_RADIUS, CLUSTER_RADIUS)
                    dy = rng.uniform(-CLUSTER_RADIUS, CLUSTER_RADIUS)
                    if dx * dx + dy * dy > CLUSTER_RADIUS * CLUSTER_RADIUS:
                        continue
                    cx, cy = ax + dx, ay + dy
                    if self._is_valid(cx, cy, already + cluster):
                        yaw = rng.choice([0.0, math.pi / 2, math.pi, 3 * math.pi / 2])
                        cluster.append(
                            PlacedObject(model_type=model_type, x=cx, y=cy, yaw=yaw, z=z_offset)
                        )
                        placed_ok = True
                        break
                if not placed_ok:
                    cluster_valid = False
                    break

            if cluster_valid and len(cluster) == count:
                return cluster

        # Fall back: place each independently using random strategy
        fallback_spec = dict(spec)
        fallback_spec["placement"] = "random"
        result: list[PlacedObject] = []
        for _ in range(count):
            obj = self._place_one(model_type, rng, already + result, fallback_spec)
            result.append(obj)
        return result

    def _place_spread(
        self,
        model_type: str,
        rng: random.Random,
        already: list[PlacedObject],
        spec: dict,
    ) -> PlacedObject:
        """Maximise minimum distance to all previously placed objects.

        Samples N=50 candidate positions, returns the one furthest from all
        already-placed objects.  Falls back to random if no valid candidate is
        found.
        """
        N_CANDIDATES = 50

        best: PlacedObject | None = None
        best_min_dist = -1.0

        for _ in range(N_CANDIDATES):
            # Sample a candidate using the standard floor placement
            try:
                candidate = self._place_one(model_type, rng, already, spec)
            except PlacementError:
                continue

            if not already:
                # No existing placements — just take the first valid candidate
                return candidate

            min_dist = min(
                math.hypot(candidate.x - p.x, candidate.y - p.y) for p in already
            )
            if min_dist > best_min_dist:
                best_min_dist = min_dist
                best = candidate

        if best is not None:
            return best

        # Last resort: standard random placement
        return self._place_one(model_type, rng, already, spec)

    def _place_cornered(
        self,
        model_type: str,
        rng: random.Random,
        already: list[PlacedObject],
        spec: dict,
    ) -> PlacedObject:
        """Bias placement toward zone boundary edges (within ~1.0 m of an edge).

        Samples x/y from a Gaussian centred on the nearest edge of a randomly
        chosen zone, with σ=0.5 m, clipped to zone bounds.  Retries up to
        MAX_ATTEMPTS times before raising PlacementError.
        """
        SIGMA = 0.5
        z_offset = float(spec.get("z_offset", 0.0))

        for _ in range(self.MAX_ATTEMPTS):
            zone = rng.choices(self._zones, weights=self._zone_weights, k=1)[0]
            x_min, x_max = zone["x_min"], zone["x_max"]
            y_min, y_max = zone["y_min"], zone["y_max"]

            # Choose a random edge to bias toward: 0=left, 1=right, 2=bottom, 3=top
            edge = rng.randint(0, 3)
            if edge == 0:   # left (x_min)
                x_center = x_min
                x = rng.gauss(x_center, SIGMA)
                y = rng.uniform(y_min, y_max)
            elif edge == 1: # right (x_max)
                x_center = x_max
                x = rng.gauss(x_center, SIGMA)
                y = rng.uniform(y_min, y_max)
            elif edge == 2: # bottom (y_min)
                y_center = y_min
                y = rng.gauss(y_center, SIGMA)
                x = rng.uniform(x_min, x_max)
            else:           # top (y_max)
                y_center = y_max
                y = rng.gauss(y_center, SIGMA)
                x = rng.uniform(x_min, x_max)

            # Clip to zone bounds
            x = max(x_min, min(x_max, x))
            y = max(y_min, min(y_max, y))

            if self._is_valid(x, y, already):
                yaw = rng.choice([0.0, math.pi / 2, math.pi, 3 * math.pi / 2])
                return PlacedObject(model_type=model_type, x=x, y=y, yaw=yaw, z=z_offset)

        raise PlacementError(
            f"Could not place {model_type!r} with strategy=cornered after "
            f"{self.MAX_ATTEMPTS} attempts."
        )

    def _place_elevated(
        self,
        model_type: str,
        rng: random.Random,
        already: list[PlacedObject],
        spec: dict,
    ) -> PlacedObject:
        """Place on a desk surface (surface_class='desk').

        Equivalent to surface='desk' with z_offset=0.  Falls back to random
        floor placement if no desk-class surface exists in the template.
        """
        desk_candidates = [
            obs for obs in self._obstacles
            if obs.get("surface_class", "") == "desk"
               and obs.get("surface_height", 0.0) > 0.0
        ]
        if not desk_candidates:
            # No desks in template — fall back to random
            return self._place_one(model_type, rng, already, spec)

        elevated_spec = dict(spec)
        elevated_spec["surface"] = "desk"
        elevated_spec["z_offset"] = 0.0
        return self._place_one(model_type, rng, already, elevated_spec)

    def _is_valid(
        self, x: float, y: float, placed: list[PlacedObject], skip_pgm: bool = False
    ) -> bool:
        """True iff (x, y) is free, wall-clear (unless *skip_pgm*), furniture-clear, and inter-object-clear."""
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

            # ── Furniture obstacle clearance ──────────────────────────────────
            for obs in self._obstacles:
                hw = obs["w"] / 2 + self._clearance
                hh = obs["h"] / 2 + self._clearance
                if abs(x - obs["x"]) < hw and abs(y - obs["y"]) < hh:
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
