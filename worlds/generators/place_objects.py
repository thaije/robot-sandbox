"""
Object placement engine — Step 1.4.

Given a world template config + object list + random seed, outputs valid
(x, y, yaw) poses for each object within the template's placement zones.
"""
from __future__ import annotations

import random
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence

import yaml


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float = 0.0


@dataclass
class PlacedObject:
    model_type: str
    pose: Pose2D


def load_template_config(template_path: Path) -> dict:
    with open(template_path) as f:
        return yaml.safe_load(f)


def place_objects(
    template_config: dict,
    objects: list[dict],   # [{"type": str, "count": int, "placement": str}, ...]
    seed: int,
    clearance: float | None = None,
) -> list[PlacedObject]:
    """Return deterministic object placements for the given config and seed."""
    raise NotImplementedError  # TODO: Step 1.4


def main() -> None:
    import argparse, json, sys

    parser = argparse.ArgumentParser(description="Place objects in a world template")
    parser.add_argument("template", help="Path to template config.yaml")
    parser.add_argument("objects", help="Path to object list YAML")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", default="-", help="Output JSON path (- for stdout)")
    args = parser.parse_args()

    config = load_template_config(Path(args.template))
    with open(args.objects) as f:
        obj_list = yaml.safe_load(f)

    placements = place_objects(config, obj_list, seed=args.seed)
    result = [
        {"type": p.model_type, "x": p.pose.x, "y": p.pose.y, "yaw": p.pose.yaw}
        for p in placements
    ]

    out = sys.stdout if args.output == "-" else open(args.output, "w")
    json.dump(result, out, indent=2)


if __name__ == "__main__":
    main()
