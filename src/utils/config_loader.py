"""Scenario config loader and validator — Step 4.1."""
from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

REQUIRED_TOP_KEYS = {"scenario", "world", "robots", "metrics", "success_criteria", "scoring", "output"}


class ConfigError(ValueError):
    pass


def load_scenario(path: str | Path) -> dict:
    """Parse and validate a scenario YAML file."""
    path = Path(path)
    if not path.exists():
        raise ConfigError(f"Scenario file not found: {path}")

    with open(path) as f:
        config = yaml.safe_load(f)

    missing = REQUIRED_TOP_KEYS - set(config)
    if missing:
        raise ConfigError(f"Scenario config missing keys: {missing}")

    _validate_scenario(config["scenario"])
    _validate_world(config["world"])
    _validate_robots(config["robots"])

    return config


def _validate_scenario(s: dict) -> None:
    for key in ("name", "timeout_seconds"):
        if key not in s:
            raise ConfigError(f"scenario.{key} is required")
    # random_seed is optional — omit it to get a fresh random layout each run.
    if s["timeout_seconds"] <= 0:
        raise ConfigError("scenario.timeout_seconds must be positive")


def _validate_world(w: dict) -> None:
    if "template" not in w:
        raise ConfigError("world.template is required")


def _validate_robots(robots: list) -> None:
    if not isinstance(robots, list) or len(robots) == 0:
        raise ConfigError("robots must be a non-empty list")
    for i, r in enumerate(robots):
        if "platform" not in r:
            raise ConfigError(f"robots[{i}].platform is required")
