"""World SDF generator — Step 1.5."""
from __future__ import annotations

from pathlib import Path
from typing import Any

from world_manager.template_loader import TemplateLoader
from world_manager.object_placer import ObjectPlacer, PlacedObject


class WorldGenerator:
    """
    Takes a scenario config → generates a complete, launchable .sdf file.
    Combines: base template SDF + placed objects + environment variations.
    """

    def __init__(self, output_dir: Path = Path("/tmp/arst_worlds")) -> None:
        self._output_dir = output_dir
        self._loader = TemplateLoader()

    def generate(self, scenario_config: dict) -> Path:
        """Return path to the generated world .sdf file."""
        raise NotImplementedError  # TODO: Step 1.5

    def _apply_lighting(self, sdf_content: str, preset: str, lighting_presets: dict) -> str:
        raise NotImplementedError

    def _inject_objects(self, sdf_content: str, placements: list[PlacedObject]) -> str:
        raise NotImplementedError
