"""World template loader — Step 1.5."""
from __future__ import annotations

from pathlib import Path

import yaml


class TemplateLoader:
    """Resolves template name → config.yaml + SDF path."""

    TEMPLATES_DIR = Path("worlds/templates")

    def __init__(self, templates_dir: Path | None = None) -> None:
        self._dir = templates_dir or self.TEMPLATES_DIR

    def load(self, template_name: str) -> dict:
        config_path = self._dir / template_name / "config.yaml"
        if not config_path.exists():
            raise FileNotFoundError(f"Template not found: {config_path}")
        with open(config_path) as f:
            return yaml.safe_load(f)

    def sdf_path(self, template_name: str) -> Path:
        return self._dir / template_name / "world.sdf"

