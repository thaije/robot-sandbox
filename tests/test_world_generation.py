"""Tests for world template loading."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import pytest
import yaml

from world_manager.template_loader import TemplateLoader


def test_load_office_template():
    loader = TemplateLoader()
    config = loader.load("indoor_office")
    assert config["template"] == "indoor_office"
    assert "placement_zones" in config
    assert len(config["placement_zones"]) > 0


def test_load_warehouse_template():
    loader = TemplateLoader()
    config = loader.load("indoor_warehouse")
    assert config["template"] == "indoor_warehouse"


def test_unknown_template_raises():
    loader = TemplateLoader()
    with pytest.raises(FileNotFoundError):
        loader.load("nonexistent_template")
