"""Unit tests for FlickerController._resolve_specs (no Gazebo required)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from scenario_runner.flicker_controller import FlickerController


_PRESET = {
    "room_lights": [
        {
            "name": "light_corridor_emergency",
            "type": "point",
            "pose": [9.5, 8.0, 3.0, 0, 0, 0],
            "diffuse": [0.9, 0.04, 0.04, 1.0],
            "range": 8.0,
            "constant": 0.5,
            "linear": 0.05,
            "quadratic": 0.005,
        }
    ]
}


def test_resolve_specs_uses_preset_diffuse():
    """on_diffuse should come from the preset diffuse when not explicitly specified."""
    specs = [{"name": "light_corridor_emergency", "period_s": 1.0}]
    resolved = FlickerController._resolve_specs(specs, _PRESET)
    assert resolved[0]["on_diffuse"] == pytest.approx([0.9, 0.04, 0.04, 1.0])


def test_resolve_specs_explicit_on_diffuse_overrides_preset():
    """Explicit on_diffuse in the spec overrides the preset value."""
    specs = [{"name": "light_corridor_emergency", "on_diffuse": [1.0, 0.0, 0.0, 1.0]}]
    resolved = FlickerController._resolve_specs(specs, _PRESET)
    assert resolved[0]["on_diffuse"] == pytest.approx([1.0, 0.0, 0.0, 1.0])


def test_resolve_specs_default_off_diffuse():
    """off_diffuse defaults to [0, 0, 0, 1] when not specified."""
    specs = [{"name": "light_corridor_emergency"}]
    resolved = FlickerController._resolve_specs(specs, _PRESET)
    assert resolved[0]["off_diffuse"] == pytest.approx([0.0, 0.0, 0.0, 1.0])


def test_resolve_specs_stores_light_cfg():
    """light_cfg must contain the full room_light dict (for EntityFactory recreation)."""
    specs = [{"name": "light_corridor_emergency"}]
    resolved = FlickerController._resolve_specs(specs, _PRESET)
    cfg = resolved[0]["light_cfg"]
    assert cfg["type"] == "point"
    assert cfg["range"] == 8.0
    assert cfg["diffuse"] == [0.9, 0.04, 0.04, 1.0]


def test_resolve_specs_unknown_light_has_empty_cfg():
    """Lights absent from the preset get an empty light_cfg and fallback diffuse."""
    specs = [{"name": "nonexistent_light"}]
    resolved = FlickerController._resolve_specs(specs, _PRESET)
    assert resolved[0]["light_cfg"] == {}
    assert resolved[0]["on_diffuse"] == pytest.approx([1.0, 1.0, 1.0, 1.0])


def test_resolve_specs_default_period():
    """period_s defaults to 1.0 when not specified."""
    specs = [{"name": "light_corridor_emergency"}]
    resolved = FlickerController._resolve_specs(specs, _PRESET)
    assert resolved[0]["period_s"] == pytest.approx(1.0)


def test_resolve_specs_multiple_lights():
    """Each spec in the list is resolved independently."""
    preset = {
        "room_lights": [
            {"name": "light_a", "diffuse": [1, 0, 0, 1]},
            {"name": "light_b", "diffuse": [0, 0, 1, 1]},
        ]
    }
    specs = [{"name": "light_a", "period_s": 0.5}, {"name": "light_b", "period_s": 2.0}]
    resolved = FlickerController._resolve_specs(specs, preset)
    assert len(resolved) == 2
    assert resolved[0]["name"] == "light_a"
    assert resolved[0]["period_s"] == pytest.approx(0.5)
    assert resolved[1]["name"] == "light_b"
    assert resolved[1]["period_s"] == pytest.approx(2.0)
