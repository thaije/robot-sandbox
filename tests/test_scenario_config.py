"""Tests for scenario config loading and validation — Step 4.7."""
import pytest
from pathlib import Path

import sys
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from utils.config_loader import load_scenario, ConfigError


OFFICE_SCENARIO = Path("config/scenarios/office_explore_detect.yaml")


def test_load_valid_scenario():
    cfg = load_scenario(OFFICE_SCENARIO)
    assert cfg["scenario"]["name"] == "office_explore_detect_001"
    assert cfg["scenario"]["timeout_seconds"] > 0


def test_missing_file_raises():
    with pytest.raises(ConfigError):
        load_scenario("nonexistent.yaml")


def test_success_criteria_structure():
    cfg = load_scenario(OFFICE_SCENARIO)
    sc = cfg["success_criteria"]
    assert sc["mode"] in ("all_of", "any_of")
    assert len(sc["conditions"]) > 0
    for cond in sc["conditions"]:
        assert "metric" in cond
        assert "operator" in cond
        assert "value" in cond
