"""Tests for WorldGenerator.

Tests that require `gz sdf -p` are skipped when gz is not on PATH.
All other tests run without any simulator.
"""
import shutil
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from world_manager.world_generator import WorldGenerator, _rgba_str, _resolve_pgm_path

PROJECT_ROOT = Path(__file__).parent.parent
GZ_AVAILABLE = shutil.which("gz") is not None

# Minimal scenario config reused across tests
_BASE_CONFIG = {
    "scenario": {"name": "test_gen", "random_seed": 42, "timeout_seconds": 300},
    "world": {"template": "indoor_office", "objects": [], "variations": {}},
    "robots": [
        {
            "platform": "derpbot",
            "name": "derpbot_0",
            "spawn_pose": {"x": 2.0, "y": 3.0, "z": 0.0, "yaw": 0.0},
        }
    ],
}


# ── ROBOT_NAME substitution (pure Python, no gz needed) ───────────────────────

def test_robot_name_substituted_in_urdf():
    """ROBOT_NAME placeholder must be gone after substitution."""
    urdf_path = PROJECT_ROOT / "robots/derpbot/urdf/derpbot.urdf"
    urdf_text = urdf_path.read_text().replace("ROBOT_NAME", "derpbot_0")
    assert "ROBOT_NAME" not in urdf_text
    assert "/derpbot_0/bumper_contact" in urdf_text
    assert "/derpbot_0/cmd_vel" in urdf_text

def test_robot_name_per_instance():
    """Each instance gets its own name — no cross-contamination."""
    urdf_path = PROJECT_ROOT / "robots/derpbot/urdf/derpbot.urdf"
    urdf_0 = urdf_path.read_text().replace("ROBOT_NAME", "derpbot_0")
    urdf_1 = urdf_path.read_text().replace("ROBOT_NAME", "derpbot_1")
    assert "/derpbot_0/" in urdf_0 and "/derpbot_1/" not in urdf_0
    assert "/derpbot_1/" in urdf_1 and "/derpbot_0/" not in urdf_1


# ── Helper functions ───────────────────────────────────────────────────────────

def test_rgba_str():
    assert _rgba_str([1, 0.5, 0, 1]) == "1.0 0.5 0.0 1.0"

def test_resolve_pgm_path_makes_absolute(tmp_path):
    cfg = {"ground_truth_map": {"pgm": "worlds/templates/indoor_office/ground_truth_map.pgm"}}
    _resolve_pgm_path(cfg, PROJECT_ROOT)
    resolved = cfg["ground_truth_map"]["pgm"]
    assert Path(resolved).is_absolute()
    assert resolved.startswith(str(PROJECT_ROOT))

def test_resolve_pgm_path_leaves_absolute_unchanged():
    cfg = {"ground_truth_map": {"pgm": "/absolute/path/map.pgm"}}
    _resolve_pgm_path(cfg, PROJECT_ROOT)
    assert cfg["ground_truth_map"]["pgm"] == "/absolute/path/map.pgm"


# ── Full generate() — requires gz ─────────────────────────────────────────────

@pytest.mark.skipif(not GZ_AVAILABLE, reason="gz not on PATH")
def test_generate_produces_valid_sdf(tmp_path):
    gen = WorldGenerator(output_dir=tmp_path, project_root=PROJECT_ROOT)
    out = gen.generate(_BASE_CONFIG)
    assert out.exists()
    # Must be parseable XML
    tree = ET.parse(out)
    assert tree.getroot().tag == "sdf"
    assert tree.getroot().find("world") is not None


@pytest.mark.skipif(not GZ_AVAILABLE, reason="gz not on PATH")
def test_generate_embeds_robot_model(tmp_path):
    """The output SDF must contain a <model name='derpbot_0'>."""
    gen = WorldGenerator(output_dir=tmp_path, project_root=PROJECT_ROOT)
    out = gen.generate(_BASE_CONFIG)
    world = ET.parse(out).getroot().find("world")
    model_names = [m.get("name") for m in world.findall("model")]
    assert "derpbot_0" in model_names, f"derpbot_0 not found in models: {model_names}"


@pytest.mark.skipif(not GZ_AVAILABLE, reason="gz not on PATH")
def test_generate_contact_sensor_topic(tmp_path):
    """The embedded robot SDF must have the correct bumper_contact topic.

    This is the specific thing that was broken (gz sdf -p sets it to
    __default_topic__ when the inner <contact><topic> is absent from the URDF).
    """
    gen = WorldGenerator(output_dir=tmp_path, project_root=PROJECT_ROOT)
    out = gen.generate(_BASE_CONFIG)
    sdf_text = out.read_text()
    assert "/derpbot_0/bumper_contact" in sdf_text, (
        "Contact sensor topic not found — ROBOT_NAME substitution may have failed "
        "or <contact><topic> is missing from the URDF."
    )
    assert "__default_topic__" not in sdf_text, (
        "__default_topic__ found — <contact><topic> is missing from the URDF sensor."
    )


@pytest.mark.skipif(not GZ_AVAILABLE, reason="gz not on PATH")
def test_generate_robot_pose(tmp_path):
    """Embedded robot must be placed at the configured spawn_pose."""
    gen = WorldGenerator(output_dir=tmp_path, project_root=PROJECT_ROOT)
    out = gen.generate(_BASE_CONFIG)
    world = ET.parse(out).getroot().find("world")
    robot = next(m for m in world.findall("model") if m.get("name") == "derpbot_0")
    pose = robot.find("pose")
    assert pose is not None
    parts = pose.text.split()
    assert float(parts[0]) == pytest.approx(2.0)   # x
    assert float(parts[1]) == pytest.approx(3.0)   # y
    assert float(parts[2]) == pytest.approx(0.0)   # z


@pytest.mark.skipif(not GZ_AVAILABLE, reason="gz not on PATH")
def test_generate_multiple_robots(tmp_path):
    """All robots in the config list must appear in the output SDF."""
    cfg = dict(_BASE_CONFIG)
    cfg["robots"] = [
        {"platform": "derpbot", "name": "derpbot_0", "spawn_pose": {"x": 1.0, "y": 1.0, "z": 0.0, "yaw": 0.0}},
        {"platform": "derpbot", "name": "derpbot_1", "spawn_pose": {"x": 3.0, "y": 1.0, "z": 0.0, "yaw": 0.0}},
    ]
    gen = WorldGenerator(output_dir=tmp_path, project_root=PROJECT_ROOT)
    out = gen.generate(cfg)
    world = ET.parse(out).getroot().find("world")
    model_names = [m.get("name") for m in world.findall("model")]
    assert "derpbot_0" in model_names
    assert "derpbot_1" in model_names
    # Each robot must have its own namespaced topic
    sdf_text = out.read_text()
    assert "/derpbot_0/bumper_contact" in sdf_text
    assert "/derpbot_1/bumper_contact" in sdf_text


@pytest.mark.skipif(not GZ_AVAILABLE, reason="gz not on PATH")
def test_generate_missing_urdf_raises(tmp_path):
    cfg = dict(_BASE_CONFIG)
    cfg["robots"] = [{"platform": "nonexistent_bot", "name": "bot_0", "spawn_pose": {}}]
    gen = WorldGenerator(output_dir=tmp_path, project_root=PROJECT_ROOT)
    with pytest.raises(FileNotFoundError, match="nonexistent_bot"):
        gen.generate(cfg)
