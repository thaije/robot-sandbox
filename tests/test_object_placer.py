"""Tests for the ObjectPlacer (Step 1.4)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

import math
import pytest
import yaml

from world_manager.template_loader import TemplateLoader
from world_manager.object_placer import ObjectPlacer, PlacedObject, PlacementError

# ── Fixtures ──────────────────────────────────────────────────────────────────

OBJECTS_SMALL = [
    {"type": "fire_extinguisher", "count": 2},
    {"type": "first_aid_kit",     "count": 1},
    {"type": "hazard_sign",       "count": 2},
]

SEED = 42


@pytest.fixture
def office_config():
    return TemplateLoader().load("indoor_office")


@pytest.fixture
def placer(office_config):
    return ObjectPlacer(office_config)


# ── Basic placement ───────────────────────────────────────────────────────────

def test_place_returns_correct_count(placer):
    placed = placer.place(OBJECTS_SMALL, SEED)
    assert len(placed) == 5  # 2 + 1 + 2


def test_placed_objects_have_correct_types(placer):
    placed = placer.place(OBJECTS_SMALL, SEED)
    types = [p.model_type for p in placed]
    assert types.count("fire_extinguisher") == 2
    assert types.count("first_aid_kit") == 1
    assert types.count("hazard_sign") == 2


def test_placed_objects_are_dataclass_instances(placer):
    placed = placer.place(OBJECTS_SMALL, SEED)
    for obj in placed:
        assert isinstance(obj, PlacedObject)
        assert isinstance(obj.x, float)
        assert isinstance(obj.y, float)
        assert isinstance(obj.yaw, float)


# ── Determinism ───────────────────────────────────────────────────────────────

def test_same_seed_same_result(placer):
    a = placer.place(OBJECTS_SMALL, SEED)
    b = placer.place(OBJECTS_SMALL, SEED)
    assert [(p.x, p.y, p.yaw) for p in a] == [(p.x, p.y, p.yaw) for p in b]


def test_different_seed_different_result(placer):
    a = placer.place(OBJECTS_SMALL, seed=1)
    b = placer.place(OBJECTS_SMALL, seed=2)
    positions_a = [(p.x, p.y) for p in a]
    positions_b = [(p.x, p.y) for p in b]
    assert positions_a != positions_b


# ── Spatial validity ──────────────────────────────────────────────────────────

def test_objects_within_world_bounds(placer, office_config):
    placed = placer.place(OBJECTS_SMALL, SEED)
    w = office_config["dimensions"]["width"]
    h = office_config["dimensions"]["depth"]
    for obj in placed:
        assert 0.0 < obj.x < w, f"{obj.model_type} x={obj.x:.2f} out of bounds"
        assert 0.0 < obj.y < h, f"{obj.model_type} y={obj.y:.2f} out of bounds"


def test_objects_within_placement_zones(placer, office_config):
    placed = placer.place(OBJECTS_SMALL, SEED)
    zones = office_config["placement_zones"]
    for obj in placed:
        in_some_zone = any(
            z["x_min"] <= obj.x <= z["x_max"] and z["y_min"] <= obj.y <= z["y_max"]
            for z in zones
        )
        assert in_some_zone, (
            f"{obj.model_type} at ({obj.x:.2f}, {obj.y:.2f}) not inside any zone"
        )


def test_inter_object_clearance(placer, office_config):
    clearance = office_config["placement_clearance"]
    placed = placer.place(OBJECTS_SMALL, SEED)
    for i, a in enumerate(placed):
        for j, b in enumerate(placed):
            if i >= j:
                continue
            dist = math.hypot(a.x - b.x, a.y - b.y)
            assert dist >= clearance, (
                f"Objects {i} and {j} too close: {dist:.3f} m < {clearance} m"
            )


def test_yaws_are_cardinal(placer):
    """Placed objects should have one of the four cardinal yaw values."""
    cardinals = {0.0, math.pi / 2, math.pi, 3 * math.pi / 2}
    placed = placer.place(OBJECTS_SMALL, SEED)
    for obj in placed:
        assert obj.yaw in cardinals, f"Non-cardinal yaw: {obj.yaw}"


# ── Stress / larger count ─────────────────────────────────────────────────────

def test_place_nine_objects(placer, office_config):
    """Mirrors the object count in the full scenario config."""
    objects = [
        {"type": "fire_extinguisher", "count": 3},
        {"type": "first_aid_kit",     "count": 2},
        {"type": "hazard_sign",       "count": 4},
    ]
    placed = placer.place(objects, SEED)
    assert len(placed) == 9
    # Verify all-pairs clearance
    clearance = office_config["placement_clearance"]
    for i, a in enumerate(placed):
        for j, b in enumerate(placed):
            if i >= j:
                continue
            dist = math.hypot(a.x - b.x, a.y - b.y)
            assert dist >= clearance


# ── Error handling ────────────────────────────────────────────────────────────

def test_impossible_placement_raises(office_config):
    """A clearance larger than any zone should trigger PlacementError."""
    placer = ObjectPlacer(office_config, clearance=50.0)  # larger than the whole world
    with pytest.raises(PlacementError):
        placer.place([{"type": "fire_extinguisher", "count": 1}], seed=0)
