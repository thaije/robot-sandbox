"""Unit tests for metric logic (no ROS required)."""
import sys
import types
from pathlib import Path
from unittest.mock import patch

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from metrics.evaluator import evaluate_condition, evaluate_criteria
from metrics.detection_metrics import DetectionMetrics
from metrics.meters_traveled import MetersTraveled
from metrics.collision_count import CollisionCount
from metrics.revisit_ratio import RevisitRatio
from metrics.exploration_coverage import ExplorationCoverage
from metrics.scoring import numeric_to_grade


# ── helpers ───────────────────────────────────────────────────────────────────

def make_odom(x, y):
    msg = types.SimpleNamespace()
    msg.pose = types.SimpleNamespace()
    msg.pose.pose = types.SimpleNamespace()
    msg.pose.pose.position = types.SimpleNamespace(x=float(x), y=float(y))
    return msg


# --- evaluator ---

def test_evaluate_condition_pass():
    passed, desc = evaluate_condition(
        {"metric": "object_detection_rate", "operator": ">=", "value": 1.0},
        {"object_detection_rate": 1.0},
    )
    assert passed
    assert "PASS" in desc


def test_evaluate_condition_fail():
    passed, _ = evaluate_condition(
        {"metric": "object_detection_rate", "operator": ">=", "value": 1.0},
        {"object_detection_rate": 0.8},
    )
    assert not passed


def test_evaluate_all_of():
    criteria = {
        "mode": "all_of",
        "conditions": [
            {"metric": "a", "operator": ">=", "value": 0.5},
            {"metric": "b", "operator": ">=", "value": 0.5},
        ],
    }
    passed, _ = evaluate_criteria(criteria, {"a": 1.0, "b": 0.3})
    assert not passed


def test_evaluate_any_of():
    criteria = {
        "mode": "any_of",
        "conditions": [
            {"metric": "a", "operator": ">=", "value": 0.5},
            {"metric": "b", "operator": ">=", "value": 0.5},
        ],
    }
    passed, _ = evaluate_criteria(criteria, {"a": 1.0, "b": 0.3})
    assert passed


# --- detection metrics ---

def test_detection_metrics_all_found():
    dm = DetectionMetrics(total_targets=3)
    dm.ingest_events([
        {"class": "fire_extinguisher", "timestamp": 30.0, "distance": 2.0},
        {"class": "first_aid_kit",     "timestamp": 60.0, "distance": 1.5},
        {"class": "hazard_sign",       "timestamp": 90.0, "distance": 3.0},
    ])
    result = dm.get_result()
    assert result["object_detection_rate"] == 1.0
    assert result["time_to_all_detections"] == 90.0
    assert result["false_positive_rate"] == 0.0


# --- meters traveled ---

def test_meters_traveled_accumulates():
    mt = MetersTraveled(min_delta=0.0)

    class FakeMsg:
        class pose:
            class pose:
                class position:
                    x = 0.0
                    y = 0.0

    msg = FakeMsg()
    mt._on_odom(msg)  # sets last_pos

    msg.pose.pose.position.x = 3.0
    msg.pose.pose.position.y = 4.0
    mt._on_odom(msg)

    result = mt.get_result()
    assert result["meters_traveled"] == 5.0  # 3-4-5 triangle


# --- scoring ---

def test_grade_boundaries():
    assert numeric_to_grade(100)[0] == "S"
    assert numeric_to_grade(95)[0] == "S"
    assert numeric_to_grade(94)[0] == "A"
    assert numeric_to_grade(85)[0] == "A"
    assert numeric_to_grade(70)[0] == "B"
    assert numeric_to_grade(55)[0] == "C"
    assert numeric_to_grade(40)[0] == "D"
    assert numeric_to_grade(39)[0] == "F"
    assert numeric_to_grade(0)[0] == "F"


# ── smoke tests — instantiate every metric, call get_result() without ROS ─────

def test_smoke_meters_traveled():
    m = MetersTraveled()
    assert m.get_result() == {"meters_traveled": 0.0}

def test_smoke_collision_count():
    m = CollisionCount()
    result = m.get_result()
    assert result["collision_count"] == 0

def test_smoke_revisit_ratio():
    m = RevisitRatio()
    assert m.get_result() == {"revisit_ratio": 0.0}

def test_smoke_exploration_coverage():
    m = ExplorationCoverage()
    assert m.get_result() == {"exploration_coverage": 0.0}

def test_smoke_detection_metrics():
    m = DetectionMetrics(total_targets=0)
    result = m.get_result()
    assert "object_detection_rate" in result


# ── CollisionCount ─────────────────────────────────────────────────────────────

def _contact_msg(n_contacts=1):
    """Fake ros_gz_interfaces/Contacts message."""
    msg = types.SimpleNamespace()
    msg.contacts = [object()] * n_contacts
    return msg

def test_collision_empty_contacts_ignored():
    cc = CollisionCount()
    cc._on_contact(_contact_msg(0))
    assert cc.get_result()["collision_count"] == 0

def test_collision_counts_event():
    cc = CollisionCount(debounce_seconds=0.5)
    with patch("metrics.collision_count.time.monotonic", return_value=1.0):
        cc._on_contact(_contact_msg())
    assert cc.get_result()["collision_count"] == 1

def test_collision_debounce_suppresses_rapid_repeat():
    cc = CollisionCount(debounce_seconds=0.5)
    with patch("metrics.collision_count.time.monotonic") as mock_t:
        mock_t.return_value = 1.0
        cc._on_contact(_contact_msg())   # counted (1)
        mock_t.return_value = 1.3        # within debounce
        cc._on_contact(_contact_msg())   # suppressed
        mock_t.return_value = 1.6        # past debounce
        cc._on_contact(_contact_msg())   # counted (2)
    assert cc.get_result()["collision_count"] == 2

def test_collision_reset():
    cc = CollisionCount()
    with patch("metrics.collision_count.time.monotonic", return_value=1.0):
        cc._on_contact(_contact_msg())
    cc.reset()
    assert cc.get_result()["collision_count"] == 0


# ── RevisitRatio ───────────────────────────────────────────────────────────────

def test_revisit_empty():
    rr = RevisitRatio()
    assert rr.get_result()["revisit_ratio"] == 0.0

def test_revisit_all_unique():
    rr = RevisitRatio(grid_resolution=1.0)
    for x in range(5):
        rr._on_odom(make_odom(x, 0))
    assert rr.get_result()["revisit_ratio"] == 0.0

def test_revisit_some_revisited():
    rr = RevisitRatio(grid_resolution=1.0)
    rr._on_odom(make_odom(0, 0))   # cell (0,0) — visit 1
    rr._on_odom(make_odom(1, 0))   # cell (1,0) — visit 1
    rr._on_odom(make_odom(0, 0))   # cell (0,0) — visit 2 → revisit
    # 2 total cells, 1 revisited → ratio 0.5
    assert rr.get_result()["revisit_ratio"] == 0.5

def test_revisit_grid_quantization():
    """Positions within the same cell should count as one cell."""
    rr = RevisitRatio(grid_resolution=0.5)
    for offset in [0.0, 0.1, 0.2, 0.4]:   # all map to cell (0,0)
        rr._on_odom(make_odom(offset, offset))
    # Only 1 distinct cell visited 4 times → revisited → ratio 1.0
    assert rr.get_result()["revisit_ratio"] == 1.0


# ── MetersTraveled — min_delta filtering ──────────────────────────────────────

def test_meters_min_delta_filters_noise():
    """Moves smaller than min_delta must not accumulate."""
    mt = MetersTraveled(min_delta=0.1)
    mt._on_odom(make_odom(0.0, 0.0))    # sets last_pos, no distance
    mt._on_odom(make_odom(0.05, 0.0))   # delta 0.05 < 0.1 → ignored (last_pos still updates)
    assert mt.get_result()["meters_traveled"] == 0.0
    mt._on_odom(make_odom(0.5, 0.0))    # delta 0.45 from (0.05,0) → counted
    assert mt.get_result()["meters_traveled"] == pytest.approx(0.45, abs=1e-6)

def test_meters_first_message_no_distance():
    mt = MetersTraveled(min_delta=0.0)
    mt._on_odom(make_odom(5.0, 5.0))
    assert mt.get_result()["meters_traveled"] == 0.0
