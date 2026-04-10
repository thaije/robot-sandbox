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
        {"metric": "found_ratio", "operator": ">=", "value": 1.0},
        {"found_ratio": 1.0},
    )
    assert passed
    assert "PASS" in desc


def test_evaluate_condition_fail():
    passed, _ = evaluate_condition(
        {"metric": "found_ratio", "operator": ">=", "value": 1.0},
        {"found_ratio": 0.8},
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
    dm = DetectionMetrics("/test/detections", node=None, total_targets=3)
    dm._tracker.get_tp_events = lambda: [
        {"label_key": "1", "class_type": "fire_extinguisher", "class_name": "fire_extinguisher #1", "timestamp": 30.0, "location_error": 0.0},
        {"label_key": "2", "class_type": "first_aid_kit",     "class_name": "first_aid_kit #1",     "timestamp": 60.0, "location_error": 0.0},
        {"label_key": "3", "class_type": "hazard_sign",       "class_name": "hazard_sign #1",       "timestamp": 90.0, "location_error": 0.0},
    ]
    dm._tracker.get_fp_count = lambda: 0
    dm._tracker.get_dp_count = lambda: 0
    dm._tracker.get_location_errors = lambda: []
    result = dm.get_result()
    assert result["found_ratio"] == 1.0
    assert result["precision"] == 1.0
    assert result["time_to_all_detections"] == 90.0


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
    m = DetectionMetrics("/test/detections", node=None, total_targets=0)
    result = m.get_result()
    assert "found_ratio" in result
    assert "precision" in result
    assert "duplicate_rate" in result


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
    cc = CollisionCount()
    with patch("metrics.collision_count.time.monotonic", return_value=1.0):
        cc._on_contact(_contact_msg())
    assert cc.get_result()["collision_count"] == 1

def test_collision_clusters_rapid_events():
    """Events within 0.5 s of each other cluster into one collision."""
    cc = CollisionCount()
    with patch("metrics.collision_count.time.monotonic") as mock_t:
        mock_t.return_value = 0.0
        cc._on_contact(_contact_msg())    # event at t=0.0
        cc._on_contact(_contact_msg(0))   # off
        mock_t.return_value = 0.3
        cc._on_contact(_contact_msg())    # event at t=0.3 → same cluster (gap 0.3s < 0.5s)
        cc._on_contact(_contact_msg(0))   # off
        mock_t.return_value = 1.1
        cc._on_contact(_contact_msg())    # event at t=1.1 → new cluster (gap 0.8s > 0.5s)
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
    """Moves smaller than min_delta must not accumulate, and must not advance
    the reference point (hysteresis) — otherwise slow motion below the noise
    floor is silently dropped forever."""
    mt = MetersTraveled(min_delta=0.1)
    mt._on_odom(make_odom(0.0, 0.0))    # sets last_pos, no distance
    mt._on_odom(make_odom(0.05, 0.0))   # delta 0.05 < 0.1 → ignored, last_pos stays at (0,0)
    assert mt.get_result()["meters_traveled"] == 0.0
    mt._on_odom(make_odom(0.5, 0.0))    # delta 0.5 from (0,0) → counted
    assert mt.get_result()["meters_traveled"] == pytest.approx(0.5, abs=1e-6)

def test_meters_slow_motion_accumulates():
    """Regression: many sub-threshold steps must accumulate via hysteresis,
    not be silently dropped (prior bug under-reported distance by ~3× at
    typical DerpBot speeds of 0.1–0.2 m/s with 20 Hz odom)."""
    mt = MetersTraveled(min_delta=0.01)
    # 100 steps of 0.005 m each — every individual delta below threshold.
    for i in range(101):
        mt._on_odom(make_odom(i * 0.005, 0.0))
    # Total path = 0.5 m; hysteresis should capture it in ~0.01 m chunks.
    assert mt.get_result()["meters_traveled"] == pytest.approx(0.5, abs=0.01)

def test_meters_first_message_no_distance():
    mt = MetersTraveled(min_delta=0.0)
    mt._on_odom(make_odom(5.0, 5.0))
    assert mt.get_result()["meters_traveled"] == 0.0
