"""Unit tests for metric logic (no ROS required)."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from metrics.evaluator import evaluate_condition, evaluate_criteria
from metrics.detection_metrics import DetectionMetrics
from metrics.meters_traveled import MetersTraveled
from metrics.scoring import numeric_to_grade


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
