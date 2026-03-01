"""Tests for the composite scoring engine (no ROS required)."""
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

from metrics.scoring import ScoringEngine, _stepwise, numeric_to_grade


# ── _stepwise ─────────────────────────────────────────────────────────────────

THRESHOLDS = [0, 2, 5, 10]
SCORES     = [100, 80, 60, 40, 20]

@pytest.mark.parametrize("value,expected", [
    (0,  100),
    (1,  80),
    (2,  80),
    (3,  60),
    (5,  60),
    (6,  40),
    (10, 40),
    (11, 20),
    (99, 20),
])
def test_stepwise(value, expected):
    assert _stepwise(value, THRESHOLDS, SCORES) == expected


# ── ScoringEngine fixtures ─────────────────────────────────────────────────────

SCORING_CFG = {
    "category_weights": {"speed": 0.25, "accuracy": 0.30, "safety": 0.20, "efficiency": 0.25},
    "par_values": {
        "time_per_detection_par": 60.0,
        "coverage_rate_par": 2.0,
        "coverage_per_meter_par": 1.5,
    },
    "collision_thresholds": [0, 2, 5, 10],
    "collision_scores":     [100, 80, 60, 40, 20],
}

SCENARIO_CFG = {
    "scenario": {"name": "test", "timeout_seconds": 600},
}

engine = ScoringEngine(SCORING_CFG)


# ── Safety score ──────────────────────────────────────────────────────────────

@pytest.mark.parametrize("collisions,near_misses,expected", [
    (0,  0,  100.0),   # perfect — no collisions, no near-misses
    (0,  3,  0.70*100 + 0.30*60),  # 0 collisions (100), 3 near-misses (60)
    (2,  0,  0.70*80  + 0.30*100), # 2 collisions (80), 0 near-misses (100)
    (11, 11, 0.70*20  + 0.30*20),  # both worst tier
])
def test_safety_score(collisions, near_misses, expected):
    metrics = {"collision_count": collisions, "near_miss_count": near_misses}
    score = engine._safety_score(metrics, SCORING_CFG)
    assert score == pytest.approx(expected, abs=0.01)


# ── Accuracy score ────────────────────────────────────────────────────────────

def test_accuracy_perfect_detection():
    metrics = {"object_detection_rate": 1.0, "false_positive_rate": 0.0}
    # det=100*0.45, fp=100*0.30, path=0*0.25 → 75
    assert engine._accuracy_score(metrics) == pytest.approx(75.0, abs=0.01)

def test_accuracy_no_detections():
    # detection_rate=0 (0.45 weight) + false_positive_rate=0 so fp_score=100 (0.30 weight) + path=0
    # = 0*0.45 + 100*0.30 + 0*0.25 = 30
    metrics = {}
    assert engine._accuracy_score(metrics) == pytest.approx(30.0, abs=0.01)

def test_accuracy_partial():
    metrics = {"object_detection_rate": 0.5, "false_positive_rate": 0.2}
    det_score = 0.45 * 50.0
    fp_score  = 0.30 * 80.0
    assert engine._accuracy_score(metrics) == pytest.approx(det_score + fp_score, abs=0.01)


# ── Full compute ──────────────────────────────────────────────────────────────

def test_compute_returns_scorecard():
    metrics = {
        "meters_traveled": 20.0,
        "task_completion_time": 300.0,
        "collision_count": 0,
        "near_miss_count": 0,
        "revisit_ratio": 0.0,
        "object_detection_rate": 1.0,
        "false_positive_rate": 0.0,
    }
    sc = engine.compute(metrics, SCENARIO_CFG)
    assert sc.scenario_name == "test"
    assert len(sc.categories) == 5
    assert 0 <= sc.overall_score <= 100
    # Zero collisions → safety = 100
    safety_cat = next(c for c in sc.categories if c.name == "safety")
    assert safety_cat.score == 100.0

def test_compute_empty_metrics_no_crash():
    """Runner calls compute() even when metrics are all missing — must not raise."""
    sc = engine.compute({}, SCENARIO_CFG)
    assert sc.overall_score >= 0

def test_compute_high_collision_degrades_safety():
    sc_clean = engine.compute({"collision_count": 0},  SCENARIO_CFG)
    sc_bad   = engine.compute({"collision_count": 15}, SCENARIO_CFG)
    clean_safety = next(c for c in sc_clean.categories if c.name == "safety").score
    bad_safety   = next(c for c in sc_bad.categories   if c.name == "safety").score
    assert bad_safety < clean_safety
