"""
Composite scoring engine — Step 3.11.
See plan §5.4 for full specification.

Category scores
---------------
Speed      (0.25) — task time, avg detection time, coverage rate
Accuracy   (0.30) — detection rate, false positive rate, path efficiency
Safety     (0.20) — collision count, near-miss count  (stepwise tiers)
Efficiency (0.25) — revisit ratio, coverage/meter, exploration completeness

Overall is a weighted sum of the four category scores.
All scores are 0–100; grades use S/A/B/C/D/F from the grade map.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

GRADE_MAP = [
    (95, "S", "Outstanding"),
    (85, "A", "Excellent"),
    (70, "B", "Good"),
    (55, "C", "Adequate"),
    (40, "D", "Poor"),
    (0,  "F", "Failed"),
]


def numeric_to_grade(score: float) -> tuple[str, str]:
    for threshold, letter, label in GRADE_MAP:
        if score >= threshold:
            return letter, label
    return "F", "Failed"


@dataclass
class CategoryScore:
    name: str
    score: float       # 0–100
    grade: str
    grade_label: str
    weight: float


@dataclass
class Scorecard:
    scenario_name: str
    status: str                    # "SUCCESS" | "FAILURE" | "TIMEOUT"
    elapsed_seconds: float
    timeout_seconds: float
    random_seed: int = 0
    categories: list[CategoryScore] = field(default_factory=list)
    overall_score: float = 0.0
    overall_grade: str = "F"
    overall_grade_label: str = "Failed"
    raw_metrics: dict[str, Any] = field(default_factory=dict)


class ScoringEngine:
    """Computes category + overall scores from raw metric results."""

    def __init__(self, scoring_config: dict) -> None:
        self._cfg = scoring_config

    def compute(self, metrics: dict[str, Any], scenario_config: dict) -> Scorecard:
        """Build a Scorecard from raw metrics and the scenario config.

        Parameters
        ----------
        metrics:
            Flat dict of metric name → value (as returned by
            ``_collect_metrics()`` in the runner).  Expected keys (all
            optional; missing keys score 0 for that component):
            ``meters_traveled``, ``task_completion_time``,
            ``average_time_per_detection``, ``exploration_coverage``,
            ``object_detection_rate``, ``false_positive_rate``,
            ``collision_count``, ``near_miss_count``, ``revisit_ratio``.
        scenario_config:
            Full scenario YAML dict; used for ``timeout_seconds`` and
            ``scenario.name``.
        """
        scenario = scenario_config["scenario"]
        timeout = float(scenario["timeout_seconds"])
        par = self._cfg.get("par_values", {})
        weights = self._cfg.get("category_weights", {})

        speed      = self._speed_score(metrics, par, timeout)
        accuracy   = self._accuracy_score(metrics)
        safety     = self._safety_score(metrics, self._cfg)
        efficiency = self._efficiency_score(metrics, par)
        eff_weights = self._cfg.get("effectiveness_weights", {})
        effectiveness = self._effectiveness_score(metrics, eff_weights)

        cats = [
            CategoryScore("speed",         speed,         *self._grade(speed),         weights.get("speed",         0.25)),
            CategoryScore("accuracy",       accuracy,      *self._grade(accuracy),      weights.get("accuracy",      0.30)),
            CategoryScore("safety",         safety,        *self._grade(safety),        weights.get("safety",        0.20)),
            CategoryScore("efficiency",     efficiency,    *self._grade(efficiency),    weights.get("efficiency",    0.25)),
            CategoryScore("effectiveness",  effectiveness, *self._grade(effectiveness), weights.get("effectiveness", 0.0)),
        ]

        total_w = sum(c.weight for c in cats) or 1.0
        overall = sum(c.score * c.weight for c in cats) / total_w
        overall_g, overall_gl = self._grade(overall)

        return Scorecard(
            scenario_name=scenario["name"],
            status="",          # runner overwrites this
            elapsed_seconds=metrics.get("task_completion_time", timeout),
            timeout_seconds=timeout,
            categories=cats,
            overall_score=round(overall, 1),
            overall_grade=overall_g,
            overall_grade_label=overall_gl,
            raw_metrics=metrics,
        )

    # ── Category scorers ───────────────────────────────────────────────────────

    def _speed_score(self, metrics: dict, par: dict, timeout: float) -> float:
        """How fast did the robot accomplish the mission?

        Component 1 (weight 0.40) — task completion time
            score = (timeout − elapsed) / timeout × 100, clamped 0–100.
        Component 2 (weight 0.35) — avg time per detection
            score = min(100, par_time / avg_time × 100).  Zero if no detections.
        Component 3 (weight 0.25) — coverage rate (% / s)
            score = min(100, rate / par_rate × 100).  Zero if no coverage data.
        """
        elapsed = float(metrics.get("task_completion_time", timeout))

        # Component 1 — task completion time
        time_score = max(0.0, min(100.0, (timeout - elapsed) / timeout * 100))

        # Component 2 — average time per detection
        avg_detect = metrics.get("average_time_per_detection")
        par_time = float(par.get("time_per_detection_par", 60.0))
        if avg_detect and float(avg_detect) > 0:
            detect_score = min(100.0, par_time / float(avg_detect) * 100)
        else:
            detect_score = 0.0

        # Component 3 — coverage rate
        coverage = float(metrics.get("exploration_coverage", 0.0))
        par_rate = float(par.get("coverage_rate_par", 2.0))
        rate = coverage / elapsed if elapsed > 0 else 0.0
        rate_score = min(100.0, rate / par_rate * 100) if par_rate > 0 else 0.0

        return round(0.40 * time_score + 0.35 * detect_score + 0.25 * rate_score, 2)

    def _accuracy_score(self, metrics: dict) -> float:
        """How precisely did the robot find what it needed to?

        Component 1 (weight 0.45) — object detection rate × 100
        Component 2 (weight 0.30) — (1 − false_positive_rate) × 100
        Component 3 (weight 0.25) — path efficiency (not yet implemented;
            scores 0 in Phase 1 until optimal path calculation is added)
        """
        det_rate = float(metrics.get("object_detection_rate", 0.0))
        det_score = min(100.0, det_rate * 100)

        fp_rate = float(metrics.get("false_positive_rate", 0.0))
        fp_score = max(0.0, (1.0 - fp_rate) * 100)

        # Path efficiency: placeholder 0 until optimal-path calculation is
        # implemented (Step 3.7 area).  The 0.25 weight drags accuracy down
        # proportionally; once implemented it will reward efficient paths.
        path_score = float(metrics.get("path_efficiency_ratio", 0.0)) * 100

        return round(0.45 * det_score + 0.30 * fp_score + 0.25 * path_score, 2)

    def _safety_score(self, metrics: dict, cfg: dict) -> float:
        """How carefully did the robot operate?

        Both collision count and near-miss count use the same stepwise tier
        structure defined by ``collision_thresholds`` and ``collision_scores``
        in the scoring config.

        Component weights: collisions 0.70, near-misses 0.30.
        """
        thresholds: list[int] = cfg.get("collision_thresholds", [0, 2, 5, 10])
        tier_scores: list[float] = cfg.get("collision_scores", [100, 80, 60, 40, 20])

        collisions  = int(metrics.get("collision_count", 0))
        near_misses = int(metrics.get("near_miss_count", 0))

        collision_score = _stepwise(collisions,  thresholds, tier_scores)
        near_miss_score = _stepwise(near_misses, thresholds, tier_scores)

        return round(0.70 * collision_score + 0.30 * near_miss_score, 2)

    def _efficiency_score(self, metrics: dict, par: dict) -> float:
        """How smartly did the robot use its resources?

        Component 1 (weight 0.35) — revisit ratio: (1 − ratio) × 100
        Component 2 (weight 0.35) — coverage per meter: (cpm / par_cpm) × 100
        Component 3 (weight 0.30) — exploration completeness: coverage %
        """
        revisit = float(metrics.get("revisit_ratio", 0.0))
        revisit_score = max(0.0, (1.0 - revisit) * 100)

        coverage = float(metrics.get("exploration_coverage", 0.0))
        meters   = float(metrics.get("meters_traveled", 0.0))
        par_cpm  = float(par.get("coverage_per_meter_par", 1.5))
        if meters > 0 and par_cpm > 0:
            cpm_score = min(100.0, (coverage / meters) / par_cpm * 100)
        else:
            cpm_score = 0.0

        completeness_score = min(100.0, coverage)

        return round(0.35 * revisit_score + 0.35 * cpm_score + 0.30 * completeness_score, 2)

    def _effectiveness_score(self, metrics: dict, eff_weights: dict) -> float:
        """How effectively did the robot detect the highest-priority objects?

        Uses ``detection_by_type`` (a dict of ``{type: {detected, total}}``)
        and per-type weights from ``scoring.effectiveness_weights`` in the
        scenario YAML (e.g. ``fire_extinguisher: 40``).

        score = Σ (type_weight / total_weight) × (detected / total) × 100

        If no weights are configured or no detection data is available, the
        score falls back to the raw ``object_detection_rate × 100``.
        """
        by_type: dict = metrics.get("detection_by_type", {})

        if not eff_weights or not by_type:
            # Fallback to overall detection rate
            det_rate = float(metrics.get("object_detection_rate", 0.0))
            return round(det_rate * 100, 2)

        total_weight = sum(float(w) for w in eff_weights.values()) or 1.0
        score = 0.0
        for obj_type, w in eff_weights.items():
            entry = by_type.get(obj_type, {})
            detected = int(entry.get("detected", 0))
            total    = int(entry.get("total", 0))
            frac = detected / total if total > 0 else 0.0
            score += (float(w) / total_weight) * frac * 100

        return round(score, 2)

    @staticmethod
    def _grade(score: float) -> tuple[str, str]:
        return numeric_to_grade(score)


# ── Module helpers ─────────────────────────────────────────────────────────────


def _stepwise(value: int, thresholds: list, scores: list) -> float:
    """Return the score tier for *value* given ascending *thresholds*.

    ``thresholds[i]`` is the upper bound for ``scores[i]``.
    Values above all thresholds get ``scores[-1]``.

    Example: thresholds=[0,2,5,10], scores=[100,80,60,40,20]
        value=0  → 100,  value=2  → 80,  value=11 → 20
    """
    for threshold, score in zip(thresholds, scores):
        if value <= threshold:
            return float(score)
    return float(scores[-1])
