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
            ``exploration_coverage``, ``found_ratio``, ``precision``,
            ``collision_count``, ``near_miss_count``,
            ``detection_by_type``.
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

        Score = min(100, par_completion_time / elapsed × 70)

        par_completion_time → score 70 (B).  Faster → higher (capped 100).
        Slower → proportionally lower.
        """
        elapsed = float(metrics.get("task_completion_time", timeout))
        par_time = float(par.get("completion_time_par", timeout * 0.5))
        if elapsed <= 0:
            return 100.0
        return round(min(100.0, par_time / elapsed * 70), 2)

    def _accuracy_score(self, metrics: dict) -> float:
        """Precision and recall of object detection.

        Component 1 (weight 0.60) — found_ratio (recall) × 100
        Component 2 (weight 0.40) — precision × 100

        Grade reference (for calibration):
            S ≥ 95 | A ≥ 85 | B ≥ 70 | C ≥ 55 | D ≥ 40 | F < 40

        found_ratio: S=1.0, A≥0.89 (8/9), B≥0.78 (7/9), C≥0.56, D≥0.33
        precision:   S≥0.90, A≥0.75, B≥0.55, C≥0.40, D≥0.25

        Note: oracle/cheat detector will score D on precision by design.
        """
        recall_score    = min(100.0, float(metrics.get("found_ratio", 0.0)) * 100)
        precision_score = min(100.0, float(metrics.get("precision", 0.0)) * 100)
        return round(0.60 * recall_score + 0.40 * precision_score, 2)

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

        Component 1 (weight 0.60) — coverage per meter vs par
            score = min(100, (coverage/meters) / par_cpm × 70)
            par_cpm → 70 (B).  Rewards tight, exploratory paths.
        Component 2 (weight 0.40) — path length vs par
            score = min(100, par_path / meters × 70)
            par_path → 70 (B).  Penalises excessive wandering.
        """
        coverage  = float(metrics.get("exploration_coverage", 0.0))
        meters    = float(metrics.get("meters_traveled", 0.0))
        par_cpm   = float(par.get("coverage_per_meter_par", 2.0))
        par_path  = float(par.get("path_length_par", 50.0))

        if meters > 0 and par_cpm > 0:
            cpm_score  = min(100.0, (coverage / meters) / par_cpm * 70)
        else:
            cpm_score  = 0.0

        if meters > 0 and par_path > 0:
            path_score = min(100.0, par_path / meters * 70)
        else:
            path_score = 0.0

        return round(0.60 * cpm_score + 0.40 * path_score, 2)

    def _effectiveness_score(self, metrics: dict, eff_weights: dict) -> float:
        """How completely did the robot accomplish the mission?

        Component 1 (weight 0.65) — detection completeness
            Per-type weighted found ratio (or plain found_ratio as fallback).
        Component 2 (weight 0.35) — spatial completeness
            Raw exploration_coverage percentage (0–100).
        """
        # Component 1: per-type weighted detection completeness
        by_type: dict = metrics.get("detection_by_type", {})
        if not eff_weights or not by_type:
            detection_score = float(metrics.get("found_ratio", 0.0)) * 100
        else:
            total_weight = sum(float(w) for w in eff_weights.values()) or 1.0
            detection_score = 0.0
            for obj_type, w in eff_weights.items():
                entry    = by_type.get(obj_type, {})
                detected = int(entry.get("detected", 0))
                total    = int(entry.get("total", 0))
                frac     = detected / total if total > 0 else 0.0
                detection_score += (float(w) / total_weight) * frac * 100

        # Component 2: spatial completeness
        coverage_score = min(100.0, float(metrics.get("exploration_coverage", 0.0)))

        return round(0.65 * detection_score + 0.35 * coverage_score, 2)

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
