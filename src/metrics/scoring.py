"""
Composite scoring engine — Step 3.11.
See plan §5.4 for full specification.
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
        raise NotImplementedError  # TODO: Step 3.11

    def _speed_score(self, metrics: dict, par: dict, timeout: float) -> float:
        raise NotImplementedError

    def _accuracy_score(self, metrics: dict) -> float:
        raise NotImplementedError

    def _safety_score(self, metrics: dict, cfg: dict) -> float:
        raise NotImplementedError

    def _efficiency_score(self, metrics: dict, par: dict) -> float:
        raise NotImplementedError

    @staticmethod
    def _grade(score: float) -> tuple[str, str]:
        return numeric_to_grade(score)
