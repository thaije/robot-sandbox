"""Scenario timing metrics."""
from __future__ import annotations

import time
from typing import Any

from metrics.base_metric import BaseMetric


class TimeMetrics(BaseMetric):
    """Tracks scenario start time and task completion time."""

    name = "time_metrics"

    def __init__(self) -> None:
        self._start: float | None = None
        self._end: float | None = None

    def start(self) -> None:
        self._start = time.monotonic()

    def mark_complete(self) -> None:
        self._end = time.monotonic()

    def update(self) -> None:
        pass

    def get_result(self) -> dict[str, Any]:
        elapsed = (self._end or time.monotonic()) - (self._start or 0.0)
        return {"task_completion_time": round(elapsed, 2)}

    def reset(self) -> None:
        self._start = None
        self._end = None
