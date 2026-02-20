"""Meters traveled metric — Step 3.2."""
from __future__ import annotations

import math
from typing import Any

from metrics.base_metric import BaseMetric


class MetersTraveled(BaseMetric):
    """Accumulates Euclidean distance from consecutive odometry readings."""

    name = "meters_traveled"

    def __init__(self, odom_topic: str = "/odom", min_delta: float = 0.01) -> None:
        self._odom_topic = odom_topic
        self._min_delta = min_delta
        self._total: float = 0.0
        self._last_pos: tuple[float, float] | None = None

    def start(self) -> None:
        raise NotImplementedError  # TODO: subscribe to self._odom_topic

    def _on_odom(self, msg: Any) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_pos is not None:
            delta = math.hypot(x - self._last_pos[0], y - self._last_pos[1])
            if delta >= self._min_delta:
                self._total += delta
        self._last_pos = (x, y)

    def update(self) -> None:
        pass  # reactive via callback

    def get_result(self) -> dict[str, Any]:
        return {"meters_traveled": round(self._total, 3)}

    def reset(self) -> None:
        self._total = 0.0
        self._last_pos = None
