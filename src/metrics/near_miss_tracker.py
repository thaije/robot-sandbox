"""Near-miss tracker metric — Step 3.10."""
from __future__ import annotations

import time
from typing import Any

from metrics.base_metric import BaseMetric


class NearMissTracker(BaseMetric):
    """
    Counts proximity events where robot comes within threshold_meters of
    an obstacle without making contact (debounced).
    """

    name = "near_miss_count"

    def __init__(
        self,
        scan_topic: str = "/scan",
        threshold_meters: float = 0.3,
        debounce_seconds: float = 1.0,
    ) -> None:
        self._scan_topic = scan_topic
        self._threshold = threshold_meters
        self._debounce = debounce_seconds
        self._count: int = 0
        self._last_event_time: float = 0.0
        self._events: list[dict] = []

    def start(self) -> None:
        raise NotImplementedError  # TODO: subscribe to /scan

    def _on_scan(self, msg: Any) -> None:
        min_range = min(r for r in msg.ranges if msg.range_min < r < msg.range_max)
        if min_range < self._threshold:
            now = time.monotonic()
            if now - self._last_event_time > self._debounce:
                self._count += 1
                self._last_event_time = now
                self._events.append({"t": now, "min_range": min_range})

    def update(self) -> None:
        pass

    def get_result(self) -> dict[str, Any]:
        return {"near_miss_count": self._count, "near_miss_events": self._events}

    def reset(self) -> None:
        self._count = 0
        self._last_event_time = 0.0
        self._events = []
