"""Collision count metric — Step 3.3."""
from __future__ import annotations

import time
from typing import Any

from metrics.base_metric import BaseMetric


class CollisionCount(BaseMetric):
    """Counts distinct collision events (debounced)."""

    name = "collision_count"

    def __init__(self, bumper_topic: str = "/bumper_contact", debounce_seconds: float = 0.5) -> None:
        self._bumper_topic = bumper_topic
        self._debounce = debounce_seconds
        self._count: int = 0
        self._last_event_time: float = 0.0
        self._events: list[dict] = []

    def start(self) -> None:
        raise NotImplementedError  # TODO: subscribe to self._bumper_topic

    def _on_contact(self, msg: Any) -> None:
        now = time.monotonic()
        if now - self._last_event_time > self._debounce:
            self._count += 1
            self._last_event_time = now
            self._events.append({"t": now, "count": self._count})

    def update(self) -> None:
        pass

    def get_result(self) -> dict[str, Any]:
        return {"collision_count": self._count, "collision_events": self._events}

    def reset(self) -> None:
        self._count = 0
        self._last_event_time = 0.0
        self._events = []
