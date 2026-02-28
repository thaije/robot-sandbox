"""Near-miss tracker metric — Step 3.10."""
from __future__ import annotations

import time
from typing import Any

from metrics.base_metric import BaseMetric


class NearMissTracker(BaseMetric):
    """
    Counts proximity events where robot comes within threshold_meters of
    an obstacle without making contact (debounced).

    Subscribes to a ``sensor_msgs/LaserScan`` topic and fires on the rising
    edge of min-range < threshold, with a debounce window.
    """

    name = "near_miss_count"

    def __init__(
        self,
        scan_topic: str = "/scan",
        threshold_meters: float = 0.3,
        debounce_seconds: float = 1.0,
        node: Any = None,
    ) -> None:
        self._scan_topic = scan_topic
        self._threshold = threshold_meters
        self._debounce = debounce_seconds
        self._node = node
        self._count: int = 0
        self._start_time: float = 0.0
        self._last_event_time: float = 0.0
        self._in_near_miss: bool = False  # rising-edge tracker
        self._events: list[dict] = []
        self._sub: Any = None

    def start(self) -> None:
        """Create a ROS 2 subscription to the LiDAR scan topic.

        Requires a ``rclpy.node.Node`` to have been passed at construction.
        """
        if self._node is None:
            raise RuntimeError(
                "NearMissTracker.start() requires a ROS 2 node. "
                "Pass node=<rclpy.node.Node> to __init__."
            )
        from sensor_msgs.msg import LaserScan  # noqa: PLC0415

        self._start_time = time.monotonic()
        self._sub = self._node.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            10,
        )

    def _on_scan(self, msg: Any) -> None:
        valid = [r for r in msg.ranges if msg.range_min < r < msg.range_max]
        if not valid:
            return
        min_range = min(valid)
        if min_range < self._threshold:
            if not self._in_near_miss:
                # Rising edge — new near-miss event
                now = time.monotonic()
                if now - self._last_event_time > self._debounce:
                    self._count += 1
                    self._last_event_time = now
                    elapsed = round(now - self._start_time, 2)
                    self._events.append({"t": elapsed, "count": self._count, "min_range": round(min_range, 3)})
            self._in_near_miss = True
        else:
            self._in_near_miss = False

    def update(self) -> None:
        pass

    def get_result(self) -> dict[str, Any]:
        return {"near_miss_count": self._count, "near_miss_events": self._events}

    def reset(self) -> None:
        self._count = 0
        self._last_event_time = 0.0
        self._in_near_miss = False
        self._events = []
