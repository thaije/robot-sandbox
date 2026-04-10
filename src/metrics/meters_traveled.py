"""Meters traveled metric — Step 3.2."""
from __future__ import annotations

import math
from typing import Any

from metrics.base_metric import BaseMetric


class MetersTraveled(BaseMetric):
    """Accumulates Euclidean distance from consecutive odometry readings.

    Parameters
    ----------
    odom_topic:
        ROS 2 topic publishing ``nav_msgs/Odometry``.
        For DerpBot use ``/derpbot_0/odom``.
    min_delta:
        Minimum position change (metres) that counts as movement.
        Filters stationary noise.
    node:
        An ``rclpy.node.Node`` instance.  Must be provided before calling
        ``start()``.  Kept as ``Any`` to avoid a hard rclpy import at module
        load time (allows use in unit tests without a ROS 2 installation).
    """

    name = "meters_traveled"

    def __init__(
        self,
        odom_topic: str = "/odom",
        min_delta: float = 0.01,
        node: Any = None,
    ) -> None:
        self._odom_topic = odom_topic
        self._min_delta = min_delta
        self._node = node
        self._total: float = 0.0
        self._last_pos: tuple[float, float] | None = None
        self._sub: Any = None

    def start(self) -> None:
        """Create a ROS 2 subscription to the odometry topic.

        Requires a ``rclpy.node.Node`` to have been passed at construction.
        """
        if self._node is None:
            raise RuntimeError(
                "MetersTraveled.start() requires a ROS 2 node. "
                "Pass node=<rclpy.node.Node> to __init__."
            )
        from nav_msgs.msg import Odometry  # noqa: PLC0415 (lazy ROS import)

        self._sub = self._node.create_subscription(
            Odometry,
            self._odom_topic,
            self._on_odom,
            10,
        )

    def _on_odom(self, msg: Any) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self._last_pos is None:
            self._last_pos = (x, y)
            return
        delta = math.hypot(x - self._last_pos[0], y - self._last_pos[1])
        # Hysteresis: only advance the reference point once motion clears the
        # noise floor. Without this, at 20 Hz odom a robot moving slower than
        # min_delta * rate (e.g. <0.2 m/s at min_delta=0.01) has every sample
        # dropped while last_pos marches forward, losing all slow motion.
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
