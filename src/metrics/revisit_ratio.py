"""Revisit ratio metric — Step 3.7."""
from __future__ import annotations

from collections import Counter
from typing import Any

from metrics.base_metric import BaseMetric


class RevisitRatio(BaseMetric):
    """
    Fraction of visited grid cells seen more than once.
    ratio = cells_visited_more_than_once / total_cells_visited
    Lower is better (0 = perfectly systematic exploration).

    Uses odometry for position (same source as MetersTraveled).
    Ground-truth Gz pose can be substituted later without changing the logic.
    """

    name = "revisit_ratio"

    def __init__(
        self,
        odom_topic: str = "/odom",
        grid_resolution: float = 0.5,
        node: Any = None,
    ) -> None:
        self._odom_topic = odom_topic
        self._resolution = grid_resolution
        self._node = node
        self._cell_visits: Counter = Counter()
        self._sub: Any = None

    def _pose_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (int(x / self._resolution), int(y / self._resolution))

    def start(self) -> None:
        if self._node is None:
            raise RuntimeError(
                "RevisitRatio.start() requires a ROS 2 node. "
                "Pass node=<rclpy.node.Node> to __init__."
            )
        from nav_msgs.msg import Odometry  # noqa: PLC0415

        self._sub = self._node.create_subscription(
            Odometry,
            self._odom_topic,
            self._on_odom,
            10,
        )

    def _on_odom(self, msg: Any) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._cell_visits[self._pose_to_cell(x, y)] += 1

    def update(self) -> None:
        pass  # reactive via callback

    def get_result(self) -> dict[str, Any]:
        if not self._cell_visits:
            return {"revisit_ratio": 0.0}
        total = len(self._cell_visits)
        revisited = sum(1 for v in self._cell_visits.values() if v > 1)
        return {"revisit_ratio": round(revisited / total, 4)}

    def reset(self) -> None:
        self._cell_visits.clear()
