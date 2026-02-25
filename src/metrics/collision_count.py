"""Collision count metric — Step 3.3."""
from __future__ import annotations

import time
from typing import Any

from metrics.base_metric import BaseMetric


class CollisionCount(BaseMetric):
    """Counts distinct collision events (debounced).

    Parameters
    ----------
    bumper_topic:
        ROS 2 topic publishing ``ros_gz_interfaces/Contacts`` from the Gazebo
        Contact system.  Add a bridge argument to ``spawn_robot.launch.py``::

            /world/indoor_office/model/<robot>/link/<link>/sensor/<sensor>/contact
            @ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts

        Until a bumper sensor is added to DerpBot, this metric will remain
        silent (no messages on the topic → zero collisions recorded).
    debounce_seconds:
        Minimum time between counted events; suppresses rapid repeated contacts
        from a single collision.
    node:
        An ``rclpy.node.Node`` instance.  Must be provided before calling
        ``start()``.  Kept as ``Any`` to avoid a hard rclpy import at module
        load time.
    """

    name = "collision_count"

    def __init__(
        self,
        bumper_topic: str = "/bumper_contact",
        debounce_seconds: float = 2.0,
        node: Any = None,
    ) -> None:
        self._bumper_topic = bumper_topic
        self._debounce = debounce_seconds
        self._node = node
        self._count: int = 0
        self._last_event_time: float = 0.0
        self._events: list[dict] = []
        self._sub: Any = None

    def start(self) -> None:
        """Create a ROS 2 subscription to the bumper/contact topic.

        Requires a ``rclpy.node.Node`` to have been passed at construction.
        Message type: ``ros_gz_interfaces/msg/Contacts`` (bridged from Gazebo
        Contact system via ``ros_gz_bridge``).
        """
        if self._node is None:
            raise RuntimeError(
                "CollisionCount.start() requires a ROS 2 node. "
                "Pass node=<rclpy.node.Node> to __init__."
            )
        from ros_gz_interfaces.msg import Contacts  # noqa: PLC0415

        self._sub = self._node.create_subscription(
            Contacts,
            self._bumper_topic,
            self._on_contact,
            10,
        )

    def _on_contact(self, msg: Any) -> None:
        # ros_gz_interfaces/Contacts has a `contacts` list; ignore empty messages
        # that Gazebo may publish when no collision is occurring.
        if hasattr(msg, "contacts") and len(msg.contacts) == 0:
            return
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
