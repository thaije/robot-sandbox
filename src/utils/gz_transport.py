"""Gazebo transport helpers — gz-transport13 / gz.msgs10 wrappers.

Centralises the repeated pattern of subscribing to Pose_V on the dynamic-pose
topic and extracting one robot's position, used by world_state.py,
robot_inspect.py, and ObjectDetectionTracker.

Two modes
---------
One-shot  (for CLI tools that need a snapshot):
    pose = gz_get_robot_pose("derpbot_0", "indoor_office", timeout=3.0)
    # → (world_x, world_y, yaw_rad) or None on timeout / gz unavailable

Continuous  (for long-running processes that track pose live):
    node = gz_subscribe_robot_pose(
        "derpbot_0", "indoor_office",
        callback=lambda x, y, yaw: setattr(obj, "pose", (x, y)),
    )
    # keep `node` alive; discard to unsubscribe
"""
from __future__ import annotations

import math
import threading
from typing import Any, Callable


def gz_get_robot_pose(
    robot: str,
    world: str,
    timeout: float = 3.0,
) -> tuple[float, float, float] | None:
    """Return ``(world_x, world_y, yaw_rad)`` from Gazebo ground truth.

    Subscribes once to ``/world/<world>/dynamic_pose/info``, waits for the
    first message that contains *robot*, then unsubscribes.

    Returns ``None`` if gz-transport is unavailable or the timeout expires.
    """
    try:
        from gz.transport13 import Node as GzNode  # noqa: PLC0415
        from gz.msgs10.pose_v_pb2 import Pose_V    # noqa: PLC0415

        gz_node = GzNode()
        result: list = []
        ev = threading.Event()

        def _cb(msg: Any) -> None:
            if not result:
                result.append(msg)
                ev.set()

        gz_node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", _cb)
        ev.wait(timeout=timeout)

        if result:
            for pose in result[0].pose:
                if pose.name == robot:
                    p, o = pose.position, pose.orientation
                    yaw = math.atan2(
                        2 * (o.w * o.z + o.x * o.y),
                        1 - 2 * (o.y * o.y + o.z * o.z),
                    )
                    return p.x, p.y, yaw
    except Exception:
        pass
    return None


def gz_subscribe_robot_pose(
    robot: str,
    world: str,
    callback: Callable[[float, float, float], None],
) -> Any:
    """Subscribe continuously; invoke *callback(x, y, yaw)* on every update.

    Returns the ``gz.transport13.Node`` — the caller **must** keep a reference
    to it for as long as updates are wanted (garbage-collecting the node stops
    the subscription).
    """
    try:
        from gz.transport13 import Node as GzNode  # noqa: PLC0415
        from gz.msgs10.pose_v_pb2 import Pose_V    # noqa: PLC0415

        gz_node = GzNode()

        def _cb(msg: Any) -> None:
            for pose in msg.pose:
                if pose.name == robot:
                    p, o = pose.position, pose.orientation
                    yaw = math.atan2(
                        2 * (o.w * o.z + o.x * o.y),
                        1 - 2 * (o.y * o.y + o.z * o.z),
                    )
                    callback(p.x, p.y, yaw)
                    return

        gz_node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", _cb)
        return gz_node
    except Exception:
        return None
