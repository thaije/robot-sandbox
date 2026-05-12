"""Proximity tracker metric — checks if the robot reaches within a radius of the target object.

Tracks the robot's world-frame position (via odometry + spawn offset)
and compares it against the ground-truth position of the target object.
When the robot enters the proximity radius, records the time and minimum
distance achieved.

For proximity-goal scenarios (goal_type: proximity), this provides the
primary success/failure metric.
"""
from __future__ import annotations

import json
import math
import threading
from pathlib import Path
from typing import Any

from metrics.base_metric import BaseMetric

_WORLD_STATE_PATH = Path("/tmp/arst_worlds/world_state.json")


class ProximityTracker(BaseMetric):
    """Track whether the robot reaches within proximity_radius of the target object.

    Uses ground-truth object positions from world_state.json and the robot's
    IMU-fused odometry for position tracking.

    Parameters
    ----------
    odom_topic : str
        ROS 2 odometry topic (e.g. ``/derpbot_0/odom``).
    target_object : str
        Object type name to track (e.g. ``sewer_pipe``).
    proximity_radius : float
        Radius in metres; success when robot centre is within this distance.
    scenario_config : dict
        Full scenario YAML config (used for fallback target lookup).
    node : Any
        rclpy Node for subscription creation.
    """

    name = "proximity_tracker"

    def __init__(
        self,
        odom_topic: str = "/odom",
        target_object: str = "",
        proximity_radius: float = 2.0,
        scenario_config: dict | None = None,
        node: Any = None,
    ) -> None:
        self._odom_topic = odom_topic
        self._target_object = target_object
        self._proximity_radius = proximity_radius
        self._scenario_config = scenario_config or {}
        self._node = node

        self._spawn_x: float = 0.0
        self._spawn_y: float = 0.0
        self._spawn_yaw: float = 0.0

        self._target_x: float | None = None
        self._target_y: float | None = None

        self._reached: bool = False
        self._time_to_target: float | None = None
        self._min_distance: float = float("inf")
        self._path_length: float = 0.0
        self._start_time: float | None = None

        self._prev_x: float | None = None
        self._prev_y: float | None = None
        self._pose_lock = threading.Lock()
        self._latest_pose: tuple[float, float] | None = None

        self._odom_sub: Any = None

    # ── Target position loading ─────────────────────────────────────────────

    def _load_target_position(self) -> None:
        """Load target object position from world_state.json label_map.

        If multiple instances of the target type exist, picks the first one
        (proximity-goal scenarios have a single primary target by convention).
        """
        try:
            state = json.loads(_WORLD_STATE_PATH.read_text())
            label_map = state.get("label_map", {})
            for label, info in label_map.items():
                if info.get("type") == self._target_object and info.get("mission_target", True):
                    self._target_x = float(info["x"])
                    self._target_y = float(info["y"])
                    return
        except Exception:
            pass

        # Fallback: look at placed object positions from ObjectPlacer world_state
        try:
            state = json.loads(_WORLD_STATE_PATH.read_text())
            label_map = state.get("label_map", {})
            for label, info in label_map.items():
                if info.get("type") == self._target_object:
                    self._target_x = float(info["x"])
                    self._target_y = float(info["y"])
                    return
        except Exception:
            pass

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        if self._node is None:
            raise RuntimeError(
                "ProximityTracker.start() requires a ROS 2 node. "
                "Pass node=<rclpy.node.Node> to __init__."
            )

        try:
            state = json.loads(_WORLD_STATE_PATH.read_text())
            sp = state.get("spawn_pose", {})
            self._spawn_x = float(sp.get("x", 0.0))
            self._spawn_y = float(sp.get("y", 0.0))
            self._spawn_yaw = float(sp.get("yaw", 0.0))
        except Exception:
            pass

        self._load_target_position()

        from nav_msgs.msg import Odometry  # noqa: PLC0415

        self._odom_sub = self._node.create_subscription(
            Odometry,
            self._odom_topic,
            self._on_odom,
            10,
        )

    # ── Pose callback ────────────────────────────────────────────────────────

    def _on_odom(self, msg: Any) -> None:
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        sy, cy = math.sin(self._spawn_yaw), math.cos(self._spawn_yaw)
        wx = self._spawn_x + cy * ox - sy * oy
        wy = self._spawn_y + sy * ox + cy * oy

        with self._pose_lock:
            self._latest_pose = (wx, wy)

    # ── Position check (called from runner poll) ────────────────────────────

    def update(self) -> None:
        if self._target_x is None or self._target_y is None:
            return

        with self._pose_lock:
            pose = self._latest_pose
        if pose is None:
            return

        wx, wy = pose

        # Accumulate path length
        if self._prev_x is not None:
            dx = wx - self._prev_x
            dy = wy - self._prev_y
            self._path_length += math.sqrt(dx * dx + dy * dy)
        self._prev_x = wx
        self._prev_y = wy

        dist = math.sqrt((wx - self._target_x) ** 2 + (wy - self._target_y) ** 2)
        if dist < self._min_distance:
            self._min_distance = dist

        if not self._reached and dist <= self._proximity_radius:
            self._reached = True

    def get_result(self) -> dict[str, Any]:
        return {
            "proximity_reached": self._reached,
            "time_to_proximity": self._time_to_target,
            "min_distance_to_target": round(self._min_distance, 3)
                if self._min_distance < float("inf") else None,
            "proximity_path_length": round(self._path_length, 3),
            "straight_line_distance": round(
                math.sqrt(
                    (self._target_x - self._spawn_x) ** 2
                    + (self._target_y - self._spawn_y) ** 2
                ), 3
            ) if self._target_x is not None else None,
        }

    def reset(self) -> None:
        self._reached = False
        self._time_to_target = None
        self._min_distance = float("inf")
        self._path_length = 0.0
        self._start_time = None
        self._prev_x = None
        self._prev_y = None
        with self._pose_lock:
            self._latest_pose = None