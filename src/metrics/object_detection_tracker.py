"""Object detection tracker — Step 3.5 (support class).

Subscribes to the bounding-box camera topic and records the first time each
object class label is detected.  Used by DetectionMetrics.

The boundingbox_camera sensor (gz-sim-sensors-system, Gazebo Harmonic) emits
gz.msgs.AnnotatedAxisAligned2DBox_V, bridged to
vision_msgs/Detection2DArray via ros_gz_bridge.

Each Detection2D.results[0].hypothesis.class_id carries the integer label
assigned by gz-sim-label-system as a string ("1", "2", "3").
  1 = fire_extinguisher
  2 = first_aid_kit
  3 = hazard_sign
"""
from __future__ import annotations

import json
import math
import sys
import time
from pathlib import Path
from typing import Any

# Camera is mounted this far forward (x) from the robot centre in robot frame.
# Must match the camera_joint origin x in robots/derpbot/urdf/derpbot.urdf.
CAMERA_FORWARD_OFFSET: float = 0.05

_LIVE_DETECTIONS_PATH = Path("/tmp/arst_worlds/detections_live.json")
_WORLD_STATE_PATH = Path("/tmp/arst_worlds/world_state.json")


def _bresenham(x0: int, y0: int, x1: int, y1: int):
    """Yield (col, row) integer grid cells on the line from (x0,y0) to (x1,y1)."""
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    sx, sy = (1 if x0 < x1 else -1), (1 if y0 < y1 else -1)
    err = dx - dy
    while True:
        yield x0, y0
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            x0 += sx
        if e2 < dx:
            err += dx
            y0 += sy

# Maps gz-sim-label-system integer labels → human-readable class names.
# Must match the <label> values in worlds/models/*/model.sdf.
LABEL_NAMES: dict[str, str] = {
    "1": "fire_extinguisher",
    "2": "first_aid_kit",
    "3": "hazard_sign",
}


class ObjectDetectionTracker:
    """Track first-detection events per object label.

    Parameters
    ----------
    detections_topic:
        ROS 2 topic, e.g. ``/derpbot_0/detections``.
    node:
        An ``rclpy.node.Node`` instance.  Must be provided before calling
        ``start()``.  Kept as ``Any`` to avoid a hard rclpy import at module
        load time.
    label_map:
        Optional dict mapping string label IDs to ``{"type": str, "instance": int}``
        dicts, as produced by ``WorldGenerator.label_map``.  When provided,
        each detected label is resolved to a human-readable name like
        ``"fire_extinguisher #1"``.  Falls back to ``LABEL_NAMES`` if absent.
    """

    def __init__(
        self,
        detections_topic: str,
        node: Any,
        label_map: dict[str, dict] | None = None,
    ) -> None:
        self._topic = detections_topic
        self._node = node
        self._label_map = label_map or {}
        self._start_time: float = 0.0
        self._first_detections: dict[str, float] = {}  # class_id → elapsed_s
        self._sub: Any = None
        self._odom_sub: Any = None
        # LOS state — camera world position (precomputed from robot pose + offset)
        self._robot_pose: tuple[float, float] | None = None  # camera world (x, y)
        self._pgm_pixels: list[list[int]] | None = None      # occupancy grid rows
        self._pgm_W: int = 0
        self._pgm_H: int = 0
        self._map_res: float = 0.5  # metres per grid cell
        # Spawn offset (world frame) — loaded from world_state.json
        self._spawn_x: float = 0.0
        self._spawn_y: float = 0.0
        self._spawn_yaw: float = 0.0  # kept alive to keep subscription active

    def start(self) -> None:
        """Subscribe to the detection topic and record start time."""
        from vision_msgs.msg import Detection2DArray  # noqa: PLC0415

        self._start_time = time.monotonic()
        self._sub = self._node.create_subscription(
            Detection2DArray,
            self._topic,
            self._on_detections,
            10,
        )
        self._load_pgm()
        self._start_pose_listener()

    # ── LOS helpers ───────────────────────────────────────────────────────────

    def _load_pgm(self) -> None:
        """Load occupancy grid from PGM for LOS ray-casting."""
        try:
            import cv2

            state = json.loads(_WORLD_STATE_PATH.read_text())
            self._map_res = float(state.get("map_resolution", 0.5))
            pgm_path = state["map_pgm"]
            grid = cv2.imread(pgm_path, cv2.IMREAD_GRAYSCALE)
            if grid is None:
                return
            self._pgm_H, self._pgm_W = grid.shape
            # Store as list-of-lists (row-major, row 0 = top of map = max world-y)
            self._pgm_pixels = grid.tolist()
        except Exception:
            pass

    def _start_pose_listener(self) -> None:
        """Subscribe to /odom and keep self._robot_pose (camera world pos) current."""
        try:
            state = json.loads(_WORLD_STATE_PATH.read_text())
            sp = state.get("spawn_pose", {})
            self._spawn_x = float(sp.get("x", 0.0))
            self._spawn_y = float(sp.get("y", 0.0))
            self._spawn_yaw = float(sp.get("yaw", 0.0))
        except Exception:
            pass

        # robot name = first segment of topic, e.g. /derpbot_0/detections → derpbot_0
        robot = self._topic.strip("/").split("/")[0]
        odom_topic = f"/{robot}/odom"

        from nav_msgs.msg import Odometry  # noqa: PLC0415

        def _on_odom(msg: Any) -> None:
            ox = msg.pose.pose.position.x
            oy = msg.pose.pose.position.y
            q = msg.pose.pose.orientation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z),
            )
            sy, cy = math.sin(self._spawn_yaw), math.cos(self._spawn_yaw)
            wx = self._spawn_x + cy * ox - sy * oy
            wy = self._spawn_y + sy * ox + cy * oy
            wyaw = self._spawn_yaw + yaw
            # Track camera world position (robot centre + forward offset along heading)
            self._robot_pose = (
                wx + CAMERA_FORWARD_OFFSET * math.cos(wyaw),
                wy + CAMERA_FORWARD_OFFSET * math.sin(wyaw),
            )

        self._odom_sub = self._node.create_subscription(Odometry, odom_topic, _on_odom, 10)

    def _has_line_of_sight(self, class_id: str) -> bool:
        """Return True if a clear sightline exists from camera to the object.

        Uses Bresenham ray-casting on the PGM occupancy grid.  A cell with
        value < 128 is considered a wall.  The ray origin is the camera's world
        position (robot centre + CAMERA_FORWARD_OFFSET along heading), not the
        robot centre, so the check works correctly when the robot is pressed
        against a wall.  Returns True (allow detection) when the grid or pose
        is unavailable so failures are non-blocking.
        """
        if self._pgm_pixels is None or self._robot_pose is None:
            return True  # data not ready — allow
        if class_id not in self._label_map:
            return True  # unknown object — allow

        res = self._map_res
        H = self._pgm_H
        rx, ry = self._robot_pose  # camera world position
        obj = self._label_map[class_id]
        ox, oy = float(obj["x"]), float(obj["y"])

        # World → grid cell.  Row 0 is the top of the map (highest world-y).
        c0, r0 = int(rx / res), H - 1 - int(ry / res)
        c1, r1 = int(ox / res), H - 1 - int(oy / res)

        pixels = self._pgm_pixels
        for c, r in _bresenham(c0, r0, c1, r1):
            if r < 0 or r >= H or c < 0 or c >= self._pgm_W:
                continue
            if pixels[r][c] < 128:  # wall cell
                return False
        return True

    def _on_detections(self, msg: Any) -> None:
        elapsed = time.monotonic() - self._start_time
        new_find = False
        for det in msg.detections:
            for hyp in det.results:
                class_id = hyp.hypothesis.class_id
                # Ignore empty or background label ("0")
                if class_id and class_id != "0" and class_id not in self._first_detections:
                    if not self._has_line_of_sight(class_id):
                        continue  # through-wall clip — reject
                    self._first_detections[class_id] = elapsed
                    new_find = True
        if new_find:
            self._write_live_state()

    def _write_live_state(self) -> None:
        """Write found class_ids to disk so agent tools can show live progress."""
        try:
            _LIVE_DETECTIONS_PATH.parent.mkdir(parents=True, exist_ok=True)
            _LIVE_DETECTIONS_PATH.write_text(
                json.dumps({"found": list(self._first_detections.keys())})
            )
        except Exception:
            pass

    def get_events(self) -> list[dict]:
        """Return first-detection events, one entry per detected instance.

        Each event: ``{"class_id": str, "class_name": str, "timestamp": float}``

        When a ``label_map`` was provided, ``class_name`` is resolved to
        ``"<type> #<1-based-instance>"`` (e.g. ``"fire_extinguisher #2"``).
        Falls back to ``LABEL_NAMES`` for type-level labels.
        """
        def _resolve(cid: str) -> str:
            if cid in self._label_map:
                entry = self._label_map[cid]
                return f"{entry['type']} #{entry['instance'] + 1}"
            return LABEL_NAMES.get(cid, f"label_{cid}")

        return [
            {
                "class_id": cid,
                "class_name": _resolve(cid),
                "timestamp": round(ts, 2),
            }
            for cid, ts in sorted(self._first_detections.items(), key=lambda x: int(x[0]))
        ]

    def reset(self) -> None:
        self._first_detections = {}
        self._start_time = 0.0
