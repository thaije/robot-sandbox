"""Object detection tracker.

Subscribes to /robot/detections (vision_msgs/Detection2DArray) and classifies
each incoming detection as TP, DP, FP, or IGN against the ground-truth label_map.

Detection2D message contract (required from agents):
  class_id  (results[0].hypothesis.class_id) : object type, e.g. "fire_extinguisher"
  id                                          : persistent per-instance tracking ID
  results[0].pose.pose.position.{x,y}        : estimated position in map/slam frame (metres);
                                                scorer applies spawn-offset transform → world frame

Oracle compatibility (dev/cheat tool):
  Numeric class_id that resolves in label_map is accepted as oracle format.
  World position and type are taken from label_map; no tracking ID needed.

Event taxonomy:
  TP  – first confirmed detection of a physical object: correct type, within
        match_threshold metres of a real object, line-of-sight clear.
  DP  – duplicate positive: correct type + location but that physical object
        is already confirmed, OR the tracking ID was already seen.
  FP  – known type, but no matching object within threshold or no line of sight.
  IGN – unknown / background type; silently dropped, not penalised.
"""
from __future__ import annotations

import json
import math
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


class ObjectDetectionTracker:
    """Track TP/DP/FP detection events against the ground-truth label map.

    Parameters
    ----------
    detections_topic:
        ROS 2 topic, e.g. ``/derpbot_0/detections``.
    node:
        An ``rclpy.node.Node`` instance. Must be provided before ``start()``.
    label_map:
        Dict mapping string label IDs to
        ``{"type": str, "instance": int, "x": float, "y": float}`` dicts,
        as produced by ``WorldGenerator.label_map``.
    match_threshold:
        Max xy distance (metres) between claimed and actual object position
        for a detection to qualify as a TP. Default 1.5 m.
    """

    def __init__(
        self,
        detections_topic: str,
        node: Any,
        label_map: dict[str, dict] | None = None,
        match_threshold: float = 1.5,
    ) -> None:
        self._topic = detections_topic
        self._node = node
        self._label_map = label_map or {}
        self._match_threshold = match_threshold
        self._start_time: float = 0.0
        self._sub: Any = None
        self._odom_sub: Any = None
        self._gz_node: Any = None

        # Valid object types derived from label_map
        self._valid_types: set[str] = {e["type"] for e in self._label_map.values()}

        # Detection state
        self._confirmed_objects: dict[str, dict] = {}  # label_key → TP data
        self._seen_tracking_ids: set[str] = set()
        self._tp_events: list[dict] = []
        self._fp_count: int = 0
        self._dp_count: int = 0
        self._location_errors: list[float] = []  # metres error per TP (non-oracle only)

        # LOS state — camera world position
        self._robot_pose: tuple[float, float] | None = None
        self._pgm_pixels: list[list[int]] | None = None
        self._pgm_W: int = 0
        self._pgm_H: int = 0
        self._map_res: float = 0.5
        # Spawn offset for odom fallback
        self._spawn_x: float = 0.0
        self._spawn_y: float = 0.0
        self._spawn_yaw: float = 0.0

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
            import numpy as np  # noqa: PLC0415

            state = json.loads(_WORLD_STATE_PATH.read_text())
            self._map_res = float(state.get("map_resolution", 0.5))
            pgm_path = state["map_pgm"]
            raw = open(pgm_path, "rb").read()  # noqa: WPS515
            lines: list[str] = []
            idx = 0
            while len(lines) < 3:
                end = raw.index(b"\n", idx)
                token = raw[idx:end].strip()
                idx = end + 1
                if not token or token.startswith(b"#"):
                    continue
                lines.append(token.decode())
            if lines[0] != "P5":
                return
            W, H = map(int, lines[1].split())
            grid = np.frombuffer(raw[idx:], dtype=np.uint8).reshape((H, W))
            self._pgm_H, self._pgm_W = grid.shape
            self._pgm_pixels = grid.tolist()
        except Exception:
            pass

    def _start_pose_listener(self) -> None:
        """Track camera world position for LOS checks.

        Primary: gz-transport ground-truth pose.
        Fallback: ROS /odom with spawn-offset correction.
        """
        robot = self._topic.strip("/").split("/")[0]
        world_name: str = ""
        try:
            state = json.loads(_WORLD_STATE_PATH.read_text())
            world_name = state.get("world_name", "")
            sp = state.get("spawn_pose", {})
            self._spawn_x = float(sp.get("x", 0.0))
            self._spawn_y = float(sp.get("y", 0.0))
            self._spawn_yaw = float(sp.get("yaw", 0.0))
        except Exception:
            pass

        if world_name:
            try:
                from utils.gz_transport import gz_subscribe_robot_pose  # noqa: PLC0415

                def _on_gz_pose(x: float, y: float, yaw: float) -> None:
                    self._robot_pose = (
                        x + CAMERA_FORWARD_OFFSET * math.cos(yaw),
                        y + CAMERA_FORWARD_OFFSET * math.sin(yaw),
                    )

                self._gz_node = gz_subscribe_robot_pose(robot, world_name, _on_gz_pose)
            except Exception:
                self._gz_node = None

        if self._gz_node is None:
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
                self._robot_pose = (
                    wx + CAMERA_FORWARD_OFFSET * math.cos(wyaw),
                    wy + CAMERA_FORWARD_OFFSET * math.sin(wyaw),
                )

            self._odom_sub = self._node.create_subscription(Odometry, odom_topic, _on_odom, 10)

    def _has_line_of_sight(self, ox: float, oy: float) -> bool:
        """Return True if a clear sightline exists from camera to (ox, oy).

        Uses Bresenham ray-casting on the PGM occupancy grid.
        Returns True (allow) when grid or robot pose is unavailable.
        """
        if self._pgm_pixels is None or self._robot_pose is None:
            return True
        res = self._map_res
        H = self._pgm_H
        rx, ry = self._robot_pose
        c0, r0 = int(rx / res), H - 1 - int(ry / res)
        c1, r1 = int(ox / res), H - 1 - int(oy / res)
        pixels = self._pgm_pixels
        for c, r in _bresenham(c0, r0, c1, r1):
            if r < 0 or r >= H or c < 0 or c >= self._pgm_W:
                continue
            if pixels[r][c] < 128:
                return False
        return True

    def _find_nearest_object(
        self, class_type: str, wx: float, wy: float
    ) -> tuple[str | None, float]:
        """Return (label_key, distance) for the nearest object of class_type.

        Searches all label_map entries (confirmed and unconfirmed) so that
        DP detection against an already-confirmed object is possible.
        Returns (None, inf) if no objects of that type exist in the map.
        """
        best_key: str | None = None
        best_dist = math.inf
        for key, entry in self._label_map.items():
            if entry["type"] != class_type:
                continue
            d = math.hypot(wx - float(entry["x"]), wy - float(entry["y"]))
            if d < best_dist:
                best_dist = d
                best_key = key
        return best_key, best_dist

    # ── Core subscription handler ──────────────────────────────────────────────

    def _on_detections(self, msg: Any) -> None:
        elapsed = time.monotonic() - self._start_time
        new_tp = False
        # Per-message FP dedup: prevents a persistently-bad track from counting
        # multiple FPs within a single DetectionArray message.
        _fp_seen_this_msg: set[str] = set()

        for det in msg.detections:
            if not det.results:
                continue
            hyp = det.results[0]
            raw_class_id: str = hyp.hypothesis.class_id
            if not raw_class_id or raw_class_id == "0":
                continue

            # ── Resolve oracle vs real-agent format ───────────────────────────
            # Oracle keys are always pure integers ("1", "2", ...).
            # Guard with isdigit() so a real agent using e.g. class_id="fire_extinguisher"
            # never accidentally hits this path.
            if raw_class_id.isdigit() and raw_class_id in self._label_map:
                # Oracle format: numeric label → translate using label_map
                entry = self._label_map[raw_class_id]
                class_type = entry["type"]
                wx = float(entry["x"])
                wy = float(entry["y"])
                tracking_id = raw_class_id   # numeric label is its own stable UID
                is_oracle = True
            else:
                # Real-agent format: class_id is type name; pose is in map/slam frame
                # (odom origin = robot spawn pose).  Convert to world frame using the
                # same spawn-offset transform applied to odom-based robot pose.
                class_type = raw_class_id
                mx = float(hyp.pose.pose.position.x)
                my = float(hyp.pose.pose.position.y)
                sy = math.sin(self._spawn_yaw)
                cy = math.cos(self._spawn_yaw)
                wx = self._spawn_x + cy * mx - sy * my
                wy = self._spawn_y + sy * mx + cy * my
                tracking_id = det.id or ""
                is_oracle = False

            # 1. Unknown type → IGN (silently dropped, not penalised)
            if class_type not in self._valid_types:
                continue

            # 2. Dedup: real-agent only — oracle uses _confirmed_objects below.
            # Only blacklist a tracking_id once it's confirmed as TP or DP.
            # FP/miss outcomes must NOT blacklist — the same track may later have a
            # better position or clear LOS and should be re-evaluated.
            if not is_oracle:
                if tracking_id and tracking_id in self._seen_tracking_ids:
                    continue

            # 3. Find nearest physical object of this type
            label_key, dist = self._find_nearest_object(class_type, wx, wy)

            if label_key is not None and dist <= self._match_threshold:
                actual = self._label_map[label_key]
                if not self._has_line_of_sight(float(actual["x"]), float(actual["y"])):
                    # LOS failure — count FP but do NOT blacklist tracking_id;
                    # robot may gain LOS on a future message.
                    if not is_oracle and tracking_id not in _fp_seen_this_msg:
                        self._fp_count += 1
                        _fp_seen_this_msg.add(tracking_id)
                    continue

                if label_key in self._confirmed_objects:
                    # Physical object already confirmed → duplicate positive
                    self._dp_count += 1
                    if not is_oracle and tracking_id:
                        self._seen_tracking_ids.add(tracking_id)
                else:
                    # True positive
                    location_error = 0.0 if is_oracle else dist
                    self._confirmed_objects[label_key] = {
                        "timestamp": elapsed,
                        "tracking_id": tracking_id,
                        "location_error": location_error,
                    }
                    if not is_oracle:
                        self._location_errors.append(dist)
                        if tracking_id:
                            self._seen_tracking_ids.add(tracking_id)
                    self._tp_events.append({
                        "label_key": label_key,
                        "class_type": class_type,
                        "class_name": f"{class_type} #{actual['instance'] + 1}",
                        "timestamp": round(elapsed, 2),
                        "location_error": round(location_error, 3),
                    })
                    new_tp = True
            else:
                # Nothing within threshold → false positive (per-message dedup)
                if not is_oracle and tracking_id not in _fp_seen_this_msg:
                    self._fp_count += 1
                    _fp_seen_this_msg.add(tracking_id)
                elif is_oracle:
                    self._fp_count += 1

        if new_tp:
            self._write_live_state()

    def _write_live_state(self) -> None:
        try:
            _LIVE_DETECTIONS_PATH.parent.mkdir(parents=True, exist_ok=True)
            found = [
                {"label_key": e["label_key"], "type": e["class_type"], "name": e["class_name"]}
                for e in self._tp_events
            ]
            _LIVE_DETECTIONS_PATH.write_text(json.dumps({"found": found}))
        except Exception:
            pass

    # ── Public API ─────────────────────────────────────────────────────────────

    def get_tp_events(self) -> list[dict]:
        """TP events sorted by timestamp."""
        return sorted(self._tp_events, key=lambda e: e["timestamp"])

    def get_fp_count(self) -> int:
        return self._fp_count

    def get_dp_count(self) -> int:
        return self._dp_count

    def get_location_errors(self) -> list[float]:
        """XY distance errors (metres) for TP detections; empty for oracle-only runs."""
        return list(self._location_errors)

    def reset(self) -> None:
        self._confirmed_objects = {}
        self._seen_tracking_ids = set()
        self._tp_events = []
        self._fp_count = 0
        self._dp_count = 0
        self._location_errors = []
        self._start_time = 0.0
