"""
Exploration coverage metric — Step 3.6.

Grid-based raycasting using odometry pose (SLAM-decoupled in simulation).
LiDAR rays are traced via Bresenham lines; cells swept by a ray are marked
as "observed".  Coverage % = observed_free / explorable_free × 100.

Pose source: ROS /odom topic + spawn offset from world_state.json.
This avoids a dependency on gz-transport Python bindings, which are only
available in the system Python (not the project venv).
"""
from __future__ import annotations

import json
import math
import threading
from pathlib import Path
from typing import Any

import numpy as np

from metrics.base_metric import BaseMetric

_WORLD_STATE_PATH = Path("/tmp/arst_worlds/world_state.json")


class ExplorationCoverage(BaseMetric):
    """
    Subscribes to /{robot_name}/scan (LaserScan) + /{robot_name}/odom.
    Converts odom pose to world frame using the spawn offset from world_state.json.
    Traces LiDAR rays via skimage.draw.line (Bresenham) onto a coverage grid.
    Computes coverage % against a PGM ground-truth free-space mask.
    """

    name = "exploration_coverage"

    def __init__(
        self,
        scan_topic: str = "/scan",
        odom_topic: str = "/odom",
        gt_map_yaml: str | None = None,
        node: Any = None,
    ) -> None:
        self._scan_topic = scan_topic
        self._odom_topic = odom_topic
        self._gt_map_yaml = gt_map_yaml
        self._node = node

        # Spawn offset (world frame) — loaded from world_state.json
        self._spawn_x: float = 0.0
        self._spawn_y: float = 0.0
        self._spawn_yaw: float = 0.0

        # Map state (set by _load_gt_map)
        self._gt_mask: np.ndarray | None = None        # bool H×W, True = free
        self._coverage_grid: np.ndarray | None = None  # bool H×W, True = observed
        self._map_h: int = 0
        self._map_w: int = 0
        self._map_res: float = 0.5
        self._map_origin: tuple[float, float] = (0.0, 0.0)

        # Latest world-frame pose (thread-safe)
        self._pose: tuple[float, float, float] | None = None  # (x, y, yaw)
        self._pose_lock = threading.Lock()

        # Cached skimage line function (imported once at start)
        self._sk_line: Any = None

        self._scan_sub: Any = None
        self._odom_sub: Any = None
        self._coverage_pct: float = 0.0

    # ── Map loading ──────────────────────────────────────────────────────────

    def _load_gt_map(self, yaml_path: str) -> None:
        """Load PGM + YAML ground truth map into self._gt_mask."""
        import yaml as _yaml  # noqa: PLC0415

        yaml_p = Path(yaml_path)
        with yaml_p.open() as f:
            cfg = _yaml.safe_load(f)

        pgm_path = yaml_p.parent / cfg["image"]
        self._map_res = float(cfg.get("resolution", 0.5))
        origin = cfg.get("origin", [0.0, 0.0, 0.0])
        self._map_origin = (float(origin[0]), float(origin[1]))
        negate = int(cfg.get("negate", 0))
        free_thresh = float(cfg.get("free_thresh", 0.196))

        # Parse P5 (binary) PGM without external image library
        raw = pgm_path.read_bytes()
        lines: list[str] = []
        idx = 0
        while len(lines) < 3:
            end = raw.index(b"\n", idx)
            token = raw[idx:end].strip()
            idx = end + 1
            if not token or token.startswith(b"#"):
                continue
            lines.append(token.decode())
        assert lines[0] == "P5", f"Expected P5 PGM, got {lines[0]!r}"
        W, H = map(int, lines[1].split())
        maxval = int(lines[2])
        pixels = np.frombuffer(raw[idx:], dtype=np.uint8).reshape((H, W))

        # gt_mask: True = free (explorable).
        # With negate=0: p_occ = (maxval - px) / maxval.
        # Free when p_occ < free_thresh  →  px > maxval * (1 - free_thresh).
        thresh = maxval * (1.0 - free_thresh)
        self._gt_mask = (pixels < thresh) if negate else (pixels > thresh)
        self._coverage_grid = np.zeros((H, W), dtype=bool)
        self._map_h, self._map_w = H, W

    # ── Coordinate helpers ───────────────────────────────────────────────────

    def _world_to_grid(self, wx: float, wy: float) -> tuple[int, int]:
        """World (x, y) → grid (row, col).  Row 0 = top of image = max world-y."""
        col = int((wx - self._map_origin[0]) / self._map_res)
        row = self._map_h - 1 - int((wy - self._map_origin[1]) / self._map_res)
        return row, col

    # ── Lifecycle ────────────────────────────────────────────────────────────

    def start(self) -> None:
        """Subscribe to scan + odom; load ground truth map."""
        if self._node is None:
            raise RuntimeError(
                "ExplorationCoverage.start() requires a ROS 2 node. "
                "Pass node=<rclpy.node.Node> to __init__."
            )

        # Load spawn pose and gt_map_yaml from world state
        try:
            state = json.loads(_WORLD_STATE_PATH.read_text())
            sp = state.get("spawn_pose", {})
            self._spawn_x = float(sp.get("x", 0.0))
            self._spawn_y = float(sp.get("y", 0.0))
            self._spawn_yaw = float(sp.get("yaw", 0.0))
            if self._gt_map_yaml is None:
                pgm = state["map_pgm"]
                self._gt_map_yaml = str(Path(pgm).with_suffix(".yaml"))
        except Exception:
            pass

        if self._gt_map_yaml:
            self._load_gt_map(self._gt_map_yaml)

        # Cache skimage line function
        from skimage.draw import line as sk_line  # noqa: PLC0415

        self._sk_line = sk_line

        from nav_msgs.msg import Odometry    # noqa: PLC0415
        from sensor_msgs.msg import LaserScan  # noqa: PLC0415

        self._odom_sub = self._node.create_subscription(
            Odometry,
            self._odom_topic,
            self._on_odom,
            10,
        )
        self._scan_sub = self._node.create_subscription(
            LaserScan,
            self._scan_topic,
            self._on_scan,
            10,
        )

    # ── Pose callback ────────────────────────────────────────────────────────

    def _on_odom(self, msg: Any) -> None:
        """Convert odom pose to world frame and store it."""
        ox = msg.pose.pose.position.x
        oy = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        # Yaw from quaternion
        odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )
        # Transform odom → world using spawn offset
        sy, cy = math.sin(self._spawn_yaw), math.cos(self._spawn_yaw)
        wx = self._spawn_x + cy * ox - sy * oy
        wy = self._spawn_y + sy * ox + cy * oy
        wyaw = self._spawn_yaw + odom_yaw
        with self._pose_lock:
            self._pose = (wx, wy, wyaw)

    # ── Scan callback ────────────────────────────────────────────────────────

    def _on_scan(self, scan: Any) -> None:
        """Trace LiDAR rays onto the coverage grid (called from ROS executor thread)."""
        if self._coverage_grid is None or self._gt_mask is None or self._sk_line is None:
            return

        with self._pose_lock:
            pose = self._pose
        if pose is None:
            return

        robot_x, robot_y, robot_yaw = pose
        r0, c0 = self._world_to_grid(robot_x, robot_y)
        if not (0 <= r0 < self._map_h and 0 <= c0 < self._map_w):
            return

        r_min = scan.range_min
        r_max = scan.range_max
        angle = scan.angle_min + robot_yaw
        inc = scan.angle_increment

        coverage = self._coverage_grid
        H, W = self._map_h, self._map_w
        sk_line = self._sk_line

        for rng in scan.ranges:
            a = angle
            angle += inc
            if not math.isfinite(rng) or rng < r_min:
                continue
            dist = min(rng, r_max)
            end_x = robot_x + dist * math.cos(a)
            end_y = robot_y + dist * math.sin(a)
            r1, c1 = self._world_to_grid(end_x, end_y)
            r1 = max(0, min(H - 1, r1))
            c1 = max(0, min(W - 1, c1))
            rr, cc = sk_line(r0, c0, r1, c1)
            valid = (rr >= 0) & (rr < H) & (cc >= 0) & (cc < W)
            coverage[rr[valid], cc[valid]] = True

        self._coverage_pct = self._compute_coverage()

    # ── Metric interface ─────────────────────────────────────────────────────

    def _compute_coverage(self) -> float:
        if self._gt_mask is None or self._coverage_grid is None:
            return 0.0
        observed = int(np.count_nonzero(self._coverage_grid & self._gt_mask))
        explorable = int(np.count_nonzero(self._gt_mask))
        return 100.0 * observed / explorable if explorable else 0.0

    def update(self) -> None:
        pass  # reactive via scan callback

    def get_result(self) -> dict[str, Any]:
        return {"exploration_coverage": round(self._coverage_pct, 2)}

    def reset(self) -> None:
        if self._coverage_grid is not None:
            self._coverage_grid[:] = False
        with self._pose_lock:
            self._pose = None
        self._coverage_pct = 0.0
