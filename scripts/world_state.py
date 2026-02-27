#!/usr/bin/env python3.12
"""world_state.py — PNG world map for agent navigation.

Shows the floor plan with robot position, object locations (found/unfound),
obstacles (furniture), and room labels — all derived from ground truth.
Intended as a cheat tool for automated testing, not for use by the robot itself.

Robot = Blue circle with an R
Hazard sign = Yellow circle
First aid kit = Green circle
Fire extinguisher = Red circle
Obstacles = Brown rectangles
Found obstacles = grey rectangles

Coordinates are in **world frame**. The robot spawns at world (1, 1) which
equals odom (0, 0). Room layout and doorway positions are visible in the
map itself — no need to hard-code them.

Prefer the PNG over the ASCII; it shows room layout and doorway positions much
more clearly at a glance.

Usage
-----
    # Save PNG (recommended — prints path on stdout):
    python3.12 scripts/world_state.py --png /tmp/map.png

    # Print object list + robot pose (no PNG):
    python3.12 scripts/world_state.py

    # Also print ASCII map (verbose):
    python3.12 scripts/world_state.py --ascii

    # Mark found objects from a results JSON:
    python3.12 scripts/world_state.py --results results/run.json --png /tmp/map.png

    # Skip ROS (no robot marker):
    python3.12 scripts/world_state.py --no-ros --png /tmp/map.png

Found objects are automatically read from /tmp/arst_worlds/detections_live.json
when a scenario is running (written by ObjectDetectionTracker on each new find).
Pass --results to override with a completed-run JSON instead.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

WORLD_STATE = Path("/tmp/arst_worlds/world_state.json")
LIVE_DETECTIONS = Path("/tmp/arst_worlds/detections_live.json")
REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PNG = REPO_ROOT / "arst_world_map.png"

# 2-char symbol per object type (unfound prefix = label number, found = ✓)
OBJ_SYM = {"fire_extinguisher": "F", "first_aid_kit": "A", "hazard_sign": "H"}

# Room labels: (world_x_centre, world_y_centre, label)
ROOMS = [
    (4.5,  3.5, "Office A"),
    (14.0, 3.5, "Office B"),
    (9.5,  8.0, "Corridor"),
    (9.5, 12.0, "Meeting Rm"),
]


# ── PGM reader ────────────────────────────────────────────────────────────────

def read_pgm(path: Path) -> tuple[list[list[int]], int, int]:
    """Parse binary or ASCII PGM. Returns (pixels[row][col], width, height)."""
    data = path.read_bytes()

    # Tokenise header (handles # comments)
    tokens: list[str] = []
    i = 0
    while len(tokens) < 4:
        # Skip whitespace
        while i < len(data) and data[i:i+1] in (b" ", b"\t", b"\n", b"\r"):
            i += 1
        if i >= len(data):
            break
        if data[i:i+1] == b"#":  # comment line
            while i < len(data) and data[i:i+1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and data[j:j+1] not in (b" ", b"\t", b"\n", b"\r"):
            j += 1
        tokens.append(data[i:j].decode())
        i = j

    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])

    if magic == "P5":  # binary — one whitespace byte separates header from data
        raw = data[i + 1:]
        if maxval <= 255:
            pixels = [[raw[r * w + c] for c in range(w)] for r in range(h)]
        else:  # 16-bit
            pixels = [[(raw[(r*w+c)*2] << 8 | raw[(r*w+c)*2+1]) for c in range(w)] for r in range(h)]
    else:  # P2 ASCII
        vals = list(map(int, data[i:].split()))
        pixels = [[vals[r * w + c] for c in range(w)] for r in range(h)]

    if maxval != 255:
        pixels = [[int(v * 255 / maxval) for v in row] for row in pixels]

    return pixels, w, h


# ── Coordinate helpers ────────────────────────────────────────────────────────

def world_to_cell(wx: float, wy: float, res: float, h: int) -> tuple[int, int]:
    """World (x right, y up) → grid (col, row) where row 0 = top."""
    col = int(wx / res)
    row = h - 1 - int(wy / res)
    return max(0, min(h - 1, col)), max(0, min(h - 1, row))


def yaw_to_arrow(yaw: float) -> str:
    deg = math.degrees(yaw) % 360
    if deg < 45 or deg >= 315:
        return ">"
    if deg < 135:
        return "^"
    if deg < 225:
        return "<"
    return "v"


# ── Pose fetch (gz ground truth → odom fallback) ──────────────────────────────

def get_robot_pose(robot: str, world: str, spawn: dict, timeout: float = 3.0):
    """Return (world_x, world_y, yaw) — Gazebo ground truth, or odom+spawn fallback."""
    # 1. Try Gazebo ground truth via gz.transport (drift-free)
    try:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V

        gz_node = GzNode()
        result: list = []
        ev = threading.Event()

        def gz_cb(msg):
            if not result:
                result.append(msg)
                ev.set()

        gz_node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", gz_cb)
        ev.wait(timeout=timeout)

        if result:
            for pose in result[0].pose:
                if pose.name == robot:
                    p = pose.position
                    o = pose.orientation
                    yaw = math.atan2(2*(o.w*o.z + o.x*o.y), 1 - 2*(o.y*o.y + o.z*o.z))
                    return p.x, p.y, yaw
    except Exception:
        pass

    # 2. Fallback: odometry + spawn offset
    try:
        import rclpy
        from nav_msgs.msg import Odometry

        rclpy.init()
        node = rclpy.create_node("ws_node")
        received: list = []
        ev2 = threading.Event()

        def odom_cb(msg):
            if not received:
                received.append(msg)
                ev2.set()

        node.create_subscription(Odometry, f"/{robot}/odom", odom_cb, 1)
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        t0 = time.monotonic()
        while not ev2.is_set() and time.monotonic() - t0 < timeout:
            ex.spin_once(timeout_sec=0.1)
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()

        if received:
            p = received[0].pose.pose.position
            q = received[0].pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return p.x + spawn.get("x", 1.0), p.y + spawn.get("y", 1.0), yaw
    except Exception:
        pass

    return None


# ── ASCII renderer ────────────────────────────────────────────────────────────

def render_ascii(
    pixels: list[list[int]], W: int, H: int, res: float,
    label_map: dict, spawn: dict, robot_pose, found: set[str],
) -> None:
    """Print an ASCII map (2 chars wide × 1 char tall per grid cell)."""
    # Build char grid
    grid = [
        ["##" if pixels[r][c] < 128 else "  " for c in range(W)]
        for r in range(H)
    ]

    # Room labels (stamped into free cells)
    for rx, ry, name in ROOMS:
        col, row = world_to_cell(rx, ry, res, H)
        # Write chars across available cells
        for k, ch in enumerate(name[:W - col]):
            if 0 <= col + k < W and grid[row][col + k] == "  ":
                grid[row][col + k] = ch + " "

    # Objects
    for lbl, info in label_map.items():
        col, row = world_to_cell(info["x"], info["y"], res, H)
        sym = OBJ_SYM.get(info["type"], "?")
        prefix = "✓" if lbl in found else lbl
        grid[row][col] = (prefix + sym)[:2]

    # Robot
    if robot_pose is not None:
        wx, wy, yaw = robot_pose
        col, row = world_to_cell(wx, wy, res, H)
        grid[row][col] = "@" + yaw_to_arrow(yaw)

    # Print
    sep = "+" + "-" * (W * 2) + "+"
    print(sep)
    for row_cells in grid:
        print("|" + "".join(row_cells) + "|")
    print(sep)
    print(f"  N↑  1 cell = {res}m   World: {W*res:.0f}m × {H*res:.0f}m")
    print("  Legend: ## wall   @> robot   F fire_ext  A first_aid  H hazard  ✓ found\n")


# ── Text summary ──────────────────────────────────────────────────────────────

def print_summary(label_map: dict, spawn: dict, robot_pose, found: set[str]) -> None:
    print("Objects (ground truth):")
    for lbl in sorted(label_map, key=int):
        info = label_map[lbl]
        sym = OBJ_SYM.get(info["type"], "?")
        status = "FOUND ✓" if lbl in found else "not found"
        print(f"  [{lbl}] {sym}  {info['type']} #{info['instance']+1}"
              f"  @ world ({info['x']:.1f}, {info['y']:.1f})   {status}")

    if robot_pose is not None:
        wx, wy, yaw = robot_pose
        print(f"\nRobot:  world ({wx:.2f}, {wy:.2f})"
              f"  yaw {math.degrees(yaw):.1f}°  facing {yaw_to_arrow(yaw)}")
    else:
        print("\nRobot:  (pose unavailable)")


# ── PNG renderer ──────────────────────────────────────────────────────────────

def render_png(
    pixels: list[list[int]], W: int, H: int, res: float,
    label_map: dict, spawn: dict, robot_pose, found: set[str],
    obstacles: list[dict], path: str,
) -> None:
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("WARNING: cv2 not available, skipping PNG", file=sys.stderr)
        return

    SCALE = 20  # px per metre
    iw, ih = int(W * res * SCALE), int(H * res * SCALE)
    img = np.full((ih, iw, 3), 200, dtype=np.uint8)

    # Draw occupancy grid (walls from PGM)
    for r in range(H):
        for c in range(W):
            y1 = int(r * res * SCALE)
            y2 = int((r + 1) * res * SCALE)
            x1 = int(c * res * SCALE)
            x2 = int((c + 1) * res * SCALE)
            color = (255, 255, 255) if pixels[r][c] >= 128 else (50, 50, 50)
            img[y1:y2, x1:x2] = color

    # Draw obstacles (furniture) as filled rectangles
    for obs in obstacles:
        cx, cy = obs["x"], obs["y"]
        hw, hh = obs["w"] / 2, obs["h"] / 2
        x1 = int((cx - hw) * SCALE)
        x2 = int((cx + hw) * SCALE)
        y1 = ih - int((cy + hh) * SCALE)
        y2 = ih - int((cy - hh) * SCALE)
        name = obs.get("name", "")
        # Desks: brown; chairs: dark grey; cabinets: blue-grey
        if "desk" in name or "table" in name:
            color = (100, 70, 40)
        elif "chair" in name:
            color = (80, 80, 80)
        else:
            color = (130, 130, 150)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), 1)

    # Robot (drawn before objects so objects always appear on top)
    if robot_pose is not None:
        wx, wy, yaw = robot_pose
        ix, iy = int(wx * SCALE), ih - int(wy * SCALE)
        cv2.circle(img, (ix, iy), 11, (200, 80, 0), -1)    # blue-filled robot
        cv2.circle(img, (ix, iy), 11, (0, 0, 0), 1)        # black outline
        # Arrow starts at circle edge, points outward in heading direction
        ax = ix + int(12 * math.cos(yaw))
        ay = iy - int(12 * math.sin(yaw))
        ex = ix + int(22 * math.cos(yaw))
        ey = iy - int(22 * math.sin(yaw))
        cv2.arrowedLine(img, (ax, ay), (ex, ey), (0, 0, 0), 2, tipLength=0.5)
        cv2.putText(img, "R", (ix - 4, iy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Object colours by type (drawn after robot so always visible on top)
    TYPE_COLOR = {
        "fire_extinguisher": (50,  50, 220),   # red (BGR)
        "first_aid_kit":     (50, 180,  50),   # green
        "hazard_sign":       (30, 200, 230),   # yellow
    }
    FOUND_COLOR = (180, 180, 180)  # grey when found

    for lbl, info in label_map.items():
        ix = int(info["x"] * SCALE)
        iy = ih - int(info["y"] * SCALE)
        color = FOUND_COLOR if lbl in found else TYPE_COLOR.get(info["type"], (180, 0, 180))
        cv2.circle(img, (ix, iy), 9, color, -1)
        cv2.circle(img, (ix, iy), 9, (0, 0, 0), 1)
        # Black text on yellow (hazard_sign) for readability; white on dark colours
        text_color = (0, 0, 0) if info["type"] == "hazard_sign" and lbl not in found else (255, 255, 255)
        cv2.putText(img, lbl, (ix - 5, iy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1)

    # Room name labels
    for rx, ry, name in ROOMS:
        ix, iy = int(rx * SCALE), ih - int(ry * SCALE)
        cv2.putText(img, name, (ix - 30, iy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)

    cv2.imwrite(path, img)
    print(path)


# ── ROS image publisher ───────────────────────────────────────────────────────

def publish_map_image(png_path: str) -> None:
    """Publish the PNG map as sensor_msgs/Image on /arst/world_map (transient-local)."""
    try:
        import cv2
        import numpy as np
        import rclpy
        from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import Image as RosImage

        img_bgr = cv2.imread(png_path)
        if img_bgr is None:
            return
        img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
        h, w = img_rgb.shape[:2]

        msg = RosImage()
        msg.header.frame_id = "map"
        msg.height = h
        msg.width = w
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = w * 3
        msg.data = img_rgb.flatten().tobytes()

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )
        rclpy.init()
        node = rclpy.create_node("world_map_publisher")
        pub = node.create_publisher(RosImage, "/arst/world_map", qos)
        pub.publish(msg)
        # Brief spin to let the message be delivered to late-joining subscribers
        time.sleep(0.3)
        node.destroy_node()
        rclpy.shutdown()
    except Exception:
        pass


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Render world map with robot and objects")
    parser.add_argument("--robot",   default="derpbot_0")
    parser.add_argument("--no-ros",  action="store_true", help="Skip ROS pose lookup")
    parser.add_argument("--ascii",   action="store_true", help="Print ASCII map (verbose)")
    parser.add_argument("--results", default=None,        help="Results JSON to mark found objects")
    parser.add_argument("--png",     default=None,        help="Save PNG to this path")
    parser.add_argument("--state",   default=str(WORLD_STATE), help="world_state.json path")
    parser.add_argument("--debug",   action="store_true", help="Print wall-clock execution time")
    args = parser.parse_args()
    t0 = time.monotonic()

    state_path = Path(args.state)
    if not state_path.exists():
        print(f"ERROR: {state_path} not found. Run a scenario first.", file=sys.stderr)
        sys.exit(1)

    state = json.loads(state_path.read_text())
    label_map: dict  = state["label_map"]
    spawn: dict      = state.get("spawn_pose", {"x": 1.0, "y": 1.0})
    res: float       = state.get("map_resolution", 0.5)
    obstacles: list  = state.get("obstacles", [])

    # Mark found objects — prefer explicit --results, else read live detections file
    found: set[str] = set()
    if args.results:
        rj = json.loads(Path(args.results).read_text())
        for ev in rj.get("raw_metrics", {}).get("detection_events", []):
            found.add(str(ev["class_id"]))
    elif LIVE_DETECTIONS.exists():
        try:
            live = json.loads(LIVE_DETECTIONS.read_text())
            found = set(str(x) for x in live.get("found", []))
        except Exception:
            pass

    # Derive Gazebo world name from map path (e.g. .../templates/indoor_office/... → "indoor_office")
    world = Path(state["map_pgm"]).parent.name

    # Robot pose — gz ground truth preferred, odom+spawn fallback
    robot_pose = None
    if not args.no_ros:
        robot_pose = get_robot_pose(args.robot, world, spawn, timeout=3.0)
        if robot_pose is None:
            print("(no pose received — robot marker omitted)", file=sys.stderr)

    pixels, W, H = read_pgm(Path(state["map_pgm"]))

    if args.ascii:
        render_ascii(pixels, W, H, res, label_map, spawn, robot_pose, found)

    print_summary(label_map, spawn, robot_pose, found)

    # Default PNG path if none specified
    png_path = args.png or str(DEFAULT_PNG)
    render_png(pixels, W, H, res, label_map, spawn, robot_pose, found, obstacles, png_path)
    print(f"\n⚠️  CHECK THE MAP BEFORE MOVING — walls and objects visible: {png_path}")
    publish_map_image(png_path)
    if args.debug:
        print(f"[debug] elapsed: {time.monotonic() - t0:.2f}s", file=sys.stderr)


if __name__ == "__main__":
    main()
