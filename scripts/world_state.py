#!/usr/bin/env python3.12
"""world_state.py — PNG world map + live status for agent navigation.

Shows the floor plan with robot position, object locations (found/unfound/visible),
obstacles, and room labels — all derived from ground truth.

Coordinates are in **world frame**. The robot spawns at world (1, 1) which
equals odom (0, 0). Room layout and doorway positions are visible in the
map itself — no need to hard-code them.

Map legend
----------
- Blue circle + arrow = robot, arrow = heading; RED ring = currently colliding
- Cyan/white ring on unfound object = currently visible in camera (LOS-checked)
- Coloured circles = unfound objects (red=fire_ext, green=first_aid, yellow=hazard)
- Grey circles = found objects
- Brown rectangles = furniture (desks, chairs, cabinets)
- Dark cells = walls

Usage
-----
    python3.12 scripts/world_state.py
    python3.12 scripts/world_state.py --png /tmp/map.png
    python3.12 scripts/world_state.py --results results/run.json
    python3.12 scripts/world_state.py --no-ros

Found objects are automatically read from /tmp/arst_worlds/detections_live.json
when a scenario is running.  Pass --results to use a completed-run JSON instead.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

# Allow importing from src/ whether the package is installed or not.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from utils.gz_transport import gz_get_robot_pose  # noqa: E402

WORLD_STATE = Path("/tmp/arst_worlds/world_state.json")
LIVE_DETECTIONS = Path("/tmp/arst_worlds/detections_live.json")
REPO_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PNG = REPO_ROOT / "arst_world_map.png"

# Camera offset from robot centre (must match camera_joint x in derpbot.urdf)
CAMERA_FORWARD_OFFSET = 0.05  # metres

# 2-char symbol per object type
OBJ_SYM = {"fire_extinguisher": "F", "first_aid_kit": "A", "hazard_sign": "H", "person": "P"}

# Room labels: (world_x_centre, world_y_centre, label)
ROOMS = [
    (4.5,  3.5, "Office A"),
    (14.0, 3.5, "Office B"),
    (9.5,  8.0, "Corridor"),
    (9.5, 12.0, "Meeting Rm"),
]


# ── Geometry helpers ───────────────────────────────────────────────────────────

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


def _has_line_of_sight(
    pixels: list[list[int]], pgm_W: int, pgm_H: int, res: float,
    robot_pose: tuple[float, float, float],
    obj_x: float, obj_y: float,
) -> bool:
    """True if the camera has a wall-clear sightline to (obj_x, obj_y).

    Uses the camera world position (robot centre + CAMERA_FORWARD_OFFSET along
    heading) so the check behaves correctly when the robot is pressed against a
    wall.  Returns True when data is unavailable (fail-open).
    """
    rx, ry, yaw = robot_pose
    cam_x = rx + CAMERA_FORWARD_OFFSET * math.cos(yaw)
    cam_y = ry + CAMERA_FORWARD_OFFSET * math.sin(yaw)

    c0, r0 = int(cam_x / res), pgm_H - 1 - int(cam_y / res)
    c1, r1 = int(obj_x / res), pgm_H - 1 - int(obj_y / res)

    for c, r in _bresenham(c0, r0, c1, r1):
        if 0 <= r < pgm_H and 0 <= c < pgm_W:
            if pixels[r][c] < 128:  # wall cell
                return False
    return True


# ── PGM reader ────────────────────────────────────────────────────────────────

def read_pgm(path: Path) -> tuple[list[list[int]], int, int]:
    """Parse binary or ASCII PGM. Returns (pixels[row][col], width, height)."""
    data = path.read_bytes()

    tokens: list[str] = []
    i = 0
    while len(tokens) < 4:
        while i < len(data) and data[i:i+1] in (b" ", b"\t", b"\n", b"\r"):
            i += 1
        if i >= len(data):
            break
        if data[i:i+1] == b"#":
            while i < len(data) and data[i:i+1] != b"\n":
                i += 1
            continue
        j = i
        while j < len(data) and data[j:j+1] not in (b" ", b"\t", b"\n", b"\r"):
            j += 1
        tokens.append(data[i:j].decode())
        i = j

    magic, w, h, maxval = tokens[0], int(tokens[1]), int(tokens[2]), int(tokens[3])

    if magic == "P5":
        raw = data[i + 1:]
        if maxval <= 255:
            pixels = [[raw[r * w + c] for c in range(w)] for r in range(h)]
        else:
            pixels = [[(raw[(r*w+c)*2] << 8 | raw[(r*w+c)*2+1]) for c in range(w)] for r in range(h)]
    else:
        vals = list(map(int, data[i:].split()))
        pixels = [[vals[r * w + c] for c in range(w)] for r in range(h)]

    if maxval != 255:
        pixels = [[int(v * 255 / maxval) for v in row] for row in pixels]

    return pixels, w, h


def yaw_to_arrow(yaw: float) -> str:
    deg = math.degrees(yaw) % 360
    if deg < 45 or deg >= 315:
        return ">"
    if deg < 135:
        return "^"
    if deg < 225:
        return "<"
    return "v"


# ── Sim state (pose + detections + collision) ──────────────────────────────────

def _ros_query(
    robot: str,
    spawn: dict,
    want_odom: bool,
    timeout: float = 2.0,
) -> tuple[tuple | None, set[str], bool | None]:
    """Single rclpy session: subscribe to detections, bumper_contact, odom.

    Returns (odom_pose, visible_ids, is_colliding).
    odom_pose = (wx, wy, yaw) or None.
    visible_ids = set of class_id strings currently in camera view (raw, no LOS).
    is_colliding = bool or None if topic never published.
    """
    try:
        import rclpy  # noqa: PLC0415
        from vision_msgs.msg import Detection2DArray  # noqa: PLC0415
        from ros_gz_interfaces.msg import Contacts    # noqa: PLC0415
    except ImportError as e:
        print(f"WARNING: ROS import failed ({e}), skipping ROS data", file=sys.stderr)
        return None, set(), None

    det_msg: list = []
    bump_msg: list = []
    odom_msg: list = []
    det_ev = threading.Event()
    bump_ev = threading.Event()
    odom_ev = threading.Event()

    try:
        rclpy.init()
    except Exception:
        pass  # already initialised

    node = rclpy.create_node("ws_node")

    def on_det(msg):
        if not det_msg:
            det_msg.append(msg)
            det_ev.set()

    # Bumper logic:
    #   The contact sensor fires for ALL contacts including the robot resting on
    #   the ground plane (always true — not a real collision).  Filter those out.
    #   Sensor publishes at ~300+ Hz so a short settle window is plenty.
    _BUMP_SETTLE_S = 0.10   # wait after first message for fresh readings
    _BUMP_MIN_COUNT = 5     # messages needed before trusting the reading
    bump_first_time: list[float | None] = [None]
    bump_count = [0]

    def _non_ground(contacts) -> list:
        return [
            c for c in (contacts or [])
            if "ground_plane" not in str(getattr(c, "collision1", ""))
            and "ground_plane" not in str(getattr(c, "collision2", ""))
        ]

    def on_bump(msg):
        bump_msg.clear()
        bump_msg.append(msg)
        bump_count[0] += 1
        if bump_first_time[0] is None:
            bump_first_time[0] = time.monotonic()
        bump_ev.set()

    def on_odom(msg):
        if not odom_msg:
            odom_msg.append(msg)
            odom_ev.set()

    node.create_subscription(Detection2DArray, f"/{robot}/detections", on_det, 1)
    node.create_subscription(Contacts, f"/{robot}/bumper_contact", on_bump, 1)
    if want_odom:
        from nav_msgs.msg import Odometry  # noqa: PLC0415
        node.create_subscription(Odometry, f"/{robot}/odom", on_odom, 1)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.05)
        odom_ok = odom_ev.is_set() if want_odom else True
        bump_settled = (
            bump_first_time[0] is not None
            and (time.monotonic() - bump_first_time[0]) >= _BUMP_SETTLE_S
        )
        if det_ev.is_set() and bump_settled and odom_ok:
            break

    executor.shutdown()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass

    # --- odom pose ---
    odom_pose = None
    if odom_msg:
        p = odom_msg[0].pose.pose.position
        q = odom_msg[0].pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        odom_pose = (p.x + spawn.get("x", 1.0), p.y + spawn.get("y", 1.0), yaw)

    # --- raw visible class ids (LOS check applied by caller) ---
    visible_ids: set[str] = set()
    if det_msg:
        for det in det_msg[0].detections:
            for hyp in det.results:
                cid = hyp.hypothesis.class_id
                if cid and cid != "0":
                    visible_ids.add(cid)

    # --- collision ---
    # Filter ground-plane contacts (always present) and require enough messages.
    is_colliding = None
    if bump_msg and bump_count[0] >= _BUMP_MIN_COUNT:
        is_colliding = bool(_non_ground(getattr(bump_msg[0], "contacts", [])))

    return odom_pose, visible_ids, is_colliding


def get_sim_state(
    robot: str,
    world: str,
    spawn: dict,
    timeout: float = 3.0,
    no_ros: bool = False,
) -> tuple[tuple | None, set[str], bool | None]:
    """Return (robot_pose, visible_ids, is_colliding).

    robot_pose = (wx, wy, yaw) or None.
    visible_ids = LOS-checked set of currently visible class_id strings.
    is_colliding = bool or None.

    Tries gz-transport for pose first; falls back to ROS odom if unavailable.
    All ROS subscriptions (odom fallback, detections, bumper) share one rclpy
    session to avoid multiple init/shutdown cycles.
    """
    if no_ros:
        return None, set(), None

    # 1. gz-transport pose (short timeout — ROS runs while we wait if needed)
    gz_pose = gz_get_robot_pose(robot, world, timeout=1.5)

    # 2. Single ROS session for detections + bumper + odom fallback
    odom_pose, visible_ids_raw, is_colliding = _ros_query(
        robot, spawn, want_odom=(gz_pose is None), timeout=1.5
    )

    robot_pose = gz_pose or odom_pose
    return robot_pose, visible_ids_raw, is_colliding


# ── Text summary ──────────────────────────────────────────────────────────────

def print_summary(
    label_map: dict,
    spawn: dict,
    robot_pose,
    found: set[str],
    visible_ids: set[str],
    is_colliding: bool | None,
) -> None:
    print("Objects (ground truth — mission targets only):")
    for lbl in sorted(label_map, key=int):
        info = label_map[lbl]
        # Backward-compat: old world_state.json without the field → treat as target
        if not info.get("mission_target", True):
            continue
        sym = OBJ_SYM.get(info["type"], "?")
        if lbl in found:
            status = "FOUND ✓"
        else:
            status = "not found"
        vis_tag = "  [visible 👁]" if lbl in visible_ids else ""
        print(f"  [{lbl}] {sym}  {info['type']} #{info['instance']+1}"
              f"  @ world ({info['x']:.1f}, {info['y']:.1f})   {status}{vis_tag}")

    if robot_pose is not None:
        wx, wy, yaw = robot_pose
        print(f"\nRobot:  world ({wx:.2f}, {wy:.2f})"
              f"  yaw {math.degrees(yaw):.1f}°  facing {yaw_to_arrow(yaw)}")
    else:
        print("\nRobot:  (pose unavailable — is the simulation running?)")

    if is_colliding is True:
        print("Status: ⚡ COLLISION — robot is currently touching an obstacle")
    else:
        # False = confirmed clear; None = sensor silent (no contact msgs) = also clear
        print("Status: no collision")


# ── PNG renderer ──────────────────────────────────────────────────────────────

def render_png(
    pixels: list[list[int]], W: int, H: int, res: float,
    label_map: dict, spawn: dict, robot_pose, found: set[str],
    visible_ids: set[str], is_colliding: bool | None,
    obstacles: list[dict], path: str,
):
    """Render map to *path*."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        print("WARNING: cv2 not available, skipping PNG", file=sys.stderr)
        return None

    SCALE = 40  # px per metre (2× → higher res, icons smaller relative to walls)
    iw, ih = int(W * res * SCALE), int(H * res * SCALE)

    ppc = max(1, int(res * SCALE))
    grid_gray = np.where(np.array(pixels, dtype=np.uint8) >= 128, 255, 50)
    grid_up = cv2.resize(grid_gray.astype(np.uint8), (W * ppc, H * ppc),
                         interpolation=cv2.INTER_NEAREST)
    img = cv2.cvtColor(grid_up, cv2.COLOR_GRAY2BGR)

    # Draw obstacles (furniture)
    for obs in obstacles:
        cx, cy = obs["x"], obs["y"]
        hw, hh = obs["w"] / 2, obs["h"] / 2
        x1 = int((cx - hw) * SCALE)
        x2 = int((cx + hw) * SCALE)
        y1 = ih - int((cy + hh) * SCALE)
        y2 = ih - int((cy - hh) * SCALE)
        name = obs.get("name", "")
        if "desk" in name or "table" in name:
            color = (100, 70, 40)
        elif "chair" in name:
            color = (80, 80, 80)
        else:
            color = (130, 130, 150)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, -1)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 0, 0), 1)

    # Robot — kept small (8 px) so wall clearance is visible
    if robot_pose is not None:
        wx, wy, yaw = robot_pose
        ix, iy = int(wx * SCALE), ih - int(wy * SCALE)
        robot_color = (200, 80, 0)  # blue-orange
        ring_color  = (0, 0, 220) if is_colliding is True else (0, 0, 0)
        ring_thick  = 3 if is_colliding is True else 2
        cv2.circle(img, (ix, iy), 8, robot_color, -1)
        cv2.circle(img, (ix, iy), 8, ring_color, ring_thick)
        ax = ix + int(9 * math.cos(yaw))
        ay = iy - int(9 * math.sin(yaw))
        ex = ix + int(17 * math.cos(yaw))
        ey = iy - int(17 * math.sin(yaw))
        # Arrow: white outline first, then coloured fill — stays visible against dark walls
        cv2.arrowedLine(img, (ax, ay), (ex, ey), (255, 255, 255), 4, tipLength=0.5)
        cv2.arrowedLine(img, (ax, ay), (ex, ey), (0, 180, 255), 2, tipLength=0.5)
        cv2.putText(img, "R", (ix - 4, iy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

    # Object colours by type
    TYPE_COLOR = {
        "fire_extinguisher": (50,  50, 220),
        "first_aid_kit":     (50, 180,  50),
        "hazard_sign":       (30, 200, 230),
    }
    FOUND_COLOR   = (180, 180, 180)
    VISIBLE_RING  = (255, 255, 255)  # white ring = currently visible

    for lbl, info in label_map.items():
        # Backward-compat: old world_state.json without the field → treat as target
        if not info.get("mission_target", True):
            continue
        ix = int(info["x"] * SCALE)
        iy = ih - int(info["y"] * SCALE)
        color = FOUND_COLOR if lbl in found else TYPE_COLOR.get(info["type"], (180, 0, 180))
        cv2.circle(img, (ix, iy), 9, color, -1)
        # Outer ring: white if currently visible (found or not)
        ring_col = VISIBLE_RING if lbl in visible_ids else (0, 0, 0)
        ring_w   = 2 if lbl in visible_ids else 1
        cv2.circle(img, (ix, iy), 9, ring_col, ring_w)
        text_color = (0, 0, 0) if info["type"] == "hazard_sign" and lbl not in found else (255, 255, 255)
        cv2.putText(img, lbl, (ix - 6, iy + 4),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, text_color, 1)

    # Room name labels
    for rx, ry, name in ROOMS:
        ix, iy = int(rx * SCALE), ih - int(ry * SCALE)
        cv2.putText(img, name, (ix - 30, iy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (100, 100, 100), 1)

    cv2.imwrite(path, img)
    return img


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Render world map with robot and objects")
    parser.add_argument("--robot",   default="derpbot_0")
    parser.add_argument("--no-ros",  action="store_true", help="Skip ROS/gz pose lookup")
    parser.add_argument("--results", default=None,        help="Results JSON to mark found objects")
    parser.add_argument("--png",     default=None,        help="Save PNG to this path")
    parser.add_argument("--state",   default=str(WORLD_STATE), help="world_state.json path")
    parser.add_argument("--debug",   action="store_true", help="Print wall-clock execution time")
    args = parser.parse_args()
    t0 = time.monotonic()

    state_path = Path(args.state)
    if not state_path.exists():
        print(
            f"ERROR: {state_path} not found.\n"
            "Simulation is not running — start one with:\n"
            "  ./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless",
            file=sys.stderr,
        )
        sys.exit(1)

    state = json.loads(state_path.read_text())
    label_map: dict  = state["label_map"]
    spawn: dict      = state.get("spawn_pose", {"x": 1.0, "y": 1.0})
    res: float       = state.get("map_resolution", 0.5)
    obstacles: list  = state.get("obstacles", [])
    world            = Path(state["map_pgm"]).parent.name

    # Found objects — explicit --results overrides live detections file
    found: set[str] = set()
    if args.results:
        rj = json.loads(Path(args.results).read_text())
        for ev in rj.get("raw_metrics", {}).get("detection_events", []):
            found.add(str(ev["class_id"]))
    elif LIVE_DETECTIONS.exists():
        try:
            live = json.loads(LIVE_DETECTIONS.read_text())
            # Build name→label_key lookup for legacy entries without label_key
            _name_lbl = {
                f"{v['type']} #{v['instance']+1}": k
                for k, v in label_map.items()
            }
            for _x in live.get("found", []):
                if isinstance(_x, dict):
                    lk = _x.get("label_key") or _name_lbl.get(_x.get("name", ""))
                    if lk:
                        found.add(lk)
                else:
                    found.add(str(_x))
        except Exception:
            pass

    def _dt(label: str, prev: float) -> float:
        now = time.monotonic()
        if args.debug:
            print(f"[debug]   {label}: {now - prev:.3f}s", file=sys.stderr)
        return now

    t = time.monotonic()

    robot_pose, visible_ids_raw, is_colliding = get_sim_state(
        args.robot, world, spawn, timeout=3.0, no_ros=args.no_ros
    )
    t = _dt("get_sim_state", t)

    if robot_pose is None and not args.no_ros:
        print(
            "⚠️  No pose received — simulation may not be running.",
            file=sys.stderr,
        )

    # LOS-filter visible_ids using robot pose + PGM (needs PGM loaded first)
    pixels, W, H = read_pgm(Path(state["map_pgm"]))
    t = _dt("read_pgm", t)

    visible_ids: set[str] = set()
    if robot_pose is not None and pixels is not None:
        for cid in visible_ids_raw:
            if cid not in label_map:
                continue
            info = label_map[cid]
            if _has_line_of_sight(pixels, W, H, res, robot_pose, info["x"], info["y"]):
                visible_ids.add(cid)

    print_summary(label_map, spawn, robot_pose, found, visible_ids, is_colliding)

    png_path = args.png or str(DEFAULT_PNG)
    render_png(pixels, W, H, res, label_map, spawn, robot_pose, found,
               visible_ids, is_colliding, obstacles, png_path)
    t = _dt("render_png", t)
    print(png_path)
    print(f"\n⚠️  CHECK THE MAP BEFORE MOVING — walls and objects visible: {png_path}")
    if args.debug:
        print(f"[debug] total elapsed: {time.monotonic() - t0:.3f}s", file=sys.stderr)


if __name__ == "__main__":
    main()
