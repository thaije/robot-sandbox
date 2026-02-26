#!/usr/bin/env python3.12
"""world_state.py — ASCII + PNG world map for agent navigation.

Shows the floor plan with robot position, object locations (found/unfound),
and room labels — all derived from ground truth. Intended as a cheat tool
for automated testing, not for use by the robot itself.

Coordinates are in **world frame**. The robot spawns at world (1, 1) which
equals odom (0, 0). Room layout and doorway positions are visible in the
map itself — no need to hard-code them.

Usage
-----
    # Live robot pose (requires running scenario):
    python3.12 scripts/world_state.py

    # Mark found objects from a results JSON:
    python3.12 scripts/world_state.py --results results/run.json

    # Skip ROS (no robot marker):
    python3.12 scripts/world_state.py --no-ros

    # Also save a PNG:
    python3.12 scripts/world_state.py --png /tmp/map.png
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


# ── ROS pose fetch ────────────────────────────────────────────────────────────

def get_robot_odom(robot: str, timeout: float = 3.0):
    """Return (odom_x, odom_y, yaw) or None if ROS unavailable."""
    try:
        import rclpy
        from nav_msgs.msg import Odometry

        rclpy.init()
        node = rclpy.create_node("ws_node")
        received: list = []
        ev = threading.Event()

        def cb(msg):
            if not received:
                received.append(msg)
                ev.set()

        node.create_subscription(Odometry, f"/{robot}/odom", cb, 1)
        ex = rclpy.executors.SingleThreadedExecutor()
        ex.add_node(node)
        t0 = time.monotonic()
        while not ev.is_set() and time.monotonic() - t0 < timeout:
            ex.spin_once(timeout_sec=0.1)
        ex.shutdown()
        node.destroy_node()
        rclpy.shutdown()

        if received:
            p = received[0].pose.pose.position
            q = received[0].pose.pose.orientation
            yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
            return p.x, p.y, yaw
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
        ox, oy, yaw = robot_pose
        wx, wy = ox + spawn.get("x", 1.0), oy + spawn.get("y", 1.0)
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
        ox, oy, yaw = robot_pose
        wx, wy = ox + spawn.get("x", 1.0), oy + spawn.get("y", 1.0)
        print(f"\nRobot:  world ({wx:.2f}, {wy:.2f})  odom ({ox:.2f}, {oy:.2f})"
              f"  yaw {math.degrees(yaw):.1f}°  facing {yaw_to_arrow(yaw)}")
    else:
        print("\nRobot:  (pose unavailable)")


# ── PNG renderer ──────────────────────────────────────────────────────────────

def render_png(
    pixels: list[list[int]], W: int, H: int, res: float,
    label_map: dict, spawn: dict, robot_pose, found: set[str], path: str,
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

    # Draw occupancy grid
    # PGM row 0 = north (high world-y) → image row 0 = top; no y-flip needed here.
    for r in range(H):
        for c in range(W):
            y1 = int(r * res * SCALE)
            y2 = int((r + 1) * res * SCALE)
            x1 = int(c * res * SCALE)
            x2 = int((c + 1) * res * SCALE)
            color = (255, 255, 255) if pixels[r][c] >= 128 else (50, 50, 50)
            img[y1:y2, x1:x2] = color

    # Room outlines (light grey)
    for rx, ry, name in ROOMS:
        pass  # room boundaries come from PGM walls, nothing extra needed

    # Object colours by type
    TYPE_COLOR = {
        "fire_extinguisher": (50,  50, 220),   # red
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

    # Robot
    if robot_pose is not None:
        ox, oy, yaw = robot_pose
        wx, wy = ox + spawn.get("x", 1.0), oy + spawn.get("y", 1.0)
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

    # Room name labels
    for rx, ry, name in ROOMS:
        ix, iy = int(rx * SCALE), ih - int(ry * SCALE)
        cv2.putText(img, name, (ix - 30, iy),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.35, (100, 100, 100), 1)

    cv2.imwrite(path, img)
    print(f"PNG saved → {path}")


# ── Entry point ───────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Render world map with robot and objects")
    parser.add_argument("--robot",   default="derpbot_0")
    parser.add_argument("--no-ros",  action="store_true", help="Skip ROS pose lookup")
    parser.add_argument("--results", default=None,        help="Results JSON to mark found objects")
    parser.add_argument("--png",     default=None,        help="Save PNG to this path")
    parser.add_argument("--state",   default=str(WORLD_STATE), help="world_state.json path")
    args = parser.parse_args()

    state_path = Path(args.state)
    if not state_path.exists():
        print(f"ERROR: {state_path} not found. Run a scenario first.", file=sys.stderr)
        sys.exit(1)

    state = json.loads(state_path.read_text())
    label_map: dict = state["label_map"]
    spawn: dict     = state.get("spawn_pose", {"x": 1.0, "y": 1.0})
    res: float      = state.get("map_resolution", 0.5)

    # Mark found objects from results JSON
    found: set[str] = set()
    if args.results:
        rj = json.loads(Path(args.results).read_text())
        for ev in rj.get("raw_metrics", {}).get("detection_events", []):
            found.add(str(ev["class_id"]))

    # Robot pose
    robot_pose = None
    if not args.no_ros:
        robot_pose = get_robot_odom(args.robot, timeout=3.0)
        if robot_pose is None:
            print("(no odom received — robot marker omitted)\n")

    pixels, W, H = read_pgm(Path(state["map_pgm"]))

    render_ascii(pixels, W, H, res, label_map, spawn, robot_pose, found)
    print_summary(label_map, spawn, robot_pose, found)

    if args.png:
        render_png(pixels, W, H, res, label_map, spawn, robot_pose, found, args.png)


if __name__ == "__main__":
    main()
