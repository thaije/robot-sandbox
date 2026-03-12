#!/usr/bin/env python3.12
"""robot_control.py — DerpBot live control utility.

Usage
-----
    # One-shot status (pose + sim time)
    python3.12 scripts/robot_control.py status

    # Save camera snapshot to /tmp/robot_snapshot.png
    python3.12 scripts/robot_control.py snapshot

    # Drive: linear m/s, angular rad/s, duration seconds (open-loop, ≤ 2 s)
    python3.12 scripts/robot_control.py drive 0.3 0.0 2.0
    python3.12 scripts/robot_control.py drive 0.0 0.6 2.0   # turn left ~70°

    # Closed-loop rotation by N degrees (+ = CCW/left, - = CW/right)
    python3.12 scripts/robot_control.py rotate 90
    python3.12 scripts/robot_control.py rotate -45

    # Closed-loop translation by N metres (+ = forward, - = backward)
    python3.12 scripts/robot_control.py move 2.0
    python3.12 scripts/robot_control.py move -0.5

Notes
-----
    - rotate/move use odom feedback and are far more accurate than drive.
    - Keep drive durations ≤ 2 s.
    - Object detections and collision status are in world_state.py, not here.

Optional flags
--------------
    --robot ROBOT_NAME   (default: derpbot_0)
    --timeout SECONDS    topic-wait timeout (default: 5.0)
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time
import threading
from pathlib import Path

# Allow importing from src/ whether the package is installed or not.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
from utils.gz_transport import gz_get_robot_pose  # noqa: E402

import rclpy
from rclpy.node import Node

_WORLD_STATE = Path("/tmp/arst_worlds/world_state.json")


def _sim_not_running_hint() -> str:
    return (
        "Simulation may not be running.\n"
        "Start one with:\n"
        "  ./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless"
    )


# ── Helpers ────────────────────────────────────────────────────────────────────

def _get_gz_world_pose(robot: str, timeout: float = 3.0):
    """Return (world_x, world_y, yaw) from Gazebo ground truth, or None."""
    try:
        state = json.loads(_WORLD_STATE.read_text())
        world = Path(state["map_pgm"]).parent.name
    except Exception:
        world = "indoor_office"
    return gz_get_robot_pose(robot, world, timeout=timeout)


def _yaw_from_quaternion(q) -> float:
    """Extract yaw (rad) from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _wait_for_message(node: Node, msg_type, topic: str, timeout: float):
    """Subscribe, spin until one message arrives, then return it (or None)."""
    received = []
    event = threading.Event()

    def cb(msg):
        if not received:
            received.append(msg)
            event.set()

    sub = node.create_subscription(msg_type, topic, cb, 1)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    deadline = time.monotonic() + timeout
    while not event.is_set() and time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.02)

    executor.shutdown()
    node.destroy_subscription(sub)
    return received[0] if received else None


# ── Subcommands ────────────────────────────────────────────────────────────────

def cmd_status(args):
    gz_pose = _get_gz_world_pose(args.robot, timeout=args.timeout)
    if gz_pose is not None:
        wx, wy, yaw = gz_pose
        print(f"Robot : {args.robot}")
        print(f"World : x={wx:.3f}  y={wy:.3f}  (ground truth)")
        print(f"Yaw   : {yaw:.3f} rad  ({math.degrees(yaw):.1f}°)")
        return 0

    from nav_msgs.msg import Odometry

    try:
        state = json.loads(_WORLD_STATE.read_text())
        spawn = state.get("spawn_pose", {"x": 1.0, "y": 1.0})
    except Exception:
        spawn = {"x": 1.0, "y": 1.0}

    rclpy.init()
    node = rclpy.create_node("rc_status")
    try:
        msg = _wait_for_message(node, Odometry, f"/{args.robot}/odom", args.timeout)
        if msg is None:
            print(
                f"ERROR: no pose from Gazebo or /{args.robot}/odom within {args.timeout}s.\n"
                + _sim_not_running_hint()
            )
            return 1
        p = msg.pose.pose.position
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        wx = p.x + spawn.get("x", 1.0)
        wy = p.y + spawn.get("y", 1.0)
        print(f"Robot : {args.robot}")
        print(f"World : x={wx:.3f}  y={wy:.3f}  (odom estimate — may drift after collisions)")
        print(f"Yaw   : {yaw:.3f} rad  ({math.degrees(yaw):.1f}°)")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def cmd_snapshot(args):
    from sensor_msgs.msg import Image
    import numpy as np
    import cv2

    rclpy.init()
    node = rclpy.create_node("rc_snapshot")
    try:
        msg = _wait_for_message(node, Image, f"/{args.robot}/rgbd/image", args.timeout)
        if msg is None:
            print(
                f"ERROR: no image on /{args.robot}/rgbd/image within {args.timeout}s.\n"
                + _sim_not_running_hint()
            )
            return 1

        dtype = np.uint8
        arr = np.frombuffer(bytes(msg.data), dtype=dtype)

        if msg.encoding in ("rgb8", "RGB8"):
            arr = arr.reshape(msg.height, msg.width, 3)
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
        elif msg.encoding in ("bgr8", "BGR8"):
            arr = arr.reshape(msg.height, msg.width, 3)
        elif msg.encoding in ("mono8", "MONO8"):
            arr = arr.reshape(msg.height, msg.width)
        else:
            print(f"WARNING: unknown encoding '{msg.encoding}', treating as rgb8")
            arr = arr.reshape(msg.height, msg.width, 3)
            arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)

        out = args.output or "/tmp/robot_snapshot.png"
        cv2.imwrite(out, arr)
        print(f"Saved {msg.width}×{msg.height} ({msg.encoding}) → {out}")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def cmd_drive(args):
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("rc_drive")
    pub = node.create_publisher(Twist, f"/{args.robot}/cmd_vel", 1)

    twist = Twist()
    twist.linear.x  = float(args.vx)
    twist.angular.z = float(args.wz)
    duration = float(args.duration)

    print(f"Driving {args.robot}: vx={args.vx}  wz={args.wz}  for {duration}s …")
    start = time.monotonic()
    try:
        while time.monotonic() - start < duration:
            pub.publish(twist)
            time.sleep(0.1)
    finally:
        stop = Twist()
        stop_deadline = time.monotonic() + 0.5
        while time.monotonic() < stop_deadline:
            pub.publish(stop)
            time.sleep(0.05)
        node.destroy_node()
        rclpy.shutdown()

    elapsed = time.monotonic() - start
    dist = abs(args.vx) * elapsed
    angle = math.degrees(args.wz * elapsed)
    print(f"Done — {elapsed:.1f}s  (~{dist:.2f}m linear / {angle:.1f}° rotation)")
    return 0


# ── Closed-loop helpers ────────────────────────────────────────────────────────

def _start_odom_listener(node, robot):
    """Subscribe to odom; return (latest_odom_list, ready_event, executor, thread)."""
    from nav_msgs.msg import Odometry

    latest = [None]
    ready = threading.Event()

    def cb(msg):
        latest[0] = msg
        ready.set()

    node.create_subscription(Odometry, f"/{robot}/odom", cb, 10)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    return latest, ready, executor, thread


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference a − b, result in (−π, π]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def cmd_rotate(args):
    """Rotate by args.degrees using closed-loop odom feedback."""
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("rc_rotate")
    pub = node.create_publisher(Twist, f"/{args.robot}/cmd_vel", 1)
    latest, ready, executor, spin_thread = _start_odom_listener(node, args.robot)

    if not ready.wait(timeout=args.timeout):
        print(
            f"ERROR: no odom on /{args.robot}/odom within {args.timeout}s.\n"
            + _sim_not_running_hint()
        )
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    initial_yaw = _yaw_from_quaternion(latest[0].pose.pose.orientation)
    target_rad = math.radians(float(args.degrees))
    KP, MAX_WZ, THRESHOLD = 2.0, 0.8, math.radians(1.5)

    print(f"Rotating {args.degrees}° ({'CCW' if args.degrees >= 0 else 'CW'}) …")
    start = time.monotonic()
    try:
        while True:
            current_yaw = _yaw_from_quaternion(latest[0].pose.pose.orientation)
            rotated = _angle_diff(current_yaw, initial_yaw)
            remaining = _angle_diff(target_rad, rotated)
            if abs(remaining) < THRESHOLD:
                break
            twist = Twist()
            twist.angular.z = max(-MAX_WZ, min(MAX_WZ, KP * remaining))
            pub.publish(twist)
            time.sleep(0.05)
    finally:
        stop = Twist()
        for _ in range(10):
            pub.publish(stop)
            time.sleep(0.05)
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    elapsed = time.monotonic() - start
    current_yaw = _yaw_from_quaternion(latest[0].pose.pose.orientation)
    actual = math.degrees(_angle_diff(current_yaw, initial_yaw))
    print(f"Done — {elapsed:.1f}s  target {args.degrees}°  actual {actual:.1f}°")
    return 0


def cmd_move(args):
    """Translate by args.metres using closed-loop odom feedback."""
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("rc_move")
    pub = node.create_publisher(Twist, f"/{args.robot}/cmd_vel", 1)
    latest, ready, executor, spin_thread = _start_odom_listener(node, args.robot)

    if not ready.wait(timeout=args.timeout):
        print(
            f"ERROR: no odom on /{args.robot}/odom within {args.timeout}s.\n"
            + _sim_not_running_hint()
        )
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    x0 = latest[0].pose.pose.position.x
    y0 = latest[0].pose.pose.position.y
    target = float(args.metres)
    KP, MAX_VX, THRESHOLD = 1.5, 0.5, 0.04  # 4 cm

    print(f"Moving {args.metres} m ({'forward' if target >= 0 else 'backward'}) …")
    start = time.monotonic()
    try:
        while True:
            p = latest[0].pose.pose.position
            displacement = math.sqrt((p.x - x0) ** 2 + (p.y - y0) ** 2)
            remaining = abs(target) - displacement
            if remaining < THRESHOLD:
                break
            vx = math.copysign(min(MAX_VX, KP * remaining), target)
            twist = Twist()
            twist.linear.x = vx
            pub.publish(twist)
            time.sleep(0.05)
    finally:
        stop = Twist()
        for _ in range(10):
            pub.publish(stop)
            time.sleep(0.05)
        executor.shutdown()
        spin_thread.join(timeout=2.0)
        node.destroy_node()
        rclpy.shutdown()

    elapsed = time.monotonic() - start
    p = latest[0].pose.pose.position
    actual = math.sqrt((p.x - x0) ** 2 + (p.y - y0) ** 2)
    print(f"Done — {elapsed:.1f}s  target {target:.2f}m  actual {actual:.2f}m")
    return 0


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="DerpBot control utility")
    parser.add_argument("--robot",   default="derpbot_0", help="Robot name (default: derpbot_0)")
    parser.add_argument("--timeout", default=5.0, type=float, help="Topic wait timeout in seconds")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status",   help="Print current pose")

    snap = sub.add_parser("snapshot", help="Save camera frame to PNG")
    snap.add_argument("--output", default=None, help="Output path (default: /tmp/robot_snapshot.png)")

    drv = sub.add_parser("drive", help="Drive for N seconds then stop (open-loop, ≤ 2 s)")
    drv.add_argument("vx",       type=float, help="Linear velocity m/s (+forward)")
    drv.add_argument("wz",       type=float, help="Angular velocity rad/s (+left)")
    drv.add_argument("duration", type=float, help="Duration in seconds")

    rot = sub.add_parser("rotate", help="Rotate N degrees with odom feedback (+ = CCW/left)")
    rot.add_argument("degrees", type=float, help="Degrees to rotate (+ CCW, - CW)")

    mv = sub.add_parser("move", help="Translate N metres with odom feedback (+ = forward)")
    mv.add_argument("metres", type=float, help="Distance in metres (+ forward, - backward)")

    parser.add_argument("--debug", action="store_true", help="Print wall-clock execution time")

    args = parser.parse_args()

    dispatch = {
        "status":   cmd_status,
        "snapshot": cmd_snapshot,
        "drive":    cmd_drive,
        "rotate":   cmd_rotate,
        "move":     cmd_move,
    }
    t0 = time.monotonic()
    ret = dispatch[args.command](args)
    if args.debug:
        print(f"[debug] elapsed: {time.monotonic() - t0:.2f}s", file=sys.stderr)
    sys.exit(ret or 0)


if __name__ == "__main__":
    main()
