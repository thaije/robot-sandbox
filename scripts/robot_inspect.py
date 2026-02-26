#!/usr/bin/env python3.12
"""robot_inspect.py — DerpBot live inspection and control utility.

Usage
-----
    # One-shot status (pose + sim time)
    python3 scripts/robot_inspect.py status

    # Save camera snapshot to /tmp/robot_snapshot.png
    python3 scripts/robot_inspect.py snapshot

    # Print current detection events
    python3 scripts/robot_inspect.py detections

    # Drive: linear m/s, angular rad/s, duration seconds (open-loop)
    python3 scripts/robot_inspect.py drive 0.3 0.0 3.0
    python3 scripts/robot_inspect.py drive 0.0 0.6 2.0   # turn left ~70 deg

    # Closed-loop rotation by N degrees (+ = CCW/left, - = CW/right)
    python3 scripts/robot_inspect.py rotate 90
    python3 scripts/robot_inspect.py rotate -45

    # Closed-loop translation by N metres (+ = forward, - = backward)
    python3 scripts/robot_inspect.py move 2.0
    python3 scripts/robot_inspect.py move -0.5

Notes
-----
    - Keep drive durations ≤ 2 s. The diff-drive controller holds the last
      velocity indefinitely; the drive command sends a stop after each call,
      but long durations risk the robot continuing into walls.
    - rotate/move use odom feedback and are much more accurate than drive.
    - Pose from `status` is in odom frame (starts at 0,0 at spawn).

Optional flags
--------------
    --robot ROBOT_NAME   (default: derpbot_0)
    --timeout SECONDS    topic-wait timeout (default: 5.0)
"""
from __future__ import annotations

import argparse
import math
import sys
import time
import threading

import rclpy
from rclpy.node import Node


# ── Helpers ────────────────────────────────────────────────────────────────────

def _get_gz_world_pose(robot: str, timeout: float = 3.0):
    """Return (world_x, world_y, yaw) from Gazebo ground truth, or None."""
    import json, threading
    from pathlib import Path
    try:
        state = json.loads(Path("/tmp/arst_worlds/world_state.json").read_text())
        world = Path(state["map_pgm"]).parent.name
    except Exception:
        world = "indoor_office"

    try:
        from gz.transport13 import Node as GzNode
        from gz.msgs10.pose_v_pb2 import Pose_V

        gz_node = GzNode()
        result: list = []
        ev = threading.Event()

        def cb(msg):
            if not result:
                result.append(msg)
                ev.set()

        gz_node.subscribe(Pose_V, f"/world/{world}/dynamic_pose/info", cb)
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
    return None


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
        executor.spin_once(timeout_sec=0.1)

    executor.shutdown()
    node.destroy_subscription(sub)
    return received[0] if received else None


# ── Subcommands ────────────────────────────────────────────────────────────────

def cmd_status(args):
    import json
    from pathlib import Path

    # Try Gazebo ground truth first (no odometry drift)
    gz_pose = _get_gz_world_pose(args.robot, timeout=args.timeout)
    if gz_pose is not None:
        wx, wy, yaw = gz_pose
        print(f"Robot : {args.robot}")
        print(f"World : x={wx:.3f}  y={wy:.3f}  (ground truth)")
        print(f"Yaw   : {yaw:.3f} rad  ({math.degrees(yaw):.1f}°)")
        return 0

    # Fallback: odometry + spawn offset
    from nav_msgs.msg import Odometry

    try:
        state = json.loads(Path("/tmp/arst_worlds/world_state.json").read_text())
        spawn = state.get("spawn_pose", {"x": 1.0, "y": 1.0})
    except Exception:
        spawn = {"x": 1.0, "y": 1.0}

    rclpy.init()
    node = rclpy.create_node("ri_status")
    try:
        msg = _wait_for_message(node, Odometry, f"/{args.robot}/odom", args.timeout)
        if msg is None:
            print(f"ERROR: no pose available (gz unreachable, no odom on /{args.robot}/odom within {args.timeout}s)")
            return 1
        p = msg.pose.pose.position
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        wx = p.x + spawn.get("x", 1.0)
        wy = p.y + spawn.get("y", 1.0)
        print(f"Robot : {args.robot}")
        print(f"World : x={wx:.3f}  y={wy:.3f}  (odom estimate — may drift after collisions)")
        print(f"Yaw   : {yaw:.3f} rad  ({math.degrees(yaw):.1f}°)")
        stamp = msg.header.stamp
        print(f"Stamp : {stamp.sec}.{stamp.nanosec // 1_000_000:03d} s")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def cmd_snapshot(args):
    from sensor_msgs.msg import Image
    import numpy as np
    import cv2

    rclpy.init()
    node = rclpy.create_node("ri_snapshot")
    try:
        msg = _wait_for_message(node, Image, f"/{args.robot}/image_raw", args.timeout)
        if msg is None:
            print(f"ERROR: no message on /{args.robot}/image_raw within {args.timeout}s")
            return 1

        # Convert raw bytes → numpy → cv2 image
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


def cmd_detections(args):
    from vision_msgs.msg import Detection2DArray

    rclpy.init()
    node = rclpy.create_node("ri_detections")
    try:
        msg = _wait_for_message(node, Detection2DArray, f"/{args.robot}/detections", args.timeout)
        if msg is None:
            print(f"No detections on /{args.robot}/detections within {args.timeout}s (nothing in view?)")
            return 0
        if not msg.detections:
            print("Message received but 0 detections in frame.")
            return 0
        print(f"Detections in current frame ({len(msg.detections)}):")
        for i, det in enumerate(msg.detections):
            for hyp in det.results:
                bb = det.bbox
                print(
                    f"  [{i}] label={hyp.hypothesis.class_id}  score={hyp.hypothesis.score:.2f}"
                    f"  bbox=({bb.center.position.x:.0f},{bb.center.position.y:.0f})"
                    f"  {bb.size_x:.0f}×{bb.size_y:.0f}"
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


def cmd_drive(args):
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("ri_drive")
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
        # Send zero-velocity for 0.5s to ensure the controller receives it
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
    """Subscribe to odom; return (latest_odom_list, executor, spin_thread).

    The executor spins in a background daemon thread so the main thread can
    run a control loop without blocking.
    """
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
    return latest, ready, executor


def _angle_diff(a: float, b: float) -> float:
    """Shortest signed angular difference a − b, result in (−π, π]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))


def cmd_rotate(args):
    """Rotate by args.degrees using closed-loop odom feedback."""
    from geometry_msgs.msg import Twist

    rclpy.init()
    node = rclpy.create_node("ri_rotate")
    pub = node.create_publisher(Twist, f"/{args.robot}/cmd_vel", 1)
    latest, ready, executor = _start_odom_listener(node, args.robot)

    if not ready.wait(timeout=args.timeout):
        print(f"ERROR: no odom on /{args.robot}/odom within {args.timeout}s")
        executor.shutdown()
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
    node = rclpy.create_node("ri_move")
    pub = node.create_publisher(Twist, f"/{args.robot}/cmd_vel", 1)
    latest, ready, executor = _start_odom_listener(node, args.robot)

    if not ready.wait(timeout=args.timeout):
        print(f"ERROR: no odom on /{args.robot}/odom within {args.timeout}s")
        executor.shutdown()
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
        node.destroy_node()
        rclpy.shutdown()

    elapsed = time.monotonic() - start
    p = latest[0].pose.pose.position
    actual = math.sqrt((p.x - x0) ** 2 + (p.y - y0) ** 2)
    print(f"Done — {elapsed:.1f}s  target {target:.2f}m  actual {actual:.2f}m")
    return 0


# ── Entry point ────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="DerpBot inspection & control utility")
    parser.add_argument("--robot",   default="derpbot_0", help="Robot name (default: derpbot_0)")
    parser.add_argument("--timeout", default=5.0, type=float, help="Topic wait timeout in seconds")

    sub = parser.add_subparsers(dest="command", required=True)

    sub.add_parser("status",     help="Print current pose and sim time")

    snap = sub.add_parser("snapshot", help="Save camera frame to PNG")
    snap.add_argument("--output", default=None, help="Output path (default: /tmp/robot_snapshot.png)")

    sub.add_parser("detections", help="Print objects visible in current frame")

    drv = sub.add_parser("drive", help="Drive for N seconds then stop (open-loop)")
    drv.add_argument("vx",       type=float, help="Linear velocity m/s (+forward)")
    drv.add_argument("wz",       type=float, help="Angular velocity rad/s (+left)")
    drv.add_argument("duration", type=float, help="Duration in seconds")

    rot = sub.add_parser("rotate", help="Rotate by N degrees using odom feedback (+ = CCW/left)")
    rot.add_argument("degrees", type=float, help="Degrees to rotate (+ CCW, - CW)")

    mv = sub.add_parser("move", help="Translate N metres using odom feedback (+ = forward)")
    mv.add_argument("metres", type=float, help="Distance in metres (+ forward, - backward)")

    args = parser.parse_args()

    dispatch = {
        "status":     cmd_status,
        "snapshot":   cmd_snapshot,
        "detections": cmd_detections,
        "drive":      cmd_drive,
        "rotate":     cmd_rotate,
        "move":       cmd_move,
    }
    sys.exit(dispatch[args.command](args))


if __name__ == "__main__":
    main()
