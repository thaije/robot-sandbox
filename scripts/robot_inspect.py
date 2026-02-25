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

    # Drive: linear m/s, angular rad/s, duration seconds
    python3 scripts/robot_inspect.py drive 0.3 0.0 3.0
    python3 scripts/robot_inspect.py drive 0.0 0.6 2.0   # turn left ~70 deg

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
    from nav_msgs.msg import Odometry

    rclpy.init()
    node = rclpy.create_node("ri_status")
    try:
        msg = _wait_for_message(node, Odometry, f"/{args.robot}/odom", args.timeout)
        if msg is None:
            print(f"ERROR: no message on /{args.robot}/odom within {args.timeout}s")
            return 1
        p = msg.pose.pose.position
        yaw = _yaw_from_quaternion(msg.pose.pose.orientation)
        yaw_deg = math.degrees(yaw)
        print(f"Robot : {args.robot}")
        print(f"Pose  : x={p.x:.3f}  y={p.y:.3f}  z={p.z:.3f}")
        print(f"Yaw   : {yaw:.3f} rad  ({yaw_deg:.1f}°)")
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

    drv = sub.add_parser("drive", help="Drive for N seconds then stop")
    drv.add_argument("vx",       type=float, help="Linear velocity m/s (+forward)")
    drv.add_argument("wz",       type=float, help="Angular velocity rad/s (+left)")
    drv.add_argument("duration", type=float, help="Duration in seconds")

    args = parser.parse_args()

    dispatch = {
        "status":     cmd_status,
        "snapshot":   cmd_snapshot,
        "detections": cmd_detections,
        "drive":      cmd_drive,
    }
    sys.exit(dispatch[args.command](args))


if __name__ == "__main__":
    main()
