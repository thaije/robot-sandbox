#!/usr/bin/env python3.12
"""map_publisher.py — watches arst_world_map.png and publishes to /arst/world_map.

Runs as a long-lived ROS 2 node alongside a scenario.  Detects file changes via
mtime polling (every 0.5 s) and republishes with TRANSIENT_LOCAL so RViz always
shows the latest map, including late-joining subscribers.

Started automatically by run_scenario.sh.  Can also be run standalone:
    python3.12 scripts/map_publisher.py [--png PATH]
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT / "src"))

import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image as RosImage


_DEFAULT_PNG = _REPO_ROOT / "arst_world_map.png"
_POLL_INTERVAL = 0.5  # seconds between mtime checks


def _read_as_ros_image(png_path: Path) -> RosImage | None:
    try:
        import cv2
        img_bgr = cv2.imread(str(png_path))
        if img_bgr is None:
            return None
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
        return msg
    except Exception as e:
        print(f"[map_publisher] read error: {e}", file=sys.stderr)
        return None


def main() -> None:
    parser = argparse.ArgumentParser(description="Watch PNG and publish to /arst/world_map")
    parser.add_argument("--png", default=str(_DEFAULT_PNG), help="PNG file to watch")
    args = parser.parse_args()

    png_path = Path(args.png)

    rclpy.init()
    node = rclpy.create_node("arst_map_publisher")
    qos = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
    )
    pub = node.create_publisher(RosImage, "/arst/world_map", qos)

    last_mtime: float = 0.0
    try:
        while rclpy.ok():
            try:
                mtime = png_path.stat().st_mtime
                if mtime != last_mtime:
                    last_mtime = mtime
                    msg = _read_as_ros_image(png_path)
                    if msg is not None:
                        pub.publish(msg)
            except FileNotFoundError:
                pass  # PNG not written yet — wait
            time.sleep(_POLL_INTERVAL)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
