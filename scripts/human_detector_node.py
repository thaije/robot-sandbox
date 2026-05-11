#!/usr/bin/env python3.12
"""Human detector node — manual perception baseline.

Publishes Detection2DArray messages from keypresses while a human teleops
DerpBot through the environment.  Each keypress registers a detection at
the robot's current odom position.

Usage
-----
    # Sim must already be running (headless or GUI):
    #   ./scripts/run_scenario.sh config/scenarios/.../easy.yaml --headless

    # Terminal 2: teleop
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel

    # Terminal 3: human detector
    python3.12 scripts/human_detector_node.py

    # Terminal 4: robot camera view (required for perception baseline)
    ros2 run rqt_image_view rqt_image_view
    # Select /derpbot_0/rgbd/image

Keybindings
-----------
    f  → fire_extinguisher
    k  → first_aid_kit
    p  → person
    ?  → show help
    q  → quit

The node queries http://localhost:7400/mission at startup to discover target
types and builds keybindings dynamically.  If the mission server is unavailable,
the hardcoded defaults above are used.

Detections use the robot's IMU-fused odom-frame (x, y) position as the estimated
object location.  The scoring engine applies the spawn-offset transform to convert
to world frame.  The EKF-fused odom corrects differential-drive yaw drift, making
detections significantly more accurate than raw wheel-encoder odometry.

Notes
-----
    - use_sim_time=True is required (same as all ROS 2 nodes in this sim).
    - Tracking IDs are auto-incremented per keypress.  The human is responsible
      for pressing only once per distinct object; duplicate presses produce
      duplicate positives that the scoring engine handles naturally.
    - rqt_image_view is recommended for seeing the robot camera feed:
          ros2 run rqt_image_view rqt_image_view
      Then select /derpbot_0/rgbd/image from the dropdown.
"""
from __future__ import annotations

import argparse
import json
import select
import sys
import termios
import time
import tty
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose

MISSION_PORT = 7400


def _fetch_mission_targets() -> list[str]:
    """Fetch target types from mission server. Returns empty list on failure."""
    try:
        url = f"http://localhost:{MISSION_PORT}/mission"
        with urllib.request.urlopen(url, timeout=3) as resp:
            data = json.loads(resp.read())
        return [t["type"] for t in data.get("targets", []) if t.get("type")]
    except Exception:
        return []


def _build_keybindings(target_types: list[str]) -> dict[str, str]:
    """Map single keypresses to target type strings.

    Generates short abbreviations for each type, preferring the first
    unique character.  Falls back to positional keys if collisions occur.
    """
    bindings: dict[str, str] = {}
    used_keys: set[str] = set()
    for ttype in target_types:
        # Prefer first character if not taken
        key = ttype[0].lower()
        if key not in used_keys:
            bindings[key] = ttype
            used_keys.add(key)
            continue
        # Try first char of each word (underscore-separated)
        for part in ttype.split("_"):
            k = part[0].lower()
            if k not in used_keys:
                bindings[k] = ttype
                used_keys.add(k)
                break
        else:
            # Fallback: number keys
            for n in range(1, 10):
                k = str(n)
                if k not in used_keys:
                    bindings[k] = ttype
                    used_keys.add(k)
                    break
    return bindings


class HumanDetectorNode(Node):
    """ROS 2 node that publishes detections from keypresses."""

    def __init__(self, robot: str, bindings: dict[str, str]) -> None:
        super().__init__(
            "human_detector",
            parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
        )
        self._robot = robot
        self._bindings = bindings
        self._track_counter = 0

        self._odom_x: float = 0.0
        self._odom_y: float = 0.0
        self._odom_ready = False

        self._pub = self.create_publisher(
            Detection2DArray, f"/{robot}/detections", 10
        )
        self.create_subscription(Odometry, f"/{robot}/odom", self._on_odom, 10)

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_x = msg.pose.pose.position.x
        self._odom_y = msg.pose.pose.position.y
        self._odom_ready = True

    def publish_detection(self, class_type: str) -> None:
        """Publish a single detection at the current robot odom position."""
        self._track_counter += 1
        tracking_id = f"human_{self._track_counter}"

        det = Detection2D()
        det.header = Header()
        det.header.frame_id = "map"
        det.id = tracking_id

        hyp = ObjectHypothesisWithPose()
        hyp.hypothesis.class_id = class_type
        hyp.hypothesis.score = 1.0
        hyp.pose.pose.position.x = self._odom_x
        hyp.pose.pose.position.y = self._odom_y
        det.results.append(hyp)

        msg = Detection2DArray()
        msg.header = det.header
        msg.detections.append(det)

        self._pub.publish(msg)
        self.get_logger().info(
            f"  [{class_type}] id={tracking_id}  pos=({self._odom_x:.2f}, {self._odom_y:.2f})"
        )


def _get_key() -> str:
    """Read a single keypress from stdin without Enter (raw mode)."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        if select.select([sys.stdin], [], [], 0.1)[0]:
            return sys.stdin.read(1)
        return ""
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Human detector node — manual perception baseline"
    )
    parser.add_argument("--robot", default="derpbot_0", help="Robot name")
    args = parser.parse_args()

    # Discover target types from mission server
    target_types = _fetch_mission_targets()
    if target_types:
        print(f"[human_detector] Targets from mission server: {target_types}")
    else:
        target_types = ["fire_extinguisher", "first_aid_kit", "person"]
        print("[human_detector] Mission server unavailable, using defaults:", target_types)

    bindings = _build_keybindings(target_types)

    rclpy.init()
    node = HumanDetectorNode(robot=args.robot, bindings=bindings)

    print()
    print("━" * 60)
    print("  Human Detector — keypress detection publisher")
    print("━" * 60)
    print()
    print("  Keybindings:")
    for key, ttype in sorted(bindings.items()):
        print(f"    {key}  →  {ttype}")
    print()
    print("    ?  →  show help")
    print("    q  →  quit")
    print()
    print("  Tip: run rqt_image_view in another terminal to see the")
    print("  robot camera feed for spotting objects:")
    print("    ros2 run rqt_image_view rqt_image_view")
    print("  Then select /derpbot_0/rgbd/image")
    print()
    print("  Press a detection key each time you see a target object.")
    print("  Press only once per distinct object instance.")
    print()
    print("━" * 60)

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.05)
            key = _get_key()
            if not key:
                continue
            if key == "q":
                break
            if key == "?":
                print()
                print("  Keybindings:")
                for k, ttype in sorted(bindings.items()):
                    print(f"    {k}  →  {ttype}")
                print("    q  →  quit")
                print()
                continue
            if key in bindings:
                ttype = bindings[key]
                if not node._odom_ready:
                    node.get_logger().warn("  No odom data yet — detection not published")
                    continue
                node.publish_detection(ttype)
            elif key in ("\x03", "\x04"):  # Ctrl-C, Ctrl-D
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print("[human_detector] done")


if __name__ == "__main__":
    main()