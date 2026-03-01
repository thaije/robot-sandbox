"""Patrol bot waypoint controller — Step 3.14.

Standalone script: drives a patrol_bot model through a closed waypoint loop
using gz-transport (no ROS 2 bridge required).

Run by SimulationLauncher as a subprocess:

    python3.12 patrol_bot_controller.py \\
        --name patrol_bot_0 \\
        --waypoints "[[5,8],[15,8],[15,3],[5,3]]" \\
        --speed 0.4 \\
        --turn_speed 0.6

Algorithm: dead-reckoning only (no feedback). Between each waypoint pair:
    1. Rotate in place to face the target heading.
    2. Drive straight to the target.
    Waypoints loop indefinitely.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time

# gz.transport13 lives in the system Python path, not in the project venv.
sys.path.insert(0, "/usr/lib/python3/dist-packages")

from gz.transport13 import Node  # type: ignore[import]
from gz.msgs10.twist_pb2 import Twist  # type: ignore[import]


def _clamp_angle(a: float) -> float:
    """Normalise *a* to (-π, π]."""
    while a > math.pi:
        a -= 2 * math.pi
    while a <= -math.pi:
        a += 2 * math.pi
    return a


def run(name: str, waypoints: list[list[float]], speed: float, turn_speed: float) -> None:
    node = Node()
    topic = f"/model/{name}/cmd_vel"
    pub = node.advertise(topic, Twist)

    def publish(linear_x: float, angular_z: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        pub.publish(msg)

    def stop() -> None:
        publish(0.0, 0.0)

    # Allow gz-sim to discover the topic before publishing
    time.sleep(2.0)

    heading = 0.0     # current heading (radians), starts aligned with +X
    wp_idx = 0
    n = len(waypoints)

    while True:
        wp_curr = waypoints[wp_idx % n]
        wp_next = waypoints[(wp_idx + 1) % n]

        dx = wp_next[0] - wp_curr[0]
        dy = wp_next[1] - wp_curr[1]
        distance = math.hypot(dx, dy)
        target_heading = math.atan2(dy, dx)

        # ── Step 1: rotate in place ──────────────────────────────────────────
        turn = _clamp_angle(target_heading - heading)
        if abs(turn) > 0.05:  # skip tiny turns
            direction = 1.0 if turn > 0 else -1.0
            turn_time = abs(turn) / turn_speed
            t_end = time.monotonic() + turn_time
            while time.monotonic() < t_end:
                publish(0.0, direction * turn_speed)
                time.sleep(0.05)
            stop()
            time.sleep(0.1)
            heading = target_heading

        # ── Step 2: drive straight ───────────────────────────────────────────
        if distance > 0.05:
            drive_time = distance / speed
            t_end = time.monotonic() + drive_time
            while time.monotonic() < t_end:
                publish(speed, 0.0)
                time.sleep(0.05)
            stop()
            time.sleep(0.1)

        wp_idx += 1


def main() -> None:
    parser = argparse.ArgumentParser(description="Patrol bot waypoint controller")
    parser.add_argument("--name",       required=True,  help="gz model name (e.g. patrol_bot_0)")
    parser.add_argument("--waypoints",  required=True,  help='JSON list of [x,y] pairs')
    parser.add_argument("--speed",      type=float, default=0.4, help="Linear speed (m/s)")
    parser.add_argument("--turn_speed", type=float, default=0.6, help="Angular speed (rad/s)")
    args = parser.parse_args()

    waypoints = json.loads(args.waypoints)
    if len(waypoints) < 2:
        sys.exit("Need at least 2 waypoints.")

    run(args.name, waypoints, args.speed, args.turn_speed)


if __name__ == "__main__":
    main()
