"""Patrol bot controller — drive-and-teleport approach.

Drives the patrol_bot from waypoint[0] toward waypoint[1] for a fixed time,
then teleports it back to waypoint[0] facing the original direction.
Repeats indefinitely.

This avoids the dead-reckoning accumulation error that caused the bot to drift
out of the corridor when using turn-based navigation.

SimulationLauncher calls ``run()`` in a daemon thread.
Can also be run standalone::

    python3 patrol_bot_controller.py \\
        --name patrol_bot_0 \\
        --waypoints "[[2,8],[18,8]]" \\
        --speed 0.5 \\
        --world indoor_office

Algorithm:
    1. Compute heading and drive time from waypoint[0] → waypoint[1].
    2. Drive forward at *speed* for *drive_time* seconds.
    3. Teleport back to waypoint[0] with the same heading.
    4. Wait briefly, then repeat.
"""
from __future__ import annotations

import argparse
import json
import math
import sys
import time


def run(
    name: str,
    waypoints: list[list[float]],
    speed: float,
    turn_speed: float,  # kept for API compatibility; not used
    world_name: str = "indoor_office",
) -> None:
    _SYS_PKG = "/usr/lib/python3/dist-packages"
    if _SYS_PKG not in sys.path:
        sys.path.insert(0, _SYS_PKG)

    from gz.transport13 import Node  # type: ignore[import]
    from gz.msgs10.twist_pb2 import Twist  # type: ignore[import]
    from gz.msgs10.pose_pb2 import Pose  # type: ignore[import]
    from gz.msgs10.boolean_pb2 import Boolean  # type: ignore[import]

    node = Node()
    cmd_topic = f"/model/{name}/cmd_vel"
    pub = node.advertise(cmd_topic, Twist)
    set_pose_svc = f"/world/{world_name}/set_pose"

    wp0 = waypoints[0]
    wp1 = waypoints[1]
    dx = wp1[0] - wp0[0]
    dy = wp1[1] - wp0[1]
    distance = math.hypot(dx, dy)
    drive_time = distance / speed

    # Quaternion for yaw = atan2(dy, dx), pitch=0, roll=0
    yaw = math.atan2(dy, dx)
    qz = math.sin(yaw / 2.0)
    qw = math.cos(yaw / 2.0)

    def _send_vel(linear_x: float) -> None:
        msg = Twist()
        msg.linear.x = linear_x
        pub.publish(msg)

    def _teleport() -> None:
        req = Pose()
        req.name = name
        req.position.x = float(wp0[0])
        req.position.y = float(wp0[1])
        req.position.z = 0.14
        req.orientation.w = float(qw)
        req.orientation.x = 0.0
        req.orientation.y = 0.0
        req.orientation.z = float(qz)
        try:
            node.request(set_pose_svc, req, Pose, Boolean, 1000)
        except Exception:
            pass  # fire-and-forget; Gazebo logs any error

    # Allow Gazebo to discover the cmd_vel topic before publishing.
    time.sleep(2.0)

    while True:
        # Drive toward wp1 for the calculated time
        t_end = time.monotonic() + drive_time
        while time.monotonic() < t_end:
            _send_vel(speed)
            time.sleep(0.05)

        # Stop, then snap back to wp0
        _send_vel(0.0)
        time.sleep(0.1)
        _teleport()
        time.sleep(0.5)


def main() -> None:
    parser = argparse.ArgumentParser(description="Patrol bot drive+teleport controller")
    parser.add_argument("--name",       required=True, help="gz model name")
    parser.add_argument("--waypoints",  required=True, help='JSON list of [x,y] pairs')
    parser.add_argument("--speed",      type=float, default=0.5)
    parser.add_argument("--world",      default="indoor_office", help="Gazebo world name")
    args = parser.parse_args()

    waypoints = json.loads(args.waypoints)
    if len(waypoints) < 2:
        sys.exit("Need at least 2 waypoints.")

    run(args.name, waypoints, args.speed, turn_speed=0.0, world_name=args.world)


if __name__ == "__main__":
    main()
