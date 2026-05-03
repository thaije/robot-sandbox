#!/usr/bin/env python3.12
"""Random wander agent — benchmark score-floor baseline.

Sends random cmd_vel commands with basic laser-based obstacle avoidance.
No SLAM, Nav2, or ML. Runs until the scenario times out.

Usage
-----
    # Sim must already be running
    python3.12 scripts/random_agent.py [--robot derpbot_0] [--seed 42] [--duration 300]

The agent polls http://localhost:7400/mission for the time limit. Override
with --duration if the mission server is unavailable.
"""
from __future__ import annotations

import argparse
import json
import math
import random
import sys
import threading
import time
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


# ── Constants ──────────────────────────────────────────────────────────────────

RESAMPLE_INTERVAL = 3.0    # seconds between new random velocity samples
OBSTACLE_DIST     = 0.40   # metres — trigger avoidance below this
AVOID_DURATION    = 2.0    # seconds to back up + spin when obstacle detected
FRONT_HALF_ANGLE  = 0.52   # radians (~30°) each side of forward axis

MAX_LINEAR        = 0.30   # m/s forward
MAX_ANGULAR       = 0.80   # rad/s

MISSION_PORT      = 7400


# ── Helpers ────────────────────────────────────────────────────────────────────

def _fetch_time_limit() -> float | None:
    try:
        url = f"http://localhost:{MISSION_PORT}/mission"
        with urllib.request.urlopen(url, timeout=3) as resp:
            return float(json.loads(resp.read())["time_limit_seconds"])
    except Exception:
        return None


def _obstacle_in_front(msg: LaserScan) -> bool:
    """Return True if any valid range within ±FRONT_HALF_ANGLE of forward is < OBSTACLE_DIST."""
    n   = len(msg.ranges)
    inc = msg.angle_increment
    start = msg.angle_min
    valid_ranges = []
    for i, r in enumerate(msg.ranges):
        angle = start + i * inc
        if abs(angle) < FRONT_HALF_ANGLE and not math.isinf(r) and not math.isnan(r) and r > 0.05:
            valid_ranges.append(r)
    return bool(valid_ranges) and min(valid_ranges) < OBSTACLE_DIST


# ── Agent node ─────────────────────────────────────────────────────────────────

class RandomWanderAgent(Node):
    """Minimal random-wander agent node."""

    def __init__(self, robot: str, seed: int) -> None:
        super().__init__("random_wander_agent")
        self._rng = random.Random(seed)

        self._pub = self.create_publisher(Twist, f"/{robot}/cmd_vel", 10)
        self.create_subscription(LaserScan, f"/{robot}/scan", self._on_scan, 10)

        self._obstacle = False
        self._avoid_until = 0.0
        self._resample_at = 0.0
        self._cmd = Twist()

        self.create_timer(0.1, self._step)

    def _on_scan(self, msg: LaserScan) -> None:
        self._obstacle = _obstacle_in_front(msg)

    def _sample_random(self) -> Twist:
        t = Twist()
        t.linear.x  = self._rng.uniform(0.05, MAX_LINEAR)
        t.angular.z = self._rng.uniform(-MAX_ANGULAR, MAX_ANGULAR)
        return t

    def _avoid_cmd(self) -> Twist:
        t = Twist()
        t.linear.x  = -0.20
        t.angular.z = self._rng.choice([-MAX_ANGULAR, MAX_ANGULAR])
        return t

    def _step(self) -> None:
        now = time.monotonic()

        if self._obstacle and now >= self._avoid_until:
            # Enter avoidance: back up + spin
            self._cmd = self._avoid_cmd()
            self._avoid_until = now + AVOID_DURATION
            self._resample_at = self._avoid_until  # resample after avoidance
        elif now < self._avoid_until:
            pass  # keep avoidance command
        elif now >= self._resample_at:
            self._cmd = self._sample_random()
            self._resample_at = now + RESAMPLE_INTERVAL

        self._pub.publish(self._cmd)


# ── Entry point ────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Random wander agent")
    parser.add_argument("--robot",    default="derpbot_0")
    parser.add_argument("--seed",     type=int, default=0)
    parser.add_argument("--duration", type=float, default=None,
                        help="Run duration in seconds. Defaults to mission time limit.")
    args = parser.parse_args()

    duration = args.duration or _fetch_time_limit()
    if duration is None:
        print("[random_agent] WARNING: could not fetch time limit; defaulting to 300 s")
        duration = 300.0

    print(f"[random_agent] robot={args.robot}  seed={args.seed}  duration={duration:.0f}s")

    rclpy.init()
    agent = RandomWanderAgent(robot=args.robot, seed=args.seed)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(agent)

    deadline = time.monotonic() + duration
    try:
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        agent._pub.publish(stop)
        agent.destroy_node()
        rclpy.shutdown()

    print("[random_agent] done")


if __name__ == "__main__":
    main()
