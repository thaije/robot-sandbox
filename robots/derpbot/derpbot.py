"""
DerpBot — concrete RobotInterface implementation.

A minimal two-wheeled differential drive robot with no sensors.
Sensors (LiDAR, camera) will be added in later phases.
"""
from __future__ import annotations

from pathlib import Path

from robots.robot_interface import RobotInterface, SensorConfig, SpawnPose

# Repo root: robots/derpbot/derpbot.py → ../../
_REPO_ROOT = Path(__file__).resolve().parents[2]


class DerpBot(RobotInterface):
    """DerpBot: simple 2-wheel differential drive, no sensors."""

    @property
    def platform_name(self) -> str:
        return "derpbot"

    @property
    def urdf_path(self) -> Path:
        return _REPO_ROOT / "robots" / "derpbot" / "urdf" / "derpbot.urdf"

    @property
    def cmd_vel_topic(self) -> str:
        return "/derpbot_0/cmd_vel"

    @property
    def odom_topic(self) -> str:
        return "/derpbot_0/odom"

    @property
    def ground_truth_pose_topic(self) -> str:
        return "/model/derpbot_0/pose"

    @property
    def sensors(self) -> dict[str, SensorConfig]:
        # No sensors in Phase 1 baseline; populated when sensors are added.
        return {}

    def default_spawn_pose(self) -> SpawnPose:
        return SpawnPose(x=1.0, y=1.0, z=0.0, yaw=0.0)
