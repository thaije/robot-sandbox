"""
Abstract robot interface.

Adding a new robot platform = subclass this + provide model files.
The interface is intentionally lightweight: it's a configuration schema,
not a runtime framework.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class SensorConfig:
    topic: str
    msg_type: str
    frame_id: str


@dataclass
class SpawnPose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0


class RobotInterface(ABC):
    """Describes what a robot provides to the simulation framework."""

    @property
    @abstractmethod
    def platform_name(self) -> str:
        """Unique identifier, e.g. 'derpbot'."""

    @property
    @abstractmethod
    def urdf_path(self) -> Path:
        """Path to the robot's URDF file."""

    @property
    @abstractmethod
    def cmd_vel_topic(self) -> str:
        """Topic name for geometry_msgs/Twist velocity commands."""

    @property
    @abstractmethod
    def odom_topic(self) -> str:
        """Topic name for nav_msgs/Odometry."""

    @property
    @abstractmethod
    def ground_truth_pose_topic(self) -> str:
        """Gazebo ground-truth pose topic (via ros_gz_bridge)."""

    @property
    @abstractmethod
    def sensors(self) -> dict[str, SensorConfig]:
        """Dict of sensor name → SensorConfig. Empty dict = no sensors."""

    def default_spawn_pose(self) -> SpawnPose:
        return SpawnPose()
