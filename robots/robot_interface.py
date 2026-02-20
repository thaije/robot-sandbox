"""
Abstract robot interface — Step 2.4.

Adding a new robot platform = subclass this + provide model files.
The interface is intentionally lightweight: it's a configuration schema,
not a runtime framework.
"""
from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass
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
        """Unique identifier, e.g. 'turtlebot4'."""

    @property
    @abstractmethod
    def urdf_path(self) -> Path:
        """Path to the robot's URDF / xacro file."""

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
        """Dict of sensor name → SensorConfig."""

    @property
    @abstractmethod
    def detection_topic(self) -> str:
        """Unified detections topic (vision_msgs/Detection2DArray)."""

    @property
    @abstractmethod
    def class_map(self) -> dict[int, str]:
        """Mapping from gz-sim-label-system integer ID → class name."""

    def default_spawn_pose(self) -> SpawnPose:
        return SpawnPose()
