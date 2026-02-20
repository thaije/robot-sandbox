"""Simulation launcher — Step 4.2."""
from __future__ import annotations

import subprocess
import time
from pathlib import Path

from utils.ros_helpers import wait_for_topic


class SimulationLauncher:
    """
    Programmatically launches Gazebo + ROS 2 nodes for a scenario.
    Handles startup sequencing and health checks.
    """

    def __init__(self, headless: bool = True) -> None:
        self._headless = headless
        self._processes: list[subprocess.Popen] = []

    def launch(self, world_sdf: Path, robot_platform: str, spawn_pose: dict) -> None:
        """Launch Gazebo and robot nodes. Blocks until ready."""
        raise NotImplementedError  # TODO: Step 4.2

    def _launch_gazebo(self, world_sdf: Path) -> subprocess.Popen:
        raise NotImplementedError

    def _launch_robot(self, platform: str, spawn_pose: dict) -> subprocess.Popen:
        raise NotImplementedError

    def _wait_until_ready(self, timeout: float = 60.0) -> None:
        """Wait for key topics to appear."""
        required = ["/scan", "/odom", "/cmd_vel"]
        for topic in required:
            if not wait_for_topic(topic, timeout=timeout):
                raise TimeoutError(f"Topic {topic!r} not available after {timeout}s")

    def shutdown(self) -> None:
        for proc in reversed(self._processes):
            proc.terminate()
        for proc in self._processes:
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        self._processes.clear()
