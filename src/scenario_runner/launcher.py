"""Simulation launcher — Step 4.2.

Programmatically launches Gazebo + ROS 2 nodes for a scenario run and handles
startup sequencing, health checks, and clean shutdown.

Design notes
------------
- Gazebo is launched via ``ros2 launch launch/arst_sim.launch.py`` so the
  existing launch file handles GZ_SIM_RESOURCE_PATH, GZ_RENDERING_PLUGIN_PATH,
  the Ogre2 workaround, and the /clock bridge automatically.
- The robot is launched via ``ros2 launch launch/spawn_robot.launch.py`` after
  Gazebo reports ready (i.e., /clock is publishing).
- All child processes are tracked in ``self._processes`` for clean shutdown.
"""
from __future__ import annotations

import subprocess
import time
from pathlib import Path

from utils.ros_helpers import wait_for_topic

# Three levels up: src/scenario_runner/ → src/ → repo root
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent
_LAUNCH_DIR = _REPO_ROOT / "launch"


class SimulationLauncher:
    """
    Programmatically launches Gazebo + ROS 2 nodes for a scenario.
    Handles startup sequencing and health checks.
    """

    def __init__(self, headless: bool = True) -> None:
        self._headless = headless
        self._processes: list[subprocess.Popen] = []

    def launch(
        self,
        world_sdf: Path,
        world_name: str,
        robot_platform: str,
        spawn_pose: dict,
        robot_name: str = "derpbot_0",
        gazebo_timeout: float = 90.0,
        robot_timeout: float = 30.0,
    ) -> None:
        """Launch Gazebo and robot nodes. Blocks until both are ready.

        Parameters
        ----------
        world_sdf:
            Absolute path to the generated world SDF (from WorldGenerator).
        world_name:
            Gazebo world name matching ``<world name="...">`` in the SDF.
            Used by spawn_robot.launch.py to target the correct world service.
        robot_platform:
            Robot type directory name under ``robots/`` (e.g. ``"derpbot"``).
        spawn_pose:
            Dict with optional keys ``x``, ``y``, ``z``, ``yaw`` (metres/rad).
        robot_name:
            Unique Gazebo model name for this instance.
        gazebo_timeout:
            Seconds to wait for ``/clock`` before raising TimeoutError.
        robot_timeout:
            Seconds to wait for ``/<robot_name>/odom`` before raising.
        """
        self._launch_gazebo(world_sdf)
        self._wait_for_gazebo(timeout=gazebo_timeout)

        self._launch_robot(robot_platform, robot_name, world_name, spawn_pose)
        self._wait_for_robot(robot_name, timeout=robot_timeout)

    # ── Private: launch ────────────────────────────────────────────────────────

    def _launch_gazebo(self, world_sdf: Path) -> subprocess.Popen:
        """Start Gazebo via the arst_sim launch file."""
        cmd = [
            "ros2", "launch",
            str(_LAUNCH_DIR / "arst_sim.launch.py"),
            f"world_sdf:={world_sdf}",
            f"headless:={'true' if self._headless else 'false'}",
        ]
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        self._processes.append(proc)
        return proc

    def _launch_robot(
        self,
        platform: str,
        robot_name: str,
        world_name: str,
        spawn_pose: dict,
    ) -> subprocess.Popen:
        """Spawn the robot and start its ROS 2 bridges."""
        cmd = [
            "ros2", "launch",
            str(_LAUNCH_DIR / "spawn_robot.launch.py"),
            f"robot:={platform}",
            f"name:={robot_name}",
            f"world:={world_name}",
            f"x:={spawn_pose.get('x', 1.0)}",
            f"y:={spawn_pose.get('y', 1.0)}",
            f"z:={spawn_pose.get('z', 0.0)}",
            f"yaw:={spawn_pose.get('yaw', 0.0)}",
        ]
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        self._processes.append(proc)
        return proc

    # ── Private: readiness checks ──────────────────────────────────────────────

    def _wait_for_gazebo(self, timeout: float = 90.0) -> None:
        """/clock appearing means Gazebo + clock bridge are both up."""
        if not wait_for_topic("/clock", timeout=timeout):
            raise TimeoutError(
                f"Gazebo /clock not available after {timeout}s. "
                "Check that ros_gz_bridge and Gazebo started successfully."
            )

    def _wait_for_robot(self, robot_name: str, timeout: float = 30.0) -> None:
        """Odom topic appearing means the robot is spawned and bridge is live."""
        topic = f"/{robot_name}/odom"
        if not wait_for_topic(topic, timeout=timeout):
            raise TimeoutError(
                f"Robot topic {topic!r} not available after {timeout}s. "
                "Check that spawn_robot.launch.py completed."
            )

    def _wait_until_ready(self, timeout: float = 60.0) -> None:
        """Wait for key topics to appear. (Legacy — use targeted checks above.)"""
        required = ["/clock", "/odom", "/cmd_vel"]
        for topic in required:
            if not wait_for_topic(topic, timeout=timeout):
                raise TimeoutError(f"Topic {topic!r} not available after {timeout}s")

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        """Terminate all child processes in reverse launch order."""
        for proc in reversed(self._processes):
            proc.terminate()
        for proc in self._processes:
            try:
                proc.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                proc.kill()
        self._processes.clear()
