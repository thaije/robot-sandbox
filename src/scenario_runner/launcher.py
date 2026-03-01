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

import os
import signal
import subprocess
import threading
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
        robots_cfg: list[dict],
        dynamic_obstacles: list[dict] | None = None,
        gazebo_timeout: float = 90.0,
        robot_timeout: float = 30.0,
    ) -> None:
        """Launch Gazebo and robot bridge nodes. Blocks until all are ready.

        Robots are expected to be embedded in *world_sdf* at generation time
        (via WorldGenerator._embed_robots).  This launcher only starts RSP and
        ros_gz_bridge for each robot — it does NOT spawn models into Gazebo.

        Parameters
        ----------
        world_sdf:
            Absolute path to the generated world SDF (from WorldGenerator).
        world_name:
            Gazebo world name matching ``<world name="...">`` in the SDF.
        robots_cfg:
            List of robot dicts from ``scenario_config["robots"]``.
            Each dict must have ``platform`` and optionally ``name``.
        gazebo_timeout:
            Seconds to wait for ``/clock`` before raising TimeoutError.
        robot_timeout:
            Seconds to wait for each ``/<robot_name>/odom`` before raising.
        """
        self._launch_gazebo(world_sdf)
        self._wait_for_gazebo(timeout=gazebo_timeout)

        robot_names = []
        for robot in robots_cfg:
            platform = robot["platform"]
            name = robot.get("name", f"{platform}_0")
            robot_names.append(name)
            self._launch_robot_bridges(platform, name, world_name)

        self._wait_for_robots(robot_names, timeout=robot_timeout)

        for obs in (dynamic_obstacles or []):
            self._launch_patrol_controller(obs, world_name=world_name)

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
            start_new_session=True,  # own process group → gz sim killed with parent
        )
        self._processes.append(proc)
        return proc

    def _launch_robot_bridges(
        self,
        platform: str,
        robot_name: str,
        world_name: str,
    ) -> subprocess.Popen:
        """Start RSP + ros_gz_bridge for a robot that is already in the world SDF.

        Passes ``spawn:=false`` so spawn_robot.launch.py skips the
        ros_gz_sim create node — the model is already embedded in the world.
        """
        cmd = [
            "ros2", "launch",
            str(_LAUNCH_DIR / "spawn_robot.launch.py"),
            f"robot:={platform}",
            f"name:={robot_name}",
            f"world:={world_name}",
            "spawn:=false",
        ]
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            start_new_session=True,  # own process group → bridges killed with parent
        )
        self._processes.append(proc)
        return proc

    def _launch_patrol_controller(
        self, obs: dict, world_name: str = "indoor_office"
    ) -> threading.Thread | None:
        """Start a patrol-bot controller as a daemon thread.

        Runs ``patrol_bot_controller.run()`` in-process (no subprocess) so
        gz-transport discovery happens in the same network context as Gazebo.
        The thread is a daemon so it is automatically reaped when the process
        exits; ``shutdown()`` doesn't need to kill it explicitly.
        """
        name = obs.get("name", obs.get("model", "patrol_bot") + "_0")
        waypoints = obs.get("patrol", [])
        if not waypoints:
            return None  # no patrol configured — model is static

        speed = float(obs.get("speed", 0.4))

        # Import here so the module's path-setup runs in the correct context.
        from scenario_runner.patrol_bot_controller import run as _patrol_run  # noqa: PLC0415

        t = threading.Thread(
            target=_patrol_run,
            args=(name, waypoints, speed, world_name),
            daemon=True,
            name=f"patrol_{name}",
        )
        t.start()
        return t

    # ── Private: readiness checks ──────────────────────────────────────────────

    def _wait_for_gazebo(self, timeout: float = 90.0) -> None:
        """/clock appearing means Gazebo + clock bridge are both up."""
        if not wait_for_topic("/clock", timeout=timeout):
            raise TimeoutError(
                f"Gazebo /clock not available after {timeout}s. "
                "Check that ros_gz_bridge and Gazebo started successfully."
            )

    def _wait_for_robots(self, robot_names: list[str], timeout: float = 30.0) -> None:
        """Wait for odom topic of every robot in *robot_names*."""
        for name in robot_names:
            topic = f"/{name}/odom"
            if not wait_for_topic(topic, timeout=timeout):
                raise TimeoutError(
                    f"Robot topic {topic!r} not available after {timeout}s. "
                    "Check that spawn_robot.launch.py bridges started."
                )

    # ── Shutdown ───────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        """Kill all child process groups (Gazebo + bridge processes).

        Robots are embedded in the world SDF and are destroyed with Gazebo —
        no explicit despawn step is needed.
        """
        # Each process was launched with start_new_session=True, so it has its
        # own process group (pgid == proc.pid).  Killing the group ensures gz sim
        # and ros_gz_bridge grandchildren are also terminated.
        #
        # pgids are captured BEFORE sending any signal: after SIGTERM the parent
        # ros2-launch process exits quickly, making getpgid() raise ProcessLookupError
        # on the SIGKILL pass and leaving gz sim alive.
        pgids = []
        for proc in reversed(self._processes):
            try:
                pgids.append(os.getpgid(proc.pid))
            except ProcessLookupError:
                pgids.append(None)

        for pgid in pgids:
            if pgid is not None:
                try:
                    os.killpg(pgid, signal.SIGTERM)
                except ProcessLookupError:
                    pass

        time.sleep(2.0)  # give gz sim time to shut down cleanly

        for pgid in pgids:
            if pgid is not None:
                try:
                    os.killpg(pgid, signal.SIGKILL)
                except ProcessLookupError:
                    pass  # already gone — SIGTERM was enough

        for proc in self._processes:
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                pass
        self._processes.clear()
