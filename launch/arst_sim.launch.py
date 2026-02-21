#!/usr/bin/env python3
"""
ARST simulation launcher — starts Gazebo with a world.  No robot is spawned.

Run this once to get a persistent simulation.  Then use spawn_robot.launch.py
to add/remove robots without restarting Gazebo.

Usage:
    # GUI mode (default):
    ros2 launch launch/arst_sim.launch.py

    # Headless (no display required, for servers / CI):
    ros2 launch launch/arst_sim.launch.py headless:=true

    # Different world:
    ros2 launch launch/arst_sim.launch.py world:=indoor_warehouse

    # Headless + specific world:
    ros2 launch launch/arst_sim.launch.py world:=indoor_office headless:=true
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ── Repo paths ────────────────────────────────────────────────────────────────
REPO_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = str(REPO_ROOT / "worlds" / "models")

ARGUMENTS = [
    DeclareLaunchArgument(
        "world",
        default_value="indoor_office",
        description="World template name — must match a sub-directory of worlds/templates/",
    ),
    DeclareLaunchArgument(
        "headless",
        default_value="false",
        choices=["true", "false"],
        description="Run Gazebo server-only (no GUI).  Uses EGL offscreen rendering.",
    ),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use simulation clock",
    ),
    DeclareLaunchArgument(
        "verbose",
        default_value="3",
        description="Gazebo verbosity level (0-4)",
    ),
]


def _make_actions(context, *args, **kwargs):
    """OpaqueFunction: resolve LaunchConfiguration values and build actions."""
    world_name = LaunchConfiguration("world").perform(context)
    headless = LaunchConfiguration("headless").perform(context).lower() == "true"
    verbose = LaunchConfiguration("verbose").perform(context)

    # ── World SDF ─────────────────────────────────────────────────────────────
    world_sdf = REPO_ROOT / "worlds" / "templates" / world_name / "world.sdf"
    if not world_sdf.exists():
        available = [d.name for d in (REPO_ROOT / "worlds" / "templates").iterdir() if d.is_dir()]
        raise FileNotFoundError(
            f"World SDF not found: {world_sdf}\nAvailable worlds: {available}"
        )

    # ── GZ_SIM_RESOURCE_PATH — add our model library ──────────────────────────
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join(filter(None, [
            MODELS_DIR,
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),
        ])),
    )

    # ── GZ_RENDERING_PLUGIN_PATH — pin to Ogre2 (EGL-capable) ─────────────────
    # Without this, gz-sim-sensors-system may load the system Ogre 1.9 package
    # (which requires X11) instead of the vendored Ogre2 (which supports EGL).
    # This causes a crash in headless mode and is a latent risk in GUI mode.
    gz_vendor_prefix = get_package_prefix("gz_rendering_vendor")
    gz_rendering_plugins_dir = os.path.join(
        gz_vendor_prefix, "opt", "gz_rendering_vendor",
        "lib", "gz-rendering-8", "engine-plugins",
    )
    gz_rendering_plugin_path = SetEnvironmentVariable(
        name="GZ_RENDERING_PLUGIN_PATH",
        value=gz_rendering_plugins_dir,
    )

    # ── Gazebo ────────────────────────────────────────────────────────────────
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    gz_sim_launch = os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")

    if headless:
        # -s  = server only (no GUI client)
        # -r  = run immediately (don't wait for play button)
        # --headless-rendering = use EGL offscreen; required for sensors without display
        gz_args = f"-s -r --headless-rendering -v {verbose} {world_sdf}"
    else:
        gz_args = f"-r -v {verbose} {world_sdf}"

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[("gz_args", gz_args)],
    )

    # ── Clock bridge — synchronise ROS 2 sim time with Gazebo ────────────────
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        parameters=[{"use_sim_time": True}],
    )

    return [gz_resource_path, gz_rendering_plugin_path, gazebo, clock_bridge]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=_make_actions))
    return ld
