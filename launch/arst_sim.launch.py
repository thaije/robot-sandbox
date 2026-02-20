#!/usr/bin/env python3
"""
ARST simulation launcher.

Replaces turtlebot4_gz_bringup/sim.launch.py for the Gazebo component so that
GZ_SIM_RESOURCE_PATH can include our custom world templates and model library.
The robot-spawn / bridge / turtlebot4_node components are still delegated to
turtlebot4_gz_bringup/turtlebot4_spawn.launch.py.

Root cause of the plain turtlebot4_gz_bringup delegation not working:
  sim.launch.py unconditionally overwrites GZ_SIM_RESOURCE_PATH with its own
  fixed list, discarding any paths we prepend.  We replicate its logic here
  while adding our extra paths, then call gz_sim.launch.py directly.

Usage:
    ros2 launch launch/arst_sim.launch.py
    ros2 launch launch/arst_sim.launch.py world:=indoor_office model:=lite
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# ── Repo paths ────────────────────────────────────────────────────────────────
REPO_ROOT = Path(__file__).resolve().parent.parent
MODELS_DIR = str(REPO_ROOT / "worlds" / "models")

ARGUMENTS = [
    DeclareLaunchArgument(
        "world",
        default_value="indoor_office",
        description="World name — must match a sub-directory of worlds/templates/",
    ),
    DeclareLaunchArgument(
        "model",
        default_value="lite",
        choices=["standard", "lite"],
        description="Turtlebot4 model variant",
    ),
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use simulation clock",
    ),
]


def _make_actions(context, *args, **kwargs):
    """OpaqueFunction: resolve LaunchConfiguration values and build actions."""
    world_name = LaunchConfiguration("world").perform(context)
    model_name = LaunchConfiguration("model").perform(context)

    # ── Package paths (mirrors what sim.launch.py computes) ──────────────────
    pkg_tb4_gz_bringup = get_package_share_directory("turtlebot4_gz_bringup")
    pkg_tb4_gz_gui = get_package_share_directory("turtlebot4_gz_gui_plugins")
    pkg_tb4_desc = get_package_share_directory("turtlebot4_description")
    pkg_create_gz_bringup = get_package_share_directory("irobot_create_gz_bringup")
    pkg_create_gz_plugins = get_package_share_directory("irobot_create_gz_plugins")
    pkg_create_desc = get_package_share_directory("irobot_create_description")
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")

    # ── World SDF: absolute path passed directly to gz sim ───────────────────
    # sim.launch.py would search by name in GZ_SIM_RESOURCE_PATH.
    # We pass the full path instead, avoiding resource-path ordering issues.
    world_sdf = str(REPO_ROOT / "worlds" / "templates" / world_name / "world.sdf")
    if not Path(world_sdf).exists():
        raise FileNotFoundError(
            f"World SDF not found: {world_sdf}\n"
            f"Available worlds: {[d.name for d in (REPO_ROOT / 'worlds' / 'templates').iterdir() if d.is_dir()]}"
        )

    # ── GZ_SIM_RESOURCE_PATH ─────────────────────────────────────────────────
    # Combine upstream paths (robot models, textures) + our model library.
    gz_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join(filter(None, [
            os.path.join(pkg_tb4_gz_bringup, "worlds"),
            os.path.join(pkg_create_gz_bringup, "worlds"),
            str(Path(pkg_tb4_desc).parent),
            str(Path(pkg_create_desc).parent),
            MODELS_DIR,                                   # our target objects
            os.environ.get("GZ_SIM_RESOURCE_PATH", ""),  # preserve any user-set paths
        ])),
    )

    # ── GZ_GUI_PLUGIN_PATH (same as sim.launch.py) ───────────────────────────
    gz_gui_plugin_path = SetEnvironmentVariable(
        name="GZ_GUI_PLUGIN_PATH",
        value=":".join([
            os.path.join(pkg_tb4_gz_gui, "lib"),
            os.path.join(pkg_create_gz_plugins, "lib"),
        ]),
    )

    # ── Gazebo (gz_sim.launch.py) ─────────────────────────────────────────────
    gz_sim_launch = os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")
    gui_config = os.path.join(pkg_tb4_gz_bringup, "gui", model_name, "gui.config")

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        launch_arguments=[
            ("gz_args", f"{world_sdf} -r -v 4 --gui-config {gui_config}"),
        ],
    )

    # ── Clock bridge ─────────────────────────────────────────────────────────
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
    )

    # ── Robot spawn + bridges + turtlebot4_node ───────────────────────────────
    spawn_launch = os.path.join(pkg_tb4_gz_bringup, "launch", "turtlebot4_spawn.launch.py")
    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(spawn_launch),
        launch_arguments=[
            ("model", model_name),
            ("x", "1.0"),
            ("y", "1.0"),
            ("z", "0.0"),
            ("yaw", "0.0"),
        ],
    )

    return [
        gz_resource_path,
        gz_gui_plugin_path,
        gazebo,
        clock_bridge,
        robot_spawn,
    ]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=_make_actions))
    return ld
