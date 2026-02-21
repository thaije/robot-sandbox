#!/usr/bin/env python3
"""
Spawn a robot into a running ARST simulation and start its ROS 2 bridges.

This launch file is designed to run AFTER arst_sim.launch.py is already
running.  It:
  1. Spawns the robot model into Gazebo via the /world/<world>/create service.
  2. Starts robot_state_publisher with the robot's URDF.
  3. Starts ros_gz_bridge nodes for cmd_vel, odom, tf, and joint_states.

To DESPAWN a robot, use scripts/despawn_robot.sh or:
    gz service -s /world/<world>/remove \\
      --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean \\
      --timeout 5000 --req 'name: "<robot_name>" type: MODEL'
Then Ctrl-C this launch to stop its bridges.

Usage examples:
    # Spawn one DerpBot at default position:
    ros2 launch launch/spawn_robot.launch.py

    # Spawn at a specific pose:
    ros2 launch launch/spawn_robot.launch.py name:=derpbot_0 x:=3.0 y:=5.0 yaw:=1.57

    # Spawn a second instance:
    ros2 launch launch/spawn_robot.launch.py name:=derpbot_1 x:=8.0 y:=2.0

    # Spawn into a different world:
    ros2 launch launch/spawn_robot.launch.py world:=indoor_warehouse
"""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

REPO_ROOT = Path(__file__).resolve().parent.parent

ARGUMENTS = [
    DeclareLaunchArgument(
        "robot",
        default_value="derpbot",
        description="Robot type — must match a sub-directory of robots/",
    ),
    DeclareLaunchArgument(
        "name",
        default_value="derpbot_0",
        description="Unique Gazebo model name for this robot instance",
    ),
    DeclareLaunchArgument(
        "world",
        default_value="indoor_office",
        description="Gazebo world name (must match the running simulation)",
    ),
    DeclareLaunchArgument(
        "x", default_value="1.0", description="Spawn X position (metres)",
    ),
    DeclareLaunchArgument(
        "y", default_value="1.0", description="Spawn Y position (metres)",
    ),
    DeclareLaunchArgument(
        "z", default_value="0.0",
        description="Spawn Z position (base_footprint frame; 0.0 = wheels on ground)",
    ),
    DeclareLaunchArgument(
        "yaw", default_value="0.0", description="Spawn yaw rotation (radians)",
    ),
    DeclareLaunchArgument(
        "use_sim_time", default_value="true", choices=["true", "false"],
    ),
]


def _make_actions(context, *args, **kwargs):
    robot_type  = LaunchConfiguration("robot").perform(context)
    robot_name  = LaunchConfiguration("name").perform(context)
    world_name  = LaunchConfiguration("world").perform(context)
    x    = LaunchConfiguration("x").perform(context)
    y    = LaunchConfiguration("y").perform(context)
    z    = LaunchConfiguration("z").perform(context)
    yaw  = LaunchConfiguration("yaw").perform(context)
    use_sim_time_str = LaunchConfiguration("use_sim_time").perform(context)
    use_sim_time = use_sim_time_str.lower() == "true"

    # ── Load URDF — substitute ROBOT_NAME placeholder ────────────────────────
    # The URDF uses the literal string ROBOT_NAME in Gazebo topic names so that
    # topics are absolute and unique per instance (e.g. /derpbot_0/cmd_vel).
    urdf_path = REPO_ROOT / "robots" / robot_type / "urdf" / f"{robot_type}.urdf"
    if not urdf_path.exists():
        raise FileNotFoundError(
            f"URDF not found: {urdf_path}\n"
            f"Expected layout: robots/{robot_type}/urdf/{robot_type}.urdf"
        )
    urdf_content = urdf_path.read_text().replace("ROBOT_NAME", robot_name)

    # ── 1. Spawn robot into Gazebo ────────────────────────────────────────────
    # ros_gz_sim create reads the URDF and calls the Gazebo /world/<world>/create
    # service.  The URDF's <gazebo> elements are converted to SDF plugins.
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        name=f"spawn_{robot_name}",
        arguments=[
            "-world", world_name,
            "-name",  robot_name,
            "-string", urdf_content,
            "-x", x,
            "-y", y,
            "-z", z,
            "-Y", yaw,     # capital Y = yaw
        ],
        output="screen",
    )

    # ── 2. Robot state publisher ──────────────────────────────────────────────
    # Subscribes to /<robot_name>/joint_states, publishes TF for all joints.
    # (Diff drive plugin publishes odom→base_footprint; RSP publishes the rest.)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=robot_name,
        parameters=[{
            "robot_description": urdf_content,
            "use_sim_time": use_sim_time,
        }],
        remappings=[
            # RSP subscribes to joint_states within its namespace by default.
            # Publish TF to the global /tf tree.
            ("tf", "/tf"),
            ("tf_static", "/tf_static"),
        ],
        output="screen",
    )

    # ── 3. ROS ↔ Gazebo bridges ───────────────────────────────────────────────
    # The URDF plugin topics already use absolute names matching the robot_name
    # (e.g. /derpbot_0/cmd_vel), so the Gazebo topic == the ROS 2 topic ==
    # the same string.  No --remap tricks needed.
    #
    # Bridge argument format: gz_topic@ros_type]gz_type  (ROS→GZ)
    #                         gz_topic@ros_type[gz_type  (GZ→ROS)
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name=f"bridge_{robot_name}",
        arguments=[
            # cmd_vel: ROS 2 → Gazebo
            f"/{robot_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            # odom: Gazebo → ROS 2
            f"/{robot_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            # TF from diff drive: Gazebo → ROS 2
            f"/{robot_name}/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V",
            # Joint states: Gazebo → ROS 2
            f"/{robot_name}/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[
            # Merge robot's TF stream into the global /tf topic
            (f"/{robot_name}/tf", "/tf"),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return [spawn, rsp, bridge]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=_make_actions))
    return ld
