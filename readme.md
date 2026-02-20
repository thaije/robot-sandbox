# Robot Sandbox

Read `agent-scripts/AGENTS.MD` first.

## What this is

Robot Sandbox — modular Gazebo-based sandbox for testing robot autonomy. Phase 1: Turtlebot 4 in indoor environments, teleoperation, metrics + composite scoring.

Full plan: [`docs/ARST_Project_Plan.md`](docs/ARST_Project_Plan.md)

## Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic (`gz-sim8`)
- Turtlebot 4 simulation packages: `ros-jazzy-turtlebot4-gz-bringup`

## Startup

### Launch the simulation

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch launch/arst_sim.launch.py world:=indoor_office model:=lite
```

This sets the complete `GZ_SIM_RESOURCE_PATH` (upstream robot resources + our `worlds/models/`),
passes the absolute world SDF path directly to Gazebo, then delegates robot spawning and
bridges to `turtlebot4_gz_bringup/turtlebot4_spawn.launch.py`.

> **Why not `ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py`?**
> The upstream `sim.launch.py` unconditionally overwrites `GZ_SIM_RESOURCE_PATH` with its own
> fixed list, so custom world paths get discarded. `arst_sim.launch.py` bypasses this by calling
> `gz_sim.launch.py` directly with the world's absolute path.

### Undock the robot

The robot spawns docked. Undock before sending velocity commands:

```bash
ros2 action send_goal /undock irobot_create_msgs/action/Undock {}
```

### Teleoperation

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=true
```

> `/cmd_vel` expects `geometry_msgs/TwistStamped` — the `stamped:=true` flag is required.

### Useful topic checks

```bash
ros2 topic echo /sim_ground_truth_pose          # ground-truth pose (TFMessage)
ros2 topic echo /scan                           # 2-D LiDAR
ros2 topic echo /oakd/rgb/preview/image_raw     # RGB camera
ros2 topic echo /bumper_contact                 # contact sensor
```

## Project structure

```
launch/                      # ROS 2 launch files
  arst_sim.launch.py         # main simulation launcher (use this)
config/
  robots/turtlebot4.yaml     # verified sensor/control topic names
  scenarios/                 # scenario definitions
  metrics/                   # metric configurations
worlds/
  templates/indoor_office/   # 20×15 m office world + ground-truth map
  models/                    # target object SDF models
    fire_extinguisher/       # class ID 1 — red cylinder
    first_aid_kit/           # class ID 2 — white box + green cross
    hazard_sign/             # class ID 3 — yellow sign on pole
src/
  metrics/                   # ROS 2 metric nodes
  scenario_runner/           # scenario orchestration
  world_manager/             # world generation + object placement
tests/                       # pytest unit tests
```

## Running tests

```bash
pytest tests/
```
