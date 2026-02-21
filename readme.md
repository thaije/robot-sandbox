# Robot Sandbox

Read `agent-scripts/AGENTS.MD` first.

## What this is

Autonomous Robotics Simulation Testbed (ARST) — modular Gazebo-based sandbox for testing robot autonomy.
Phase 1: DerpBot (simple two-wheeled diff-drive) in indoor environments, teleoperation, metrics + composite scoring.

Full plan: [`docs/ARST_Project_Plan.md`](docs/ARST_Project_Plan.md)

## Prerequisites

- Ubuntu 24.04 LTS
- ROS 2 Jazzy (`source /opt/ros/jazzy/setup.bash`)
- Gazebo Harmonic (`ros-jazzy-ros-gz*` packages)
- `robot_state_publisher`, `ros_gz_bridge`, `ros_gz_sim`, `teleop_twist_keyboard`

---

## Running the simulation

The simulator and robots are decoupled: **Gazebo runs persistently**, and robots are spawned/despawned on demand without restarting it.

### Step 1 — Start Gazebo (once, leave it running)

**GUI mode (default):**
```bash
ros2 launch launch/arst_sim.launch.py
```

**Headless mode** (no display required — for servers, SSH sessions, CI):
```bash
ros2 launch launch/arst_sim.launch.py headless:=true
```

**Options:**
```
world:=indoor_office       world template (default: indoor_office)
world:=indoor_warehouse    alternative world
headless:=true/false       GUI vs server-only (default: false)
verbose:=0..4              Gazebo log verbosity (default: 3)
```

---

### Step 2 — Spawn a robot

```bash
ros2 launch launch/spawn_robot.launch.py
```

**Options:**
```
name:=derpbot_0        unique Gazebo model name (default: derpbot_0)
x:=1.0 y:=1.0         spawn position in metres (default: 1.0, 1.0)
z:=0.0                 spawn height — 0.0 puts wheels on the floor
yaw:=0.0               heading in radians
world:=indoor_office   must match the running simulation (default: indoor_office)
robot:=derpbot         robot type / URDF directory (default: derpbot)
```

**Spawn a second instance** (separate terminal):
```bash
ros2 launch launch/spawn_robot.launch.py name:=derpbot_1 x:=5.0 y:=3.0
```

This launch also starts:
- `robot_state_publisher` — publishes TF tree from URDF + joint states
- `bridge_<name>` — `ros_gz_bridge` for cmd_vel, odom, TF, and joint states

---

### Step 3 — Teleoperate

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel
```

> The terminal running teleop must have keyboard focus.
> Default keys: `i`/`,` = forward/back, `j`/`l` = rotate, `k` = stop.

---

### Despawn a robot

```bash
scripts/despawn_robot.sh                          # removes derpbot_0 from indoor_office
scripts/despawn_robot.sh derpbot_1                # remove by name
scripts/despawn_robot.sh derpbot_0 indoor_warehouse  # name + world
```

Then **Ctrl-C** the `spawn_robot.launch.py` terminal to stop its bridges.

---

## Verifying the setup

### Check topics are live

```bash
ros2 topic list | grep derpbot        # should show cmd_vel, odom, joint_states, tf
```

### Check odometry

```bash
ros2 topic echo /derpbot_0/odom       # nav_msgs/Odometry — updates as robot moves
```

### Check cmd_vel reaches the robot

In one terminal send a constant forward command:
```bash
ros2 topic pub /derpbot_0/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}" --rate 10
```
The robot should drive forward in Gazebo. `Ctrl-C` to stop.

### Check TF tree

```bash
ros2 run tf2_tools view_frames        # writes frames.pdf — open it
```
Expected tree: `odom → base_footprint → base_link → {left_wheel, right_wheel, caster_wheel}`

### Check joint states (wheel angles)

```bash
ros2 topic echo /derpbot_0/joint_states
```

### Inspect Gazebo topics directly

```bash
gz topic -l                                        # list all Gazebo topics
gz topic -e -t /derpbot_0/cmd_vel                  # echo Gazebo-side cmd_vel
gz topic -e -t /derpbot_0/odom                     # echo Gazebo-side odom
gz model --list                                    # list spawned models
gz model -m derpbot_0 -p                           # print derpbot_0 pose
```

---

## Project structure

```
launch/
  arst_sim.launch.py         # start Gazebo (world only, no robot)
  spawn_robot.launch.py      # spawn a robot + its ROS bridges
scripts/
  despawn_robot.sh           # remove a robot from the running sim
config/
  robots/derpbot.yaml        # DerpBot topic names, speeds, spawn defaults
  scenarios/                 # scenario YAML definitions
  metrics/                   # metric configurations
robots/
  derpbot/
    urdf/derpbot.urdf        # two-wheel diff-drive URDF (ROBOT_NAME placeholder)
    derpbot.py               # RobotInterface implementation
  robot_interface.py         # abstract base class
worlds/
  templates/indoor_office/   # 20×15 m office world + ground-truth map
  templates/indoor_warehouse/ # warehouse world (template only)
  models/                    # target object SDF models
    fire_extinguisher/       # class ID 1
    first_aid_kit/           # class ID 2
    hazard_sign/             # class ID 3
src/
  metrics/                   # ROS 2 metric nodes
  scenario_runner/           # scenario orchestration
  world_manager/             # world generation + object placement
  utils/
tests/                       # pytest unit tests
docs/
  ARST_Project_Plan.md       # full project plan
```

## Running tests

```bash
pytest tests/
```
