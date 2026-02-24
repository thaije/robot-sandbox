# Robot Sandbox — ARST

Autonomous Robotics Simulation Testbed. Gazebo Harmonic + ROS 2 Jazzy. DerpBot (two-wheeled diff-drive) in indoor environments with metrics + composite scoring.

Full plan: [`docs/ARST_Project_Plan.md`](docs/ARST_Project_Plan.md) · Agent state: [`docs/AGENT_HANDOFF.md`](docs/AGENT_HANDOFF.md)

## Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (`ros-jazzy-ros-gz*`)
- `robot_state_publisher`, `ros_gz_bridge`, `ros_gz_sim`, `teleop_twist_keyboard`
- **Python 3.12** — `rclpy` is compiled for 3.12; `python3` may resolve to a different interpreter

## Run a scenario (automated)

```bash
./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless [--timeout N] [--gui]
```

Handles full lifecycle: generates world SDF (with robot embedded), starts Gazebo, collects metrics, prints scorecard, cleans up. Results → `results/<scenario_name>.json`.

## Manual simulation

```bash
# Start Gazebo
ros2 launch launch/arst_sim.launch.py [headless:=true]

# Spawn robot (manual/dev use only — automated runs embed robot in world SDF)
ros2 launch launch/spawn_robot.launch.py [name:=derpbot_0] [x:=1.0] [y:=1.0] [yaw:=0.0]

# Teleoperate
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel

# Despawn
scripts/despawn_robot.sh [name] [world]
```

## Tests

```bash
python3.12 -m pytest tests/
```
