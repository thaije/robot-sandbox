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

## Interactive dev session (GUI + teleoperation)

```bash
./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --gui
```

Then in a separate terminal:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel
```

> Dynamic spawn via `spawn_robot.launch.py` also works but contact sensors won't fire (gz-sim 8.10.0 bug). Use `run_scenario.sh --gui` for full fidelity.

## Visualization (RViz2)

```bash
rviz2 -d config/derpbot.rviz
```

Requires `spawn_robot.launch.py` (or `run_scenario.sh --gui`) to be running first — it provides `robot_state_publisher` and the TF tree.

**Displays included:**

| Display | Topic | Notes |
|---------|-------|-------|
| RobotModel | `/derpbot_0/robot_description` | Full URDF with camera mount visible |
| TF | `/tf` | `odom → base_footprint → base_link → camera_link` |
| Odometry | `/derpbot_0/odom` | Red arrows, last 100 poses |
| Camera Image | `/derpbot_0/image_raw` | Live RGB feed from forward-facing camera |

## Robot inspection & control (automated sessions)

```bash
source /opt/ros/jazzy/setup.bash
export PYTHONPATH=src:$PYTHONPATH
python3.12 scripts/robot_inspect.py <command>
```

| Command | Example | What it does |
|---------|---------|--------------|
| `status` | `robot_inspect.py status` | Print x/y/yaw + sim stamp |
| `snapshot` | `robot_inspect.py snapshot [--output /tmp/out.png]` | Save camera frame to PNG |
| `detections` | `robot_inspect.py detections` | Print objects visible in current frame |
| `drive` | `robot_inspect.py drive 0.3 0.0 2.0` | Drive `vx` m/s, `wz` rad/s for N seconds then stop |

All commands accept `--robot NAME` (default `derpbot_0`) and `--timeout SEC`.

Drive examples:
```bash
python3.12 scripts/robot_inspect.py drive 0.3 0.0 2.0   # forward ~0.6 m
python3.12 scripts/robot_inspect.py drive 0.0 0.6 2.6   # turn left ~90°
python3.12 scripts/robot_inspect.py drive 0.0 -0.6 2.6  # turn right ~90°
```

> Keep drive durations ≤ 2 s — the controller holds the last velocity until a new command arrives.

## Tests

```bash
python3.12 -m pytest tests/
```
