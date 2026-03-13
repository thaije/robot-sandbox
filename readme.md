# Robot Sandbox — ARST

Autonomous Robotics Simulation Testbed. Gazebo Harmonic + ROS 2 Jazzy. DerpBot (two-wheeled diff-drive) in indoor environments with metrics + composite scoring.

Full plan: [`docs/ARST_Project_Plan.md`](docs/ARST_Project_Plan.md) · Agent state: [`docs/AGENT_HANDOFF.md`](docs/AGENT_HANDOFF.md)

## Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (`ros-jazzy-ros-gz*`)
- `robot_state_publisher`, `ros_gz_bridge`, `ros_gz_sim`, `teleop_twist_keyboard`
- **Python 3.12** — `rclpy` is compiled for 3.12; `python3` may resolve to a different interpreter
- **uv** — Python package/tool runner (`pip install uv` or see [docs.astral.sh/uv](https://docs.astral.sh/uv)): `uv venv` > source venv > `uv pip install -r requirements.txt`
- **ast-grep** — structural code search (`npm install -g @ast-grep/cli`)
- **Serena MCP** — symbol-level code navigation for AI agents (auto-installed via uv/uvx; see `~/.claude.json`)

## Run a scenario (automated)

```bash
./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless [--timeout N] [--gui] [--seed N] [--speed N] [--enable-oracle] [--enable-pointcloud]
```

Handles full lifecycle: generates world SDF (with robot embedded), starts Gazebo, collects metrics, prints scorecard, cleans up. Results → `results/<scenario_name>.json`.

## Difficulty tiers — office_explore_detect

All tiers use the same indoor office map and object set; difficulty is controlled by lighting, door states, obstacles, and time limit.

| Tier | YAML | Lighting | Doors | Extras | Timeout |
|------|------|----------|-------|--------|---------|
| Easy | `office_explore_detect/easy.yaml` | bright | open | — | 900 s |
| Medium | `office_explore_detect/medium.yaml` | normal | random | — | 600 s |
| Hard | `office_explore_detect/hard.yaml` | dim + localized | closed | elevated objects | 300 s |
| Brutal | `office_explore_detect/brutal.yaml` | dim + flicker | closed | patrol bot + elevated | 180 s |
| Perception stress | `office_explore_detect/perception_stress.yaml` | flicker + red emergency | random | patrol bot + smoke | 600 s |

```bash
# Run a specific tier (all accept --headless / --gui / --timeout / --seed):
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --headless
```

Pass `--seed N` to pin a specific layout; omit for a fresh random layout each run.

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
| RobotModel | `/derpbot_0/robot_description` | Full URDF with lidar drum visible |
| TF | `/tf` | `odom → base_footprint → base_link → lidar_link / camera_link` |
| Odometry | `/derpbot_0/odom` | Red arrows, last 100 poses |
| Camera Image | `/derpbot_0/rgbd/image` | Live RGB feed from forward-facing RGBD camera |
| World Map | `/arst/world_map` | Top-down floor plan with robot + object positions |
| LiDAR Scan | `/derpbot_0/scan` | Cyan point cloud, 360° 12 m range |

## Human play

Drive the scenario yourself to create a baseline for the robot autonomy to compare to..

The human player gets **only the forward camera** — no map, no object positions, no collision indicators.  Use `rqt_image_view` rather than RViz so there is no information leak.

**Terminal 1 — simulation (headless; metrics still collected):**
```bash
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --headless --timeout 300
```

**Terminal 2 — camera-only view:**
```bash
ros2 run rqt_image_view rqt_image_view /derpbot_0/rgbd/image
```

**Terminal 3 — keyboard teleop:**
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel
```

Teleop keys (default): `i` forward · `,` backward · `j`/`l` rotate · `k` stop · `u`/`o`/`m`/`.` diagonals.
Speed is capped at 0.5 m/s and 2.0 rad/s by the diff-drive plugin regardless of what you send.

Navigate the four rooms and locate all objects as quickly as possible.  When the timeout expires (or all objects are found) the scorecard and JSON result are printed. 

## Sim diagnostics

```bash
# Realtime factor (headless-friendly)
gz topic -e -t /stats
```

## Tests

```bash
python3.12 -m pytest tests/
```


To fix: 
- bugs:

- extensions:
  - add dummy objects?
  - add some variation in signs e.g. also on wall etc.
- new features:
  - add sound? E.g. for detecting victims
  - add sensor jitter, e.g. odom wheels error / slipping  




