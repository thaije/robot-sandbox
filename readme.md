# Robot Sandbox — ARST

Autonomous Robotics Simulation Testbed. Gazebo Harmonic + ROS 2 Jazzy. DerpBot (two-wheeled diff-drive) in indoor environments with metrics + composite scoring.

**Autonomous agent instructions: [`docs/AUTONOMOUS_AGENT_GUIDE.md`](docs/AUTONOMOUS_AGENT_GUIDE.md)** — task, grading, sensors, running scenarios, benchmark submission.

State: [`docs/STATE.md`](docs/STATE.md) · Roadmap: [`docs/ROADMAP.md`](docs/ROADMAP.md)

---

## [**Leaderboard →**](https://thaije.github.io/robot-sandbox/leaderboard.html)

## Prerequisites

- Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic (`ros-jazzy-ros-gz*`)
- `robot_state_publisher`, `ros_gz_bridge`, `ros_gz_sim`, `teleop_twist_keyboard`
- **Python 3.12** — `rclpy` is compiled for 3.12; `python3` may resolve to a different interpreter
- **uv** — Python package/tool runner (`pip install uv` or see [docs.astral.sh/uv](https://docs.astral.sh/uv)): `uv venv` > source venv > `uv pip install -r requirements.txt`
- **ast-grep** — structural code search (`npm install -g @ast-grep/cli`)
- **Serena MCP** — symbol-level code navigation for AI agents (auto-installed via uv/uvx; see `~/.claude.json`)

## Run a scenario (automated)

```bash
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --headless [--timeout N] [--gui] [--seed N] [--speed N] [--enable-oracle] [--enable-pointcloud]
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
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --gui
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
| Odometry | `/derpbot_0/odom` | IMU-fused (EKF), red arrows, last 100 poses |
| Camera Image | `/derpbot_0/rgbd/image` | Live RGB feed from forward-facing RGBD camera |
| World Map | `/arst/world_map` | Top-down floor plan with robot + object positions |
| LiDAR Scan | `/derpbot_0/scan` | Cyan point cloud, 360° 12 m range |

## Human baselines

Two baseline modes compare human performance against autonomous agents. Both run **headless** — no Gazebo GUI needed. The human views the robot camera via `rqt_image_view` which subscribes to the ROS image topic directly.

### Oracle mode — navigation only (cheat detections)

Human drives; the oracle camera auto-detects objects. Measures human navigation ceiling with perfect perception.

```bash
# Terminal 1 — full batch (5 difficulties × 5 seeds):
./scripts/run_human_baseline.sh oracle

# Or single run:
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --headless --enable-oracle

# Terminal 2 — teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel

# Terminal 3 — camera view (optional, helps navigation):
ros2 run rqt_image_view rqt_image_view
# Select /derpbot_0/rgbd/image
```

### Perception mode — navigation + manual detections

Human drives **and** reports detections via keypresses. Measures human nav + perception + localization ceiling. Detection position uses IMU-fused odom (EKF, minimal drift).

```bash
# Terminal 1 — full batch:
./scripts/run_human_baseline.sh perception

# Or single run (no --enable-oracle):
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --headless

# Terminal 2 — teleop:
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel

# Terminal 3 — detection keypresses:
python3.12 scripts/human_detector_node.py

# Terminal 4 — robot camera view (required to spot objects):
ros2 run rqt_image_view rqt_image_view
# Select /derpbot_0/rgbd/image
```

Press `f` (fire_extinguisher), `a` (first_aid_kit), `p` (person) each time you see a target object. Press only once per distinct object. Use `rqt_image_view` — not RViz — to avoid information leaks.

Results go to `results/submissions/human-baseline-oracle/` or `results/submissions/human-baseline-perception/`. 

## Sim diagnostics

```bash
# Realtime factor (headless-friendly)
gz topic -e -t /stats
```

## Benchmark submission

Seeds 1–5 × 3 runs/seed = 15 runs per difficulty. Full protocol, result naming convention, submission YAML format, and validation instructions: **[`docs/AUTONOMOUS_AGENT_GUIDE.md §7`](docs/AUTONOMOUS_AGENT_GUIDE.md#7-benchmark-submission)**.

```bash
python3.12 scripts/validate_submission.py path/to/benchmark_submission.yaml
```

## Leaderboard

**[View the live leaderboard →](https://thaije.github.io/robot-sandbox/leaderboard.html)**

After adding or updating submissions in `results/submissions/`, regenerate the leaderboard:

```bash
python3.12 scripts/generate_leaderboard.py
```

This reads all `benchmark_submission.yaml` files, aggregates per-difficulty stats, and writes:
- `docs/leaderboard.json` — structured data
- `docs/leaderboard.html` — viewable leaderboard page with per-cell hover tooltips

The leaderboard is auto-published to GitHub Pages from `docs/` on every push to `main`.

## Tests

```bash
# Unit tests (no sim required)
python3.12 -m pytest tests/

# Integration test (requires live sim + ROS 2)
pytest -m integration tests/test_integration.py
```




