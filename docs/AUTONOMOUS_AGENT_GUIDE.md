# Autonomous Agent Guide — ARST

Gazebo Harmonic + ROS 2 Jazzy simulation testbed. Your task: deploy a fully autonomous agent and complete the scenario.

---

## 1. Task

You will be placed in an unknown indoor environment. Before each run you receive a **mission description** with the mission goal listing for example the target object types to locate. Your agent must explore the environment, detect all instances of every target type, and report detections — all within a **time limit**.

Scenarios come in multiple **difficulty tiers**. The environment may challenge your agent in various ways — expect conditions to differ between tiers and runs. Object placement and spawn position are **randomised each run** unless a seed is fixed. Doors are passable or permanently locked; no unlocking or interaction is required.

> **Design advice:** Build a general-purpose agent. Tiers, object types, and environment conditions may be extended or changed at any time. Agents that adapt to what they observe will hold up far better than those tuned to specific scenarios.

**Success** = all required instances detected before the time limit.

---

## 2. Grading

Runs are scored 0–100 across five categories, combined as a weighted sum.

| Category | What it measures |
|---|---|
| **Speed** | Time-to-detect and exploration rate relative to par |
| **Accuracy** | Detection rate, false-positive rate, path efficiency |
| **Safety** | Collisions and near-misses |
| **Efficiency** | Redundant traversal, area-per-metre coverage |
| **Effectiveness** | Per-type detection completeness (types have configurable weights) |

Grade thresholds: **S** ≥ 95 · **A** ≥ 85 · **B** ≥ 70 · **C** ≥ 55 · **D** ≥ 40 · **F** < 40.
A B-grade means *competent performance for that difficulty level*.

Full score breakdown is written to `results/` as JSON after each run.

---

## 3. Platform — DerpBot

Ground robot (differential drive — currently the only supported model).

| Property | Value |
|---|---|
| Body | 30 × 20 × 10 cm |
| Max linear speed | 0.5 m/s |
| Max angular speed | 2.0 rad/s |
| Drive command | `geometry_msgs/Twist` |

### Sensors

| Sensor | Topic | Rate | Details |
|---|---|---|---|
| 2-D LiDAR | `/derpbot_0/scan` | ~10 Hz | 360°, 720 rays, 0.5° resolution, 0.15–12 m range, `sensor_msgs/LaserScan` |
| IMU | `/derpbot_0/imu` | 100 Hz | `sensor_msgs/Imu`, **BEST_EFFORT QoS** |
| RGB camera | `/derpbot_0/image_raw` | 10 Hz | 640 × 480, 90° horizontal FOV, forward-facing, `sensor_msgs/Image` |
| Odometry | `/derpbot_0/odom` | — | `nav_msgs/Odometry`, wheel-encoder dead-reckoning |

### Control & TF

| Topic/Frame | Details |
|---|---|
| `/derpbot_0/cmd_vel` | `geometry_msgs/Twist` — drive command |
| `/derpbot_0/joint_states` | `sensor_msgs/JointState` |
| TF tree | `odom → base_footprint → base_link` |

### Detection output (ground-truth oracle)

`/derpbot_0/detections` publishes `vision_msgs/Detection2DArray` from a ground-truth bounding-box camera (same FOV/resolution as the RGB camera, handles occlusion). This is the reference output used for scoring. You may use your own vision pipeline on `/derpbot_0/image_raw` and remap to the detections topic; or consume the oracle directly and focus your autonomy on navigation and exploration.

---

## 4. Running a scenario

**Requirements:** ROS 2 Jazzy, Gazebo Harmonic, Python 3.12, a sourced workspace.

```bash
# Single run, headless, random seed
./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless

# Pin layout for reproducibility
./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless --seed 42

# Shorten timeout for iteration
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --headless --timeout 300
```

Available difficulty tiers (ascending): `easy` · `medium` · `hard` · `brutal` · `perception_stress`
All YAML files are under `config/scenarios/office_explore_detect/`.

Startup takes ~5 s. The scenario ends on `SUCCESS` or `TIME_LIMIT`; scorecard prints to stdout and JSON is written to `results/`.

### ROS 2 node requirements

- Create your node with `use_sim_time=True` or sensor messages will be silently dropped.
- Subscribe to IMU with `ReliabilityPolicy.BEST_EFFORT`.
- Isolated parallel runs: prefix with `ROS_DOMAIN_ID=N`.

---

## 5. What your agent must do

1. **Receive the mission** — fetch the mission description (see section 4) before or at run start.
2. **Explore** the environment — no map is provided; build it from LiDAR and odometry.
3. **Detect objects** — publish `vision_msgs/Detection2DArray` on `/derpbot_0/detections`, or rely on the oracle.
4. **Navigate safely** — collisions and near-misses reduce your Safety score.
5. **Stop or let the runner end the episode** — the runner polls detections and terminates automatically.

---

## 6. Development tips

- Start with `easy.yaml` for a more forgiving baseline.
- Use `--seed N` to freeze the layout while iterating on your algorithm.
- `scripts/robot_control.py` — manual drive / camera snapshot (dev/cheat tool).
- `scripts/world_state.py` — PNG map + object ground-truth positions (dev/cheat tool).
- Reference docs: [`docs/AGENT_HANDOFF.md`](AGENT_HANDOFF.md) (architecture, gotchas).
