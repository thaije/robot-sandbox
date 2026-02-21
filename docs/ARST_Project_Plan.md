# Project Plan: Autonomous Robotics Simulation Testbed (ARST)

## 1. Project Overview

### 1.1 Vision

Build a modular, extensible simulation environment for testing autonomous robots across diverse scenarios, paired with an autonomy stack that enables robots to explore unknown environments and locate target objects. The system serves as an academic research platform for developing and benchmarking robust robotic autonomy.

### 1.2 Project Goals

1. **Simulation Environment:** Create a configurable digital sandbox where robotic scenarios can be defined, executed, reset, and evaluated programmatically. The environment must support multiple world templates, configurable metrics, and headless operation.
2. **Robot Integration:** Integrate DerpBot (a minimal two-wheeled differential drive robot) as the first platform, controllable via ROS 2 topics.  Sensors (LiDAR, camera) are added in later steps once the simulation pipeline is stable.
3. **First Use Case — Explore & Detect:** Implement the scenario "autonomously explore an unknown indoor environment and detect all target objects of specified types," with configurable success/failure criteria and comprehensive metrics logging.

### 1.3 Scope Boundaries

**In scope for Phase 1:**
- Simulation environment with Gazebo + ROS 2 (persistent simulator + spawn/despawn API)
- DerpBot: minimal two-wheeled differential drive robot (no sensors in baseline; sensors added incrementally)
- Indoor world templates (office-like, warehouse-like) with object placement variation
- Metrics framework with configurable success/failure criteria
- Headless (CLI-driven) operation
- Manual teleoperation of the robot (autonomy is NOT in scope for Phase 1)

**In scope for later phases (design accommodated now, implementation deferred):**
- Autonomous exploration and object detection algorithms
- Additional robot platforms (Spot, drones, humanoids)
- Additional use cases (search & rescue, inspection, security, multi-robot relay, hazmat)
- Migration path to Isaac Sim or other high-fidelity simulators
- Multi-instance parallel simulation
- Photorealistic rendering

**Out of scope:**
- Physical robot deployment and sim-to-real transfer (deferred)
- Building simulators or autonomy stacks from scratch
- Real-time cloud deployment

### 1.4 Hardware Available

| Component | Specification |
|-----------|--------------|
| GPUs | 3x NVIDIA RTX 2070 Super (8 GB VRAM each, 24 GB total VRAM) |
| CPUs | 2x Intel Xeon E5-2630 v4 (10 cores / 20 threads each, 2.2 GHz base / 3.1 GHz turbo, 40 threads total) |
| RAM | 96 GB |
| Storage | 2 TB NVMe SSD |
| OS | Ubuntu 24.04 LTS (required for ROS 2 Jazzy + Gazebo Harmonic) |

**Hardware notes:** The Xeon E5-2630 v4 is a Broadwell-EP (2016) server CPU. Single-thread performance is moderate by current standards, but the 40 total threads provide excellent parallelism for ROS 2 node execution and multi-instance simulation. The 96 GB RAM is very generous for simulation workloads. The 3x RTX 2070 Super GPUs can be allocated one-per-simulation-instance if parallel execution is implemented, or combined for rendering-heavy workloads. Note: these CPUs do not support AVX-512 (only AVX2), which may matter for some ML inference libraries — verify compatibility if deploying neural network-based perception later.

---

## 2. Requirements

Requirements are categorized using MoSCoW prioritization: Must Have (M), Should Have (S), Nice to Have (N).

### 2.1 Simulation Environment Requirements

| ID | Requirement | Priority | Notes |
|----|------------|----------|-------|
| SIM-01 | Gazebo-based simulation with ROS 2 integration | M | Gazebo Harmonic or Fortress recommended |
| SIM-02 | Headless / CLI operation (no GUI required to run) | M | Essential for scripted experiments and CI |
| SIM-03 | Programmatic scenario configuration (YAML/JSON config files) | M | Define world, objects, robot spawn, success criteria |
| SIM-04 | World templates: indoor structured (office/corridor) | M | First template |
| SIM-05 | World templates: indoor unstructured (cluttered/disaster) | S | Second template |
| SIM-06 | World templates: outdoor urban | N | Future phase |
| SIM-07 | World templates: outdoor natural (forest, terrain) | N | Future phase |
| SIM-08 | Randomized object placement within templates | M | For robustness testing |
| SIM-09 | Configurable environment variations (lighting, obstacles, layout changes) | S | Beyond just object placement |
| SIM-10 | World and robot models use standard formats (SDF/URDF) transferable to other simulators | M | Enables future Isaac Sim migration |
| SIM-11 | Scenario reset (full environment reset to initial state programmatically) | M | |
| SIM-12 | Multiple simulator instances running in parallel | N | For batch experiments |
| SIM-13 | Deterministic replay (same seed produces same scenario) | S | For reproducibility |
| SIM-14 | Simulation speed control (faster than real-time if possible) | S | Accelerates experiments |

### 2.2 Robot Integration Requirements

| ID | Requirement | Priority | Notes |
|----|------------|----------|-------|
| ROB-01 | DerpBot: minimal two-wheeled differential drive robot in simulation | M | Custom URDF in robots/derpbot/; no external packages required |
| ROB-02 | RGB camera sensor | S | Added in a later step once baseline is stable |
| ROB-03 | LiDAR sensor (2D) | S | Added in a later step |
| ROB-04 | IMU sensor | N | Added when needed |
| ROB-05 | Contact / bumper sensors | S | Added when needed |
| ROB-06 | Motor / joint control via ROS 2 cmd_vel and standard interfaces | M | Provided by gz-sim-diff-drive-system plugin |
| ROB-07 | Sensor data accessible via standard ROS 2 topics | M | Via ros_gz_bridge; topics namespaced per robot instance |
| ROB-08 | Teleoperation capability (keyboard) | M | teleop_twist_keyboard with /<name>/cmd_vel |
| ROB-09 | Persistent simulator: Gazebo runs continuously; robots are spawned/despawned without restart | M | arst_sim.launch.py (sim only) + spawn_robot.launch.py |
| ROB-10 | Architecture supports adding new robot platforms without restructuring | M | Abstract RobotInterface; add robot type = new sub-directory |
| ROB-11 | Turtlebot 4 integration | N | Future phase; requires turtlebot4_simulator package |
| ROB-12 | Spot robot model integration | N | Future phase |
| ROB-13 | Quadcopter drone model integration | N | Future phase |
| ROB-14 | Multi-robot spawning in same scenario | N | Supported by spawn_robot.launch.py with unique name: parameter |

### 2.3 Metrics and Evaluation Requirements

| ID | Requirement | Priority | Notes |
|----|------------|----------|-------|
| MET-01 | Metrics framework that logs data during scenario execution | M | |
| MET-02 | Metric: meters traveled (odometry-based path length) | M | |
| MET-03 | Metric: time until all target objects detected | M | |
| MET-04 | Metric: task completion time (all areas explored) | M | |
| MET-05 | Metric: average time per detection | M | |
| MET-06 | Metric: exploration coverage percentage | M | Area explored / total explorable area |
| MET-07 | Metric: object detection rate (objects found / total objects) | M | |
| MET-08 | Metric: false positive detection rate | M | Important for detection reliability |
| MET-09 | Metric: path efficiency ratio (optimal path / actual path) | S | Requires computing shortest path |
| MET-10 | Metric: collision count | M | Safety-relevant |
| MET-11 | Metric: computational resource usage (CPU/GPU/memory during run) | S | For profiling |
| MET-12 | Configurable success/failure criteria per scenario (e.g., "find all objects of type X and Y" or "explore all areas") | M | Defined in scenario config |
| MET-13 | Metrics output to structured format (CSV, JSON, or database) | M | For automated analysis |
| MET-14 | Configurable timeout for scenario execution | M | Failure condition |
| MET-15 | Aggregate statistics across multiple runs (mean, std dev, min, max) | S | For batch experiments |
| MET-16 | Metric: energy consumption estimate (based on motor commands) | N | Useful for real-world feasibility |
| MET-17 | Composite scores combining raw metrics into category scores and an overall score | M | Gamified scorecard; see Section 5.4 |
| MET-18 | Composite score: Speed | M | Combines task completion time, avg time per detection, coverage rate |
| MET-19 | Composite score: Accuracy | M | Combines object detection rate, false positive rate, path efficiency |
| MET-20 | Composite score: Safety | M | Based on collision count and near-miss proximity events |
| MET-21 | Composite score: Efficiency | M | Combines revisit ratio, meters traveled vs. optimal, coverage per meter |
| MET-22 | Composite score: Overall | M | Weighted combination of category scores |
| MET-23 | Composite score weights configurable per scenario | M | Different use cases emphasize different categories |
| MET-24 | Score display as letter grade (S/A/B/C/D/F) and numeric (0-100) | M | Both human-readable and machine-comparable |

### 2.4 Documentation and Reproducibility Requirements

| ID | Requirement | Priority | Notes |
|----|------------|----------|-------|
| DOC-01 | Installation guide (dependencies, setup steps) | M | |
| DOC-02 | User guide: how to configure and run a scenario | M | |
| DOC-03 | Developer guide: how to add new worlds, robots, metrics | M | |
| DOC-04 | Architecture documentation with design decisions | M | |
| DOC-05 | All configurations version-controlled (Git) | M | |
| DOC-06 | Reproducible environment setup (Docker or detailed env specs) | S | |
| DOC-07 | Example scenarios with expected outputs | S | Serves as integration tests |

---

## 3. Architecture Overview

### 3.1 High-Level Architecture

```
┌─────────────────────────────────────────────────────┐
│                 Scenario Runner (CLI)                │
│  - Reads scenario config (YAML)                     │
│  - Launches simulator with specified world + robot   │
│  - Manages lifecycle (start, monitor, stop, reset)  │
│  - Collects metrics and writes results              │
└────────────┬───────────────────────────┬────────────┘
             │                           │
             ▼                           ▼
┌────────────────────────┐  ┌────────────────────────┐
│   Simulation Layer     │  │   Metrics Collector    │
│  - Gazebo simulator    │  │  - Subscribes to ROS   │
│  - World templates     │  │    topics              │
│  - Robot models        │  │  - Tracks odometry,    │
│  - Sensor plugins      │  │    detections, time    │
│  - Object placement    │  │  - Evaluates success/  │
│    engine              │  │    failure criteria    │
└────────────┬───────────┘  └────────────┬───────────┘
             │                           │
             ▼                           ▼
┌────────────────────────┐  ┌────────────────────────┐
│     ROS 2 Layer        │  │   Scoring & Results    │
│  - Standard topics     │  │  - Composite scoring   │
│  - tf2 transforms      │  │    engine (S/A/B/C/D/F)│
│  - Nav2 (future)       │  │  - Scorecard display   │
│  - Sensor messages     │  │  - JSON / CSV logs     │
│                        │  │  - Per-run + aggregate │
└────────────────────────┘  └────────────────────────┘
```

### 3.2 Key Abstractions

These abstractions enable extensibility to new robots, worlds, and use cases without restructuring:

1. **Scenario Configuration:** A YAML file that fully describes a test scenario — which world template, which robot, object types and placements, success/failure criteria, metrics to collect, and timeout.

2. **World Template:** A parameterized environment description. Base SDF world file + a configuration layer that specifies object placements, variations, and regions of interest (for coverage calculation).

3. **Robot Interface:** A standardized description of what a robot provides (sensor topics, control topics, URDF/SDF model). Adding a new robot means providing this description plus the model files.

4. **Metrics Plugin Interface:** Each metric is a self-contained module that subscribes to relevant ROS 2 topics, computes its value over time, and reports a final result. New metrics can be added without modifying existing code.

5. **Success Criteria Evaluator:** A configurable rule engine that takes metric outputs and determines pass/fail. Rules are specified in the scenario config (e.g., `object_detection_rate >= 1.0`, `exploration_coverage >= 0.95`).

### 3.3 Scenario Configuration Example

```yaml
scenario:
  name: "office_explore_and_detect_001"
  description: "Explore an office environment and find all target objects"
  timeout_seconds: 600
  random_seed: 42

world:
  template: "indoor_office"
  variant: "medium_clutter"
  size: "20x15"  # meters
  objects:
    - type: "fire_extinguisher"
      count: 3
      placement: "random"  # random within valid positions
    - type: "first_aid_kit"
      count: 2
      placement: "random"
    - type: "hazard_sign"
      count: 4
      placement: "random"
  variations:
    lighting: "normal"  # normal, dim, bright
    door_states: "random"  # open, closed, random

robot:
  platform: "derpbot"
  spawn_pose:
    x: 1.0
    y: 1.0
    z: 0.0
    yaw: 0.0
  sensors: []  # no sensors in Phase 1 baseline

metrics:
  collect:
    - meters_traveled
    - time_to_all_detections
    - task_completion_time
    - average_time_per_detection
    - exploration_coverage
    - object_detection_rate
    - false_positive_rate
    - collision_count
    - near_miss_count
    - revisit_ratio
  
success_criteria:
  mode: "all_of"  # all_of, any_of
  conditions:
    - metric: "object_detection_rate"
      operator: ">="
      value: 1.0
      description: "All target objects must be detected"

scoring:
  category_weights:
    speed: 0.25
    accuracy: 0.30
    safety: 0.20
    efficiency: 0.25
  par_values:
    time_per_detection_par: 60.0
    coverage_rate_par: 2.0
    coverage_per_meter_par: 1.5
    near_miss_threshold: 0.3

output:
  format: "json"
  directory: "./results/"
  include_trajectory: true
  include_detection_log: true
  include_scorecard: true
```

### 3.4 Directory Structure

```
arst/
├── README.md
├── docs/
│   ├── architecture.md
│   ├── installation.md
│   ├── user_guide.md
│   └── developer_guide.md
├── config/
│   ├── scenarios/                  # Scenario YAML files
│   │   ├── office_explore_detect.yaml
│   │   └── warehouse_explore_detect.yaml
│   ├── robots/                     # Robot configuration files
│   │   └── turtlebot4.yaml
│   └── metrics/                    # Metrics definitions
│       └── explore_detect_metrics.yaml
├── worlds/
│   ├── templates/                  # Base world SDF files
│   │   ├── indoor_office/
│   │   │   ├── world.sdf
│   │   │   ├── config.yaml         # Template parameters
│   │   │   └── regions.yaml        # Explorable regions for coverage calc
│   │   └── indoor_warehouse/
│   ├── models/                     # Reusable SDF models (objects, furniture)
│   │   ├── fire_extinguisher/
│   │   ├── first_aid_kit/
│   │   └── ...
│   └── generators/                 # Scripts for world variation generation
│       └── place_objects.py
├── robots/
│   ├── derpbot/
│   │   ├── urdf/
│   │   │   └── derpbot.urdf        # Two-wheel diff-drive URDF (no sensors)
│   │   ├── __init__.py
│   │   └── derpbot.py              # Concrete RobotInterface implementation
│   └── robot_interface.py          # Abstract robot interface
├── src/
│   ├── scenario_runner/            # Main CLI application
│   │   ├── runner.py
│   │   ├── launcher.py             # Gazebo + ROS 2 launch management
│   │   └── reset.py                # Scenario reset logic
│   ├── metrics/
│   │   ├── base_metric.py          # Abstract metric class
│   │   ├── meters_traveled.py
│   │   ├── exploration_coverage.py
│   │   ├── object_detection_tracker.py
│   │   ├── detection_metrics.py
│   │   ├── collision_count.py
│   │   ├── near_miss_tracker.py
│   │   ├── revisit_ratio.py
│   │   ├── time_metrics.py
│   │   ├── scoring.py              # Composite scoring engine
│   │   ├── evaluator.py            # Success/failure evaluator
│   │   └── reporter.py             # Results output + scorecard display
│   ├── world_manager/
│   │   ├── template_loader.py
│   │   ├── object_placer.py        # Randomized object placement
│   │   └── world_generator.py      # Generates final SDF from template + config
│   └── utils/
│       ├── ros_helpers.py
│       ├── config_loader.py
│       └── logging_setup.py
├── results/                        # Output directory for run results
├── scripts/
│   ├── run_scenario.sh
│   ├── run_batch.sh                # Run multiple scenarios
│   └── aggregate_results.py
├── tests/
│   ├── test_scenario_config.py
│   ├── test_metrics.py
│   └── test_world_generation.py
├── docker/                         # Optional Docker setup
│   └── Dockerfile
├── requirements.txt
└── setup.py
```

---

## 4. Technology Stack

### 4.1 Core Technologies

| Component | Technology | Version | Rationale |
|-----------|-----------|---------|-----------|
| Robot middleware | ROS 2 Jazzy | Jazzy Jalisco (LTS) | LTS release, EOL May 2029; native to Ubuntu 24.04 |
| Simulator | Gazebo Harmonic | Harmonic (LTS) | Paired with Jazzy; EOL Sept 2028; Python plugins, Simulation Reset API, gz-usd Isaac Sim tooling |
| Programming language | Python 3.10+ | 3.10 | ROS 2 Humble default, good for rapid prototyping |
| Secondary language | C++ 17 | For performance-critical plugins if needed |
| Configuration | YAML | Standard in ROS ecosystem |
| Version control | Git | |
| Containerization | Docker (optional) | For reproducible setup |

### 4.2 Key ROS 2 Packages

| Package | Purpose |
|---------|---------|
| `ros_gz_bridge` | Gazebo-ROS 2 message bridging |
| `ros_gz_sim` | Gazebo simulation launch + spawn service (`create` executable) |
| `robot_state_publisher` | Publishes TF tree from URDF + joint states |
| `teleop_twist_keyboard` | Keyboard teleoperation |
| `nav2` (future) | Navigation stack for autonomous movement |
| `slam_toolbox` (future) | SLAM for mapping |
| `tf2_ros` | Transform management |

### 4.3 Research Decisions Needed

Several technology choices require investigation before committing. These are flagged as research tasks in the step-by-step plan (Section 6).

| Decision | Options | Considerations |
|----------|---------|----------------|
| Gazebo version | Fortress vs. Harmonic | Fortress is LTS and paired with Humble, but Harmonic has better features. Check Turtlebot 4 simulator compatibility. |
| World format | ✅ **Resolved: URDF for robots, SDF for worlds (parallel operation strategy)** | No automated SDF→USD pipeline exists. Author robot descriptions in URDF (both simulators consume it). Keep world geometry separate from plugin config. Use Gazebo for integration testing; Isaac Sim for RL/photorealistic rendering. Revisit in ~1–2 years as tooling matures. |
| Object detection ground truth | ✅ **Resolved: visible 2D bounding box camera** | Logical camera disqualified (sees through walls). Bounding box camera handles occlusion natively and outputs `vision_msgs/Detection2DArray` — same type as YOLO. Swap to real perception = one topic remap. |
| Coverage calculation | ✅ **Resolved: grid-based LiDAR raycasting with Gazebo ground-truth pose** | SLAM-decoupled. Uses `skimage.draw.line()` Bresenham raycasting at 0.5m resolution. Ground truth stored as PGM + YAML. Processes full 360-ray scan in under 2ms. |
| Metrics transport | ROS 2 topics vs. Gazebo transport vs. direct API | How metrics collector gets data from simulation. |

---

## 5. Detailed Requirements for Phase 1 Use Case

### 5.1 Use Case: Explore and Detect

**Description:** A DerpBot is placed in an unknown indoor environment. The task is to explore the entire environment and detect all target objects of specified types. In Phase 1, the robot is teleoperated (no autonomy), and sensors (LiDAR/camera) are added incrementally. The simulation framework records all metrics and evaluates success/failure.

**Actors:**
- Human operator (via teleoperation interface)
- Simulation system (manages scenario lifecycle, records metrics, evaluates success)

**Flow:**
1. Operator specifies scenario configuration (YAML file or CLI arguments)
2. System generates the world from template + configuration (random seed for object placement)
3. System spawns Gazebo world and robot
4. System starts metrics collection
5. Operator controls the robot via teleoperation
6. System continuously monitors: robot pose, sensor data, object proximity/detection
7. When success criteria are met OR timeout is reached, system stops the scenario
8. System outputs metrics report

**Ground Truth for Detection:**
For Phase 1, object detection uses a `boundingbox_camera` sensor (visible 2D mode) co-located with the TurtleBot 4's OAKD camera. This rendering-based sensor handles occlusion natively — objects behind walls are not detected. It outputs `vision_msgs/Detection2DArray` on `/ground_truth/detections`. A thin **detection source abstraction node** republishes on a unified `/detections` topic, parameterized to select ground-truth or real-perception source. When a real perception system is integrated later, only the parameter changes — all downstream code is unaffected. Target objects must be tagged with the `gz-sim-label-system` plugin in their SDF.

**Ground Truth for Coverage:**
The explorable area is pre-defined per world template as a **PGM + YAML** ground truth map (standard ROS 2 map format). Coverage is computed by a dedicated `coverage_metrics_node` that uses Gazebo's ground-truth robot pose (zero drift) and LiDAR rays to trace which 0.5m grid cells have been observed via Bresenham raycasting (`skimage.draw.line()`). The ground truth map and coverage grid are invisible to the robot — they exist solely for the metrics system. This node works identically for teleoperation and autonomous operation.

### 5.2 Metrics Specification

| Metric | Definition | Unit | Collection Method |
|--------|-----------|------|-------------------|
| Meters traveled | Cumulative Euclidean distance between consecutive odometry readings | meters | Subscribe to `/odom` topic, integrate |
| Time to all detections | Elapsed sim time from scenario start to last target object first detected | seconds | Timestamp each detection event |
| Task completion time | Elapsed sim time from scenario start to success criteria met or timeout | seconds | Scenario runner clock |
| Average time per detection | Mean of (detection timestamp - scenario start) across all detected objects | seconds | Computed from detection event log |
| Exploration coverage | Fraction of explorable grid cells observed by robot sensors | percentage (0-100) | Compare swept cells to ground truth grid |
| Object detection rate | Number of target objects detected / total target objects placed | ratio (0.0-1.0) | Detection event log vs. scenario config |
| False positive rate | Number of false detections / total detections reported | ratio (0.0-1.0) | Relevant when perception system is integrated; for ground-truth oracle this is 0 by definition |
| Collision count | Number of times robot makes contact with environment or objects | count | Subscribe to contact/bumper sensor topics |
| Revisit ratio | Fraction of cells visited more than once / total cells visited | ratio | Derived from trajectory; indicates exploration inefficiency |

### 5.3 Success / Failure Criteria Configuration

Success and failure are defined per scenario in the YAML config. The evaluator supports:

- **Comparison operators:** `>=`, `<=`, `==`, `>`, `<`
- **Logical combination modes:** `all_of` (all conditions must be met), `any_of` (at least one condition)
- **Implicit failure:** Timeout reached without success criteria met = failure
- **Nested conditions (future):** For complex multi-objective scenarios

Examples:

```yaml
# Example 1: Find all objects
success_criteria:
  mode: "all_of"
  conditions:
    - metric: "object_detection_rate"
      operator: "=="
      value: 1.0

# Example 2: Explore 95% of area
success_criteria:
  mode: "all_of"
  conditions:
    - metric: "exploration_coverage"
      operator: ">="
      value: 95.0

# Example 3: Combined objective
success_criteria:
  mode: "all_of"
  conditions:
    - metric: "object_detection_rate"
      operator: "=="
      value: 1.0
    - metric: "exploration_coverage"
      operator: ">="
      value: 90.0
```

### 5.4 Composite Scoring System

Raw metrics are useful for analysis but hard to compare across scenarios and don't give an intuitive sense of "how well did the robot do." The composite scoring system combines raw metrics into category scores and an overall score, providing a gamified scorecard for each run.

#### Score Categories

**🏎️ Speed Score** — How fast did the robot accomplish the mission?

| Input Metric | Weight | Scoring Logic |
|-------------|--------|---------------|
| Task completion time | 0.40 | Ratio of scenario timeout to actual time. Faster = higher. `score = (timeout - completion_time) / timeout * 100`. Capped at 100. |
| Average time per detection | 0.35 | Inverse of avg detection time, normalized against a configurable "par time" per object. `score = min(100, par_time / avg_time * 100)` |
| Coverage rate (coverage % / elapsed time) | 0.25 | How quickly the robot gains coverage. Normalized against a configurable "par rate." |

**🎯 Accuracy Score** — How precisely did the robot find what it needed to?

| Input Metric | Weight | Scoring Logic |
|-------------|--------|---------------|
| Object detection rate | 0.45 | Direct: `detection_rate * 100`. Finding all objects = 100. |
| False positive rate | 0.30 | Inverse: `(1 - false_positive_rate) * 100`. Zero false positives = 100. |
| Path efficiency ratio | 0.25 | `(optimal_path / actual_path) * 100`. Perfect path = 100. Capped at 100. |

**🛡️ Safety Score** — How carefully did the robot operate?

| Input Metric | Weight | Scoring Logic |
|-------------|--------|---------------|
| Collision count | 0.70 | Stepwise penalty: 0 collisions = 100, 1-2 = 80, 3-5 = 60, 6-10 = 40, 11+ = 20. (Thresholds configurable.) |
| Near-miss events (within configurable proximity threshold) | 0.30 | Similar stepwise penalty based on count. |

**⚡ Efficiency Score** — How smartly did the robot use its resources?

| Input Metric | Weight | Scoring Logic |
|-------------|--------|---------------|
| Revisit ratio | 0.35 | Inverse: `(1 - revisit_ratio) * 100`. No revisits = 100. |
| Coverage per meter traveled | 0.35 | `(coverage_percentage / meters_traveled)` normalized against a configurable par value. |
| Exploration completeness | 0.30 | Final exploration coverage percentage. 100% coverage = 100. |

**⭐ Overall Score** — Weighted combination of all category scores.

Default weights (configurable per scenario):

| Category | Default Weight | Rationale |
|----------|---------------|-----------|
| Speed | 0.25 | Important but not at the cost of accuracy or safety |
| Accuracy | 0.30 | Core mission objective: find the right things |
| Safety | 0.20 | Collisions matter, but less critical in simulation than real world |
| Efficiency | 0.25 | Smart exploration is key to scaling to larger environments |

#### Letter Grades

Scores map to letter grades for quick human-readable assessment:

| Score Range | Grade | Label |
|------------|-------|-------|
| 95-100 | S | Outstanding |
| 85-94 | A | Excellent |
| 70-84 | B | Good |
| 55-69 | C | Adequate |
| 40-54 | D | Poor |
| 0-39 | F | Failed |

#### Configuration

Weights, par times, and grade thresholds are all configurable per scenario:

```yaml
scoring:
  category_weights:
    speed: 0.25
    accuracy: 0.30
    safety: 0.20
    efficiency: 0.25
  
  par_values:
    time_per_detection_par: 60.0    # seconds — "par" time to detect one object
    coverage_rate_par: 2.0          # percent per second — "par" exploration rate
    coverage_per_meter_par: 1.5     # percent per meter — "par" efficiency
    near_miss_threshold: 0.3        # meters — proximity that counts as near-miss
  
  collision_thresholds: [0, 2, 5, 10]  # boundaries for stepwise penalty
  collision_scores: [100, 80, 60, 40, 20]  # score at each tier
  
  grade_thresholds:
    S: 95
    A: 85
    B: 70
    C: 55
    D: 40
    # Below D = F
```

#### Example Scorecard Output

```
╔══════════════════════════════════════════════════════════╗
║             SCENARIO RESULTS: office_explore_001        ║
╠══════════════════════════════════════════════════════════╣
║  Status: SUCCESS ✓   Time: 342s / 600s timeout         ║
╠══════════════════════════════════════════════════════════╣
║                                                          ║
║  🏎️  Speed        78 / 100   ████████░░   B  Good       ║
║  🎯  Accuracy     95 / 100   ██████████   S  Outstanding║
║  🛡️  Safety       80 / 100   ████████░░   B  Good       ║
║  ⚡  Efficiency   62 / 100   ██████░░░░   C  Adequate   ║
║                                                          ║
║  ─────────────────────────────────────────────────       ║
║  ⭐  OVERALL      79 / 100   ████████░░   B  Good       ║
║                                                          ║
╠══════════════════════════════════════════════════════════╣
║  Raw Metrics:                                            ║
║    Meters traveled:       47.3 m                         ║
║    Objects found:         5 / 5  (100%)                  ║
║    Exploration coverage:  87.2%                          ║
║    Collisions:            2                              ║
║    Revisit ratio:         0.38                           ║
║    Avg time/detection:    52.1 s                         ║
╚══════════════════════════════════════════════════════════╝
```

#### Design Notes

The scoring system is intentionally decoupled from the success/failure evaluator. A run can succeed (all objects found) but still receive a poor efficiency score (too many revisits, long path). This separation ensures the scores are useful for optimization — you always know whether you passed, and separately, how well you passed.

The par values serve as "expected reasonable performance" baselines. They should be calibrated per world template after initial testing (run the scenario a few times with teleoperation, observe typical values, set par slightly better than average human teleoperation). This ensures the scores are meaningful rather than arbitrary.

For future multi-robot scenarios, individual robot scores and a team score can coexist. The team score would add coordination-specific metrics (handoff success rate, communication efficiency, task overlap).

---

## 6. Step-by-Step Implementation Plan

The plan is organized into phases. Each phase has numbered steps. Steps marked with 🔍 are research tasks where investigation is needed before implementation. Steps marked with 🤖 indicate tasks where Claude can be asked to help identify the best approach.

### Phase 0: Research and Foundation (Weeks 1-2)

The goal of this phase is to resolve open technology questions and set up the development environment.

#### Step 0.1: Confirm Hardware and OS Setup
- Install Ubuntu 24.04 LTS (required for ROS 2 Jazzy + Gazebo Harmonic)
- Confirm GPU drivers are installed and working (NVIDIA driver 525+ recommended for RTX 2070 Super)
- Verify CUDA toolkit version (11.7+ for RTX 2070 Super)
- Confirm multi-GPU setup: verify all 3x RTX 2070 Super are recognized (`nvidia-smi`)
- Confirmed specs: 2x Xeon E5-2630 v4 (40 threads total), 96 GB RAM, 2 TB NVMe SSD
- Note: Xeon E5-2630 v4 supports AVX2 but not AVX-512 — verify compatibility with any ML libraries planned for later phases
- **Deliverable:** Hardware spec sheet and confirmed Ubuntu 24.04 installation with GPU drivers verified

#### Step 0.2: ✅ RESOLVED — Gazebo Version Selection
- **Decision: Ubuntu 24.04 + ROS 2 Jazzy + Gazebo Harmonic**
- Rationale: Fortress EOLs Sept 2026 (~7 months away). The `turtlebot4_simulator` jazzy branch is now the default; no supported Humble+Harmonic path exists. Harmonic adds Python system plugins, Simulation Reset API, and is the only version supported by `gz-usd` (Isaac Sim migration tooling). Jazzy+Harmonic provides 31–39 months of LTS runway.
- Headless rendering: both versions use EGL with `--headless-rendering`; equivalent capability on RTX 2070 Super hardware.
- Multi-instance: use Docker + NVIDIA Container Toolkit for GPU isolation (`--gpus '"device=N"'`). The `GZ_PARTITION` env var prevents topic collisions between instances.
- Launch command for TB4: `ros2 launch turtlebot4_gz_bringup turtlebot4_gz.launch.py`
- **Deliverable:** Research complete; see `docs/ Gazebo Fortress vs. Harmonic.md` for full analysis

#### Step 0.3: ✅ RESOLVED — SDF/URDF to USD Conversion Path / Isaac Sim Strategy
- **Decision: Parallel operation strategy. No wholesale migration planned.**
- Rationale: No reliable automated SDF-to-USD pipeline exists. Isaac Sim has no SDF importer. The `gz-usd` tool (14 commits, requires building OpenUSD from source) and `gz-omni` connector (unmaintained since 2022) are not production-ready. The URDF importer is production-quality but handles robot structure only — all sensors, Gazebo plugins, and world elements must be manually recreated. Budget weeks for a single robot, months for a complete world.
- **Recommended strategy:** Keep Gazebo/ROS 2 as the primary integration testing and metrics environment. Use Isaac Sim selectively for tasks it uniquely enables: GPU-accelerated RL policy training, photorealistic synthetic data generation, RTX sensor simulation.
- **Key design decisions this informs:**
  - Author robot descriptions in **URDF** (both simulators consume URDF; SDF-only locks out Isaac Sim's importer)
  - Keep environment geometry separate from plugin/behavior config in world files — minimizes future migration pain
  - For TB4 in Isaac Sim: start from the **iRobot Create 3** USD model (TB4's base, already in Isaac Sim asset library with correct physics), add TB4-specific components on top
- The convergence point (reliable SDF↔USD tooling via NVIDIA's Simulation Interfaces standard + Newton project) is expected in ~1–2 years.
- **Deliverable:** Research complete; see `docs/Gazebo-to-Isaac Sim migration.md` for full analysis

#### Step 0.4: ✅ RESOLVED — Object Detection Ground Truth Approach
- **Decision: Visible 2D bounding box camera (`boundingbox_camera` sensor) with `vision_msgs/Detection2DArray` output**
- Rationale: The logical camera sees through walls (disqualifying for indoor exploration). The bounding box camera is rendering-based, handles occlusion natively, and outputs `vision_msgs/Detection2DArray` — the same message type YOLO and other real detectors publish. Swapping ground truth for real perception later is a one-line topic remap, not a refactor.
- Objects must be tagged with the `gz-sim-label-system` plugin (integer class ID per object type). Maintain a YAML config mapping IDs to class names (e.g., `{1: "fire_extinguisher", 2: "first_aid_kit"}`).
- Mount the bbox sensor co-located with the TurtleBot 4's OAKD camera, matching its FOV and resolution.
- Build a **detection source abstraction node** (`detection_source_node`): subscribes to either `/ground_truth/detections` (sim) or `/yolo/detections` (real) via a parameter, republishes on unified `/detections`. All downstream code subscribes only to `/detections`.
- Requires Ogre2 rendering even in headless mode (EGL offscreen on RTX 2070 Super hardware — fine).
- **Deliverable:** Research complete; see `docs/Ground-truth object detection in Gazebo.md` for full analysis

#### Step 0.5: ✅ RESOLVED — Coverage Calculation Method
- **Decision: Grid-based raycasting using Gazebo ground-truth pose (SLAM-decoupled)**
- Rationale: No off-the-shelf package exists for this stack. SLAM-comparison approach ties coverage quality to SLAM quality — unacceptable for a neutral metrics system. Gazebo publishes zero-drift ground-truth pose via `ros_gz_bridge`, enabling a fully decoupled metrics node.
- Implementation: `coverage_metrics_node` subscribes to `/scan` (LaserScan) and `/model/turtlebot4/pose` (TFMessage from Gazebo). Vectorizes 360 ray endpoints with NumPy, traces each with `skimage.draw.line()` (Cython Bresenham). Grid resolution: **0.5m** (upgrade to 0.25m if finer granularity needed — both run in real-time easily). Full scan processed in under 2ms.
- Ground truth format: **PGM + YAML** (standard ROS 2 map format). One file pair per world template. Load with `cv2.imread()`, convert to binary mask with `np.flipud()` (PGM origin is top-left; grid origin is bottom-left).
- Coverage % formula: `100 * np.count_nonzero(coverage_grid & gt_mask) / np.count_nonzero(gt_mask)` — runs in microseconds.
- Optionally publish coverage grid as `nav_msgs/OccupancyGrid` for RViz visualization (minimal cost, significant debug value).
- **Deliverable:** Research complete; see `docs/Computing exploration coverage in Gazebo.md` for full analysis

#### Step 0.6: Install Core Dependencies
- Install Ubuntu 24.04 LTS; install ROS 2 Jazzy following official guide
- Install Gazebo Harmonic following official guide (`ros-jazzy-ros-gz*` packages)
- Install `ros_gz_bridge`, `ros_gz_sim`, `robot_state_publisher` packages
- Install `teleop_twist_keyboard`
- Install NVIDIA Container Toolkit (for future multi-instance GPU isolation)
- Add user to `render` group for EGL headless rendering: `sudo usermod -aG render $USER`
- Verify base install: `gz sim -r --headless-rendering <world.sdf>` loads without error
- Verify DerpBot spawn: `ros2 launch launch/arst_sim.launch.py` + `ros2 launch launch/spawn_robot.launch.py`
- Teleoperate: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel`
- **Deliverable:** Working ROS 2 Jazzy + Gazebo Harmonic + DerpBot. Robot spawns, moves, publishes odom in both GUI and headless modes.

#### Step 0.7: Initialize Project Repository
- Create Git repository with directory structure from Section 3.4
- Set up Python package structure (`setup.py` or `pyproject.toml`)
- Add `.gitignore` for ROS 2, Python, Gazebo artifacts
- Create initial `README.md` with project overview
- (Optional) Create Dockerfile with all dependencies
- **Deliverable:** Initialized Git repository with clean structure

---

### Phase 1: World Template System (Weeks 3-5)

The goal of this phase is to build the world generation pipeline: load a template, place objects, generate a runnable Gazebo world.

#### Step 1.1: Create First World Template — Indoor Office
- Study Gazebo SDF world format and existing example worlds
- Design a simple office layout: rooms, corridors, doors, furniture
- Build the base SDF file with static geometry (walls, floor, ceiling)
- Ensure proper collision and visual meshes
- Test: launch in Gazebo, visually verify, walk robot through
- **Deliverable:** `worlds/templates/indoor_office/world.sdf` — a static office environment that loads in Gazebo

#### Step 1.2: Define Explorable Regions for Coverage
- Create a **PGM + YAML** ground truth map for the indoor office template (standard ROS 2 map format)
  - PGM image: white pixels = free space, black = walls/obstacles
  - YAML: specifies resolution (0.5m), origin (world coordinates of bottom-left corner), free/occupied thresholds
- This format is natively compatible with `nav2_map_server`, `cv2.imread()`, and the coverage metrics node
- Load at startup with `cv2.imread()`, convert to binary mask: `gt_mask = (img > 127).astype(np.uint8)` then `np.flipud()` (PGM top-left origin vs. grid bottom-left origin)
- For each world template, this is a one-time preprocessing step. Can be created manually in an image editor, or by surveying the world with the robot + SLAM and using that map as ground truth.
- **Deliverable:** `worlds/templates/indoor_office/ground_truth_map.pgm` + `ground_truth_map.yaml` — ground truth explorable area in ROS 2 map format

#### Step 1.3: Create Target Object Models
- Source or create SDF models for at least 3 target object types (e.g., fire extinguisher, first aid kit, hazard sign)
- Each model needs: visual mesh, collision mesh, and a `gz-sim-label-system` plugin with a unique integer class ID
- Maintain a YAML config mapping integer IDs to class names (e.g., `{1: "fire_extinguisher", 2: "first_aid_kit"}`) in `config/metrics/`
- Check Gazebo Fuel (model repository) for existing models first
- Ensure objects are appropriately scaled for the environment
- **Deliverable:** At least 3 object models in `worlds/models/`, each with SDF, meshes, and label plugin; class ID mapping YAML

#### Step 1.4: Build Object Placement Engine
- Write Python script that takes: world template + object configuration (types, counts, placement mode) + random seed
- Script reads the template's valid placement zones (from regions file or a separate placement config)
- Script generates randomized but valid positions (not inside walls, not overlapping, on floor surface)
- Script outputs a modified SDF world file (or SDF include directives) with objects placed
- **Deliverable:** `src/world_manager/object_placer.py` — given a config and seed, produces consistent object placements
- **Test:** Run with same seed twice, verify identical placement. Run with different seeds, verify variation.

#### Step 1.5: Build World Generator
- Write the main world generation script that combines: base template SDF + placed objects + any environment variations (lighting, door states)
- Output: a complete, launchable SDF world file
- Support configuration via YAML (the `world:` section of the scenario config)
- **Deliverable:** `src/world_manager/world_generator.py` — takes scenario config, produces final `.sdf` file
- **Test:** Generate world, launch in Gazebo, verify objects present at expected locations

#### Step 1.6: Add Environment Variations
- Implement lighting variations: adjust light sources in SDF (intensity, color temperature)
- Implement door state variations: doors open, closed, or randomized
- Implement clutter variations: different amounts of non-target objects (tables, chairs, boxes)
- All controlled via scenario YAML config
- **Deliverable:** Variation system working for lighting, doors, and clutter level

#### Step 1.7: Create Second World Template — Indoor Warehouse
- Design a warehouse-like layout: open areas, shelving units, aisles, loading docks
- Build SDF with static geometry
- Define explorable regions
- Test with object placement engine
- **Deliverable:** Second world template, verifying that the template system generalizes

---

### Phase 2: Robot Integration (Weeks 4-6, overlaps with Phase 1)

The goal of this phase is to get DerpBot fully operational in the simulation:
persistent Gazebo + spawn/despawn + teleoperation.

#### Step 2.1: Verify DerpBot Spawns and Moves
- Launch persistent sim: `ros2 launch launch/arst_sim.launch.py`
- Spawn DerpBot: `ros2 launch launch/spawn_robot.launch.py`
- Verify topics are publishing: `/derpbot_0/cmd_vel`, `/derpbot_0/odom`, `/tf`
- Test teleoperation: `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel`
- Verify robot moves, TF tree is correct (`ros2 run tf2_tools view_frames`)
- **Deliverable:** DerpBot spawns, moves on cmd_vel, publishes odom and TF

#### Step 2.2: Verify Spawn/Despawn Cycle
- Spawn robot: `ros2 launch launch/spawn_robot.launch.py`
- Despawn robot: `scripts/despawn_robot.sh`
- Respawn at a different pose
- Verify Gazebo remains running throughout and no state leaks
- **Deliverable:** Clean spawn/despawn cycle confirmed

#### Step 2.3: Create DerpBot Config File
- Document confirmed topic names, message types, and speed limits in `config/robots/derpbot.yaml`
- **Deliverable:** `config/robots/derpbot.yaml` (already created; update with verified values)

#### Step 2.4: Robot Interface Abstraction (already done)
- `robots/robot_interface.py` — abstract base class
- `robots/derpbot/derpbot.py` — DerpBot concrete implementation
- Adding a new robot = new sub-directory + subclass
- **Deliverable:** Completed in Phase 1 prep

#### Step 2.5: Test DerpBot in Generated Worlds
- Spawn DerpBot in both world templates (office, warehouse)
- Teleoperate through environment
- Verify: robot physics stable, no clipping through walls
- **Deliverable:** Confirmed DerpBot works correctly in generated worlds

#### Step 2.6: Add Sensors (deferred — not in baseline)
- Add LiDAR (2D scan) to DerpBot URDF when coverage metrics are implemented
- Add RGB/depth camera when object detection is implemented
- Follow the same spawn/bridge pattern: one bridge argument per sensor topic

---

### Phase 3: Metrics Framework and Scoring (Weeks 6-9)

The goal of this phase is to build the metrics collection, evaluation, and reporting system.

#### Step 3.1: Design Metrics Plugin Architecture
- Define a base metric class/interface with methods: `start()`, `update()`, `get_result()`, `reset()`
- Each metric subscribes to relevant ROS 2 topics in `start()`
- `update()` is called periodically or on message receipt
- `get_result()` returns the final computed value
- Metrics are instantiated based on scenario config
- **Deliverable:** `src/metrics/base_metric.py` with abstract interface and plugin registration mechanism

#### Step 3.2: Implement Meters Traveled Metric
- Subscribe to `/odom` (nav_msgs/Odometry)
- Accumulate Euclidean distance between consecutive position readings
- Handle: initial pose, teleport resets, noisy odometry
- **Deliverable:** `src/metrics/meters_traveled.py` — tested with manual teleoperation

#### Step 3.3: Implement Collision Count Metric
- Subscribe to bumper/contact sensor topic
- Count distinct collision events (debounce rapid repeated contacts)
- Log each collision with timestamp and location
- **Deliverable:** `src/metrics/collision_count.py`

#### Step 3.4: Implement Object Detection Tracking
- Add a `boundingbox_camera` sensor to the TurtleBot 4 URDF/SDF, co-located with the OAKD camera, using `box_type: visible_2d`
- Bridge to ROS 2 as `vision_msgs/Detection2DArray` on `/ground_truth/detections`
- Build the **detection source abstraction node** (`src/metrics/detection_source_node.py`): reads a `source` parameter (`ground_truth` or `yolo`), subscribes to the appropriate detections topic, republishes on unified `/detections` topic. All downstream nodes subscribe only to `/detections`.
- In the tracker node: subscribe to `/detections`, track per-object first-detection timestamp, total detection count, and distance at detection (using ground-truth pose via `/model/turtlebot4/pose`)
- Publish detection events on `/metrics/detection_events` for other metrics to consume
- **Deliverable:** `src/metrics/object_detection_tracker.py`, `src/metrics/detection_source_node.py`; bounding box sensor config in TB4 URDF

#### Step 3.5: Implement Detection-Based Metrics
- Object detection rate: detected count / total count
- Time to all detections: timestamp of last detection event
- Average time per detection: mean of detection timestamps
- False positive rate: placeholder (0.0 for ground truth oracle; ready for real perception integration)
- **Deliverable:** `src/metrics/detection_metrics.py`

#### Step 3.6: Implement Exploration Coverage Metric
- Implement `coverage_metrics_node` as a dedicated ROS 2 node (Python)
- Subscribes to `/scan` (sensor_msgs/LaserScan) and `/model/turtlebot4/pose` (tf2_msgs/TFMessage from Gazebo ground truth via `ros_gz_bridge`)
- Loads the world template's PGM + YAML ground truth mask at startup (path passed as ROS 2 parameter)
- Maintains a 2D NumPy coverage grid at **0.5m resolution**
- On each scan callback: vectorize 360 ray endpoints with NumPy, trace each ray with `skimage.draw.line()`, mark traversed cells as observed
- Publish `std_msgs/Float32` on `/metrics/coverage_percent` at 1 Hz
- Optionally publish coverage grid as `nav_msgs/OccupancyGrid` on `/metrics/coverage_grid` for RViz visualization
- Uses Gazebo ground-truth pose only — completely decoupled from the robot's SLAM/navigation stack
- **Deliverable:** `src/metrics/exploration_coverage.py`; requires `scikit-image` in `requirements.txt`

#### Step 3.7: Implement Revisit Ratio Metric
- Track visited cells over time
- Compute: cells visited more than once / total cells visited
- **Deliverable:** `src/metrics/revisit_ratio.py`

#### Step 3.8: Build Success/Failure Evaluator
- Read success criteria from scenario config
- After scenario ends (or continuously during run), evaluate conditions against current metric values
- Support `all_of` and `any_of` logical modes
- Support comparison operators: `>=`, `<=`, `==`, `>`, `<`
- Return: overall pass/fail, per-condition results
- **Deliverable:** `src/metrics/evaluator.py`

#### Step 3.9: Build Metrics Reporter
- Collect all metric results at scenario end
- Write structured output (JSON and/or CSV):
  - Per-run summary: all metric values, pass/fail, scenario config, timestamps
  - Detailed logs: trajectory, detection events, coverage progression over time
- **Deliverable:** `src/metrics/reporter.py` — produces result files in `results/` directory
- **Test:** Run a full scenario manually (teleop), verify all metrics are recorded correctly

#### Step 3.10: Implement Near-Miss Detection
- Track minimum distance between robot and obstacles/objects over time
- Flag events where distance drops below configurable threshold (default: 0.3m) without making contact
- Log near-miss events with timestamp, location, and proximity distance
- **Deliverable:** `src/metrics/near_miss_tracker.py`

#### Step 3.11: Build Composite Scoring Engine
- Implement the scoring system defined in Section 5.4
- For each category (Speed, Accuracy, Safety, Efficiency), compute weighted score from input metrics
- Compute overall score as weighted combination of category scores
- Map numeric scores to letter grades (S/A/B/C/D/F)
- All weights, par values, and thresholds loaded from scenario config with sensible defaults
- **Deliverable:** `src/metrics/scoring.py`
- **Test:** Feed known metric values, verify scores compute correctly. Edge cases: perfect run (all S), worst run (all F), timeout failure.

#### Step 3.12: Build Scorecard Display
- Generate ASCII scorecard for terminal output (see Section 5.4 for example format)
- Include: category scores with bar visualization, letter grades, overall score, and raw metrics summary
- Also write scorecard to results JSON for programmatic access
- For batch runs: show comparison table across runs with score distributions
- **Deliverable:** Scorecard integrated into `src/metrics/reporter.py`; displays after each run

#### Step 3.13: Calibrate Par Values
- Run the first world template (indoor office) 5-10 times with teleoperation
- Record all raw metrics
- Set par values based on observed performance: par should represent "competent but not optimal" teleoperation
- Document calibration methodology so it can be repeated for new world templates
- **Deliverable:** Calibrated default par values in `config/scenarios/` example files; calibration procedure documented

---

### Phase 4: Scenario Runner and CLI (Weeks 9-11)

The goal of this phase is to build the orchestration layer that ties everything together and enables headless operation.

#### Step 4.1: Build Scenario Configuration Loader
- Parse scenario YAML config file
- Validate required fields, types, and value ranges
- Resolve references (template names to file paths, robot names to configs)
- Report clear error messages for invalid configs
- **Deliverable:** `src/utils/config_loader.py` — validated config objects

#### Step 4.2: Build Simulation Launcher
- Programmatically launch Gazebo in headless mode (no GUI) with the generated world
- Launch ROS 2 nodes (bridges, robot state publisher, etc.) via launch file or subprocess
- Handle startup sequencing: Gazebo must be ready before spawning robot
- Implement health checks: verify key topics are publishing before declaring "ready"
- **Deliverable:** `src/scenario_runner/launcher.py`

#### Step 4.3: Build Scenario Lifecycle Manager
- Implement the full scenario lifecycle: configure → generate world → launch sim → spawn robot → start metrics → run → evaluate → stop → report
- Implement timeout handling
- Implement clean shutdown: stop metrics, kill Gazebo and ROS processes, release resources
- **Deliverable:** `src/scenario_runner/runner.py`

#### Step 4.4: Build CLI Interface
- Command-line entry point: `python -m arst.run --scenario config/scenarios/my_scenario.yaml`
- Support flags: `--headless` (default: true), `--gui` (launch with Gazebo GUI for debugging), `--seed` (override random seed), `--timeout` (override timeout), `--output-dir` (override output directory)
- Support batch mode: `--batch config/scenarios/batch.yaml` runs multiple scenarios sequentially
- **Deliverable:** CLI that runs a full scenario from a single command

#### Step 4.5: Implement Scenario Reset
- After a scenario completes, cleanly tear down and allow re-launch
- For batch mode: reset between scenarios without restarting the entire process (if possible; may require full restart)
- **Deliverable:** Reset functionality integrated into runner

#### Step 4.6: Implement Batch Execution and Aggregation
- Run N repetitions of a scenario with different random seeds
- Aggregate metrics across runs: compute mean, standard deviation, min, max for each metric
- Output aggregate summary alongside per-run results
- **Deliverable:** Batch execution script + aggregation script

#### Step 4.7: End-to-End Integration Test
- Define a reference scenario with known expected behavior
- Run it headless from CLI
- Verify: world generates correctly, robot spawns, teleoperation works (or use a scripted movement pattern), metrics are recorded, success/failure evaluates correctly, results are written
- Fix any integration issues
- **Deliverable:** Passing end-to-end test; documented test procedure

---

### Phase 5: Documentation and Polish (Weeks 11-12)

#### Step 5.1: Write Installation Guide
- Step-by-step: OS setup, ROS 2 installation, Gazebo installation, project dependencies, verification
- Include troubleshooting for common issues (GPU drivers, ROS 2 environment sourcing, Gazebo model paths)
- **Deliverable:** `docs/installation.md`

#### Step 5.2: Write User Guide
- How to configure a scenario (YAML reference)
- How to run a scenario (CLI reference)
- How to interpret results
- How to add objects to the world
- Example walkthrough: run the office exploration scenario
- **Deliverable:** `docs/user_guide.md`

#### Step 5.3: Write Developer Guide
- Architecture overview with diagrams
- How to add a new world template
- How to add a new robot platform
- How to add a new metric
- How to add a new use case / success criteria type
- Code conventions and contribution guidelines
- **Deliverable:** `docs/developer_guide.md`

#### Step 5.4: Write Architecture Document
- Design decisions and rationale (including research task outcomes)
- Abstraction layer descriptions
- Data flow diagrams
- Future extension points
- **Deliverable:** `docs/architecture.md`

#### Step 5.5: Create Example Scenarios
- At least 2 example scenarios with different templates and configurations
- Include expected approximate metric ranges (from manual testing)
- Serve as both documentation and regression tests
- **Deliverable:** Example YAML files in `config/scenarios/` + documented expected results

---

### Phase 6: Future Extensions (Post Phase 5, not scheduled)

These are documented here for planning purposes. Each would be its own project phase.

#### 6.1 Autonomy Integration
- Integrate SLAM (slam_toolbox or RTAB-Map)
- Integrate navigation (Nav2 stack)
- Implement frontier-based exploration
- Integrate object detection (YOLO or similar)
- Replace ground-truth detection oracle with perception-based detection

#### 6.2 Additional Robot Platforms
- Spot robot model (requires sourcing or creating URDF/SDF)
- Quadcopter drone (e.g., PX4 SITL + Gazebo integration)
- Humanoid (depending on availability of models)
- Multi-robot scenarios (multiple robots in same world)

#### 6.3 Additional Use Cases
- Search and rescue: find "victims" (mannequin models), report locations
- Security patrol: detect anomalies (moved objects, open doors that should be closed)
- Inspection: systematic coverage of surfaces, anomaly detection
- Nuclear/hazmat: navigate contaminated zones, avoid hazard areas, take measurements
- Multi-robot relay: drone scouts, ground robot retrieves

#### 6.4 Isaac Sim Integration (Parallel Operation, not Migration)
- **Strategy: parallel operation, not wholesale migration.** Maintain Gazebo as primary testbed; use Isaac Sim for tasks where it uniquely excels.
- **Near-term opportunities (no migration needed):**
  - RL policy training in Isaac Sim → validate in Gazebo → deploy on real robot (established pattern)
  - Photorealistic synthetic data generation for training visual detectors
- **Robot model path:** Start from the iRobot Create 3 USD model in Isaac Sim's asset library (TB4's base platform, already correctly configured). Add TB4-specific components (OAK-D camera, RPLIDAR A1, shell) by importing from TB4 URDF. Community workflow documented (Jan 2025 blog post by Samuel Ho).
- **World environment path:** Rebuild using NVIDIA's asset library (warehouse, office templates) as starting point. No automated SDF→USD conversion — budget months of effort for full environment rebuild.
- **Tooling to monitor:** NVIDIA Simulation Interfaces standard + Newton project (DeepMind/NVIDIA) USD tooling. Expected to mature within 1–2 years, potentially making SDF↔USD conversion a solved problem.
- **Prerequisite:** Author all robot descriptions in URDF (done from Phase 2). Keep world geometry SDF files separate from plugin/behavior config.

#### 6.5 Advanced Simulation Features
- Parallel multi-instance execution (GPU partitioning or sequential GPU sharing)
- Dynamic environments (moving people, opening/closing doors during scenario)
- Weather and time-of-day effects for outdoor templates
- Degraded conditions (sensor noise, communication dropouts, actuator failures)

---

## 7. Risk Register

| Risk | Likelihood | Impact | Mitigation |
|------|-----------|--------|-----------|
| DerpBot URDF → SDF conversion produces unexpected physics (inertia, collision) | Low | Medium | Use `check_urdf` and inspect SDF via `gz model --list` after spawn. Tune inertia values if robot tips over or spins in place. |
| Headless Gazebo has undocumented issues or limitations | Low | High | Test headless operation early (Step 0.6); Gazebo Fortress/Harmonic both officially support headless |
| GPU insufficient for simulator + metrics collection | Low | Medium | RTX 2070 Super is adequate for Gazebo; monitor GPU usage early; disable unnecessary rendering in headless mode |
| SDF to USD conversion is lossy or manual | Low | Low | Resolved by strategy: parallel operation, not migration. Gazebo remains primary testbed. Robot descriptions authored in URDF (consumed by both simulators). No SDF→USD automation dependency in project plan. Revisit when Newton/Simulation Interfaces tooling matures (~1–2 years). |
| Coverage calculation is computationally expensive at high resolution | Medium | Low | Use coarse grid (0.5m cells) initially; optimize later if needed |
| Object placement in complex worlds produces invalid positions (inside walls) | Medium | Medium | Use predefined valid placement zones per template; implement collision checking in placer |
| ROS 2 topic naming or message format changes between ROS 2 versions | Low | Medium | Pin to specific ROS 2 release (Humble); document all topic names |
| Xeon E5-2630 v4 single-thread performance limits simulation speed | Medium | Low | Broadwell-era CPUs have ~60% the single-thread performance of modern cores. Gazebo physics stepping is largely single-threaded. May limit faster-than-real-time simulation. Mitigated by 40 total threads for parallel workloads and the fact that real-time simulation is sufficient for Phase 1 |
| ML inference performance limited by older CPU (no AVX-512) and RTX 2070 Super VRAM (8 GB) | Low | Medium | Only relevant in future phases when neural network perception is added. RTX 2070 Super handles YOLO-class models well. Larger models (foundation models) may require model optimization or offloading. Flag early if planning to use large vision-language models |
| Composite scoring par values are poorly calibrated, making scores meaningless | Medium | Medium | Mitigated by Step 3.13 (calibration runs). Par values are configurable per scenario, so they can be tuned iteratively. Document calibration methodology. |

---

## 8. Timeline Summary

| Phase | Description | Estimated Duration | Dependencies |
|-------|------------|-------------------|-------------|
| Phase 0 | Research and Foundation | Weeks 1-2 | None |
| Phase 1 | World Template System | Weeks 3-5 | Phase 0 |
| Phase 2 | Robot Integration | Weeks 4-6 | Phase 0 (Step 0.6) |
| Phase 3 | Metrics Framework + Scoring | Weeks 6-9 | Phase 1, Phase 2 |
| Phase 4 | Scenario Runner and CLI | Weeks 9-11 | Phase 1, 2, 3 |
| Phase 5 | Documentation and Polish | Weeks 11-12 | Phase 4 |

**Total estimated duration: 12 weeks** for a functional Phase 1 system (simulation testbed with Turtlebot 4, two world templates, metrics framework with composite scoring, CLI operation, and documentation). This assumes approximately half-time effort; full-time effort could compress to 7-8 weeks.

Note: Phases 1 and 2 overlap intentionally — robot integration does not depend on the full world template system, only on a working Gazebo installation.

---

## 9. Glossary

| Term | Definition |
|------|-----------|
| SDF | Simulation Description Format — XML format used by Gazebo to describe worlds, models, and sensors |
| URDF | Unified Robot Description Format — XML format for describing robot kinematics, used by ROS |
| USD | Universal Scene Description — file format used by NVIDIA Omniverse/Isaac Sim |
| ROS 2 | Robot Operating System 2 — middleware for robot software, providing pub/sub messaging, service calls, and tools |
| nav2 | ROS 2 Navigation stack — provides autonomous navigation capabilities |
| SLAM | Simultaneous Localization and Mapping |
| FoV | Field of View |
| MoSCoW | Must have, Should have, Could have, Won't have — prioritization framework |
| Gazebo Fuel | Online repository of 3D models for Gazebo simulation |
| ARST | Autonomous Robotics Simulation Testbed — working name for this project |
