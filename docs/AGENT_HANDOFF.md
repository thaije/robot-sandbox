# Agent Handoff — ARST

Gazebo Harmonic + ROS 2 Jazzy simulation testbed. DerpBot (custom diff-drive).
Full plan + remaining work: [`docs/ARST_Project_Plan.md`](ARST_Project_Plan.md).

---

## What works (verified)

- **Full scenario pipeline**: `./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless`
  - WorldGenerator assigns unique per-instance labels (1..N); exposes `label_map` to metrics
  - Live metrics: `meters_traveled`, `collision_count` (rising-edge, not debounce), `revisit_ratio`, detection metrics
  - Scorecard: Speed / Accuracy / Safety / Efficiency / **Effectiveness** (per-type weights)
  - Ends on `SUCCESS` (all instances found) or `TIME_LIMIT` (600 s)
  - Scorecard printed + JSON written to `results/`
- **Manual sim**: persistent Gazebo + `spawn_robot.launch.py` + teleoperation
- **World**: indoor_office (20×15 m, 4 rooms, furniture, randomised object placement)
- **Objects**: fire_extinguisher (×3), first_aid_kit (×2), hazard_sign (×4) — 9 instances total with unique labels
- **LiDAR**: `/derpbot_0/scan` → `sensor_msgs/LaserScan` @ 9.8 Hz, 720 samples, 360°, 0.15–12 m, `frame_id=lidar_link`
- **IMU**: `/derpbot_0/imu` → `sensor_msgs/Imu` @ 100 Hz, `frame_id=imu_link`, **BEST_EFFORT QoS** (subscribe with `ReliabilityPolicy.BEST_EFFORT`)
- **Exploration coverage** (MET-06): `ExplorationCoverage` in `src/metrics/exploration_coverage.py` — LiDAR raycasting + odom-based pose (world frame = spawn offset + odom transform), Bresenham cells onto 40×30 binary grid, coverage % against PGM free-space mask. Registered in `_build_metrics` / `_avg_metrics`.

**Running as agent:** `./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless --timeout 300` (startup ~5s).   
Navigation → invoke the `/arst-nav` skill. It has all documentation.

---

## Key gotchas

- **Python interpreter**: Always `python3.12`. `python3` may resolve to another venv. `run_scenario.sh` handles this.
- **gz-transport Python bindings not in venv**: `gz.transport13` lives in `/usr/lib/python3/dist-packages/` (system), excluded when the project venv is active. Use ROS odom topic + spawn offset instead of `gz_subscribe_robot_pose` for metrics that need world-frame pose.
- **`use_sim_time`**: rclpy metrics node must be created with `use_sim_time=True` or messages are silently dropped as future-dated.
- **Contact sensor topic**: `<contact><topic>` (NOT `<sensor><topic>`) controls what gz transport publishes. Must be `/ROBOT_NAME/bumper_contact` inside the `<contact>` block. If absent, `gz sdf -p` sets it to a scoped default that doesn't match the bridge.
- **URDF→SDF joint lumping**: `gz sdf -p` lumps `base_footprint→base_link`. Collision name becomes `base_footprint_fixed_joint_lump__base_link_collision_collision`. Contact sensor reference must use the post-lumping name.
- **Dual casters**: Robot has front caster (x=+0.13) AND rear caster (x=-0.13) — required for pitch stability under acceleration. Removing either causes the robot to tilt when driving.
- **Camera wall-clipping**: camera_joint x moved from 0.15 → 0.05 m (10 cm behind front collision face). Camera is well inside the robot body and cannot reach a wall surface, so it physically cannot look through walls. The software LOS check in `ObjectDetectionTracker` and `world_state.py` adds defence-in-depth using Bresenham ray-casting from the camera world position.
- **Dynamic spawn contact sensors (gz-sim 8.10.0 bug)**: `EachNew<ContactSensor>` never fires for dynamically spawned models. Fixed by embedding robots in world SDF at generation time — already implemented in `WorldGenerator._embed_robots()`.
- **Process cleanup**: `SimulationLauncher.shutdown()` captures pgids *before* SIGTERM (process may exit before SIGKILL). Uses `os.killpg()` to kill entire process groups. Runner wrapped in `try/finally` so shutdown always runs.
- **Scenario config schema**: `robots:` is a list (not `robot:`). Each entry: `{platform, name, spawn_pose: {x,y,z,yaw}}`.
- **Parallel sessions**: Isolate with `ROS_DOMAIN_ID=N ./scripts/run_scenario.sh ...`. gz transport is isolated per process; only ROS DDS needs the domain ID.
- **Door states**: `door_states` YAML field is a stub — not implemented. Doors are always open (physical wall gaps, no door models). Closed doors require injecting box panels at generation time.
- **IMU system plugin**: IMU is a physics-based sensor — it needs `gz-sim-imu-system` in `world.sdf` in addition to `gz-sim-sensors-system` (which handles GPU-rendered sensors like LiDAR). Missing it → IMU sensor initialises but never publishes.
- **IMU QoS**: `ros_gz_bridge` bridges IMU as **BEST_EFFORT**. Subscribers must use `ReliabilityPolicy.BEST_EFFORT` or they receive no messages.

---

## Architecture

```
ScenarioRunner.run()
  ├── WorldGenerator.generate()    # SDF from template + objects + embedded robots (gz sdf -p)
  ├── SimulationLauncher.launch()  # gz sim + RSP + bridges (no dynamic spawn)
  ├── rclpy metrics node (thread)  # MetersTraveled, CollisionCount, RevisitRatio per robot
  ├── _run_until_done()            # poll success criteria every 2s
  ├── ScoringEngine.compute()      # Speed / Accuracy / Safety / Efficiency
  ├── render_scorecard()
  └── write_results()              # JSON to results/
```

**Key design decisions:**
- `ROBOT_NAME` placeholder in URDF replaced at SDF generation time → unique topics per instance
- Robots embedded in world SDF (not dynamically spawned) — required for contact sensors
- `spawn_robot.launch.py` with `spawn:=false` starts RSP + bridges only (used by automated runs; robot already in world SDF). True dynamic spawn works for teleop/odom/TF but contact sensor won't fire.
- `boundingbox_camera` for ground-truth detection — handles occlusion, outputs `vision_msgs/Detection2DArray` (same as YOLO → easy swap later)
- Coverage: raycasting at 0.5m res using `skimage.draw.line()` + Gazebo ground-truth pose (zero drift, SLAM-decoupled)
- TF frames unprefixed (`odom → base_footprint → base_link`) — fine for single robot

---

## Key files

```
launch/
  arst_sim.launch.py            # Gazebo; accepts world_sdf override arg
  spawn_robot.launch.py         # RSP + bridges; spawn:=false for automated runs (robot in world SDF)
scripts/
  run_scenario.sh               # main entry point; handles python3.12, cleanup, results
  robot_control.py              # drive / status / snapshot (no detections — dev/cheat tool)
  world_state.py                # PNG map + object positions + found/visible status (dev/cheat tool)
  despawn_robot.sh
robots/derpbot/urdf/derpbot.urdf  # ROBOT_NAME placeholder; contact sensor; no LiDAR yet
config/
  robots/derpbot.yaml           # robot platform config
  scenarios/office_explore_detect.yaml  # main scenario; see also environment_variations.md
worlds/
  templates/indoor_office/      # world.sdf + config.yaml + ground_truth_map.pgm/.yaml
  models/                       # fire_extinguisher, first_aid_kit, hazard_sign SDF models
src/
  metrics/                      # base_metric, evaluator, reporter, scoring, meters_traveled,
                                #   collision_count, revisit_ratio, detection_metrics,
                                #   object_detection_tracker; exploration_coverage stub (needs LiDAR)
  scenario_runner/              # launcher, runner, __main__
  world_manager/                # world_generator, object_placer, template_loader
  utils/                        # config_loader, ros_helpers, logging_setup
docs/
  ARST_Project_Plan.md          # vision, requirements, remaining steps, scoring reference
  environment_variations.md     # robustness variation designs + priority order
```
