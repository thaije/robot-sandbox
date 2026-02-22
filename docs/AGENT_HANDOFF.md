# Agent Handoff — ARST Project

## What this project is

Autonomous Robotics Simulation Testbed (ARST). Gazebo Harmonic + ROS 2 Jazzy simulation environment for testing robot autonomy. Full plan: `docs/ARST_Project_Plan.md`.

**Stack:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Python 3.12.

---

## Current state

### What works end-to-end (verified running)

- **Persistent Gazebo** — `ros2 launch launch/arst_sim.launch.py [headless:=true]`
- **Spawn / despawn DerpBot** — `ros2 launch launch/spawn_robot.launch.py [name:=derpbot_0 x:= y:= yaw:=]` / `scripts/despawn_robot.sh`
- **Teleoperation** — `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel`
- **Bridges** — `/derpbot_0/cmd_vel`, `/derpbot_0/odom`, `/tf`, `/derpbot_0/joint_states` all live
- **Indoor office world** — 20×15 m, 4 rooms, furniture, correct physics
- **Ground-truth map** — `worlds/templates/indoor_office/ground_truth_map.pgm` + `.yaml` (0.5 m resolution)
- **Object models** — `fire_extinguisher` (class 1), `first_aid_kit` (class 2), `hazard_sign` (class 3) — all SDF with label plugins

### Full scenario pipeline (verified running, 2026-02)

Run via:
```bash
./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless [--timeout N] [--gui]
```

The script handles `python3.12` and `PYTHONPATH` automatically. Full lifecycle:
1. Generate world SDF (WorldGenerator)
2. Launch Gazebo with the generated SDF (SimulationLauncher → `ros2 launch` subprocess)
3. Despawn any stale robot model with the same name (prevents ghost accumulation)
4. Spawn DerpBot at the configured pose
5. Collect `meters_traveled`, `collision_count`, and `revisit_ratio` metrics via rclpy subscriptions
6. Poll success criteria every 2 s until met or timeout
7. Evaluate, score all four categories (Speed/Accuracy/Safety/Efficiency), print ASCII scorecard, write JSON results
8. Despawn robot from Gazebo; kill entire Gazebo + bridge process tree (no orphans)

### DerpBot robot

Two-wheeled diff-drive with a contact/bumper sensor referenced from `base_footprint` (post-lumping). URDF at `robots/derpbot/urdf/derpbot.urdf`. Uses `ROBOT_NAME` placeholder substituted at spawn time so topic names are per-instance (`/derpbot_0/cmd_vel` etc.). TF frames are unprefixed (`odom → base_footprint → base_link`). LiDAR and camera to be added in later steps.

Yellow strip on the front face (+x) marks the forward direction. Contact sensor fires on collisions from any side (the body collision covers the full chassis box).

### Implementation status of key modules

| File | Status | Notes |
|------|--------|-------|
| `src/metrics/base_metric.py` | COMPLETE | Abstract base; plugin interface |
| `src/metrics/meters_traveled.py` | COMPLETE | Odom subscription; registered in runner |
| `src/metrics/collision_count.py` | COMPLETE | Contacts subscription on `/{robot}/bumper_contact`; registered in runner |
| `src/metrics/revisit_ratio.py` | COMPLETE | Odom subscription; registered in runner |
| `src/metrics/exploration_coverage.py` | STUB | All methods raise NotImplementedError — needs LiDAR |
| `src/metrics/detection_metrics.py` | STUB | Logic written but not wired; needs camera sensor |
| `src/metrics/evaluator.py` | COMPLETE | Success/failure evaluation fully works |
| `src/metrics/scoring.py` | COMPLETE | All four category scorers + overall + Scorecard |
| `src/metrics/reporter.py` | COMPLETE | ASCII scorecard + JSON output work |
| `src/scenario_runner/launcher.py` | COMPLETE | launch(), shutdown() kills full process groups (pgid fix) |
| `src/scenario_runner/runner.py` | COMPLETE | run(), _build_metrics() registers meters_traveled, collision_count, revisit_ratio |
| `src/scenario_runner/__main__.py` | COMPLETE | CLI entry point; argparse wired to runner |
| `src/world_manager/object_placer.py` | COMPLETE | Random placement with collision avoidance |
| `src/world_manager/world_generator.py` | COMPLETE | generate(), lighting + object injection |
| `src/utils/config_loader.py` | COMPLETE | YAML load + validation |
| `launch/arst_sim.launch.py` | COMPLETE | Added `world_sdf` override arg |

---

## Where we are in the plan

| Phase | Step | Status |
|-------|------|--------|
| 0 | 0.1–0.7 Research + Foundation | ✅ Done |
| 1 | 1.1 Indoor office world | ✅ Done |
| 1 | 1.2 Ground-truth map | ✅ Done |
| 1 | 1.3 Target object models | ✅ Done |
| 1 | 1.4 Object placement engine | ✅ Done |
| 1 | 1.5 World generator | ✅ Done |
| 1 | 1.6 Environment variations | ⬜ Not started |
| 1 | 1.7 Warehouse template | ⬜ Config file only |
| 2 | 2.1–2.5 Robot integration | ✅ Done (DerpBot replaces TB4) |
| 3 | 3.1 Metrics plugin arch | ✅ Done |
| 3 | 3.2 Meters traveled | ✅ Done |
| 3 | 3.3 Collision count | ✅ Done — bumper sensor on DerpBot, bridge live |
| 3 | 3.4 Object detection tracking | ⬜ Not started (needs bounding box camera on DerpBot) |
| 3 | 3.5 Detection metrics | ⬜ Stub wired, not registered (needs camera) |
| 3 | 3.6 Exploration coverage | ⬜ Stub (needs LiDAR sensor on DerpBot) |
| 3 | 3.7 Revisit ratio | ✅ Done — uses odom, registered in runner |
| 3 | 3.8 Success/failure evaluator | ✅ Done |
| 3 | 3.9 Metrics reporter | ✅ Done |
| 3 | 3.10 Near-miss detection | ⬜ Not started |
| 3 | 3.11 Composite scoring | ✅ Done |
| 3 | 3.12 Scorecard display | ✅ Done (in reporter.py) |
| 3 | 3.13 Calibrate par values | ⬜ After first successful run |
| 4 | 4.1 Config loader | ✅ Done |
| 4 | 4.2 Simulation launcher | ✅ Done |
| 4 | 4.3 Scenario lifecycle manager | ✅ Done |
| 4 | 4.4 CLI entry point | ✅ Done |
| 4 | 4.5 Scenario reset | ⬜ Stub |
| 4 | 4.6 Batch execution | ⬜ Not started |
| 4 | 4.7 End-to-end integration test | ✅ Done |
| 5 | Documentation | ⬜ Not started |

---

## Known issues fixed (2026-02)

- **`python3` wrong interpreter** — `python3` resolves to a text-generation-webui venv (Python 3.11). ROS 2 Jazzy's `rclpy` C extension requires Python 3.12. `run_scenario.sh` uses `python3.12` explicitly.
- **Ghost robot models accumulating** — `SimulationLauncher.shutdown()` was killing the `ros2 launch` process but not removing the Gazebo model. After 3 runs, 3 `derpbot_0` instances were all publishing on `/derpbot_0/odom`. Fixed: `_despawn_if_exists()` called at both pre-spawn and shutdown.
- **Orphaned `gz sim` / bridge processes (round 1)** — `proc.terminate()` on the `ros2 launch` parent did not kill the `gz sim` grandchild. Fixed: both launch subprocesses use `start_new_session=True`; `shutdown()` uses `os.killpg()` to kill the entire process group.
- **Orphaned `gz sim` processes (round 2)** — After SIGTERM, `ros2 launch` exits quickly; subsequent `os.getpgid(proc.pid)` then raises `ProcessLookupError`, skipping SIGKILL and leaving `gz sim` alive. Fixed: pgids are captured *before* sending any signal and reused for SIGKILL. Verified zero lingering processes.
- **`meters_traveled` explosion (216,344 m)** — consequence of ghost models; consecutive odom messages from robots at different world positions created huge apparent movement. Fixed by the above.

## Fixed (2026-02) — metrics now update during automated runs

**Root cause:** `use_sim_time` mismatch. The bridge runs with `use_sim_time=true`, stamping odom
messages with Gazebo sim time. The rclpy metrics node was created without `use_sim_time`, so
messages appeared to be in the future and were silently dropped.

**Fix applied** in `src/scenario_runner/runner.py`:
```python
node = rclpy.create_node(
    "arst_metrics",
    parameter_overrides=[Parameter("use_sim_time", Parameter.Type.BOOL, True)],
)
```
Also added `time.sleep(2.0)` after spin thread start for DDS discovery.

**Verified:** `meters_traveled: 20.401` for a 40s run at 0.5 m/s (was 0.0 before fix).

---

## Fixed (2026-02) — collision_count always 0 / contact sensor "No publishers"

**Symptom:** `collision_count: 0` in every scorecard even after repeated wall collisions.
`gz topic -t /derpbot_0/bumper_contact --info` showed "No publishers" — the sensor entity
existed in the ECS but never initialized its gz transport publisher.

**Root cause:** URDF→SDF fixed-joint lumping. `gz sdf -p` merges `base_footprint` and
`base_link` (the fixed joint between them is lumped), renaming the body collision from
`base_link_collision` to `base_footprint_fixed_joint_lump__base_link_collision_collision`.
The contact sensor's `<contact><collision>` still referenced `base_link_collision` (the
pre-lumping name). Gazebo's contact system could not find the collision entity, so it
silently skipped publisher initialization.

Note: adding a tiny inertial (`mass=0.001`) to `base_footprint` does NOT prevent lumping —
`gz sdf -p` ignores it and lumps anyway.

**Fix applied** in `robots/derpbot/urdf/derpbot.urdf`:
```xml
<!-- Before (wrong — pre-lumping name): -->
<gazebo reference="base_link">
  <sensor name="bumper" type="contact">
    <contact>
      <collision>base_link_collision</collision>
    </contact>
  </sensor>
</gazebo>

<!-- After (correct — post-lumping name): -->
<gazebo reference="base_footprint">
  <sensor name="bumper" type="contact">
    <contact>
      <collision>base_footprint_fixed_joint_lump__base_link_collision_collision</collision>
    </contact>
  </sensor>
</gazebo>
```

**How to re-derive the correct name** if the URDF changes:
```bash
gz sdf -p robots/derpbot/urdf/derpbot.urdf | grep "collision name="
```
The body collision will always be the one on `base_footprint` with the `lump__` prefix.

**Status:** Fix applied, awaiting user verification (next test run).

---

## Recommended next steps (in order)

### 1. Add bounding box camera to DerpBot (unlock detection metrics + scenario SUCCESS)

This is the highest-priority step: without it `object_detection_rate` is always `[MISSING]` and the scenario can never reach SUCCESS.

Add a `boundingbox_camera` sensor to `robots/derpbot/urdf/derpbot.urdf` — a Gazebo rendering sensor that outputs `vision_msgs/Detection2DArray` (handles occlusion; objects behind walls are NOT detected). This is a ground-truth oracle in Phase 1; swap for real perception later with a single topic remap.

Steps:
1. Add camera link + Gazebo sensor element to URDF:
   ```xml
   <gazebo reference="camera_link">
     <sensor name="bbox_camera" type="boundingbox_camera">
       <topic>/ROBOT_NAME/detections</topic>
       <update_rate>10</update_rate>
       <camera>...</camera>
     </sensor>
   </gazebo>
   ```
2. Add bridge in `spawn_robot.launch.py`:
   ```
   /{robot_name}/detections@vision_msgs/msg/Detection2DArray[gz.msgs.AnnotatedAxisAligned2DBox_V
   ```
3. Implement `src/metrics/object_detection_tracker.py` (file does not exist yet) — tracks first-detection timestamp per object class ID.
4. Wire `DetectionMetrics` + `ObjectDetectionTracker` into `runner._build_metrics()`.
5. Set `total_targets` from the scenario config (`world.objects` count).

### 2. Add 2D LiDAR to DerpBot (unlock exploration_coverage)

Add a LiDAR link + `gpu_lidar` sensor element to URDF, bridge `/derpbot_0/scan`, and add a ground-truth pose bridge (requires `PosePublisher` plugin in URDF):
```
/{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
/model/{robot_name}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose
```

Then implement `src/metrics/exploration_coverage.py` (currently a full stub):
- Subscribe to scan + ground-truth pose
- Grid-based raycasting with `skimage.draw.line()` onto the PGM ground-truth map (0.5 m resolution)
- See `docs/ARST_Project_Plan.md` §5.2 for full algorithm spec

Note: `revisit_ratio` is already live (uses odom). Once LiDAR pose bridge is added, `revisit_ratio` could optionally be upgraded to use ground-truth pose — not required.

### 3. Calibrate par values (Step 3.13)

After the first manual teleoperation run with meters_traveled and coverage working:
1. Record typical values: meters traveled, coverage %, time taken
2. Update `config/scenarios/office_explore_detect.yaml` `par_values` so a competent operator scores ~70 (B)
3. Document in the scenario YAML comments

---

## Architecture of the full pipeline

```
__main__.py (CLI)
  └── ScenarioRunner.run()
        ├── WorldGenerator.generate()          # SDF generation
        │     ├── TemplateLoader.load()         # reads config.yaml + world.sdf
        │     ├── ObjectPlacer.place()          # deterministic object placement
        │     ├── _apply_lighting()             # XML mutation
        │     └── _inject_objects()             # XML mutation
        ├── SimulationLauncher.launch()         # subprocess management
        │     ├── ros2 launch arst_sim.launch.py world_sdf:=<path>
        │     └── ros2 launch spawn_robot.launch.py name:=derpbot_0 ...
        ├── rclpy metrics node (spin in thread)
        │     ├── MetersTraveled  → /derpbot_0/odom subscription
        │     └── CollisionCount  → /bumper_contact subscription
        ├── _run_until_done()                   # poll evaluate_criteria every 2s
        ├── evaluate_criteria()                 # pass/fail check
        ├── ScoringEngine.compute()             # Speed/Accuracy/Safety/Efficiency
        ├── render_scorecard()                  # ASCII output
        └── write_results()                     # JSON to results/
```

## Key design decisions already made

- **ROBOT_NAME placeholder in URDF** — `robots/derpbot/urdf/derpbot.urdf` uses the literal string `ROBOT_NAME` in all Gazebo topic names. `spawn_robot.launch.py` calls `.replace("ROBOT_NAME", robot_name)` before spawning. This gives unique absolute topic names per instance without requiring `parameter_bridge` remappings.

- **No sensors on DerpBot yet** — sensors (LiDAR, camera) are added later as separate steps. The `Sensors` plugin is in `world.sdf` but DerpBot has no sensor links. Add sensors by adding links + `<gazebo>` sensor elements to the URDF and additional bridge arguments in `spawn_robot.launch.py`.

- **DerpBot replaces Turtlebot4** — all TB4 packages/configs are removed. DerpBot is a custom URDF with no external package dependencies.

- **Bridge direction syntax** — `]gz.msgs.Type` = ROS→GZ, `[gz.msgs.Type` = GZ→ROS. The bridge topic name is the Gazebo topic name (same as ROS 2 side when using absolute topics).

- **TF frames are unprefixed** — `odom`, `base_footprint`, `base_link` (not `derpbot_0/odom` etc.). Works for single robot; multi-robot would need prefixed frames.

- **Ground truth uses Gazebo pose, not SLAM** — coverage metrics use `/model/<name>/pose` from Gazebo (zero drift) not the robot's own odometry. Bridge this topic when implementing coverage: add `/{robot_name}/gz_pose@geometry_msgs/msg/Pose[gz.msgs.Pose` to the bridge arguments and bridge `/model/derpbot_0/pose` in Gazebo.

- **world_sdf override in arst_sim.launch.py** — added `world_sdf` arg that takes a full path to a generated SDF, overriding the `world` template name lookup. Launcher passes this. Backward compatible — old `world:=indoor_office` usage still works.

- **rclpy lazy import in runner** — `import rclpy` is inside `run()` so the module is importable without a ROS 2 installation (useful for unit tests of non-ROS logic).

---

## File structure (key files only)

```
launch/
  arst_sim.launch.py          # Gazebo (+ world_sdf override arg)
  spawn_robot.launch.py       # spawn robot + bridges
scripts/
  despawn_robot.sh
robots/
  derpbot/urdf/derpbot.urdf   # ROBOT_NAME placeholder; no sensors yet
  robot_interface.py
config/
  robots/derpbot.yaml
  scenarios/office_explore_detect.yaml
worlds/
  templates/indoor_office/world.sdf + config.yaml + ground_truth_map.*
  models/{fire_extinguisher,first_aid_kit,hazard_sign}/
src/
  metrics/
    base_metric.py            # DONE
    evaluator.py              # DONE
    reporter.py               # DONE
    scoring.py                # DONE — all four category scorers
    meters_traveled.py        # DONE — odom subscription
    collision_count.py        # DONE — contacts subscription
    exploration_coverage.py   # STUB — implement after adding LiDAR
  world_manager/
    object_placer.py          # DONE
    world_generator.py        # DONE
    template_loader.py        # DONE
  scenario_runner/
    launcher.py               # DONE
    runner.py                 # DONE
    __main__.py               # DONE — CLI entry point
    reset.py                  # STUB
  utils/
    config_loader.py          # DONE
    ros_helpers.py            # DONE
    logging_setup.py          # DONE
```
