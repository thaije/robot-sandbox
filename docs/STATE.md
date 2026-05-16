# STATE — robot-sandbox

Gazebo Harmonic + ROS 2 Jazzy sim testbed. DerpBot (custom diff-drive) explores indoor environments, detects objects or navigates to targets. Scored on Speed/Accuracy/Safety/Efficiency/Effectiveness (explore-detect) or Success/Time/Safety/Efficiency (proximity-goal).
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits.

Agent-facing spec: [`AUTONOMOUS_AGENT_GUIDE.md`](AUTONOMOUS_AGENT_GUIDE.md)

## Environment

| Component   | Version              |
|-------------|----------------------|
| OS          | Ubuntu 24.04 LTS     |
| Python      | 3.12.3               |
| ROS         | Jazzy Jalisco        |
| Gazebo      | Harmonic (gz-sim 8.11.0) |

Pinned Python deps: [`requirements.txt`](../requirements.txt)

---

## What's running

```
ScenarioRunner — scripts/run_scenario.sh config/scenarios/<scenario>/<tier>.yaml [--headless|--gui] [--seed N]
  WorldGenerator       SDF from template + placed objects + embedded robots (gz sdf -p)
  SimulationLauncher   gz sim + robot_state_publisher + ros_gz bridges
  rclpy metrics node   MetersTraveled, CollisionCount, RevisitRatio, ExplorationCoverage, DetectionMetrics, NearMissTracker, ProximityTracker
  MissionServer        HTTP :7400/mission — mission brief (explore_detect or proximity type) + live status
  ScoringEngine        explore_detect: Speed/Accuracy/Safety/Efficiency/Effectiveness → JSON
                        proximity-goal: Success/Time/Safety/Efficiency → JSON
                        Par values from human perception baseline (n=5/tier). Par = B grade (~70).

worlds/templates/indoor_office/   20×15 m, 4 rooms, PGM ground-truth map
worlds/templates/basement/       6×8 m (E-W × N-S), 3 rooms (main 4×8m + SE 2×3m + NE 2×5m, two arched passages), 0.8 m ceiling, PGM ground-truth map
scenario types                    explore_detect (find all targets) / proximity (reach target)
objects                           fire_extinguisher, first_aid_kit, hazard_sign, exit_sign, person, drink_can, drill, mouse, conduit_yellow, pipe_sewer, pipe_sewer_floor, pipe_water, suitcase, suitcase_static — unique per-instance labels
difficulty tiers                  easy / medium / hard / brutal / perception_stress (office)
                                   easy / medium (basement_find — proximity-goal)
                                   lighting, door states, dynamic obstacles, timeout — all YAML-driven

DerpBot topics (/derpbot_0/...)
  scan               LaserScan @ 9.8 Hz, 720 samples, 360°, 0.15–12 m
  imu                Imu @ 100 Hz, BEST_EFFORT QoS
  rgbd/image         RGB 640×480 @ 10 Hz, 90° H-FOV
  rgbd/depth_image   depth float32 m, 0.15–6.0 m
  rgbd/camera_info   intrinsics (always on)
  rgbd/points        off by default (--enable-pointcloud)
  detections         oracle bbox camera, off by default (--enable-oracle)
  odom               IMU-fused (EKF), published by robot_localization
  odom_raw           raw wheel-encoder dead-reckoning (for custom sensor fusion)
  cmd_vel            diff-drive command
```

TF: `odom → base_footprint → base_link → lidar_link / camera_link` (EKF publishes odom→base_footprint)
Ends on `SUCCESS` (`found_ratio` = 1.0) or `TIME_LIMIT`.

Benchmark submission: `scripts/validate_submission.py` + `results/submissions/`. Protocol + format → `docs/AUTONOMOUS_AGENT_GUIDE.md §7`.
Leaderboard: run `python3.12 scripts/generate_leaderboard.py` after adding/updating submissions — writes `docs/leaderboard.json` + `docs/leaderboard.html`. Ranking by `found_ratio` (mission targets only; environmental objects excluded).

---

## Invariants (will bite again — keep in context)

Anything in committed config/code is omitted. Only things a fresh agent would rediscover the hard way.

### Runtime / ROS 2
- **Python interpreter: always `python3.12`.** `python3` may resolve to another venv. `run_scenario.sh` handles this.
- **`use_sim_time=True` required.** rclpy metrics node must use sim time or messages are silently dropped as future-dated.
- **Odometry is IMU-fused via EKF.** `/derpbot_0/odom` is published by `robot_localization` (fuses wheel encoders + IMU yaw). Raw wheel-encoder odom is on `/derpbot_0/odom_raw`. The EKF also publishes `odom→base_footprint` TF; the raw diff-drive TF is remapped to `/derpbot_0/tf_raw` and should NOT be republished to `/tf`.
- **IMU is BEST_EFFORT QoS.** Subscribe with `ReliabilityPolicy.BEST_EFFORT` or receive nothing.
- **gz-transport Python bindings**: `gz.transport13` lives in `/usr/lib/python3/dist-packages/` (system). `src/utils/gz_transport.py` appends this path so it works with venv active.
- **Parallel sessions**: isolate with `ROS_DOMAIN_ID=N ./scripts/run_scenario.sh ...`. gz transport is isolated per process; only ROS DDS needs the domain ID.
- **Detection pose frame**: tracker expects agent detections in odom/map frame (odom origin = robot spawn). Applies spawn offset to convert to world frame. Oracle uses GT world positions directly.
- **Leaderboard `detection_by_type` includes `mission_target`**: per-object-type boolean marking whether that type counts toward `found_ratio`. Leaderboard generator falls back to scenario YAML config for legacy JSONs lacking this field.
- **Easy timeout is 900s** (not 300s). `easy_900s.yaml` has been merged into `easy.yaml`.
- **Proximity-goal scenarios**: `goal_type: proximity` in scenario YAML switches scoring from 5-category explore-detect to 4-category (Success/Time/Safety/Efficiency). ProximityTracker uses ground-truth positions from `world_state.json` + robot odom. Success requires BOTH `proximity_reached == true` AND `found_ratio >= 1.0` — the robot must reach within `proximity_radius` metres of the target AND publish a valid Detection2DArray for it. Proximity radius changed from 2.0 m to 1.0 m.
- **`target_pool` randomization**: object specs with `target_pool: true` form a candidate pool; WorldGenerator randomly picks ONE type (seed-deterministic via `rng(seed + 5555)`) as the mission target. Chosen type gets `mission_target: true`, others in pool get `false`. `scenario.target_object` is set automatically. Non-pool objects keep their existing `mission_target` value.
- **Basement is 6 m × 8 m** (E-W × N-S). Three rooms: R1 (west, 4×8 m), R2 (SE, 2×3 m), R3 (NE, 2×5 m). Two arched passages connect R1↔R2 (at y≈1.5) and R1↔R3 (at y≈5.5). Ceiling 0.8 m — DerpBot camera at z=0.18 m, only 0.62 m clearance above camera.

### Gazebo / SDF
- **Dynamic spawn contact sensors (gz-sim 8.10.0 bug)**: `EachNew<ContactSensor>` never fires for dynamically spawned models. Robots must be embedded in world SDF at generation time.
- **Contact sensor topic**: use `<contact><topic>` (NOT `<sensor><topic>`) inside the `<contact>` block, or gz-transport publishes to an unmatched scoped topic.
- **URDF→SDF joint lumping**: `gz sdf -p` lumps `base_footprint→base_link`. Contact sensor collision reference must use post-lump name: `base_footprint_fixed_joint_lump__base_link_collision_collision`.
- **Particle emitter**: `gz-sim-particle-emitter-system` must be in `world.sdf` or emitters silently do nothing. Use PBR `<albedo_map>` — NOT `<material><script>` (ogre1 syntax, renders black in ogre2).
- **FlickerController**: `/world/<world>/light_config` is a SERVICE, not a topic. gz-sim 8.10.0 bug: only works on lights created dynamically via EntityFactory, not SDF-baked lights — flicker lights must be recreated at startup.
- **Only one sim run at a time.** Hardware cannot sustain two Gazebo/ROS 2 stacks simultaneously. Always run seeds sequentially, never in parallel.

### Human baselines
- **Two modes**: `oracle` (nav-only, free detections) and `perception` (human keypresses for detections via `scripts/human_detector_node.py`).
- **Detection node** publishes `Detection2DArray` with robot odom position (IMU-fused via EKF). Keybindings from mission server; defaults to `f`=fire_extinguisher, `a`=first_aid_kit, `p`=person.
- **Odometry is IMU-fused**: `/odom` comes from `robot_localization` EKF (wheel encoders + IMU gyro). Raw wheel-encoder data is on `/odom_raw` for agents that want to do their own fusion.

### Robot geometry
- **Camera min height z=0.18 m.** Below this, the DerpBot chassis re-enters the frame and blocks the lower image portion.
- **Dual casters required.** Both DerpBot (x=±0.13) and PatrolBot (x=±0.20) need front AND rear casters for pitch stability; without them the robot pitches forward under accel.

### PatrolBot
- **DiffDrive topic**: do NOT set `<topic>cmd_vel</topic>` in the DiffDrive plugin SDF — resolves to global `/cmd_vel`. Omit it; DiffDrive defaults to `/model/<name>/cmd_vel`.
- **Spawn = first waypoint exactly.** Controller drives `distance/speed` seconds from waypoint[0]→[1]. Mismatch → overshoot → wall collision.
- **Multi-waypoint dead-reckoning drifts.** Solution: drive straight for fixed time then teleport back to waypoint[0] via `/world/<world>/set_pose`. Already implemented.

---

## How to run

```bash
# Automated (agent)
./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless --timeout 300 [--seed N]
./scripts/run_scenario.sh config/scenarios/basement_find/easy.yaml --headless --seed 1

# Dev session (GUI + teleop)
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --gui
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel

# Human baseline — oracle (nav-only, cheat detections)
./scripts/run_human_baseline.sh oracle

# Human baseline — perception (nav + manual detections)  
./scripts/run_human_baseline.sh perception
# + human_detector_node.py in terminal 3
# + rqt_image_view in terminal 4

# Unit tests (no sim required)
python3.12 -m pytest tests/

# Integration test (requires live sim + ROS 2)
pytest -m integration tests/test_integration.py

# Post-run detection visualisation (requires results JSON + world_state.json)
python3.12 scripts/detection_viz.py --results results/<run>.json
```

Navigation: invoke the `arst-nav` skill — full docs inside.
Results: `results/<scenario_name>.json`.

---

## Issue tracker

Everything with a lifecycle lives in GitHub issues.
- **Active / next work:** [`ROADMAP.md`](ROADMAP.md) — short TOC with links.
- **Before a major change, check closed dead-ends:** `gh issue list --state closed --label dead-end --search <topic>`
- **Backlog / known bugs:** `gh issue list --state open --label backlog`
