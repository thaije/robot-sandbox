# Agent Handoff — ARST

Gazebo Harmonic + ROS 2 Jazzy simulation testbed. DerpBot (custom diff-drive).
Full plan + remaining work: [`docs/ARST_Project_Plan.md`](ARST_Project_Plan.md).

---

## What works (verified)

- **Full scenario pipeline**: `./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless`
  - Difficulty tiers: `easy` / `medium` / `hard` / `brutal` / `perception_stress` — all under `config/scenarios/office_explore_detect/`
  - Each tier YAML sets lighting, door states, dynamic obstacles, and timeout. Pass `--seed N` to pin a layout; omit for a fresh random layout each run.
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

**Running as agent:** `./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml --headless --timeout 300 [--seed N]` (startup ~5s).
Navigation → invoke the `/arst-nav` skill. It has all documentation.

---

## Key gotchas

- **Python interpreter**: Always `python3.12`. `python3` may resolve to another venv. `run_scenario.sh` handles this.
- **gz-transport Python bindings**: `gz.transport13` is in `/usr/lib/python3/dist-packages/` (system). `src/utils/gz_transport.py` appends this path so it works with venv active.
- **`use_sim_time`**: rclpy metrics node must be created with `use_sim_time=True` or messages are silently dropped as future-dated.
- **Contact sensor topic**: Use `<contact><topic>` (NOT `<sensor><topic>`) inside the `<contact>` block, or gz-transport publishes to an unmatched scoped default topic.
- **URDF→SDF joint lumping**: `gz sdf -p` lumps `base_footprint→base_link`. Contact sensor collision reference must use the post-lump name: `base_footprint_fixed_joint_lump__base_link_collision_collision`.
- **Dual casters**: Both DerpBot (x=±0.13) and PatrolBot (x=±0.20) need front AND rear casters for pitch stability. Without the rear caster the robot pitches forward under acceleration, causing inaccurate turns.
- **Camera: robot body in frame**: Do not lower the DerpBot camera below z=0.18 m or the chassis re-enters the frame and blocks the lower portion of the image.
- **Dynamic spawn contact sensors (gz-sim 8.10.0 bug)**: `EachNew<ContactSensor>` never fires for dynamically spawned models. Fixed by embedding robots in world SDF at generation time — already implemented in `WorldGenerator._embed_robots()`.
- **Parallel sessions**: Isolate with `ROS_DOMAIN_ID=N ./scripts/run_scenario.sh ...`. gz transport is isolated per process; only ROS DDS needs the domain ID.
- **Door states**: `door_states: open | closed | random`. Closed doors are injected as static panels at SDF generation time; BFS connectivity check ensures the scenario stays solvable.
- **IMU**: Needs `gz-sim-imu-system` plugin in `world.sdf` (separate from `gz-sim-sensors-system`); bridge publishes **BEST_EFFORT** QoS — subscribe with `ReliabilityPolicy.BEST_EFFORT` or receive nothing.
- **Patrol bot DiffDrive topic**: Do NOT set `<topic>cmd_vel</topic>` in the DiffDrive plugin SDF — it resolves to the global `/cmd_vel`. Omit it and DiffDrive defaults to `/model/<name>/cmd_vel`.
- **Patrol bot spawn = first waypoint**: Controller drives for `distance/speed` seconds from waypoint[0]→[1]. Spawn must match waypoint[0] exactly or the bot overshoots and hits a wall.
- **Patrol bot drive+teleport**: Pure diff-drive dead-reckoning for multi-waypoint patrol quickly goes off-route — heading drift per turn pushes the bot into walls and rooms. Solved by driving straight for a fixed time then teleporting back to waypoint[0] via `/world/<world>/set_pose`. No turns, no drift.
- **Particle emitter plugin**: `gz-sim-particle-emitter-system` must be in `world.sdf` or emitters silently do nothing. Use PBR `<albedo_map>` / `<diffuse>` — NOT `<material><script>` (ogre1 syntax, renders black in ogre2).
- **FlickerController**: `/world/<world>/light_config` is a **SERVICE**, not a topic — use `node.request()`. gz-sim 8.10.0 bug: service only works on lights created dynamically via EntityFactory (`/world/<world>/create`), not SDF-baked lights. `FlickerController._recreate_lights()` handles this at startup; flicker lights are excluded from the SDF by `WorldGenerator._apply_lighting()`. The `light_config` request must send all light fields (diffuse, attenuation, `is_light_off`, intensity) — partial updates are ignored.

---

## Development guidelines

Try to test new features as much as possible before deeming them complete. Preferably use Python tests. 
If not possible, e.g. for visual features (smoke, lights, materials, furniture), use this flow: create a minimal scenario in `config/scenarios/tests/` with fixed lighting, open doors, and an impossible success condition so it runs to timeout; find a seed that places objects in the target zone by sampling `ObjectPlacer` directly; launch headless in tmux with a long timeout; navigate with `robot_control.py` and snapshot the camera; iterate kill → edit → relaunch → snapshot. Use `world_state.py` to confirm object positions and robot pose at each step.

---

## Architecture

```
ScenarioRunner.run()
  ├── WorldGenerator.generate()    # SDF from template + objects + embedded robots (gz sdf -p)
  ├── SimulationLauncher.launch()  # gz sim + RSP + bridges (no dynamic spawn)
  ├── rclpy metrics node (thread)  # MetersTraveled, CollisionCount, RevisitRatio per robot
  ├── _run_until_done()            # poll success criteria every 2s
  ├── ScoringEngine.compute()      # Speed / Accuracy / Safety / Efficiency
  ├── MissionServer.start()        # HTTP :7400/mission — mission brief + live status
  ├── render_scorecard()
  └── write_results()              # JSON to results/
```

**Key design decisions:**
- `ROBOT_NAME` placeholder in URDF replaced at SDF generation time → unique topics per instance
- Robots embedded in world SDF (not dynamically spawned) — required for contact sensors
- `spawn_robot.launch.py` with `spawn:=false` starts RSP + bridges only (used by automated runs; robot already in world SDF). True dynamic spawn works for teleop/odom/TF but contact sensor won't fire.
- `boundingbox_camera` for ground-truth detection — handles occlusion, outputs `vision_msgs/Detection2DArray` (same as YOLO → easy swap later)
- Coverage: raycasting at 0.5m res using `skimage.draw.line()` + odom pose converted to world frame via spawn offset (near-zero drift in sim)
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
robots/derpbot/urdf/derpbot.urdf  # ROBOT_NAME placeholder; contact sensor; LiDAR; boundingbox camera
config/
  robots/derpbot.yaml           # robot platform config
  scenarios/office_explore_detect.yaml  # legacy base scenario
  scenarios/office_explore_detect/      # difficulty tiers: easy / medium / hard / brutal / perception_stress
worlds/
  templates/indoor_office/      # world.sdf + config.yaml + ground_truth_map.pgm/.yaml
  models/                       # fire_extinguisher, first_aid_kit, hazard_sign SDF models
src/
  metrics/                      # base_metric, evaluator, reporter, scoring, meters_traveled,
                                #   collision_count, revisit_ratio, detection_metrics,
                                #   object_detection_tracker, exploration_coverage, near_miss_tracker
  scenario_runner/              # launcher, runner, __main__, mission_server
  world_manager/                # world_generator, object_placer, template_loader
  utils/                        # config_loader, ros_helpers, logging_setup
docs/
  ARST_Project_Plan.md          # vision, requirements, remaining steps, scoring reference
  AUTONOMOUS_AGENT_GUIDE.md     # external guide for autonomous agent developers (sensors, topics, mission endpoint)
  environment_variations.md     # robustness variation designs + priority order
config/scenarios/tests/         # isolated test scenarios (smoke_test.yaml, …)
```
