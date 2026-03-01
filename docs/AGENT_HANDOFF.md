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
- **gz-transport Python bindings**: `gz.transport13` in `/usr/lib/python3/dist-packages/` (system). `src/utils/gz_transport.py` now appends this path so ground-truth pose works even with venv active. Metrics still use ROS odom (intentionally — odom is available without gz-transport and sufficient for coverage/distance; world_state.py prefers ground-truth via gz_get_robot_pose).
- **`use_sim_time`**: rclpy metrics node must be created with `use_sim_time=True` or messages are silently dropped as future-dated.
- **Contact sensor topic**: `<contact><topic>` (NOT `<sensor><topic>`) controls what gz transport publishes. Must be `/ROBOT_NAME/bumper_contact` inside the `<contact>` block. If absent, `gz sdf -p` sets it to a scoped default that doesn't match the bridge.
- **URDF→SDF joint lumping**: `gz sdf -p` lumps `base_footprint→base_link`. Collision name becomes `base_footprint_fixed_joint_lump__base_link_collision_collision`. Contact sensor reference must use the post-lumping name.
- **Dual casters**: Robot has front caster (x=+0.13) AND rear caster (x=-0.13) — required for pitch stability under acceleration. Removing either causes the robot to tilt when driving.
- **Camera wall-clipping**: camera at x=+0.05 m (10 cm behind front face). Near clip set to **0.05 m** (was 0.12 m). With 0.12 m the wall face (only 0.10 m away when pressed against it) was inside the blind zone and objects behind the wall were rendered. 0.05 m ensures walls render before the clip plane. LOS check in `ObjectDetectionTracker` and `world_state.py` adds defence-in-depth.
- **Dynamic spawn contact sensors (gz-sim 8.10.0 bug)**: `EachNew<ContactSensor>` never fires for dynamically spawned models. Fixed by embedding robots in world SDF at generation time — already implemented in `WorldGenerator._embed_robots()`.
- **Process cleanup**: `SimulationLauncher.shutdown()` captures pgids *before* SIGTERM (process may exit before SIGKILL). Uses `os.killpg()` to kill entire process groups. Runner wrapped in `try/finally` so shutdown always runs.
- **Scenario config schema**: `robots:` is a list (not `robot:`). Each entry: `{platform, name, spawn_pose: {x,y,z,yaw}}`.
- **Parallel sessions**: Isolate with `ROS_DOMAIN_ID=N ./scripts/run_scenario.sh ...`. gz transport is isolated per process; only ROS DDS needs the domain ID.
- **Door states**: `door_states: open | closed | random` is fully implemented. Closed doors are injected as static box panels at SDF generation time. BFS connectivity check ensures the scenario stays solvable. Connectivity uses `rooms` (full room extents in config.yaml) not placement_zones — spawns anywhere in a room are correctly tracked.
- **Spawn randomization**: `spawn_pose: {random: true, zones: [...]}` in scenario YAML. `WorldGenerator._resolve_spawn_poses()` picks a zone then a pre-validated pose from `config.yaml:spawn_poses`. Resolved before door-state assignment so connectivity guarantee uses the actual spawn. Easy: office_a only; medium: office_a/b; hard/brutal/perception_stress: any room.
- **Ceiling**: static flat model at z=3.0 m added to `world.sdf`. `_apply_lighting` sets `ceiling_color` + scene `background` from each lighting preset. dim_localized (hard/brutal): near-black ceiling [0.03,0.03,0.03], background [0.02,0.02,0.02]. Rooms now have visible contrast: office_a moderate warm, office_b very dim blue, corridor bright red emergency, meeting_room bright neutral.
- **IMU system plugin**: IMU is a physics-based sensor — it needs `gz-sim-imu-system` in `world.sdf` in addition to `gz-sim-sensors-system` (which handles GPU-rendered sensors like LiDAR). Missing it → IMU sensor initialises but never publishes.
- **IMU QoS**: `ros_gz_bridge` bridges IMU as **BEST_EFFORT**. Subscribers must use `ReliabilityPolicy.BEST_EFFORT` or they receive no messages.
- **Camera FOV / body in frame**: DerpBot camera is at z=0.20 from `base_link` (body top = 0.10). With horizontal_fov=90° and 640×480, the bottom FOV edge is −36.87°. At z=0.20 the bottom-centre ray exits the body x-extent (±0.15 m) at x=0.183 m. Do NOT lower the camera below z=0.18 or the robot body re-enters frame. Prior value was 0.15 → caused lower 25% of image to show blue chassis.
- **Patrol bot controller**: Runs as a **daemon thread** inside the launcher process (not a subprocess). `patrol_bot_controller.run()` imports gz.transport13 lazily inside the function (not at module level) so the module is safely importable. The thread uses the same network context as the Gazebo process — no discovery delay.
- **Patrol bot DiffDrive topic**: Do NOT set `<topic>cmd_vel</topic>` in the DiffDrive plugin SDF. That resolves to the global `/cmd_vel`, not `/model/<name>/cmd_vel`. Without a `<topic>` element, DiffDrive defaults to `/model/<model_name>/cmd_vel`, which matches what `patrol_bot_controller.py` publishes to. This was the root cause of patrol bot never moving.
- **Patrol bot spawn = first waypoint**: `patrol_bot_controller.py` drives for `distance/speed` seconds computed from waypoint[0]→waypoint[1]. Spawn must match waypoint[0] or the bot overshoots and hits a wall. Always set `spawn: [wp0_x, wp0_y, z]`.
- **Patrol bot dual casters**: PatrolBot needs both front (x=+0.20) and rear (x=-0.20) casters for pitch stability. With only front caster, bot pitches −19° → inaccurate turns → y drift. Rear caster reduces pitch to −5.8°.
- **Patrol bot drive+teleport**: Controller drives forward for `dist/speed` seconds then calls `/world/<world>/set_pose` to teleport back to waypoint[0]. No turns = zero heading drift. y stays exactly on path. `world_name` is passed from launcher → controller via `_launch_patrol_controller(obs, world_name)`.
- **Patrol bot waypoints must stay in one room**: Dead-reckoning drives straight; cannot open doors or avoid walls. Corridor patrol: keep all points within y=6.7–9.3.
- **Particle emitter plugin**: `gz-sim-particle-emitter-system` must be in world.sdf plugins for `<particle_emitter>` elements to work. Without it the emitter silently does nothing. Material format for Gazebo Harmonic: `<material><diffuse>r g b a</diffuse><specular>r g b a</specular></material>` (no `<script>` — that's Gazebo Classic/ogre1 syntax and renders as black in ogre2).
- **FlickerController**: `/world/<world>/light_config` is a **SERVICE** (not a pub/sub topic) provided by the UserCommands system. Using `advertise()`+`publish()` sends to a topic nobody subscribes to. Use `node.request(service, req, Light, Boolean, timeout_ms)` to call the service. Falls back to `gz service` CLI if transport init fails. 3-second startup delay in the flicker thread avoids calling the service before Gazebo registers it.

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
  scenario_runner/              # launcher, runner, __main__
  world_manager/                # world_generator, object_placer, template_loader
  utils/                        # config_loader, ros_helpers, logging_setup
docs/
  ARST_Project_Plan.md          # vision, requirements, remaining steps, scoring reference
  environment_variations.md     # robustness variation designs + priority order
```
