# STATE — robot-sandbox

Gazebo Harmonic + ROS 2 Jazzy sim testbed. DerpBot (custom diff-drive) explores indoor office + detects objects, scored on Speed / Accuracy / Safety / Efficiency / Effectiveness.
Load this every session. What's next lives in [`ROADMAP.md`](ROADMAP.md); history lives in GitHub issues + commits.

Agent-facing spec: [`AUTONOMOUS_AGENT_GUIDE.md`](AUTONOMOUS_AGENT_GUIDE.md)

---

## What's running

```
ScenarioRunner — scripts/run_scenario.sh config/scenarios/<tier>/<name>.yaml [--headless|--gui] [--seed N]
  WorldGenerator       SDF from template + placed objects + embedded robots (gz sdf -p)
  SimulationLauncher   gz sim + robot_state_publisher + ros_gz bridges
  rclpy metrics node   MetersTraveled, CollisionCount, RevisitRatio, ExplorationCoverage, DetectionMetrics, NearMissTracker
  MissionServer        HTTP :7400/mission — mission brief + live status
  ScoringEngine        Speed 0.20 / Accuracy 0.30 / Safety 0.20 / Efficiency 0.10 / Effectiveness 0.15 → JSON in results/

worlds/templates/indoor_office/   20×15 m, 4 rooms, PGM ground-truth map
objects                           fire_extinguisher ×3, first_aid_kit ×2, hazard_sign ×4 — unique per-instance labels
difficulty tiers                  easy / medium / hard / brutal / perception_stress
                                  lighting, door states, dynamic obstacles, timeout — all YAML-driven

DerpBot topics (/derpbot_0/...)
  scan               LaserScan @ 9.8 Hz, 720 samples, 360°, 0.15–12 m
  imu                Imu @ 100 Hz, BEST_EFFORT QoS
  rgbd/image         RGB 640×480 @ 10 Hz, 90° H-FOV
  rgbd/depth_image   depth float32 m, 0.15–6.0 m
  rgbd/camera_info   intrinsics (always on)
  rgbd/points        off by default (--enable-pointcloud)
  detections         oracle bbox camera, off by default (--enable-oracle)
  cmd_vel / odom     diff-drive
```

TF: `odom → base_footprint → base_link → lidar_link / camera_link`
Ends on `SUCCESS` (`found_ratio` = 1.0) or `TIME_LIMIT`.

---

## Invariants (will bite again — keep in context)

Anything in committed config/code is omitted. Only things a fresh agent would rediscover the hard way.

### Runtime / ROS 2
- **Python interpreter: always `python3.12`.** `python3` may resolve to another venv. `run_scenario.sh` handles this.
- **`use_sim_time=True` required.** rclpy metrics node must use sim time or messages are silently dropped as future-dated.
- **IMU is BEST_EFFORT QoS.** Subscribe with `ReliabilityPolicy.BEST_EFFORT` or receive nothing.
- **gz-transport Python bindings**: `gz.transport13` lives in `/usr/lib/python3/dist-packages/` (system). `src/utils/gz_transport.py` appends this path so it works with venv active.
- **Parallel sessions**: isolate with `ROS_DOMAIN_ID=N ./scripts/run_scenario.sh ...`. gz transport is isolated per process; only ROS DDS needs the domain ID.
- **Detection pose frame**: tracker expects agent detections in odom/map frame (odom origin = robot spawn). Applies spawn offset to convert to world frame. Oracle uses GT world positions directly.

### Gazebo / SDF
- **Dynamic spawn contact sensors (gz-sim 8.10.0 bug)**: `EachNew<ContactSensor>` never fires for dynamically spawned models. Robots must be embedded in world SDF at generation time.
- **Contact sensor topic**: use `<contact><topic>` (NOT `<sensor><topic>`) inside the `<contact>` block, or gz-transport publishes to an unmatched scoped topic.
- **URDF→SDF joint lumping**: `gz sdf -p` lumps `base_footprint→base_link`. Contact sensor collision reference must use post-lump name: `base_footprint_fixed_joint_lump__base_link_collision_collision`.
- **Particle emitter**: `gz-sim-particle-emitter-system` must be in `world.sdf` or emitters silently do nothing. Use PBR `<albedo_map>` — NOT `<material><script>` (ogre1 syntax, renders black in ogre2).
- **FlickerController**: `/world/<world>/light_config` is a SERVICE, not a topic. gz-sim 8.10.0 bug: only works on lights created dynamically via EntityFactory, not SDF-baked lights — flicker lights must be recreated at startup.

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

# Dev session (GUI + teleop)
./scripts/run_scenario.sh config/scenarios/office_explore_detect/easy.yaml --gui
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel

# Tests
python3.12 -m pytest tests/
```

Navigation: invoke the `arst-nav` skill — full docs inside.
Results: `results/<scenario_name>.json`.

---

## Issue tracker

Everything with a lifecycle lives in GitHub issues.
- **Active / next work:** [`ROADMAP.md`](ROADMAP.md) — short TOC with links.
- **Before a major change, check closed dead-ends:** `gh issue list --state closed --label dead-end --search <topic>`
- **Backlog / known bugs:** `gh issue list --state open --label backlog`
