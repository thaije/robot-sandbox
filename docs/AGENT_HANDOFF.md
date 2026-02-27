# Agent Handoff — ARST

Gazebo Harmonic + ROS 2 Jazzy simulation testbed. DerpBot (custom diff-drive). Full plan: [`docs/ARST_Project_Plan.md`](ARST_Project_Plan.md).

---

## What works (verified)

- **Full scenario pipeline**: `./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless`
  - WorldGenerator assigns unique per-instance labels (1..N); exposes `label_map` to metrics
  - Live metrics: `meters_traveled`, `collision_count` (rising-edge, not debounce), `revisit_ratio`, detection metrics
  - Scorecard: Speed / Accuracy / Safety / Efficiency / **Effectiveness** (per-type weights)
  - Ends on `SUCCESS` (all instances found) or `TIME_LIMIT` (600 s)
  - Scorecard printed + JSON written to `results/`
- **Manual sim**: persistent Gazebo + `spawn_robot.launch.py` + teleoperation
- **Inspection**: `scripts/robot_inspect.py` — snapshot, status, detections, drive (use ≤ 2 s drive calls)
- **Agent nav tools**: `scripts/world_state.py` + `scripts/robot_inspect.py` — see section below
- **World**: indoor_office (20×15 m, 4 rooms, furniture, randomised object placement)
- **Objects**: fire_extinguisher (×3), first_aid_kit (×2), hazard_sign (×4) — 9 instances total with unique labels

## Implementation status

| Module | Status |
|--------|--------|
| `src/metrics/{base_metric,evaluator,reporter,scoring}.py` | ✅ |
| `src/metrics/{meters_traveled,collision_count,revisit_ratio}.py` | ✅ |
| `src/metrics/exploration_coverage.py` | ⬜ Stub — needs LiDAR |
| `src/metrics/detection_metrics.py` | ✅ instance-level tracking + detection_by_type |
| `src/metrics/object_detection_tracker.py` | ✅ label_map → "fire_extinguisher #2" display |
| `src/scenario_runner/{launcher,runner,__main__}.py` | ✅ |
| `src/world_manager/{world_generator,object_placer,template_loader}.py` | ✅ |
| `src/utils/{config_loader,ros_helpers,logging_setup}.py` | ✅ |

---

---

## Running the scenario as an agent (cheat tools)

These tools give ground-truth information not available to the real robot. Use them to verify the pipeline works, not to chase a high score.

**Launch:** `./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless --timeout 300` (startup ~5s)
Writes `/tmp/arst_worlds/world_state.json` at startup. See each script's own docstring for usage and flags.

- `scripts/world_state.py` — PNG map + summary. Map contains walls, furniture, objects to be found (coloured), and found objects (grey, auto-updated). 
- `scripts/robot_inspect.py` — drive, status, detections currently in view.

### Navigation approach
1. **Start with the map**: `world_state.py`
2. **Plan by room, not exact waypoints.** The map shows doorways; navigate room-by-room.
3. **Move → check loop**: drive 1–2 s, then re-fetch and reread the map or call `detections`. ALWAYS CHECK THE MAP AFTER MOVING to avoid collisions. Adjust based on what you see. Don't plan the full route upfront.
4. **Run tools often** — it's fast and keeps the user informed. Refresh the map often to prevent collisions. 
5. **Goal is completion, not score.**
6. **Objects in doorways**: small objects (first_aid_kit, fire_extinguisher) are narrower than a standard door opening and the robot can physically squeeze past, but there is a real risk of getting stuck against the object and losing time. Only attempt this if no clear alternative route exists — otherwise find another door or approach angle.
7. **Drive straight through doorways**: Be sure to go straight through door openings and continue for a meter or so, to prevent the probability of getting stuck on the doorpost.
8. **Study the tools before starting**: check the tools and their usage in detail before starting. 
9. **Tip for efficient detection**: Doing a 360 around the center of a room can spot most objects (unless occluded by furniture) in one go.


---

## Next steps (priority order)

### 1. Add 2D LiDAR to DerpBot
Unlocks `exploration_coverage`. Add `gpu_lidar` sensor to URDF, bridge:
```
/{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
/model/{robot_name}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose
```
Implement `exploration_coverage.py`: grid raycasting with `skimage.draw.line()` against `worlds/templates/indoor_office/ground_truth_map.pgm` at 0.5m resolution.

### 3. Calibrate par values (Step 3.13)
After metrics work: teleop the scenario, record typical values, set `par_values` in `config/scenarios/office_explore_detect.yaml` so a competent operator scores ~70 (B).

---

## Key gotchas

- **Python interpreter**: Always `python3.12`. `python3` may resolve to another venv. `run_scenario.sh` handles this.
- **`use_sim_time`**: rclpy metrics node must be created with `use_sim_time=True` or messages are silently dropped as future-dated.
- **Contact sensor topic**: `<contact><topic>` (NOT `<sensor><topic>`) controls what gz transport publishes. Must be `/ROBOT_NAME/bumper_contact` inside the `<contact>` block. If absent, `gz sdf -p` sets it to a scoped default that doesn't match the bridge.
- **URDF→SDF joint lumping**: `gz sdf -p` lumps `base_footprint→base_link`. Collision name becomes `base_footprint_fixed_joint_lump__base_link_collision_collision`. Contact sensor reference must use the post-lumping name.
- **Dynamic spawn contact sensors (gz-sim 8.10.0 bug)**: `EachNew<ContactSensor>` never fires for dynamically spawned models. Fixed by embedding robots in world SDF at generation time — already implemented in `WorldGenerator._embed_robots()`.
- **Process cleanup**: `SimulationLauncher.shutdown()` captures pgids *before* SIGTERM (process may exit before SIGKILL). Uses `os.killpg()` to kill entire process groups. Runner wrapped in `try/finally` so shutdown always runs.
- **Scenario config schema**: `robots:` is a list (not `robot:`). Each entry: `{platform, name, spawn_pose: {x,y,z,yaw}}`.
- **Parallel sessions**: Isolate with `ROS_DOMAIN_ID=N ./scripts/run_scenario.sh ...`. gz transport is isolated per process; only ROS DDS needs the domain ID.

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
  arst_sim.launch.py            # Gazebo (world_sdf override arg)
  spawn_robot.launch.py         # RSP + bridges; spawn:=false for automated runs
scripts/
  run_scenario.sh
  robot_inspect.py      # status / snapshot / detections / drive
  world_state.py        # PNG map (--ascii opt-in); obstacles + found status; path to stdout
  despawn_robot.sh
robots/derpbot/urdf/derpbot.urdf  # ROBOT_NAME placeholder; contact sensor; no LiDAR/camera yet
config/
  robots/derpbot.yaml
  scenarios/office_explore_detect.yaml
worlds/templates/indoor_office/   # world.sdf + config.yaml + ground_truth_map.pgm/.yaml
worlds/models/                    # fire_extinguisher, first_aid_kit, hazard_sign
src/
  metrics/
  scenario_runner/
  world_manager/
  utils/
```
