# Agent Handoff — ARST Project

## What this project is

Autonomous Robotics Simulation Testbed (ARST). Gazebo Harmonic + ROS 2 Jazzy simulation environment for testing robot autonomy. Full plan: `docs/ARST_Project_Plan.md`.

**Stack:** Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, Python 3.12.

---

## Current state

### What works end-to-end (verified running)

- **Persistent Gazebo** — `ros2 launch launch/arst_sim.launch.py [headless:=true]`
- **Manual spawn DerpBot** — `ros2 launch launch/spawn_robot.launch.py [name:=derpbot_0 x:= y:= yaw:=]` / despawn: `scripts/despawn_robot.sh`. In automated scenario runs the robot is embedded in the world SDF; `spawn_robot.launch.py` is started with `spawn:=false` (bridges only).
- **Teleoperation** — `ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel`
- **Bridges** — `/derpbot_0/cmd_vel`, `/derpbot_0/odom`, `/tf`, `/derpbot_0/joint_states` all live
- **Indoor office world** — 20×15 m, 4 rooms, furniture, correct physics
- **Ground-truth map** — `worlds/templates/indoor_office/ground_truth_map.pgm` + `.yaml` (0.5 m resolution)
- **Object models** — `fire_extinguisher` (class 1), `first_aid_kit` (class 2), `hazard_sign` (class 3) — all SDF with label plugins

### Full scenario pipeline (updated 2026-02)

Run via:
```bash
./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless [--timeout N] [--gui]
```

The script handles `python3.12` and `PYTHONPATH` automatically. Full lifecycle:
1. Generate world SDF — WorldGenerator embeds robots from the `robots:` list directly in the SDF (required for contact sensors; see "gz-sim dynamic-spawn bug" section)
2. Launch Gazebo with the generated SDF (SimulationLauncher → `ros2 launch` subprocess)
3. Start RSP + ros_gz_bridge for each robot (`spawn:=false` — model is already in world SDF)
4. Collect `meters_traveled`, `collision_count`, and `revisit_ratio` metrics via rclpy subscriptions
5. Poll success criteria every 2 s until met or timeout
6. Evaluate, score all four categories (Speed/Accuracy/Safety/Efficiency), print ASCII scorecard, write JSON results
7. Kill entire Gazebo + bridge process tree (no orphans; no explicit despawn needed — robots are destroyed with the world)

### DerpBot robot

Two-wheeled diff-drive with a contact/bumper sensor referenced from `base_footprint` (post-lumping). URDF at `robots/derpbot/urdf/derpbot.urdf`. Uses `ROBOT_NAME` placeholder substituted at spawn time so topic names are per-instance (`/derpbot_0/cmd_vel` etc.). TF frames are unprefixed (`odom → base_footprint → base_link`). LiDAR and camera to be added in later steps.

Yellow strip on the front face (+x) marks the forward direction. Contact sensor fires on collisions from any side (the body collision covers the full chassis box).

### Implementation status of key modules

| File | Status | Notes |
|------|--------|-------|
| `src/metrics/base_metric.py` | COMPLETE | Abstract base; plugin interface |
| `src/metrics/meters_traveled.py` | COMPLETE | Odom subscription; registered in runner |
| `src/metrics/collision_count.py` | PENDING TEST | Subscription wired; contact sensor now embedded in world SDF (architectural fix applied — needs live test to confirm) |
| `src/metrics/revisit_ratio.py` | COMPLETE | Odom subscription; registered in runner |
| `src/metrics/exploration_coverage.py` | STUB | All methods raise NotImplementedError — needs LiDAR |
| `src/metrics/detection_metrics.py` | STUB | Logic written but not wired; needs camera sensor |
| `src/metrics/evaluator.py` | COMPLETE | Success/failure evaluation fully works |
| `src/metrics/scoring.py` | COMPLETE | All four category scorers + overall + Scorecard |
| `src/metrics/reporter.py` | COMPLETE | ASCII scorecard + JSON output work |
| `src/scenario_runner/launcher.py` | COMPLETE | launch() accepts robots list, starts bridges only (no spawn); shutdown() kills full process groups |
| `src/scenario_runner/runner.py` | COMPLETE | run(), _build_metrics() per-robot; _collect_metrics() aggregates across robots |
| `src/scenario_runner/__main__.py` | COMPLETE | CLI entry point; argparse wired to runner |
| `src/world_manager/object_placer.py` | COMPLETE | Random placement with collision avoidance |
| `src/world_manager/world_generator.py` | COMPLETE | generate(), lighting + object injection + robot embedding via gz sdf -p |
| `src/utils/config_loader.py` | COMPLETE | YAML load + validation (uses `robots:` list) |
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
| 3 | 3.3 Collision count | 🟡 PENDING TEST — architectural fix applied (robots embedded in world SDF); needs live run to confirm |
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

## OPEN BUG — collision_count always 0 (contact sensor broken for dynamic spawning)

**Symptom:** `collision_count: 0` in every scorecard even after repeated wall collisions.
`gz topic -i /derpbot_0/bumper_contact` shows "No publishers". Same for the scoped topic
`/world/indoor_office/model/derpbot_0/link/base_footprint/sensor/bumper/contact`.

**Deeply investigated (2026-02) — NOT fixed yet.** Reproduced and root-caused by driving
the robot into a wall and observing the bumper topic, then testing progressively simpler
cases down to a 5-line native SDF model.

### What was tried and ruled out

1. **URDF collision name (FIXED, not the root bug)** — `gz sdf -p` lumps the
   `base_footprint→base_link` fixed joint, renaming the body collision from
   `base_link_collision` to `base_footprint_fixed_joint_lump__base_link_collision_collision`.
   The URDF was already updated to use the post-lumping name (`<gazebo reference="base_footprint">`
   + the long lumped name). Confirmed correct via `gz sdf -p | grep "collision name="`.

2. **URDF→SDF runtime conversion** — Also tested spawning the pre-converted SDF via
   `ros2 run ros_gz_sim create -file /tmp/derpbot_0.sdf`. Same "No publishers" result.
   The SDF file is correct; the spawn mechanism is not the problem.

3. **World plugins** — `gz-sim-contact-system`, `gz-sim-physics-system`, and
   `gz-sim-sensors-system` are all present in the world SDF.

4. **TouchPlugin alternative** — Investigated as an alternative. `gz-sim-touchplugin-system`
   also fails because it depends on `components::ContactSensorData` being present on collision
   entities, which is only set by the contact system — same broken path.

### Actual root cause: gz-sim 8.10.0 dynamic-spawn bug

**Confirmed by testing a minimal 5-line SDF** (`<sensor type="contact">` in a box with a
directly named `body_collision` — no URDF, no lumping). When this model is embedded **in the
world SDF at startup**, the contact sensor has 2 publishers and fires correctly on contact.
When the **same SDF is spawned dynamically** via `ros2 run ros_gz_sim create`, the sensor
has 0 publishers and never fires.

**Code path analysis** (gz-sim8 source read from GitHub):
- `SdfEntityCreator` creates `components::ContactSensor` unconditionally for all `<sensor type="contact">` entities, whether at startup or dynamic spawn. ✓
- `gz-sim-contact-system` uses `_ecm.EachNew<components::ContactSensor>()` in `PreUpdate()` to detect new sensors and call `ContactSensor::Load()`, which creates the gz transport publisher and the `ContactSensorData` component on the collision.
- **Bug:** `EachNew<ContactSensor>` never fires for models spawned via the UserCommands create service in a running simulation. `ContactSensor::Load()` is therefore never called → no publisher → no `ContactSensorData` → no contacts detected.
- This is a gz-sim 8.10.0 bug. It may be fixed in later versions.

### Design fix required

The fix is architectural: **embed robot models in the world SDF at generation time** instead
of spawning them dynamically. This ensures contact sensors are initialized by `EachNew` in
the first simulation tick (the normal path for statically loaded worlds).

See the "Architectural redesign: pre-loaded robots" section below for the proposed approach
and options.

### Status: ARCHITECTURALLY RESOLVED (2026-02) — pending live test.

Robots are now embedded in the world SDF by `WorldGenerator._embed_robots()` (via `gz sdf -p`).
`SimulationLauncher` no longer calls `ros_gz_sim create`; it starts RSP + bridge with
`spawn:=false`. Contact sensors should now fire on tick 1. Verify by running a scenario,
driving into a wall, and checking `collision_count > 0` in the scorecard.

---

---

## Architectural redesign: pre-loaded robots (IMPLEMENTED 2026-02)

The dynamic-spawn bug forces a rethink of how robots enter the simulation. The fundamental
constraint is: **contact sensors only work for models that are in the world SDF when Gazebo
starts**, not models added later via the create service.

### Background: current design

Each scenario run:
1. WorldGenerator writes a world SDF (furniture, obstacles, lighting)
2. SimulationLauncher starts `gz sim` with that world
3. `ros_gz_sim create` dynamically spawns DerpBot into the running world
4. Bridge + RSP start and connect to the robot's topics

Step 3 is where things break for contact sensors.

### Option A — Embed robot(s) in world SDF at generation time *(recommended)*

`WorldGenerator.generate()` converts the robot URDF to SDF (via `gz sdf -p`) and injects
the `<model>` element directly into the world SDF before writing it to disk, at the configured
spawn pose. `SimulationLauncher._launch_robot()` then only needs to start the bridge and RSP
— no `ros_gz_sim create` step.

**Pros:**
- Contact sensors work immediately (model is in world from tick 1)
- Minimal code delta — `WorldGenerator` and `Launcher` get small targeted changes
- Each scenario run still gets a fresh Gazebo with its own world; no shared state
- Multi-robot: just embed N models with names `derpbot_0`, `derpbot_1`, etc.

**Cons:**
- Can no longer hot-swap a robot mid-run (not currently needed)
- `spawn_robot.launch.py` becomes unused for programmatic runs (keep for manual dev use)

**For parallel scenarios:** Each scenario run is already a separate Gazebo process. Five
concurrent scenarios = five Gazebo processes, each with their own world and robots. No
instance-count problem.

**For multi-robot in one world:** Scenario config lists N robots; WorldGenerator embeds N
model instances with separate names and poses. Each gets its own bridge.

### Option B — Parking-lot with teleport *(heavier, useful only if Gazebo must stay running)*

Pre-load a fixed number of robot slots (e.g. `derpbot_0..derpbot_4`) in every world SDF, all
parked off-map at `(−100, 0)`. "Spawning" becomes a `set_pose` gz service call to move the
robot to the scenario start position; "despawning" teleports it back to the parking lot.

**Pros:** Can reset between runs without restarting Gazebo (faster iteration if startup is slow).

**Cons:**
- All N robot instances exist and consume physics budget even when idle
- Slot contention: need a registry to assign slots to concurrent users
- Sensors from inactive robots could interfere
- More complex to implement and operate

**Verdict:** Only worth it if Gazebo startup time becomes a bottleneck (currently ~10–15 s).
With Option A, each scenario restarts Gazebo, which is acceptable for now.

### Recommended path

Implement **Option A**. The changes are:

1. **`WorldGenerator.generate()`** — after building the world XML tree, convert the robot
   URDF to SDF via `subprocess.run(['gz', 'sdf', '-p', urdf_path])`, parse the model
   element, set its name and pose from `robot_cfg`, and `world_elem.append(model_elem)`.
2. **`SimulationLauncher._launch_robot()`** — remove the `ros_gz_sim create` node; keep
   RSP and bridge nodes only.
3. **`SimulationLauncher._despawn_if_exists()`** — teleport instead of remove, or skip
   (world SDF regenerated per run anyway, so no ghost accumulation).
4. **`SimulationLauncher._wait_for_robot()`** — unchanged; still waits for odom topic.

Estimated delta: ~40 lines changed across two files.

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

## Running scenarios

### Single robot

The default scenario has one DerpBot. Just run:

```bash
./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless
```

The `robots:` list in the YAML drives everything — WorldGenerator embeds the robot in the
world SDF and `SimulationLauncher` starts its bridges. No extra arguments needed.

### Multiple robots in one world

Add more entries to the `robots:` list in the scenario YAML:

```yaml
robots:
  - platform: "derpbot"
    name: "derpbot_0"
    spawn_pose: {x: 1.0, y: 1.0, z: 0.0, yaw: 0.0}
  - platform: "derpbot"
    name: "derpbot_1"
    spawn_pose: {x: 3.0, y: 1.0, z: 0.0, yaw: 1.57}
```

WorldGenerator embeds both models in the world SDF. Each gets its own RSP + bridge
(namespaced `/derpbot_0/...` and `/derpbot_1/...`). Metrics are collected per robot
and aggregated: `meters_traveled` and `collision_count` are summed; `revisit_ratio`
is averaged. Per-robot breakdown is included as `{metric}__derpbot_0` etc. in the results.

### Parallel sessions (separate ROS_DOMAIN_ID)

Each `./scripts/run_scenario.sh` launch starts its own Gazebo process, so Gazebo topics are
already isolated. The only thing that could collide is the ROS 2 DDS layer — all nodes on the
same domain ID can see each other's topics, causing metric cross-contamination.

Isolate each session with a unique `ROS_DOMAIN_ID` (0–232):

```bash
# Terminal 1 — scenario A on domain 0 (default)
./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless

# Terminal 2 — scenario B on domain 1
ROS_DOMAIN_ID=1 ./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless

# Terminal 3 — scenario C on domain 2
ROS_DOMAIN_ID=2 ./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless
```

Each `run_scenario.sh` exports `ROS_DOMAIN_ID` before launching, so all child processes
(Gazebo bridges, RSP, metrics rclpy node) share the same domain. Processes on different
domains are completely invisible to each other — no namespace collisions, no cross-contamination.

For scripted parallel runs use a distinct ID per process, e.g.:

```bash
for i in $(seq 0 4); do
    ROS_DOMAIN_ID=$i ./scripts/run_scenario.sh my_scenario.yaml --headless &
done
wait
```

**Note:** `gz transport` (Gazebo's internal bus) is isolated by process by default —
separate `gz sim` processes never share topics regardless of `ROS_DOMAIN_ID`.

---

## Architecture of the full pipeline

```
__main__.py (CLI)
  └── ScenarioRunner.run()
        ├── WorldGenerator.generate()          # SDF generation
        │     ├── TemplateLoader.load()         # reads config.yaml + world.sdf
        │     ├── ObjectPlacer.place()          # deterministic object placement
        │     ├── _apply_lighting()             # XML mutation
        │     ├── _inject_objects()             # XML mutation
        │     └── _embed_robots()              # gz sdf -p → inject <model> per robot
        ├── SimulationLauncher.launch()         # subprocess management
        │     ├── ros2 launch arst_sim.launch.py world_sdf:=<path>
        │     └── ros2 launch spawn_robot.launch.py name:=derpbot_0 spawn:=false ...
        │           (RSP + bridge only — robot already in world SDF)
        ├── rclpy metrics node (spin in thread)
        │     ├── MetersTraveled  → /derpbot_0/odom subscription (per robot)
        │     ├── CollisionCount  → /derpbot_0/bumper_contact subscription (per robot)
        │     └── RevisitRatio    → /derpbot_0/odom subscription (per robot)
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
