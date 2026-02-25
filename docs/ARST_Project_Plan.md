# ARST Project Plan

## Vision

Build a modular Gazebo simulation testbed for testing robot autonomy across diverse scenarios. Phase 1: DerpBot explores an indoor office and locates target objects, evaluated by configurable metrics and composite scoring (Speed / Accuracy / Safety / Efficiency). Teleoperation only in Phase 1; autonomy stack deferred.

**Current state:** See [`docs/AGENT_HANDOFF.md`](AGENT_HANDOFF.md).

---

## Requirements (not yet met)

| ID | Requirement | Notes |
|----|-------------|-------|
| SIM-09 | Configurable environment variations (lighting, door states) | Lighting stub only |
| SIM-11 | Scenario reset without full restart | Stub in `reset.py` |
| SIM-12 | Multiple simulator instances in parallel | Future |
| SIM-13 | Deterministic replay (same seed = same scenario) | Not verified |
| ROB-02 | RGB bounding box camera | Next step |
| ROB-03 | 2D LiDAR | Next step |
| MET-03 | Time to all detections | Needs camera |
| MET-04 | Task completion time | Needs camera (success depends on detection) |
| MET-05 | Average time per detection | Needs camera |
| MET-06 | Exploration coverage % | Needs LiDAR |
| MET-07 | Object detection rate | Needs camera |
| MET-08 | False positive rate | Needs camera |
| MET-10 | Near-miss detection | Not started |
| MET-13 | Calibrated par values | After first working run |
| DOC-01–07 | Documentation | Not started |

---

## Remaining implementation steps

### Step 3.4 — Object detection tracking

Add `boundingbox_camera` sensor to `robots/derpbot/urdf/derpbot.urdf` (`box_type: visible_2d`). Handles occlusion natively. Output: `vision_msgs/Detection2DArray` on `/{ROBOT_NAME}/detections`. Objects must have `gz-sim-label-system` plugin (class IDs: 1=fire_extinguisher, 2=first_aid_kit, 3=hazard_sign).

Bridge in `spawn_robot.launch.py`:
```
/{robot_name}/detections@vision_msgs/msg/Detection2DArray[gz.msgs.AnnotatedAxisAligned2DBox_V
```

Implement `src/metrics/object_detection_tracker.py`: subscribe to `/detections`, track first-detection timestamp per class ID.

### Step 3.5 — Detection metrics

Wire `DetectionMetrics` into `runner._build_metrics()` with `total_targets` from scenario config. Computes: `object_detection_rate`, `time_to_all_detections`, `average_time_per_detection`, `false_positive_rate` (0.0 for ground-truth oracle).

### Step 3.6 — Exploration coverage

Add `gpu_lidar` sensor to DerpBot URDF. Bridge:
```
/{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
/model/{robot_name}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose
```

Implement `exploration_coverage.py`: subscribe to `/scan` + ground-truth pose. Load `worlds/templates/indoor_office/ground_truth_map.pgm` (binary mask, 0.5m resolution). Per scan: vectorise ray endpoints, trace each with `skimage.draw.line()`, mark cells observed. Coverage % = `100 * observed_cells / explorable_cells`. Publish `Float32` at 1 Hz.

### Step 3.10 — Near-miss detection

Subscribe to ground-truth pose. Count events where robot-to-obstacle proximity < threshold (default 0.3m) without contact. Log timestamp, location, distance. Source proximity from collision geometry or a separate obstacle distance node.

### Step 3.13 — Calibrate par values

Run office scenario 3–5 times with teleoperation. Set `par_values` in scenario YAML so a competent operator scores ~70 (B). Document baseline values in YAML comments.

### Steps 4.5–4.7

- **4.5 Scenario reset**: implement `reset.py` (currently a stub)
- **4.6 Batch execution**: `scripts/run_batch.sh` — N runs with different seeds, aggregate mean/std/min/max
- **4.7 Integration test**: scripted movement pattern, verify all metrics record correctly

### Phase 5 — Documentation

`docs/installation.md`, `docs/user_guide.md`, `docs/developer_guide.md`, `docs/architecture.md`, example scenarios with expected metric ranges.

---

## Phase 6 — Future (post Phase 5)

- **Autonomy**: SLAM (slam_toolbox) + Nav2 + frontier exploration + YOLO detection (swap ground-truth oracle via topic remap)
- **Additional robots**: Spot, quadcopter (PX4 SITL)
- **Additional use cases**: search & rescue, inspection, security patrol, multi-robot relay
- **Parallel simulation**: Docker + `GZ_PARTITION` for GPU isolation
- **Isaac Sim**: parallel operation for RL training and photorealistic data (not migration — no reliable SDF→USD pipeline)

---

## Scoring system reference

Overall = weighted sum of categories, normalised by total weight. All scores 0–100.
Grades: S≥95, A≥85, B≥70, C≥55, D≥40, F<40.

| Category        | Default weight | Formula |
|-----------------|---------------|---------|
| Speed           | 0.20 | `0.40 × (timeout−elapsed)/timeout + 0.35 × par_time/avg_detect_time + 0.25 × coverage_rate/par_rate` |
| Accuracy        | 0.25 | `0.45 × detection_rate + 0.30 × (1−fp_rate) + 0.25 × path_efficiency` (path_efficiency=0 until implemented) |
| Safety          | 0.20 | `0.70 × collision_tier + 0.30 × near_miss_tier`; tiers: 0→100, 1–2→80, 3–5→60, 6–10→40, 11+→20 |
| Efficiency      | 0.20 | `0.35 × (1−revisit_ratio) + 0.35 × (coverage/meters)/par_cpm + 0.30 × coverage_pct` |
| Effectiveness   | 0.15 | `Σ (type_weight / Σweights) × (detected_instances / total_instances) × 100`; per-type weights configurable |

All weights are configurable per scenario via `scoring.category_weights` and `scoring.effectiveness_weights`.
Full implementation in [src/metrics/scoring.py](../src/metrics/scoring.py).
