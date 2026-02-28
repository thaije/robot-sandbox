# ARST Project Plan

## Vision

Build a modular Gazebo simulation testbed for testing robot autonomy across diverse scenarios. Phase 1: DerpBot explores an indoor office and locates target objects, evaluated by configurable metrics and composite scoring (Speed / Accuracy / Safety / Efficiency). Teleoperation only in Phase 1; autonomy stack deferred.

**Current state:** See [`docs/AGENT_HANDOFF.md`](AGENT_HANDOFF.md).

---

## Requirements (not yet met)

| ID | Requirement | Notes |
|----|-------------|-------|
| SIM-09 | Configurable environment variations (lighting, door states, dynamic obstacles) | Lighting only; door_states/dynamic obstacles not implemented — see [`docs/environment_variations.md`](environment_variations.md) |
| SIM-11 | Scenario reset without full restart | Sim must be restarted; reset.py stub removed |
| SIM-12 | Multiple simulator instances in parallel | Future |
| SIM-13 | Deterministic replay (same seed = same scenario) | Not verified |
| ~~ROB-03~~ | ~~2D LiDAR~~ | Done |
| ~~MET-06~~ | ~~Exploration coverage %~~ | Done — `ExplorationCoverage` in `src/metrics/exploration_coverage.py` |
| ~~MET-10~~ | ~~Near-miss detection~~ | Done — `NearMissTracker` in `src/metrics/near_miss_tracker.py`; wired into runner + scorecard |
| MET-13 | Calibrated par values | After first working run |
| DOC-01–07 | Documentation | Not started |

---

## Remaining implementation steps

### Step 3.6 — Exploration coverage

Add `gpu_lidar` sensor to DerpBot URDF. Bridge:
```
/{robot_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan
/model/{robot_name}/pose@geometry_msgs/msg/Pose[gz.msgs.Pose
```

Implement `exploration_coverage.py`: subscribe to `/scan` + ground-truth pose. Load `worlds/templates/indoor_office/ground_truth_map.pgm` (binary mask, 0.5m resolution). Per scan: vectorise ray endpoints, trace each with `skimage.draw.line()`, mark cells observed. Coverage % = `100 * observed_cells / explorable_cells`. Publish `Float32` at 1 Hz.

### Step 3.13 — Calibrate par values

Run office scenario 3–5 times with teleoperation. Set `par_values` in scenario YAML so a competent operator scores ~70 (B). Document baseline values in YAML comments.

### Step 3.14 — Environment variations

Implement the robustness variations described in [`docs/environment_variations.md`](environment_variations.md). Priority order from that doc:

1. Localized lighting (per-room point/spot lights in SDF)
2. Vertical object placement (`z_offset` in object_placer + scenario YAML)
3. Flickering lights (gz-transport `UserCommands` timer in scenario runner)
4. Compound scenario YAMLs (easy / hard / brutal / perception_stress)
5. Patrol bot dynamic obstacle (scripted model, embedded in world SDF)
6. Object distribution strategies (placer strategy dispatch)
7. Smoke / particle emitters (forward-looking; no metric impact until visual detection)

Also covers door state implementation: inject door-panel models at generation time when `door_states: closed` or `random` (currently always open — gaps in walls, no door models).

### Steps 4.6–4.7

- **4.6 Batch execution**: `scripts/run_batch.sh` — N runs with different seeds, aggregate mean/std/min/max
- **4.7 Integration test**: scripted movement pattern, verify all metrics record correctly

---

## Future (post Phase 4)

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
