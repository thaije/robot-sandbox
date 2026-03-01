# ARST Project Plan

## Vision

Build a modular Gazebo simulation testbed for testing robot autonomy across diverse scenarios. Phase 1: DerpBot explores an indoor office and locates target objects, evaluated by configurable metrics and composite scoring (Speed / Accuracy / Safety / Efficiency). Teleoperation only in Phase 1; autonomy stack deferred.

For the external autonomous agent interface (sensors, topics, mission endpoint), see [`docs/AUTONOMOUS_AGENT_GUIDE.md`](AUTONOMOUS_AGENT_GUIDE.md).

**Current state:** See [`docs/AGENT_HANDOFF.md`](AGENT_HANDOFF.md).

---

## Requirements (not yet met)

| ID | Requirement | Notes |
|----|-------------|-------|
| SIM-09 | Configurable environment variations (lighting, door states, dynamic obstacles) | **Done** — door states (connected-graph guaranteed), localized lighting (4 presets + 2 compound), vertical/surface placement, flickering lights, patrol bot, smoke emitters, 5 tier YAMLs |
| SIM-12 | Multiple simulator instances in parallel | Future |
| SIM-13 | Deterministic replay (same seed = same scenario) | Not verified |
| MET-13 | Calibrated par values | After first working run |

---

## Remaining implementation steps


### Step 3.15 — Calibrate par values (after all tiers are built)

Run each tier 3–5× with teleoperation (or a reference agent), take median performance per tier, set `par_values` in each tier YAML so median ≈ 70 (B). Grades mean the same thing across tiers: B = competent performance *for this difficulty level*.

**What scales with difficulty** (tune per tier):
- `time_per_detection_par` — higher for hard tiers (dim/closed-door runs take longer)
- `timeout_seconds` — already tier-specific (see table above)
- `category_weights` — based on reference performance.

**What stays constant** (do not vary by tier):
- `coverage_per_meter_par` — physics/geometry constant
- `near_miss_threshold` — not difficulty-dependent
- Grade thresholds (S/A/B/C/D) — same scale for all tiers

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

## Difficulty tiers reference

| File | Lighting | Doors | Extras | Timeout | Spawn |
|---|---|---|---|---|---|
| `easy.yaml` | bright, uniform | open | — | 900 s | (1,1) |
| `medium.yaml` | normal | random | — | 600 s | (1,1) |
| `hard.yaml` | dim + localized | closed | vertical objects | 300 s | (18,13) |
| `brutal.yaml` | flickering + dim | closed | patrol bot + vertical | 180 s | (18,13) |
| `perception_stress.yaml` | flicker + red emergency | random | smoke + patrol | 600 s | (1,1) |

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
