# ARST Project Plan

## Vision

Build a modular Gazebo simulation testbed for testing robot autonomy across diverse scenarios. Phase 1: DerpBot explores an indoor office and locates target objects, evaluated by configurable metrics and composite scoring (Speed / Accuracy / Safety / Efficiency). Teleoperation only in Phase 1; autonomy stack deferred.

For the external autonomous agent interface (sensors, topics, mission endpoint), see [`docs/AUTONOMOUS_AGENT_GUIDE.md`](AUTONOMOUS_AGENT_GUIDE.md).

**Current state:** See [`docs/AGENT_HANDOFF.md`](AGENT_HANDOFF.md).

---

## Requirements (not yet met)

| ID | Requirement | Notes |
|----|-------------|-------|
| SIM-12 | Multiple simulator instances in parallel | Future |

---

## Remaining implementation steps

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
| Speed           | 0.20 | `min(100, completion_time_par / elapsed × 70)`; par → B (70) |
| Accuracy        | 0.30 | `0.60 × found_ratio × 100 + 0.40 × precision × 100` |
| Safety          | 0.20 | `0.70 × collision_tier + 0.30 × near_miss_tier`; tiers: 0→100, 1–2→80, 3–5→60, 6–10→40, 11+→20 |
| Efficiency      | 0.10 | `0.60 × cpm_score + 0.40 × path_score`; each `min(100, par/actual × 70)` — par → B (70) |
| Effectiveness   | 0.15 | `0.65 × detection_completeness + 0.35 × exploration_coverage%`; detection uses per-type weights |

All weights are configurable per scenario via `scoring.category_weights` and `scoring.effectiveness_weights`.
Full implementation in [src/metrics/scoring.py](../src/metrics/scoring.py).
