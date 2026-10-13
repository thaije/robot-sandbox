# Brutal-tier batch — derpbot-explorer-v5-master-brutal

Agent: `derpbot-explorer-v5-master-brutal` · Tier: brutal · 2026-10-13

## Setup

- Code: derpbot-explorer worktree `/tmp/derpbot-v5-tiers` pinned at commit
  **24f76581b15c9a7b88dd7e5aa53a1980ed811e08** (24f7658), clean
  (`git status` → no modifications). Sandbox HEAD at run time:
  **7a90174288d29c7b7765076ade5cc3c3544d6357** (robot-sandbox main).
- Protocol: 5 seeds × 2 repeats = 10 runs, 180 s sim each (all TIME_LIMIT),
  speed 2×, pkg1 pin, one sim at a time, sequential, resumable.
- Result dir: `~/Projects/robot-sandbox/results/submissions/derpbot-explorer-v5-master-brutal/`
- Batch wall time ~22 min (15:42:47Z → 16:04:40Z), 10 completed / 0 failed,
  no resumable stop needed.

## Per-run table

| run        | status     | score | grade | found | FP | coll | cov%  | path m |
|------------|-----------|-------|-------|-------|----|------|-------|--------|
| seed1_run1 | TIME_LIMIT | 62.1  | C     | 0.00  | 0  | 0    | 34.3  | 17.1   |
| seed1_run2 | TIME_LIMIT | 62.1  | C     | 0.00  | 0  | 0    | 19.6  | 0.9    |
| seed2_run1 | TIME_LIMIT | 58.3  | C     | 0.17  | 2  | 0    | 29.6  | 17.6   |
| seed2_run2 | TIME_LIMIT | 57.9  | C     | 0.17  | 2  | 0    | 29.5  | 18.8   |
| seed3_run1 | TIME_LIMIT | 55.8  | C     | 0.17  | 0  | 9    | 60.6  | 8.9    |
| seed3_run2 | TIME_LIMIT | 61.1  | C     | 0.17  | 2  | 0    | 53.6  | 17.4   |
| seed4_run1 | TIME_LIMIT | 63.6  | C     | 0.33  | 0  | 4    | 76.5  | 11.7   |
| seed4_run2 | TIME_LIMIT | 71.9  | B     | 0.33  | 0  | 0    | 51.0  | 18.9   |
| seed5_run1 | TIME_LIMIT | 68.6  | C     | 0.17  | 0  | 0    | 58.7  | 17.6   |
| seed5_run2 | TIME_LIMIT | 67.6  | C     | 0.17  | 0  | 0    | 44.5  | 20.9   |

## Means (n=10)

| metric        | value |
|---------------|-------|
| mean score    | 62.9  |
| mean grade    | 9× C, 1× B (seed4_run2 71.9) |
| mean found_ratio | 0.167 (1.00 / 6 objects) |
| mean FP       | 0.6   |
| mean collisions | 1.30 |
| mean coverage | 45.8% |
| mean path     | 15.0 m |

## Completion criterion — found_ratio == 1.0

**0 / 10 runs** completed (all 10 hit the 180 s TIME_LIMIT with ≤ 2 of 6
objects found). No run got close: max found_ratio 0.33 (2/6).

## DOA count

**0** — no dead-on-arrival runs (all statuses are normal TIME_LIMIT).

## Per-class recall

| class             | detected ≥ 1 in runs |
|-------------------|----------------------|
| fire_extinguisher | 6 / 10 |
| first_aid_kit     | 2 / 10 |
| person            | 2 / 10 |

(exit_sign / drill / drink_can / trash_can not observed in any run — brutal
tier mixes additional object classes; detection_by_type in run JSONs is the
source of truth.)

## Comparison vs existing brutal-tier entries

| agent (n) | score | found_ratio | FP | coll | cov% |
|-----------|-------|------------|----|------|------|
| derpbot-explorer-v1 (15)        | 59.8 | 0.056 | 0.40 | 1.00 | 50.7 |
| derpbot-explorer-v2-imu-odom (15) | 62.2 | 0.078 | 0.27 | 0.93 | 50.9 |
| derpbot-explorer-v3-detection-aware-exploration (15) | 62.0 | 0.056 | 0.20 | 0.93 | 43.1 |
| derpbot-explorer-v4-bumper-fix (15) | 61.0 | 0.100 | 0.80 | 0.87 | 42.7 |
| random-baseline (15)            | 52.1 | 0.000 | 0.00 | 4.87 | 56.1 |
| **derpbot-explorer-v5-master-brutal (10, this)** | **62.9** | **0.167** | 0.60 | 1.30 | 45.8 |

### Delta vs v4-bumper-fix (bootstrap CI — both sides ≥ 10 runs)

- found_ratio: +0.067 (0.167 vs 0.100), 95% bootstrap CI **[−0.022, +0.156]** →
  **not significant** at n=10 vs n=15.
- score: +1.9 (62.9 vs 61.0).
- v5 also clears v1–v3 on found_ratio (0.167 vs 0.056/0.078/0.056) but with n=10
  those are raw-number comparisons only.

### Par context (grade thresholds S95/A85/B70/C55/D40; par is human-baseline-derived)

- Human Teleop (perception), brutal tier, n=5: mean score **73.3**, **4.4/6**
  objects found (found_ratio 0.733), coverage ~99.96%, 0.4 collisions.
- v5 at 62.9 = **86% of human-teleop par score**; found_ratio 0.167 vs human
  0.733 — the gap is detection recall under the 180 s budget, not score
  mechanics: v5 travels 15 m/run vs human 64 m, and brutal-tier coverage
  (45.8%) is far below human (~100%).
- Grade mean C (62.9) sits in the C band (55–70); one B run shows the ceiling
  with current recall.

## Verdict

Brutal-tier v5 clean-master is the **best brutal-tier agent entry on
found_ratio to date** (0.167, prior best v4 0.100), but the delta vs v4 is not
significant at this sample size, coverage lags prior entries slightly, and 0/10
runs complete. Collisions roughly unchanged from v1–v4 (1.3 vs ~0.9–1.0);
the seed3_run1 9-collision run is an outlier, rest ≤ 4.

## Artifacts

- Results: `~/Projects/robot-sandbox/results/submissions/derpbot-explorer-v5-master-brutal/` (10 run JSONs + benchmark_submission.yaml + benchmark_run.log)
- Analysis script: `analyze_brutal.py` (this workspace)
- Submission validation: **PASS**

## Provenance

- derpbot-explorer code commit: 24f76581b15c9a7b88dd7e5aa53a1980ed811e08 (24f7658)
- robot-sandbox HEAD: 7a90174288d29c7b7765076ade5cc3c3544d6357 (main)
- Batch: `bash /tmp/derpbot-v5-tiers/scripts/run_benchmark.sh --agent-name derpbot-explorer-v5-master-brutal --difficulties "brutal" --seeds "1 2 3 4 5" --runs-per-seed 2 --speed 2`