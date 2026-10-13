# perception_stress-tier v5 clean-master batch report (derpbot-explorer-v5-master-perception_stress)

**Date:** 2026-10-13
**Agent:** derpbot-explorer-v5-master-perception_stress
**Code:** derpbot-explorer commit `24f76581b15c9a7b88dd7e5aa53a1980ed811e08` (pinned worktree /tmp/derpbot-v5-tiers, clean — never HEAD)
**robot-sandbox HEAD:** `7a90174288d29c7b7765076ade5cc3c3544d6357` (main, v1.0.3)
**Protocol:** 5 seeds × 2 repeats = 10 runs, 600 sim-s each, speed 2×, one sim at a time, resumable
**Submission validation:** PASS (validate_submission.py)
**Wall time:** ~58 min single continuous batch (no resumable stop needed)

## Failed first attempt (documented for the protocol)

The first launch (16:08Z) produced a DOA-class failure: run 1 timed out at 600 s wall
with no result file. Root cause: the batch tmux session was started from a
non-ROS-sourced shell, so `fastdds discovery` inside start_stack.sh failed silently
(`libfastrtps.so.2.14: cannot open shared object file`) and the stack ran without a
discovery server — degraded discovery, scenario result never written. The failure was
diagnosed from the missing `office_perception_stress_001_*.json` file + reproduced the
`fastdds` shared-library failure outside a sourced env. Fix: launch via
`bash -lc 'source /opt/ros/jazzy/setup.bash; …'`. Relaunch (16:29Z) bound port 11811
immediately and all 10 runs completed normally. The one lost run is NOT part of the 10
below (runner counts it as failed; the relaunched batch ran 10/10 fresh, failed=0).

## Per-run results

| run | score | grade | found_ratio | FP | coll | cov % | precision | status |
|---|---|---|---|---|---|---|---|---|
| seed1 run1 | 44.4 | D | 0.50 | 4 | 25 | 63.7 | 0.43 | TIME_LIMIT |
| seed1 run2 | 50.5 | D | 0.50 | 4 | 0 | 63.6 | 0.43 | TIME_LIMIT |
| seed2 run1 | 45.6 | D | 0.17 | 0 | 8 | 67.3 | 1.00 | TIME_LIMIT |
| seed2 run2 | 65.3 | C | 0.83 | 2 | 0 | 75.5 | 0.71 | TIME_LIMIT |
| seed3 run1 | 49.9 | D | 0.50 | 7 | 0 | 73.1 | 0.30 | TIME_LIMIT |
| seed3 run2 | 50.2 | D | 0.50 | 5 | 0 | 71.0 | 0.38 | TIME_LIMIT |
| seed4 run1 | 45.9 | D | 0.33 | 2 | 0 | 61.1 | 0.50 | TIME_LIMIT |
| seed4 run2 | 53.7 | D | 0.33 | 0 | 0 | 79.9 | 1.00 | TIME_LIMIT |
| seed5 run1 | 44.1 | D | 0.00 | 0 | 0 | 59.9 | 1.00 | TIME_LIMIT |
| seed5 run2 | 42.3 | D | 0.00 | 0 | 10 | 72.3 | 1.00 | TIME_LIMIT |

**Means:** score 49.2 ± 6.7 (grade D) · found_ratio 0.367 (2.20/6) · cov 68.7 % ·
collisions 4.3 · false positives 2.4 · 0 completions (all TIME_LIMIT at 600 s)
**Grade distribution:** 9× D, 1× C
**Completion criterion (found_ratio == 1.0):** 0/10 runs
**DOA count:** 0 DOA-class reliability events in this batch (one start_stack failure
attributable to launch env, not to the explorer's SLAM startup path; see above)

## Per-class recall (over 10 runs, detected/total)

| class | recall |
|---|---|
| fire_extinguisher | 9/30 (0.30) |
| first_aid_kit | 8/20 (0.40) |
| person | 5/10 (0.50) |

Scenario: 3 fire extinguishers + 2 first aid kits + 1 person = 6 mission targets.
Flicker red emergency lighting (0.8 s period) + smoke: detector recall is the
bottleneck, not coverage (cov is a healthy 60–80 %, but objects are missed).

## Comparison vs prior perception_stress-tier entries

| agent | n | score | found_ratio | cov % | grades |
|---|---|---|---|---|---|
| **v5 clean-master (this)** | 10 | **49.2 ± 6.7** | **0.367** | 68.7 | 9D/1C |
| v4-bumper-fix | 15 | 45.9 ± 8.2 | 0.200 | 68.4 | 2C/9D/4F |
| v3-detection-aware | 15 | 44.7 ± 9.5 | 0.167 | 55.9 | 3C/6D/6F |
| v1 | 15 | 45.2 ± 6.7 | 0.100 | 56.5 | 1C/10D/4F |
| human-baseline-perception (par) | 5 | 86.6 ± 3.7 | 1.000 | 100.0 | 3A/2B |

Both sides ≥10 runs → delta with bootstrap 95 % CI (10,000 resamples, seed 42):
- score: v5 49.2 vs v4 45.9 → **+3.3, CI [−2.0, +9.1] — not significant**
- found_ratio: v5 0.367 vs v4 0.200 → **+0.167, CI [−0.028, +0.356] — not significant**

v5 is the highest perception_stress found_ratio to date (nearly 2× v4, 3.7× v1) and the
only perception_stress agent entry with zero F-grade runs, but with n=10 the delta does
not clear significance. Note the v5 entry also has no F runs at all while every prior
agent had 4–6.

**Par context (grade thresholds S95/A85/B70/C55/D40; par is human-baseline-derived):**
v5's mean 49.2 sits between D(40) and C(55). Human teleop par on the same tier is
86.6 (A-band, 6/6 found, 100 % coverage in 600 s) — v5 is at 57 % of par score,
with the gap almost entirely in detector recall under flicker+smoke (human finds
6/6; v5 averages 2.2/6) rather than navigation/coverage.

## Weak run

seed5 (both runs): found_ratio 0.00 — the detector never confirmed a single mission
target in 600 s despite 60–72 % coverage. Precision 1.00 in those runs (no spam
submissions), suggesting the threshold gating held but the detector simply could not
see the targets under the stress lighting this seed produced.

## Reproduction

```
git worktree add /tmp/derpbot-v5-tiers 24f7658   # in derpbot-explorer
bash -lc 'source /opt/ros/jazzy/setup.bash; cd /tmp/derpbot-v5-tiers && \
  bash scripts/run_benchmark.sh --agent-name derpbot-explorer-v5-master-perception_stress \
  --difficulties perception_stress --seeds "1 2 3 4 5" --runs-per-seed 2 --speed 2'
```
Results: `~/Projects/robot-sandbox/results/submissions/derpbot-explorer-v5-master-perception_stress/`
Log: `benchmark_run.log` (10× START/DONE lines, failed=0)