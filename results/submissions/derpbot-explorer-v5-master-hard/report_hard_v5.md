# Hard-tier v5 clean-master batch — report (t_9b96c668)

**Agent:** `derpbot-explorer-v5-master-hard` · **Code:** derpbot-explorer pinned at `24f76581b15c9a7b88dd7e5aa53a1980ed811e08` (worktree `/tmp/derpbot-v5-tiers`, clean, left intact) · **Sandbox:** robot-sandbox HEAD `7a90174288d29c7b7765076ade5cc3c3544d6357` (main), sandbox v1.0.3 (local tag = a53295a)
**Protocol:** hard tier, 5 seeds x 2 repeats = 10 runs, 300 s sim budget each, speed 2x, pkg1 pin, one sim at a time, resumable runner. 10/10 runs completed, 0 failed, no resumable stop needed. Submission validation: **PASS**. Master repo untouched (batch ran entirely from the pinned worktree).

## Per-run table (all TIME_LIMIT at 300 s — none completed)

| run | status | score | grade | found | FP | coll | cov% | path m | s |
|---|---|---|---|---|---|---|---|---|---|
| hard_seed1_run1 | TIME_LIMIT | 45.3 | D | 0.00 | 0 | 0 | 17.8 | 0.0 | 300 |
| hard_seed1_run2 | TIME_LIMIT | 59.9 | C | 0.33 | 1 | 0 | 43.8 | 29.1 | 301 |
| hard_seed2_run1 | TIME_LIMIT | 59.2 | C | 0.33 | 1 | 0 | 36.1 | 26.0 | 300 |
| hard_seed2_run2 | TIME_LIMIT | 59.6 | C | 0.33 | 1 | 0 | 36.4 | 24.0 | 301 |
| hard_seed3_run1 | TIME_LIMIT | 56.4 | C | 0.33 | 4 | 0 | 55.5 | 37.8 | 301 |
| hard_seed3_run2 | TIME_LIMIT | 66.2 | C | 0.50 | 3 | 0 | 71.6 | 32.5 | 301 |
| hard_seed4_run1 | TIME_LIMIT | 64.3 | C | 0.33 | 0 | 0 | 51.5 | 26.6 | 301 |
| hard_seed4_run2 | TIME_LIMIT | 72.1 | B | 0.67 | 3 | 0 | 69.5 | 27.8 | 300 |
| hard_seed5_run1 | TIME_LIMIT | 67.6 | C | 0.33 | 0 | 0 | 76.6 | 32.5 | 301 |
| hard_seed5_run2 | TIME_LIMIT | 60.6 | C | 0.17 | 0 | 0 | 58.7 | 36.9 | 301 |

**Means (n=10):** score **61.1** (band C; thresholds S95/A85/B70/C55/D40) · found_ratio **0.333** (2.00/6) · FP **1.3** · collisions **0.00** · coverage **51.8 %** · path **27.3 m**

**Completion criterion found_ratio==1.0: 0/10** (all runs hit the 300 s TIME_LIMIT; no COMPLETED runs, same as the medium batch).
**DOA count: 0** (every run reached TIME_LIMIT normally; no DEAD/SLAM-loss runs — slam sentinel chain held).

## Per-class recall (mission-target classes; hard tier GT = 3 fire_extinguisher + 2 first_aid_kit + 1 person, plus non-target clutter)

| class | detections across runs | found in runs |
|---|---|---|
| fire_extinguisher | 8 | 8/10 runs |
| first_aid_kit | 8 | 6/10 runs |
| person | 4 | 4/10 runs |

Anomaly worth noting: seed1_run1 shows meters_traveled = 0.0 yet coverage 17.8 % and zero detections — the agent barely moved in the 300 s budget (likely slow startup/SLAM init eating the short hard budget; not a DOA, but the weakest run and the main hard-tier risk: with only 300 s, a slow start is unrecoverable).

## Comparison vs prior hard-tier entries (comparison basis: results/v5_tier_comparison_basis.md)

| entry | n | mean found_ratio | mean score |
|---|---|---|---|
| **v5 clean-master (this batch)** | 10 | **0.333** (2.00/6) | **61.1** |
| v4-bumper-fix | 15 | 0.089 (0.53/6) | 54.3 |
| v3-detection-aware | 15 | 0.122 | — |
| v2-imu-odom | 15 | 0.133 | — |
| v1 | 15 | 0.122 | — |
| human-baseline-perception (par context, n=5 → no CI) | 5 | 0.967 (5.8/6) | — |
| random-baseline | 15 | 0.000 | — |

**Delta vs v4-bumper-fix: +0.244 found_ratio (0.333 vs 0.089), 95 % bootstrap CI [+0.117, +0.367] (10 000 resamples) — CI excludes 0 ⇒ significant.** Mean score delta +6.8 (61.1 vs 54.3).

Both sides have ≥10 runs so the bootstrap CI is valid per the comparison-basis note.

**Flag from the note, checked:** v4 was the *worst* hard-tier agent entry (v1–v3 at 0.122–0.133). v5 at 0.333 clears all of them — v5 is roughly 2.4–3.7x the prior agent entries' found_ratio, so no below-prior-generation flag is needed.

**Par context:** human-baseline-perception hard is 0.967 (n=5, context only). v5 sits at 0.333, i.e. ~34 % of human par on hard — a large tier gap vs the human teleop baseline remains, consistent with the time budget (300 s vs 212.9 s human completion-time par) and low coverage (51.8 % vs human-scale coverage-per-meter).

## Verdict

Hard-tier v5 clean-master: 10/10 runs valid, mean found_ratio 0.333 (+0.244 vs v4, significant by bootstrap CI), 0 DOA, 0 collisions, mean grade C (61.1). Best result of any agent entry on hard tier to date; still far from human par. Results in `results/submissions/derpbot-explorer-v5-master-hard/` (10 run JSONs + benchmark_submission.yaml + benchmark_run.log).