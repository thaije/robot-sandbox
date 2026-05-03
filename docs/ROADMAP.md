# ROADMAP — robot-sandbox

Table of contents for the issue tracker. Each task gets a short entry here with a link to the GitHub issue where the full plan and discussion live. Do not copy issue content into this file.

Current state lives in [`STATE.md`](STATE.md). History lives in closed issues + commits.

---

## Ground rules (apply to all upcoming work)

- **No hardcoded scenario values.** Difficulty, sensors, and metrics stay YAML-driven and configurable.
- **No dead-end retries.** `gh issue list --state closed --label dead-end` before proposing a change in an area with prior attempts.

---

## Next

### Benchmark: random agent baseline runs · [#8](https://github.com/thaije/robot-sandbox/issues/8)
Run #7 across full benchmark protocol (seeds 1–5 × 3 runs × 5 difficulties). Depends on #7, #6.

### Benchmark: human teleop baseline runs · [#9](https://github.com/thaije/robot-sandbox/issues/9)
Teleop sessions for easy + medium, seeds 1–5 × 3 runs. Oracle detections. Depends on #6.

### Benchmark: static leaderboard page · [#10](https://github.com/thaije/robot-sandbox/issues/10)
`results/leaderboard.json` + `docs/leaderboard.html`. Depends on #6, #8, #9.

### Benchmark: versioned release v1.0-benchmark · [#11](https://github.com/thaije/robot-sandbox/issues/11)
Freeze scenario/scoring/seeds/protocol; git tag. Depends on #6–#10.

---

## Open backlog

Known issues not currently prioritised. Full details in the linked issues.

- [#1](https://github.com/thaije/robot-sandbox/issues/1) — Add sound to simulation
- [#2](https://github.com/thaije/robot-sandbox/issues/2) — Add sensor jitter
- [#3](https://github.com/thaije/robot-sandbox/issues/3) — Batch execution script (autonomy devs run their own parallel instances; see [derpbot-explorer/scripts/start_stack.sh](https://github.com/thaije/derpbot-explorer/blob/master/scripts/start_stack.sh) as reference)
- [#5](https://github.com/thaije/robot-sandbox/issues/5) — Multiple simulator instances (SIM-12, parallel runs via Docker + GZ_PARTITION)

Run `gh issue list --state open --label backlog` for the live list.

---

## Workflow

- **Starting a task:** read `STATE.md`, `ROADMAP.md`, and the task's issue. Check closed dead-ends in the same area.
- **During a task:** log findings and decisions as comments on the issue, not in this doc.
- **New finding:** `gh issue create` with `bug` / `dead-end` / `backlog` label. Cross-link related issues.
- **Completing a task:** close the issue with a final comment (outcome + commit SHA). Delete the task entry from "Next" here. Update `STATE.md` only if a new *invariant* came out of it.
- **Commits:** reference the issue, e.g. `feat(scripts): batch runner (#3)`. GitHub auto-links.
