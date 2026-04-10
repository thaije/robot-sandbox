# ROADMAP — robot-sandbox

Table of contents for the issue tracker. Each task gets a short entry here with a link to the GitHub issue where the full plan and discussion live. Do not copy issue content into this file.

Current state lives in [`STATE.md`](STATE.md). History lives in closed issues + commits.

---

## Ground rules (apply to all upcoming work)

- **No hardcoded scenario values.** Difficulty, sensors, and metrics stay YAML-driven and configurable.
- **No dead-end retries.** `gh issue list --state closed --label dead-end` before proposing a change in an area with prior attempts.

---

## Next

### Batch execution · [#3](https://github.com/thaije/robot-sandbox/issues/3)
`scripts/run_batch.sh` — N runs with different seeds, aggregate mean/std/min/max. Builds on `run_scenario.sh` + existing results JSON format.

### Integration test · [#4](https://github.com/thaije/robot-sandbox/issues/4)
Scripted movement pattern, verify all metrics record correctly end-to-end. Prefer Python pytest; minimal scenario in `config/scenarios/tests/`.

---

## Open backlog

Known issues not currently prioritised. Full details in the linked issues.

- [#1](https://github.com/thaije/robot-sandbox/issues/1) — Add sound to simulation
- [#2](https://github.com/thaije/robot-sandbox/issues/2) — Add sensor jitter
- [#5](https://github.com/thaije/robot-sandbox/issues/5) — Multiple simulator instances (SIM-12, parallel runs via Docker + GZ_PARTITION)

Run `gh issue list --state open --label backlog` for the live list.

---

## Workflow

- **Starting a task:** read `STATE.md`, `ROADMAP.md`, and the task's issue. Check closed dead-ends in the same area.
- **During a task:** log findings and decisions as comments on the issue, not in this doc.
- **New finding:** `gh issue create` with `bug` / `dead-end` / `backlog` label. Cross-link related issues.
- **Completing a task:** close the issue with a final comment (outcome + commit SHA). Delete the task entry from "Next" here. Update `STATE.md` only if a new *invariant* came out of it.
- **Commits:** reference the issue, e.g. `feat(scripts): batch runner (#3)`. GitHub auto-links.
