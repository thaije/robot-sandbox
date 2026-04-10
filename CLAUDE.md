# CLAUDE.md — robot-sandbox

## Session start

Tjalling owns this. Start: say hi + 1 motivating line.

Read in order before doing anything:
1. This doc — working style
2. `docs/STATE.md` — what's built, current state, invariants
3. `docs/ROADMAP.md` — next work, TOC of the issue tracker

Do NOT skip reading these files!

Open and closed work lives in GitHub issues (`gh issue list`). Before proposing a change in a previously-touched area, check closed issues: `gh issue list --state closed --label dead-end`.

---

## Hard rules

**Testbed quality**
The goal is a configurable, extensible sim testbed — not a one-off scenario.
- Difficulty, sensors, and metrics are YAML-driven; no hardcoded scenario-specific values.
- Fix root cause (not band-aids). Same approach failing twice: stop, summarise what you know, list ≥3 alternatives, ask Tjalling.

**Research before building**
Web-search first for any ROS 2, Gazebo, or sim approach. Quote exact errors; prefer 2025–2026 sources. Don't invent APIs from memory.

**Root-cause fixes only**
No band-aids or workarounds that mask the real problem. If the same approach fails twice: stop, summarise what you know, list ≥ 3 alternative approaches, and ask Tjalling before continuing.

**Environment is changeable**
Launch files, config, SDF templates, metrics — all fair game. If something is a fundamental bottleneck (CPU, GPU), flag it explicitly. Don't silently work around it.

---

## Git

- Commit after any significant improvement using `committer` (Conventional Commits: `feat|fix|refactor|docs|perf|chore`).
- Don't push unless Tjalling asks.
- Keep commits small and reviewable; no repo-wide search/replace scripts.
- Branch changes require user consent.
- Destructive ops forbidden unless explicit (`reset --hard`, `clean`, `restore`, `rm`).

---

## Docs

**One fact, one home.** Don't duplicate content across files.

- **`docs/STATE.md`** — invariants + current architecture/config. ≤3 lines each, telegraph-style. Update only when a behaviour/config/API change produces a *new invariant* future agents need in-context.
- **`docs/ROADMAP.md`** — short TOC of the issue tracker. One entry per active task with a link. Full plans live in the GitHub issue, not here.
- **GitHub issues** — anything with a lifecycle: tasks, bugs, dead-ends, backlog. Labels: `bug`, `backlog`, `dead-end`. Cross-link related issues.
- **Commit messages** — the "why" of code changes. Reference the issue (`feat(metrics): X (#3)`). Commits + closed issues = project history.
- No new doc files without asking.
- Update docs when behaviour/API changes — don't ship without docs.

---

## Code quality

- Files approaching ~500 LOC: split or refactor before adding more.
- After any major addition: check whether existing code can be simplified, merged, or removed.
- Prefer end-to-end verification; if blocked, say what's missing.
- New deps: quick health check (recent releases, adoption).

---

## Way of working
- Style: telegraph; noun-phrases ok; drop grammar; min tokens.
- Web: search early; quote exact errors; prefer 2025–2026 sources.
- Unsure: read more code; if still stuck, ask w/ short options.
- Conflicts: call out; pick safer path.
- Leave breadcrumb notes in thread.
- Unrecognised changes: assume other agent; keep going; focus your changes. If causes issues, stop + ask.

---

## Flow & Runtime
- Python interpreter: always `python3.12`. `python3` may resolve to another venv.

---

## Tools

### gh
- GitHub CLI for issues/PRs/CI/releases. Given issue/PR URL: use `gh`, not web search.
- `gh issue view <url> --comments`, `gh pr view <url> --comments --files`

### tmux
- Use for persistent/interactive tasks.
- Start: `tmux new -s <name> -d '<command>'`
- View: `tmux capture-pane -t <name> -p -S -30`
- Manage: `tmux ls` / `tmux kill-session -t <name>`

### Serena MCP (symbol navigation)
- **Use for Python**: `find_symbol`, `find_referencing_symbols`, `replace_symbol_body` instead of Read+Grep when you know the symbol name.
- **Skip for**: YAML, SDF, URDF, shell scripts, launch files — no language server benefit.
- First run downloads language server via uvx — normal, wait it out.

### ast-grep
- Structural code search: `ast-grep --lang python -p '$FUNC($$$)' src/`
- Use over grep when matching code structure matters.

### Skills
- **`arst-nav`**: drive DerpBot through a scenario (navigation, snapshot, detection).
- **`arst-test`**: observer/tester — start sim, watch an agent run, read scores.
