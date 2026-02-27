---
name: arst-nav
description: Navigate DerpBot through an ARST scenario (robot_inspect.py + world_state.py). Use when running any ARST scenario as an agent — fetching maps, driving, checking detections, finding all objects within the time limit.
user_invocable: true
---

# arst-nav — DerpBot Navigation Skill

ARST scenario: find all objects within the time limit. Layout, object types, counts, and time limit vary per scenario — read from the scenario config or the initial map.

All commands run from repo root. PYTHONPATH and ROS env are set by hooks — no prefix needed.

---

## Tool: world_state.py — map + object list

```bash
# PNG always written (default: arst_world_map.png in repo root); also prints object list + robot pose
python3.12 scripts/world_state.py
# Then immediately read the image:
# Read arst_world_map.png

# Override output path:
python3.12 scripts/world_state.py --png /tmp/map.png
```

**Map legend:**
- Blue circle + arrow = robot, arrow = heading
- Coloured circles = unfound objects (colours per type)
- Grey = found (already counted), Brown = furniture, Dark = walls

**Never call `status` just for pose — the map already shows it.**

---

## Tool: robot_inspect.py — drive + detections

```bash
# Closed-loop rotation (accurate; + = CCW/left, - = CW/right)
python3.12 scripts/robot_inspect.py rotate <degrees>

# Closed-loop translation (accurate; + = forward, - = backward)
python3.12 scripts/robot_inspect.py move <metres>

# Open-loop drive (use only for brief corrections ≤ 2 s)
python3.12 scripts/robot_inspect.py drive <vx> <wz> <seconds>

# What's in the camera frame right now
python3.12 scripts/robot_inspect.py detections

# Camera snapshot (PNG)
python3.12 scripts/robot_inspect.py snapshot
```

Prefer `rotate` + `move` over `drive` — they use odom feedback and are far more accurate.

---

## Navigation protocol

### 1. Session start

If the scenario isn't running yet:
```bash
./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless --timeout 300
```
Startup ~5 s. Then fetch the initial map:
```
python3.12 scripts/world_state.py   →   Read arst_world_map.png
```
Identify unfound objects + their rooms. Note the time limit. Plan room order.

### 2. Core loop (repeat until all found or time runs out)
```
move / rotate  →  world_state.py  →  Read map  →  detections (if near objects)
```
- Refresh map **after every move**. Don't skip — catches collisions and pose drift.
- Call `detections` and `world_state.py` **in the same tool-call batch** when possible.
- React to what the map shows; don't plan the full route upfront.

### 3. Room search pattern
**Rotate 360° at room centre** — spots most objects in one pass.
```bash
python3.12 scripts/robot_inspect.py rotate 360
```
Then check detections. Furniture may occlude; circle the perimeter if needed.

### 4. Doorways
- Aim straight at the gap; `move` 1–2 m to clear the frame fully before turning.
- Small objects may be in doorways — risk getting stuck. Use another door if available.

### 5. Object found confirmation
`world_state.py` auto-reads `/tmp/arst_worlds/detections_live.json`. An object turns grey on the map as soon as the scenario records it — no extra action needed.

---

## Efficiency rules

- **No redundant status calls** — map shows pose.
- **Batch parallel tool calls**: detections + map refresh in one message.
- **Prefer rotate+move** over drive for precise manoeuvres.
- **360 sweep** at each room centre before detailed search.
- **Move through doorways decisively** — hesitation causes doorpost collisions.
- **Goal is completion within the time limit** — don't over-optimise path.
