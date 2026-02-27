---
name: arst-nav
description: Navigate DerpBot through an ARST scenario (robot_control.py + world_state.py). Use when running any ARST scenario as an agent — fetching maps, driving, checking detections, finding all objects within the time limit.
user_invocable: true
---

# arst-nav — DerpBot Navigation Skill

ARST scenario: find all objects within the time limit. Layout, object types, counts, and time limit vary per scenario — read from the scenario config or the initial map.

All commands run from repo root. PYTHONPATH and ROS env are set by hooks — no prefix needed.

---

## Tool: world_state.py — map + full status

```bash
# PNG always written (default: arst_world_map.png in repo root)
# Also prints: object list, visible tags, collision status, robot pose
python3.12 scripts/world_state.py
# Then immediately read the image:
# Read arst_world_map.png

# Override output path:
python3.12 scripts/world_state.py --png /tmp/map.png
```

**Output includes:**
- Object list with status: `not found`, `FOUND ✓`, or `not found  [visible 👁]`
- Robot pose (world frame, yaw, facing direction)
- Collision status: `⚡ COLLISION — robot is currently touching an obstacle` or `no collision`
- Clear error if simulation is not running

**Map legend:**
- Blue circle + arrow = robot; RED ring on robot = currently colliding
- White ring on unfound object = currently visible in camera (LOS-checked)
- Coloured circles = unfound objects (red=fire_ext, green=first_aid, yellow=hazard)
- Grey = found, Brown = furniture, Dark = walls

**This is the single source of truth for detections — no separate detections call needed.**

---

## Tool: robot_control.py — drive only

```bash
# Closed-loop rotation (accurate; + = CCW/left, - = CW/right)
python3.12 scripts/robot_control.py rotate <degrees>

# Closed-loop translation (accurate; + = forward, - = backward)
python3.12 scripts/robot_control.py move <metres>

# Open-loop drive (use only for brief corrections ≤ 2 s)
python3.12 scripts/robot_control.py drive <vx> <wz> <seconds>

# Camera snapshot (PNG)
python3.12 scripts/robot_control.py snapshot
```

Prefer `rotate` + `move` over `drive` — they use odom feedback and are far more accurate.
Both scripts print a clear message if the simulation is not running.

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
move / rotate  →  world_state.py  →  Read map
```
- Refresh map **after every move** — shows collision status, visible tags, pose.
- No separate detections call needed — visible objects are tagged in world_state.py output.
- React to what the map shows; don't plan the full route upfront.

### 3. Room search pattern
**Rotate 360° at room centre** — spots most objects in one pass.
```bash
python3.12 scripts/robot_control.py rotate 360
```
Then call world_state.py and check for `[visible 👁]` tags. Furniture may occlude; circle the perimeter if needed.

### 4. Doorways
- Aim straight at the gap; `move` 1–2 m to clear the frame fully before turning.
- Move through doorways decisively — hesitation causes doorpost collisions.

### 5. Object found confirmation
An object turns grey on the map as soon as the scenario records it.
Objects tagged `[visible 👁]` are currently in the camera's view (wall-LOS-checked).

### 6. Wall clipping
The camera is physically mounted 10 cm behind the robot's front collision face, so it cannot
clip through walls. The LOS check in world_state.py additionally filters any detections
that cross a wall cell in the occupancy grid.

---

## Efficiency rules

- **No redundant status calls** — map shows pose.
- **world_state.py after every move** — replaces separate detections + map calls.
- **Prefer rotate+move** over drive for precise manoeuvres.
- **360 sweep** at each room centre before detailed search.
- **Move through doorways decisively** — hesitation causes doorpost collisions.
- **Goal is completion within the time limit** — don't over-optimise path.
