# Environment Variations — ARST Robustness Testing

**Context**: `arst-nav` cheat tools (100% accurate detection, full ground-truth map) are dev/eval only.
The real robot has camera + odometry + some other real sensors only. Variations here are designed to stress future perception-based agents,
not just the current ground-truth pipeline. Implement in SDF/config; the scenario runner picks them up.

---

## Variations

### 1. Localized Lighting
**What**: Replace single directional ceiling light with per-room point/spot lights. Assign distinct intensities and colours per zone (e.g. one office bright, one dim, corridor emergency-red, corridor exit sign bright-green).

**Implementation**: Pure SDF. Add `<light type="point">` or `<light type="spot">` models per room. Drop `<scene><ambient>` to a low baseline so unlit rooms are genuinely dark. Add named presets to `config.yaml` `lighting_presets`. `WorldGenerator` injects the matching preset block.

**Note**: ground-truth bbox camera ignores lighting — effect is on future visual detector + navigation. Coloured lights (red emergency, green exit) can bias real RGB-based detectors → interesting false-positive stress.

**Priority**: High. Pure SDF, no code beyond config wiring.

---

### 2. Flickering Lights
**What**: One or more lights strobes at a configurable period (e.g. 1 Hz, 5 Hz). Models a failing fluorescent or emergency strobe. Creates temporal detection instability for any frame-based detector.

**Implementation**: Scenario runner side — Python gz-transport client publishes to the `UserCommands` service (`gz.msgs.Light`) on a timer thread. Light name(s) and period come from scenario YAML (`variations.flicker: [{name: office_a_light, period_s: 0.5}]`). No SDF plugin needed; `gz::sim::systems::UserCommands` is already loaded.

**Note**: pairs naturally with localized lighting above (flicker a specific room's model light, not the global directional).

**Priority**: High. ~50 lines Python in scenario runner, zero SDF change once localized lights exist.

---

### 3. Smoke / Particle Effects
**What**: Localised particle emitter (smoke, haze) in a zone — corridor, one office, or meeting room. Degrades visibility for any camera-based detector.

**Implementation**: SDF `<particle_emitter>` block inside a static model. Ogre2 renders it natively. Emitter position, rate, lifetime, size come from scenario YAML. `WorldGenerator` injects emitter models.

**Note**: **No effect on current ground-truth bbox camera** — semantic labels render regardless of particles. Only meaningful once real RGB + YOLO detection is in. Implement now as forward-looking infrastructure; metric impact follows.

**Priority**: Medium. Low SDF effort but zero current metric impact. Good to wire up early.

---

### 4. Vertical Object Placement
**What**: Target objects placed on top of furniture (desk surface z=0.75 m, cabinet top z=1.0 m) instead of only on the floor. Robot must approach from the correct angle to get line-of-sight.

**Implementation**: Add optional `z_offset` and `surface` fields to object config entries. `object_placer.py` reads `z_offset`; `surface` can pin placement to a specific furniture zone (cross-reference with `config.yaml` obstacle list for surface height). Also verify bbox camera pose in URDF covers elevated objects.

```yaml
# scenario YAML example
objects:
  - type: fire_extinguisher
    count: 1
    placement: surface
    surface: desk   # picks any desk-class obstacle
```

**Priority**: High. Real spatial challenge; couples navigation and detection.

---

### 5. Dynamic Obstacles — Patrol Model
**What**: One or more autonomous models (vacuum robot disc, human silhouette, dog) patrolling scripted waypoint loops. Has real physics (bumper collisions register). Texture/label chosen to stress future false-positive behaviour in visual detectors.

**Implementation (Option B — scripted autonomous model)**:
- SDF model: simple geometry (cylinder for vacuum bot, capsule for human) with `DiffDrive` plugin and collision geometry. Textured to look like the target entity.
- Embedded in world SDF at generation time (same pattern as DerpBot — required for contact sensor correctness).
- Patrol controller: lightweight Python ROS 2 node publishing `cmd_vel` on a waypoint loop; started by `SimulationLauncher` alongside other bridges.
- Scenario YAML: `dynamic_obstacles: [{model: vacuum_robot, patrol: [[x1,y1],[x2,y2],...]}]`.

**Texture options and detection implications**:
| Model | Visual similarity to target | Expected future effect |
|---|---|---|
| Vacuum disc | Low | Collision/near-miss only |
| Human silhouette | Medium (person near hazard sign?) | False positive risk for visual detector |
| Dog | Low-medium | False positive for small objects |

**Note**: actors (`<actor>` SDF) are non-collidable by default — avoid them. The scripted model approach gives real physics and correctly fires `collision_count`.

**Priority**: Medium-High. Most complex, highest payoff for collision/avoidance testing.

---

### 6. Object Distribution Strategies
**What**: Named placement strategies beyond random. Controlled via scenario YAML, no code change needed beyond `object_placer.py` strategy dispatch.

| Strategy | Description | Tests |
|---|---|---|
| `random` (current) | uniform random across all zones | baseline |
| `clustered` | all objects in one named zone | over-exploration, early-termination |
| `spread` | one object per zone, far corners | path efficiency, coverage |
| `cornered` | objects at zone boundary extremes | wall-following, approach precision |
| `elevated` | all on surfaces (requires item 4 above) | vertical coverage |

**Implementation**: `object_placer.py` strategy pattern. Each strategy is a placement function keyed by name.

**Priority**: Medium. YAML-driven once placer supports strategies; useful for systematic sweeps.

---

### 7. Spawn Variations & Seed Sweeps
**What**: Vary robot starting position and random seed to separate agent skill from luck.

**Implementation**: Already supported in scenario YAML (`spawn_pose`, `random_seed`). Add named compound scenario files (see Difficulty Tiers below). Seed sweep = shell loop, no code needed.

**Priority**: Low effort, already wired. Just needs scenario YAML files.

---

## Compound Difficulty Tiers

Combining the above into named scenario variants (each a YAML file in `config/scenarios/`):

| Tier | Lighting | Doors | Clutter | Timeout | Spawn | Extras |
|---|---|---|---|---|---|---|
| `easy` | bright, uniform | open | low | 900 s | (1,1) | — |
| `medium` *(current)* | normal | random | medium | 600 s | (1,1) | — |
| `hard` | dim + localized | closed | high | 300 s | (18,13) | vertical objects |
| `brutal` | flickering + dim | closed | high | 180 s | (18,13) | patrol bot + vertical |
| `perception_stress` | flicker + red emergency | random | medium | 600 s | (1,1) | smoke + human patrol |

---

## Priority Order

| # | Item | Effort | Current metric impact | Future impact |
|---|---|---|---|---|
| 1 | Localized lights | Low | Navigation | Visual detection |
| 2 | Vertical placement | Medium | Occlusion/approach | Same |
| 3 | Flickering lights | Low (after #1) | Navigation | Visual detection |
| 4 | Compound scenario YAMLs | Low | Systematic coverage | Same |
| 5 | Patrol bot | Medium-High | Collision/near-miss | False positives |
| 6 | Distribution strategies | Medium | Path efficiency | Same |
| 7 | Smoke | Low | None yet | Visual detection |
