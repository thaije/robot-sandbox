# Visual Testing Workflow

For iterating on smoke, lights, materials, furniture rendering, etc.

---

## Pattern

1. **Create a minimal test scenario YAML** (`config/scenarios/<name>_test.yaml`)
   - Fix lighting to `normal` unless you're testing lighting
   - Fix `door_states: open`
   - Place only the relevant objects/obstacles
   - Use an impossible success condition so it runs until timeout:
     ```yaml
     success_criteria:
       mode: "any_of"
       conditions:
         - metric: "meters_traveled"
           operator: ">="
           value: 9999
     ```

2. **Find a seed that places objects where you need them**
   ```python
   python3.12 -c "
   import sys, yaml, random; sys.path.insert(0, 'src')
   from world_manager.object_placer import ObjectPlacer
   with open('worlds/templates/indoor_office/config.yaml') as f:
       cfg = yaml.safe_load(f)
   placer = ObjectPlacer(cfg)
   specs = [{'type': 'fire_extinguisher', 'count': 1, 'placement': 'random'}]
   for seed in range(1, 100):
       placed = placer.place(specs, seed=seed)
       # adjust bounds to match target zone (e.g. inside smoke, near a light, etc.)
       hits = [p for p in placed if 7.5 <= p.x <= 11.5 and 6.8 <= p.y <= 9.2]
       if hits: print(f'Seed {seed}: {[(p.model_type,round(p.x,1),round(p.y,1)) for p in placed]}')
   "
   ```

3. **Launch headless**
   ```bash
   tmux new-session -d -s vis_test \
     "./scripts/run_scenario.sh config/scenarios/<name>_test.yaml --headless --seed <N>"
   sleep 10
   ```

4. **Position robot and snapshot**
   ```bash
   python3.12 scripts/world_state.py          # confirm object positions + robot pose
   python3.12 scripts/robot_control.py move <m>
   python3.12 scripts/robot_control.py rotate <deg>
   python3.12 scripts/robot_control.py snapshot   # → /tmp/robot_snapshot.png
   # Read /tmp/robot_snapshot.png
   ```

5. **Iterate**
   - Kill: `tmux kill-session -t vis_test`
   - Edit the parameter in `src/world_manager/world_generator.py` (smoke), `worlds/templates/indoor_office/config.yaml` (lights, presets), or the model SDF (materials/geometry)
   - Relaunch from step 3

---

## Key locations

| What | Where |
|------|-------|
| Smoke emitter SDF + parameters | `src/world_manager/world_generator.py` → `smoke_emitter` block |
| Lighting presets (normal, dim, flicker…) | `worlds/templates/indoor_office/config.yaml` → `lighting_presets` |
| World plugins (particle, IMU, etc.) | `worlds/templates/indoor_office/world.sdf` |
| Object/furniture model SDFs | `worlds/models/<name>/model.sdf` |
| Smoke texture | `worlds/media/textures/smoke.png` |

---

## Smoke tuning reference

| Parameter | Effect | Current |
|-----------|--------|---------|
| `color_start` alpha | Per-particle opacity | 0.025 |
| `rate` | Particles/sec — more = denser cloud | 15 (test) / 20 (scenarios) |
| `particle_size` | Initial particle diameter (m) | 0.4 |
| `scale_rate` | Growth rate (m/s) — larger → bigger overlap → opaque sooner | 0.20 |
| `lifetime` | Seconds a particle lives | 5.0 |
| emitter `size` | Bounding box of spawn region | [1.5, 2.0, 1.0] |
