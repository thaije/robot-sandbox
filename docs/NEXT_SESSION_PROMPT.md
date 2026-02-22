# Next session prompt

You are continuing the ARST robot sandbox project. Read `agent-scripts/AGENTS.MD` first,
then `docs/AGENT_HANDOFF.md` for full project state.

## What was just completed

- Bumper/contact sensor added to DerpBot URDF (`base_link_collision` named, sensor block added)
- `/{robot}/bumper_contact` bridge added to `spawn_robot.launch.py`
- `RevisitRatio` metric fully implemented (odom subscription, cell tracking)
- `collision_count` bumper topic fixed to `/{robot_name}/bumper_contact` in runner
- `revisit_ratio` registered in `runner._build_metrics()`
- `SimulationLauncher.shutdown()` pgid bug fixed (pgids now captured before SIGTERM so SIGKILL reliably fires)
- Smoke tests pass, zero orphan processes after shutdown

## Immediate task for next session — fix metrics not updating during runs

**Symptom:** In automated scenario runs, `meters_traveled` and `revisit_ratio` show 0.0 / 1.0
even while the robot moves. The robot definitely moves (confirmed by `ros2 topic echo /derpbot_0/odom`
and user teleop). The rclpy metrics node's odom callbacks may not be firing.

**Most likely fix:** `use_sim_time` mismatch. The bridge runs with `use_sim_time=true`, stamping
messages with sim time. The rclpy metrics node is created without `use_sim_time`, so it uses wall
time and may silently drop messages whose timestamps appear to be in the future.

Open `src/scenario_runner/runner.py` and find:
```python
node = rclpy.create_node("arst_metrics")
```
Try changing to:
```python
from rclpy.parameter import Parameter
node = rclpy.create_node(
    "arst_metrics",
    parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
)
```
Then test with a 30s scenario run while publishing cmd_vel from tmux, and verify
`meters_traveled` is non-zero.

If that doesn't fix it, see the "Open issue" section in `docs/AGENT_HANDOFF.md` for
the next suspects (DDS discovery race, spin thread race).

## After fixing metrics — next feature

Add the bounding box camera to DerpBot (see step 1 in `docs/AGENT_HANDOFF.md`
"Recommended next steps"). This unlocks detection metrics and lets the scenario reach SUCCESS.

## How to run smoke tests

```bash
cd /home/plip/Projects/robot-sandbox

# Quick headless test (10s, no movement):
./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless --timeout 10

# Movement test (30s) — in tmux, start scenario then publish cmd_vel in a second window:
tmux new-session -d -s arst -x 220 -y 50
tmux send-keys -t arst:0 "./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml --headless --timeout 30 2>&1 | tee /tmp/run.log" Enter
tmux new-window -t arst
tmux send-keys -t arst:1 "set +u; source /opt/ros/jazzy/setup.bash; set -u && until ros2 topic list 2>/dev/null | grep -q '/derpbot_0/odom'; do sleep 0.5; done && ros2 topic pub --rate 10 /derpbot_0/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.5}}'" Enter
# Wait ~45s, then: tmux capture-pane -t arst:0 -p -S -30
```

## Check for orphan processes after every test run

```bash
ps aux | grep -E "gz sim|parameter_bridge" | grep -v grep | wc -l  # should be 0
```
