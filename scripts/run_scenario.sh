#!/usr/bin/env bash
# Run a single scenario from the command line.
#
# Usage:
#   ./scripts/run_scenario.sh <scenario.yaml> [extra args]
#
# Optional env vars:
#   ROS_DOMAIN_ID   — ROS 2 DDS domain ID (0–232).  Processes on different
#                     domain IDs are completely invisible to each other, which
#                     cleanly isolates parallel scenario runs on the same host.
#                     If not set, the shell's existing ROS_DOMAIN_ID is used
#                     (default 0 if unset by ROS).
#
# Examples:
#   # Single robot, default domain:
#   ./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml
#
#   # Run a specific difficulty tier:
#   ./scripts/run_scenario.sh config/scenarios/office_explore_detect/hard.yaml
#
#   # Parallel run on domain 1 (separate terminal):
#   ROS_DOMAIN_ID=1 ./scripts/run_scenario.sh config/scenarios/office_explore_detect/medium.yaml
#
#   # Auto-assign a free domain ID (for scripted parallel runs):
#   ROS_DOMAIN_ID=$(( RANDOM % 200 + 10 )) ./scripts/run_scenario.sh ...
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# ROS setup scripts reference uninitialized vars; disable -u around the source.
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Export ROS_DOMAIN_ID so all child processes (Gazebo, bridges, metrics node)
# share the same domain.  If already set in the environment, honour it.
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
echo "[run_scenario] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"

cd "$REPO_ROOT"

# source python
source .venv/bin/activate 

# python3 on this machine may resolve to a non-system venv (e.g. text-generation-webui).
# ROS 2 Jazzy's rclpy C extension is compiled for Python 3.12 — use it explicitly.
export PYTHONPATH="$REPO_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"

# Start the map image publisher in the background.  It polls arst_world_map.png
# for changes and republishes to /arst/world_map (TRANSIENT_LOCAL) so RViz
# always shows the current map without world_state.py needing to touch ROS.
python3.12 "$SCRIPT_DIR/map_publisher.py" &
MAP_PUB_PID=$!
_cleanup() {
    kill "$MAP_PUB_PID" 2>/dev/null || true
    wait "$MAP_PUB_PID" 2>/dev/null || true
}
trap _cleanup EXIT INT TERM

python3.12 -m scenario_runner --scenario "$@"
