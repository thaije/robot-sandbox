#!/usr/bin/env bash
# Run a single scenario from the command line.
# Usage: ./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml [extra args]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# ROS setup scripts reference uninitialized vars; disable -u around the source.
set +u
source /opt/ros/jazzy/setup.bash
set -u

cd "$REPO_ROOT"
# python3 on this machine may resolve to a non-system venv (e.g. text-generation-webui).
# ROS 2 Jazzy's rclpy C extension is compiled for Python 3.12 — use it explicitly.
export PYTHONPATH="$REPO_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
python3.12 -m scenario_runner --scenario "$@"
