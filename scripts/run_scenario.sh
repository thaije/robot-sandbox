#!/usr/bin/env bash
# Run a single scenario from the command line.
# Usage: ./scripts/run_scenario.sh config/scenarios/office_explore_detect.yaml [extra args]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

source /opt/ros/jazzy/setup.bash

cd "$REPO_ROOT"
python -m scenario_runner --scenario "$@"
