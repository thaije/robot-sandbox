#!/usr/bin/env bash
# Run a scenario N times with sequential seeds.
# Usage: ./scripts/run_batch.sh config/scenarios/office_explore_detect.yaml 5
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

SCENARIO="${1:?Usage: run_batch.sh <scenario.yaml> <N>}"
N="${2:?Usage: run_batch.sh <scenario.yaml> <N>}"

source /opt/ros/jazzy/setup.bash
cd "$REPO_ROOT"

for i in $(seq 0 $((N - 1))); do
    echo "=== Run $((i + 1)) / $N (seed=$i) ==="
    python -m scenario_runner --scenario "$SCENARIO" --seed "$i"
done

echo "=== Batch complete. Aggregating results... ==="
python scripts/aggregate_results.py results/
