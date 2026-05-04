#!/usr/bin/env bash
# Human teleop baseline runner — easy + medium, seeds 1–5, 1 run each = 10 runs
#
# Usage:
#   ./scripts/run_human_baseline.sh
#
# Each scenario starts automatically. Drive the robot in a second terminal:
#   ros2 run teleop_twist_keyboard teleop_twist_keyboard \
#     --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel
#
# Oracle detections are enabled — just navigate; the sim scores object finds.
# Resumable: skips any run whose output file already exists.

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

DIFFICULTIES=(easy medium)
SEEDS=(1 2 3 4 5)
RUN=1   # one run per seed

OUTPUT_DIR="${REPO_ROOT}/results/submissions/human-baseline"
mkdir -p "$OUTPUT_DIR"

log() { echo "[$(date +%H:%M:%S)] $*"; }

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
cd "$REPO_ROOT"

TOTAL=$(( ${#DIFFICULTIES[@]} * ${#SEEDS[@]} ))
run_num=0
n_ok=0
n_skip=0

for difficulty in "${DIFFICULTIES[@]}"; do
    for seed in "${SEEDS[@]}"; do
        (( run_num++ )) || true

        outfile="${OUTPUT_DIR}/${difficulty}_seed${seed}_run${RUN}.json"

        if [[ -f "$outfile" ]]; then
            log "[$run_num/$TOTAL] SKIP  $difficulty  seed=$seed  (exists)"
            (( n_skip++ )) || true
            continue
        fi

        echo ""
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo "  Run $run_num / $TOTAL  —  $difficulty  seed=$seed"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "  In a second terminal, start teleop once Gazebo is open:"
        echo ""
        echo "    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\"
        echo "      --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel"
        echo ""
        echo "  Press Enter to start the scenario, Ctrl-C to abort."
        read -r

        before=$(ls -t "${REPO_ROOT}/results"/*.json 2>/dev/null | head -1 || true)

        log "Starting sim: $difficulty  seed=$seed  (oracle detections on)"
        "${SCRIPT_DIR}/run_scenario.sh" \
            "config/scenarios/office_explore_detect/${difficulty}.yaml" \
            --gui --seed "$seed" --enable-oracle &
        SIM_PID=$!

        wait "$SIM_PID" || true

        after=$(ls -t "${REPO_ROOT}/results"/*.json 2>/dev/null | head -1 || true)

        if [[ -n "$after" && "$after" != "$before" ]]; then
            cp "$after" "$outfile"
            score=$(python3.12 -c \
                "import json, sys; d=json.load(open(sys.argv[1])); print(d.get('overall_score','?'))" \
                "$after" 2>/dev/null || echo "?")
            log "✓ score=${score}  → $(basename "$outfile")"
            (( n_ok++ )) || true
        else
            log "✗ No result file produced — run not saved"
        fi
    done
done

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
log "DONE  ok=${n_ok}  skip=${n_skip}  total=${TOTAL}"
if (( n_ok + n_skip == TOTAL )); then
    echo ""
    echo "  All runs complete. Next steps:"
    echo "    python3.12 scripts/summarize_random_baseline.py \\"
    echo "      --results-dir results/submissions/human-baseline"
    echo "    python3.12 scripts/validate_submission.py \\"
    echo "      results/submissions/human-baseline/benchmark_submission.yaml"
fi
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
