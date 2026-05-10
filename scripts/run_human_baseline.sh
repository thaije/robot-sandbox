#!/usr/bin/env bash
# Human teleop baseline runner — two modes:
#
#   oracle      — teleop + oracle detections (navigation-only baseline)
#   perception  — teleop + human keypress detections (full-task baseline)
#
# Usage:
#   ./scripts/run_human_baseline.sh oracle
#   ./scripts/run_human_baseline.sh perception
#
# All difficulties, seeds 1–5, 1 run each = 25 runs per mode.
# Resumable: skips any run whose output file already exists.
#
# Oracle mode:
#   Drive in a second terminal:
#     ros2 run teleop_twist_keyboard teleop_twist_keyboard \
#       --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel
#
# Perception mode:
#   Drive in terminal 2:
#     ros2 run teleop_twist_keyboard teleop_twist_keyboard \
#       --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel
#
#   Detect objects in terminal 3:
#     python3.12 scripts/human_detector_node.py
#
#   View robot camera in terminal 4:
#     ros2 run rqt_image_view rqt_image_view
#     Then select /derpbot_0/rgbd/image

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

MODE="${1:-oracle}"
if [[ "$MODE" != "oracle" && "$MODE" != "perception" ]]; then
    echo "Usage: $0 [oracle|perception]"
    echo "  oracle      — teleop + oracle detections (navigation baseline)"
    echo "  perception  — teleop + human keypress detections (full-task baseline)"
    exit 1
fi

DIFFICULTIES=(easy medium hard brutal perception_stress)
SEEDS=(1 2 3 4 5)
RUN=1   # one run per seed

OUTPUT_DIR="${REPO_ROOT}/results/submissions/human-baseline-${MODE}"
mkdir -p "$OUTPUT_DIR"

log() { echo "[$(date +%H:%M:%S)] $*"; }

set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

export PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
cd "$REPO_ROOT"

TOTAL=$(( ${#DIFFICULTIES[@]} * ${#SEEDS[@]} ))
run_num=0
n_ok=0
n_skip=0

if [[ "$MODE" == "oracle" ]]; then
    ORACLE_FLAG="--enable-oracle"
    MODE_DESC="oracle — navigate, detections automatic"
else
    ORACLE_FLAG=""
    MODE_DESC="perception — navigate AND detect objects manually"
fi

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "  Human baseline: $MODE_DESC"
echo "  ${#DIFFICULTIES[@]} difficulties × ${#SEEDS[@]} seeds × ${RUN} run = ${TOTAL} total"
echo "  Output: ${OUTPUT_DIR}"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

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
        echo "  Run $run_num / $TOTAL  —  $difficulty  seed=$seed  mode=$MODE"
        echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
        echo ""
        echo "  Terminal 2 — start teleop once Gazebo is open:"
        echo ""
        echo "    ros2 run teleop_twist_keyboard teleop_twist_keyboard \\"
        echo "      --ros-args --remap cmd_vel:=/derpbot_0/cmd_vel"

        if [[ "$MODE" == "perception" ]]; then
            echo ""
            echo "  Terminal 3 — start human detector (press key when you see an object):"
            echo ""
            echo "    python3.12 scripts/human_detector_node.py"
            echo ""
            echo "  Terminal 4 — view robot camera (optional but recommended):"
            echo ""
            echo "    ros2 run rqt_image_view rqt_image_view"
            echo "    Select /derpbot_0/rgbd/image"
        fi

        echo ""
        echo "  Press Enter to start the scenario, Ctrl-C to abort."
        read -r

        before=$(ls -t "${REPO_ROOT}/results"/*.json 2>/dev/null | head -1 || true)

        log "Starting sim: $difficulty  seed=$seed  mode=$MODE"
        "${SCRIPT_DIR}/run_scenario.sh" \
            "config/scenarios/office_explore_detect/${difficulty}.yaml" \
            --gui --seed "$seed" $ORACLE_FLAG &
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
    echo "      --results-dir ${OUTPUT_DIR}"
    echo "    python3.12 scripts/validate_submission.py \\"
    echo "      ${OUTPUT_DIR}/benchmark_submission.yaml"
fi
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
