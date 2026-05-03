#!/usr/bin/env bash
# Batch runner: random-baseline — 5 difficulties × 5 seeds × 3 runs = 75 runs
#
# Usage:
#   ./scripts/run_random_baseline.sh [--speed N] [--dry-run]
#
# Runs are sequential (hardware can't sustain parallel Gazebo stacks).
# Resumable: skips any run whose output file already exists.
# Results land in results/submissions/random-baseline/<difficulty>_seed<N>_run<K>.json
#
# After all runs, generate the summary:
#   python3.12 scripts/summarize_random_baseline.py

set -uo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(dirname "$SCRIPT_DIR")"

# ── Config ─────────────────────────────────────────────────────────────────────

DIFFICULTIES=(easy medium hard brutal perception_stress)
SEEDS=(1 2 3 4 5)
RUNS_PER_SEED=3
SPEED=3            # sim real-time factor; scores are in sim-seconds and comparable
READY_TIMEOUT=120  # seconds to wait for mission server before aborting run
DRY_RUN=0

for arg in "$@"; do
    case "$arg" in
        --speed=*) SPEED="${arg#--speed=}" ;;
        --speed)   shift; SPEED="$1" ;;
        --dry-run) DRY_RUN=1 ;;
    esac
done

OUTPUT_DIR="${REPO_ROOT}/results/submissions/random-baseline"
LOG_DIR="${OUTPUT_DIR}/logs"
mkdir -p "$OUTPUT_DIR" "$LOG_DIR"

LOGFILE="${LOG_DIR}/batch_$(date +%Y%m%dT%H%M%S).log"

log() { echo "[$(date +%H:%M:%S)] $*" | tee -a "$LOGFILE"; }

# ── Environment ────────────────────────────────────────────────────────────────

# ROS setup (unset check disabled — setup.bash uses unbound vars).
set +u
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
set -u

export PYTHONPATH="${REPO_ROOT}/src${PYTHONPATH:+:$PYTHONPATH}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

cd "$REPO_ROOT"

# ── Helpers ────────────────────────────────────────────────────────────────────

wait_for_mission_server() {
    local deadline=$(( $(date +%s) + READY_TIMEOUT ))
    while [[ $(date +%s) -lt $deadline ]]; do
        if curl -sf http://localhost:7400/mission > /dev/null 2>&1; then
            return 0
        fi
        sleep 2
    done
    return 1
}

newest_json() {
    ls -t "${REPO_ROOT}/results"/*.json 2>/dev/null | head -1 || true
}

# ── Main loop ──────────────────────────────────────────────────────────────────

TOTAL=$(( ${#DIFFICULTIES[@]} * ${#SEEDS[@]} * RUNS_PER_SEED ))
log "random-baseline batch: ${#DIFFICULTIES[@]} difficulties × ${#SEEDS[@]} seeds × ${RUNS_PER_SEED} runs = ${TOTAL} total (speed=${SPEED}×)"

run_num=0
n_ok=0
n_skip=0
n_fail=0

for difficulty in "${DIFFICULTIES[@]}"; do
    for seed in "${SEEDS[@]}"; do
        for run in $(seq 1 $RUNS_PER_SEED); do
            (( run_num++ )) || true

            outfile="${OUTPUT_DIR}/${difficulty}_seed${seed}_run${run}.json"

            if [[ -f "$outfile" ]]; then
                log "[$run_num/$TOTAL] SKIP  $difficulty  seed=$seed  run=$run  (exists)"
                (( n_skip++ )) || true
                continue
            fi

            log "[$run_num/$TOTAL] START $difficulty  seed=$seed  run=$run"

            if [[ $DRY_RUN -eq 1 ]]; then
                log "  (dry-run — skipping actual execution)"
                continue
            fi

            before=$(newest_json)

            # Start sim in background
            SIM_LOG="${LOG_DIR}/${difficulty}_seed${seed}_run${run}_sim.log"
            "${SCRIPT_DIR}/run_scenario.sh" \
                "config/scenarios/office_explore_detect/${difficulty}.yaml" \
                --headless --seed "$seed" --speed "$SPEED" \
                > "$SIM_LOG" 2>&1 &
            SIM_PID=$!

            # Wait for mission server
            if ! wait_for_mission_server; then
                log "  ✗ mission server never ready — aborting run"
                kill "$SIM_PID" 2>/dev/null || true
                wait "$SIM_PID" 2>/dev/null || true
                (( n_fail++ )) || true
                sleep 5
                continue
            fi

            # Start random agent
            AGENT_LOG="${LOG_DIR}/${difficulty}_seed${seed}_run${run}_agent.log"
            python3.12 "${SCRIPT_DIR}/random_agent.py" --seed "$seed" \
                > "$AGENT_LOG" 2>&1 &
            AGENT_PID=$!

            # Wait for sim to finish
            wait "$SIM_PID" || true
            kill "$AGENT_PID" 2>/dev/null || true
            wait "$AGENT_PID" 2>/dev/null || true

            # Find the newly written result
            after=$(newest_json)

            if [[ -n "$after" && "$after" != "$before" ]]; then
                cp "$after" "$outfile"
                score=$(python3.12 -c \
                    "import json, sys; d=json.load(open(sys.argv[1])); print(d.get('overall_score','?'))" \
                    "$after" 2>/dev/null || echo "?")
                log "  ✓ score=${score}  → $(basename "$outfile")"
                (( n_ok++ )) || true
            else
                log "  ✗ no result file produced"
                (( n_fail++ )) || true
            fi

            # Brief cool-down: let gz processes release ports/sockets
            sleep 5
        done
    done
done

log "=== DONE  ok=${n_ok}  skip=${n_skip}  fail=${n_fail}  total=${TOTAL} ==="
if (( n_ok + n_skip == TOTAL && n_fail == 0 )); then
    log "All runs complete. Next: python3.12 scripts/summarize_random_baseline.py"
else
    log "Some runs failed or are still pending. Re-run to retry failed runs."
fi
