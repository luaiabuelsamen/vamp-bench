#!/usr/bin/env bash
# GPU session 2026-10-13 (orchestrator: GPU released by interp 14:17; hold until 09:00 tomorrow at most).
# v2.1 S2 seed dump: cuRobo v1 plan_single_js on held-out A6 at seed_k 0, recording its trajopt seeds per attempt.
# Feasibility runner keeps running (1 worker): nothing here is timed for a result.
cd "$(dirname "$0")/.."
log(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
H=6aea6eecf76a173d7ebdf548104aaec854b6b6c7
DEADLINE=$(date -d 'tomorrow 08:40' +%s)
log "seed dump starts; MemAvailable $(awk '/MemAvailable/ {printf "%.1f", $2/1048576}' /proc/meminfo) GB"
unset PYTHONPATH; export PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only
~/projects/curobo_berkeley/venv/bin/python bench/v2_curobo_seeds.py --freeze $H --deadline $DEADLINE 2>&1 | grep -vE "Warning|warn"
log "seed dump rc=${PIPESTATUS[0]}"
log "SEEDS DONE"
