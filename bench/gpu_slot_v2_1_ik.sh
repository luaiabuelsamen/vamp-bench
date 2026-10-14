#!/usr/bin/env bash
# GPU slot 2026-10-13/14 (orchestrator: GPU free since 22:35; touch GPU_RELEASED by 09:00).
# v2.1 IK comparator: cuRobo v1 IKSolver on the 300 held-out A6 goal poses (bench/v2_1_curobo_ik.py).
cd "$(dirname "$0")/.."
log(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
trap 'touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"' EXIT
H=6aea6eecf76a173d7ebdf548104aaec854b6b6c7
DEADLINE=$(date -d 'tomorrow 08:40' +%s)
rm -f ~/projects/_ops/GPU_RELEASED
log "IK comparator starts; MemAvailable $(awk '/MemAvailable/ {printf "%.1f", $2/1048576}' /proc/meminfo) GB"
unset PYTHONPATH; export PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only
~/projects/curobo_berkeley/venv/bin/python bench/v2_1_curobo_ik.py --freeze $H --deadline $DEADLINE "$@" 2>&1 | grep -vE "Warning|warn"
log "ik rc=${PIPESTATUS[0]}"
log "SESSION DONE"
