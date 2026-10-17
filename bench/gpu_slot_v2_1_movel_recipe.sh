#!/usr/bin/env bash
# GPU slot 2026-10-13/14 (orchestrator: GPU free since 22:35; touch GPU_RELEASED by 09:00).
# v2.1 amendment 7g: PyRoki MoveL (480 segments) with PyRoki's own endpoint recipe. No new segment after 08:40; kill 08:50.
cd "$(dirname "$0")/.."
log(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
DEADLINE=$(date -d '2026-10-14 08:40' +%s); KILL_AT=$(date -d '2026-10-14 08:50' +%s)
export PYROKI_MARGIN=$(cat results/v2_1/margin.txt) PYROKI_MOVEL_ENDS=recipe
cleanup(){ kill $WD 2>/dev/null; touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"; }
trap cleanup EXIT
( while [ $(date +%s) -lt $KILL_AT ]; do sleep 30; done; pkill -TERM -f "^$PWD/.venv-pyroki/bin/python bench/v2_1_pyroki\.py" && echo "$(date '+%H:%M') watchdog killed PyRoki run" ) &
WD=$!
rm -f ~/projects/_ops/GPU_RELEASED
log "MoveL recipe rerun starts; margin $PYROKI_MARGIN m; MemAvailable $(awk '/MemAvailable/ {printf "%.1f", $2/1048576}' /proc/meminfo) GB"
bench/pyroki_env.sh python bench/v2_1_pyroki.py movel --tag _recipe --deadline $DEADLINE 2>&1 | grep -vE "INFO|^\s" | tail -3
log "movel recipe rc=${PIPESTATUS[0]}"
log "SESSION DONE"
