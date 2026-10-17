#!/usr/bin/env bash
# GPU session 2026-10-13/14 (GPU released by interp 14:17; hold until 09:00 at most).
# v2.1 PyRoki on held-out A6, GPU: IK (a, b, c) -> MoveL (480 segments) -> trajopt S1 -> trajopt S2.
# Margin from the gantry dev sweep (results/v2_1/margin.txt, amendment 7d). Feasibility runner keeps running (1 worker);
# load recorded per result (7c). No new item after 08:40; GPU_RELEASED touched on exit.
cd "$(dirname "$0")/.."
log(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
DEADLINE=$(date -d 'tomorrow 08:40' +%s); KILL_AT=$(date -d 'tomorrow 08:50' +%s)
export PYROKI_MARGIN=$(cat results/v2_1/margin.txt)
cleanup(){ kill $WD 2>/dev/null; touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"; }
trap cleanup EXIT
( while [ $(date +%s) -lt $KILL_AT ]; do sleep 30; done; pkill -TERM -f "^$PWD/.venv-pyroki/bin/python bench/v2_1_pyroki\.py" && echo "$(date '+%H:%M') watchdog killed PyRoki run" ) &
WD=$!
log "session starts; margin $PYROKI_MARGIN m; MemAvailable $(awk '/MemAvailable/ {printf "%.1f", $2/1048576}' /proc/meminfo) GB"
for step in "ik" "movel" "trajopt --seeds s1" "trajopt --seeds s2"; do
  [ $(date +%s) -ge $DEADLINE ] && { log "deadline: not starting $step"; break; }
  bench/pyroki_env.sh python bench/v2_1_pyroki.py $step --deadline $DEADLINE 2>&1 | grep -vE "INFO|^\s" | tail -3
  log "$step rc=${PIPESTATUS[0]}"
done
log "SESSION DONE"
