#!/usr/bin/env bash
# GPU session 2026-10-12 night (orchestrator: GPU free, flagship queue done; release by 09:00).
# The evening session's v1 heldout_a6 step crashed at start (subset-lookup bug, fixed); V2 heldout_a6 completed there.
# Order: v1 heldout_a6 smoke (--limit 1, scratch) -> jerk-peak diagnostic (one pick-place cell, v1 at max_jerk 500 and
# 250, --dump, scratch) -> v1 heldout_a6 (all queries, 10 seeds). No new query after 08:40, kill at 08:50.
cd "$(dirname "$0")/.."
log(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
H=6aea6eecf76a173d7ebdf548104aaec854b6b6c7
FEAS='^\.venv/bin/python bench/v2_feasibility_par\.py'
DEADLINE=$(date -d 'tomorrow 08:40' +%s); KILL_AT=$(date -d 'tomorrow 08:50' +%s)
cleanup(){
  M=$(pgrep -f "$FEAS" | tr '\n' ' '); [ -n "$M" ] && kill -CONT $M 2>/dev/null && log "resumed reference runner: $M"
  kill $WD 2>/dev/null; touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"
}
trap cleanup EXIT
M=$(pgrep -f "$FEAS" | tr '\n' ' '); [ -n "$M" ] && kill -STOP $M && log "suspended reference runner: [$M]"
log "session starts; MemAvailable $(awk '/MemAvailable/ {printf "%.1f", $2/1048576}' /proc/meminfo) GB"
( while [ $(date +%s) -lt $KILL_AT ]; do sleep 30; done; pkill -TERM -f '^[^ ]*python[0-9.]* bench/v2_(pickplace_curobo|curobo)\.py' && echo "$(date '+%H:%M') watchdog killed cuRobo run" ) &
WD=$!
unset PYTHONPATH; C1=~/projects/curobo_berkeley/venv/bin/python
export PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only
SM=results/v2/diag_jerk; mkdir -p $SM
$C1 bench/v2_curobo.py --version v1 --freeze $H --set heldout_a6 --limit 1 --seeds 1 --out $SM/smoke_heldout_a6.json > $SM/smoke_heldout_a6.log 2>&1
RC=$?; log "v1 heldout_a6 smoke rc=$RC"; tail -3 $SM/smoke_heldout_a6.log | sed 's/^/  /'
[ $RC = 0 ] || { log "smoke failed: not starting v1 heldout_a6"; exit 1; }
for J in 500 250; do
  $C1 bench/v2_pickplace_curobo.py --version v1 --freeze $H --seeds 1 --limit 1 --max-jerk $J --out $SM/pp_jerk$J.jsonl --dump $SM/dump_jerk$J > $SM/pp_jerk$J.log 2>&1
  log "diagnostic dump max_jerk $J rc=$?"
done
python3 bench/powerlog.py results/v2/raw/curobo1_heldout_a6.power.json -- $C1 bench/v2_curobo.py --version v1 --freeze $H --set heldout_a6 --deadline $DEADLINE
log "v1 heldout_a6 rc=$? (0 complete, 3 partial)"
log "SESSION DONE"
