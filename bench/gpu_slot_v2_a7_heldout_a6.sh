#!/usr/bin/env bash
# GPU session 2026-10-12 evening (orchestrator: GPU free, flagship QUEUE_DONE; starts now).
# Order: VAMP pass 2a on heldout_a6 (quiet CPU, A4) -> A7 J1 (v1 pick-place, max_jerk 250) -> A7 J2 (v1 pick-place,
# jerk-continue) -> heldout_a6 cuRobo v1 (10 seeds) -> heldout_a6 cuRoboV2 (original subsample indices, 3 seeds).
# Cap: no new cell/query after start+9h40; runners killed at start+9h50; partials kept. Reference runner suspended
# during the session; GPU_RELEASED touched on exit.
cd "$(dirname "$0")/.."
log(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
memgb(){ awk '/MemAvailable/ {printf "%.1f", $2/1048576}' /proc/meminfo; }
H=6aea6eecf76a173d7ebdf548104aaec854b6b6c7
FEAS='^\.venv/bin/python bench/v2_feasibility_par\.py'
START=$(date +%s); DEADLINE=$((START + 9*3600 + 40*60)); KILL_AT=$((START + 9*3600 + 50*60))
FEAS_KILLED=0
cleanup(){
  M=$(pgrep -f "$FEAS" | tr '\n' ' ')
  [ -n "$M" ] && kill -CONT $M 2>/dev/null && log "resumed my CPU jobs: $M"
  if [ $FEAS_KILLED = 1 ]; then
    nohup .venv/bin/python bench/v2_feasibility_par.py --freeze $H --workers 2 --max-workers 6 >> logs/v2_feas_par.log 2>&1 &
    log "restarted reference runner (pid $!)"
  fi
  kill $WD 2>/dev/null
  touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"
}
trap cleanup EXIT
M=$(pgrep -f "$FEAS" | tr '\n' ' ')
[ -n "$M" ] && kill -STOP $M && log "suspended reference runner: [$M]"
log "session starts; no new work after $(date -d @$DEADLINE '+%a %H:%M'), kill at $(date -d @$KILL_AT '+%H:%M'); MemAvailable $(memgb) GB"
ps -eo pcpu,rss,args --sort=-pcpu | head -6 | cut -c1-150 | sed 's/^/load: /'
if [ "$(memgb | cut -d. -f1)" -lt 4 ] && [ -n "$M" ]; then
  kill -TERM $M 2>/dev/null; kill -CONT $M 2>/dev/null; sleep 5; FEAS_KILLED=1
  log "MemAvailable under 4 GB with the reference runner suspended: stopped it (restarts after the session); now $(memgb) GB"
fi
( while [ $(date +%s) -lt $KILL_AT ]; do sleep 30; done; pkill -TERM -f '^[^ ]*python[0-9.]* bench/v2_(pickplace_curobo|curobo)\.py' && echo "$(date '+%H:%M') watchdog killed cuRobo run" ) &
WD=$!

# 1. VAMP pass 2a (quiet CPU timing) on heldout_a6
.venv/bin/python bench/v2_vamp_pass2a.py --freeze $H --sets heldout_a6 --check-out results/v2/pass2a_check_a6.json --deadline $DEADLINE
log "pass 2a heldout_a6 rc=$?"

unset PYTHONPATH
C1=~/projects/curobo_berkeley/venv/bin/python; C2=~/projects/clean_env/venv/bin/python
export PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only
SM=$(mktemp -d)
# 2. A7 J1 smoke (one cell, one seed, scratch output), then J1, then J2
$C1 bench/v2_pickplace_curobo.py --version v1 --freeze $H --seeds 1 --limit 1 --max-jerk 250 --out $SM/j1.jsonl > $SM/j1.log 2>&1
log "J1 smoke rc=$?"; grep "A7:" $SM/j1.log | sed 's/^/  /'
python3 - $SM/j1.jsonl <<'EOF' | sed 's/^/  /'
import json, sys
r = [json.loads(l) for l in open(sys.argv[1])]
print("picks", len(r), "success", sum(x["success"] for x in r), "jerk ratios", [round(s.get("max_jerk_ratio", -1), 2) for x in r for s in x["segments"] if "max_jerk_ratio" in s][:12])
EOF
for J in jerk250 jerkcontinue; do
  [ $(date +%s) -gt $DEADLINE ] && { log "deadline reached before $J"; break; }
  if [ $J = jerk250 ]; then X="--max-jerk 250"; else X="--jerk-continue"; fi
  python3 bench/powerlog.py results/v2/raw/curobo1_pickplace.A6.$J.power.json -- $C1 bench/v2_pickplace_curobo.py --version v1 --freeze $H --seeds 5 $X \
    --out results/v2/raw/curobo1_pickplace.A6.$J.jsonl --deadline $DEADLINE
  log "A7 $J rc=$? (0 complete, 3 partial)"
done

# 3. heldout_a6: v1 (all queries, 10 seeds), then V2 (original subsample indices, 3 seeds)
[ $(date +%s) -lt $DEADLINE ] && {
  python3 bench/powerlog.py results/v2/raw/curobo1_heldout_a6.power.json -- $C1 bench/v2_curobo.py --version v1 --freeze $H --set heldout_a6 --deadline $DEADLINE
  log "v1 heldout_a6 rc=$? (0 complete, 3 partial)"; }
export PYTHONPATH=bench:.vamp_only
[ $(date +%s) -lt $DEADLINE ] && {
  python3 bench/powerlog.py results/v2/raw/curobo2_heldout_a6.power.json -- $C2 bench/v2_curobo.py --version v2 --freeze $H --set heldout_a6 \
    --subset results/v2/v2_subsample.json --seeds 3 --deadline $DEADLINE
  log "V2 heldout_a6 rc=$? (0 complete, 3 partial)"; }
log "SESSION DONE"
