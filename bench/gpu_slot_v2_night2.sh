#!/usr/bin/env bash
# GPU slot, night of 2026-10-09: (1) A4 pass 2a quiet-CPU re-timing of VAMP successes, (2) cuRoboV2 stratified
# subsample (300 queries x 3 seeds, A2 budget T = 30 s, max_attempts 100).
# Gate as last night: no python train_local.py process (interpreter-anchored match) and GPU idle 10 min.
# My CPU jobs (A3 reference runs) are suspended for the whole slot and resumed on exit.
# Deadline: no new query after 07:50; watchdog at 07:58; GPU_RELEASED touched on exit if the slot started.
cd "$(dirname "$0")/.."
log(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
H=6aea6eecf76a173d7ebdf548104aaec854b6b6c7
DEADLINE=$(date -d 'tomorrow 07:50' +%s); [ $(date +%H) -lt 8 ] && DEADLINE=$(date -d 'today 07:50' +%s)
KILL_AT=$((DEADLINE + 480))
TRAIN='^[^ ]*python[0-9.]* [^ ]*train_local\.py'
while pgrep -f "$TRAIN" >/dev/null; do sleep 60; done
log "no python train_local.py process"
idle=0
while [ $idle -lt 10 ]; do
  pgrep -f "$TRAIN" >/dev/null && { idle=0; sleep 60; continue; }
  g=$(timeout 5 tegrastats --interval 1000 | head -3 | grep -oE "GR3D_FREQ [0-9]+" | awk '{s+=$2} END {print int(s/NR)}')
  if [ "${g:-100}" -le 5 ]; then idle=$((idle+1)); else idle=0; fi
  log "GR3D=${g}% idle_min=$idle"; sleep 55
  [ $(date +%s) -gt $DEADLINE ] && { log "deadline passed before the gate opened; not starting"; exit 0; }
done
MINE=$(pgrep -f '^\.venv/bin/python bench/v2_feasibility_par\.py' | tr '\n' ' ')
cleanup(){
  [ -n "$MINE" ] && kill -CONT $MINE 2>/dev/null && log "resumed my CPU jobs: $MINE"
  touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"
}
trap cleanup EXIT
[ -n "$MINE" ] && kill -STOP $MINE && log "suspended my CPU jobs: $MINE"
log "slot starts; deadline $(date -d @$DEADLINE '+%H:%M')"
ps -eo pcpu,args --sort=-pcpu | head -6 | sed 's/^/load: /'
unset PYTHONPATH
# (1) pass 2a: quiet CPU, nothing of ours running
.venv/bin/python bench/v2_vamp_pass2a.py --freeze $H --deadline $DEADLINE; log "pass 2a rc=$?"
# (2) cuRoboV2 subsample
C2=~/projects/clean_env/venv/bin/python
export PYTHONPATH=bench:.vamp_only
SM=$(mktemp -d)
$C2 bench/v2_curobo.py --version v2 --freeze $H --set mbm --subset results/v2/v2_subsample.json --limit 1 --seeds 1 --out $SM/smoke.json; rc=$?
log "V2 smoke rc=$rc"
[ $rc -ne 0 ] && { log "V2 SMOKE FAILED; stopping slot"; exit 1; }
( while [ $(date +%s) -lt $KILL_AT ]; do sleep 30; done; pkill -TERM -f '^[^ ]*python[0-9.]* bench/v2_curobo\.py' && echo "$(date '+%H:%M') watchdog killed V2 run" ) &
WD=$!
for s in heldout mbm mpinets; do
  [ $(date +%s) -gt $DEADLINE ] && { log "deadline reached before $s"; break; }
  python3 bench/powerlog.py results/v2/raw/curobo2_$s.power.json -- $C2 bench/v2_curobo.py --version v2 --freeze $H --set $s \
    --subset results/v2/v2_subsample.json --seeds 3 --deadline $DEADLINE
  log "V2 $s rc=$? (0 complete, 3 partial)"
done
kill $WD 2>/dev/null
log "SLOT DONE"
