#!/usr/bin/env bash
# GPU slot, night of 2026-10-10 -> 11: (1) A4 pass 2a repeat on a quiet CPU, (2) VAMP pick-place (timed, quiet CPU),
# (3) cuRobo v1 pick-place, (4) cuRoboV2 pick-place until the deadline (docs/V2_PICKPLACE_PREREGISTRATION.md).
# Gate: no python train_local.py process (interpreter-anchored match), GPU idle and MemAvailable >= 4 GB for 10 min.
# My CPU jobs (A3 reference runs, pick-place cell generation if still running) are suspended for the slot; if
# MemAvailable stays under 4 GB with them suspended, the reference runner is stopped instead (it resumes from its
# checkpoint after the slot; references are deterministic).
# Deadline: no new cell/query after Sun 07:50; watchdog at 07:58; GPU_RELEASED touched on exit once the slot started.
cd "$(dirname "$0")/.."
log(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
memgb(){ awk '/MemAvailable/ {printf "%.1f", $2/1048576}' /proc/meminfo; }
H=6aea6eecf76a173d7ebdf548104aaec854b6b6c7
DEADLINE=$(date -d '2026-10-11 07:50' +%s); KILL_AT=$((DEADLINE + 480))
TRAIN='^[^ ]*python[0-9.]* [^ ]*train_local\.py'
FEAS='^\.venv/bin/python bench/v2_feasibility_par\.py'
GEN='^\.venv/bin/python bench/v2_pickplace_gen\.py'
while pgrep -f "$TRAIN" >/dev/null; do sleep 60; done
log "no python train_local.py process"
idle=0
while [ $idle -lt 10 ]; do
  pgrep -f "$TRAIN" >/dev/null && { idle=0; sleep 60; continue; }
  g=$(timeout 5 tegrastats --interval 1000 | head -3 | grep -oE "GR3D_FREQ [0-9]+" | awk '{s+=$2} END {print int(s/NR)}')
  m=$(memgb)
  # flagship's evaluation can leave the GPU idle for minutes; while it holds the RAM it has not paused
  if [ "${g:-100}" -le 5 ] && [ "${m%.*}" -ge 4 ]; then idle=$((idle+1)); else idle=0; fi
  log "GR3D=${g}% MemAvailable=${m}GB idle_min=$idle"; sleep 55
  [ $(date +%s) -gt $DEADLINE ] && { log "deadline passed before the gate opened; not starting"; exit 0; }
done
FEAS_KILLED=0
cleanup(){
  M=$(pgrep -f "$FEAS" | tr '\n' ' '); G=$(pgrep -f "$GEN" | tr '\n' ' ')
  [ -n "$M$G" ] && kill -CONT $M $G 2>/dev/null && log "resumed my CPU jobs: $M $G"
  if [ $FEAS_KILLED = 1 ]; then
    echo "=== $(date '+%F %T') restart after slot ===" >> logs/v2_feas_par.log
    nohup .venv/bin/python bench/v2_feasibility_par.py --freeze $H --workers 2 --max-workers 6 --mem-gate-gb 4 >> logs/v2_feas_par.log 2>&1 &
    log "restarted reference runner (pid $!)"
  fi
  touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"
}
trap cleanup EXIT
M=$(pgrep -f "$FEAS" | tr '\n' ' '); G=$(pgrep -f "$GEN" | tr '\n' ' ')
[ -n "$M$G" ] && kill -STOP $M $G && log "suspended my CPU jobs: feas [$M] gen [$G]"
log "slot starts; deadline $(date -d @$DEADLINE '+%a %H:%M'); MemAvailable $(memgb) GB"
ps -eo pcpu,rss,args --sort=-pcpu | head -8 | cut -c1-150 | sed 's/^/load: /'
unset PYTHONPATH
# (1) pass 2a repeat, quiet CPU (night-2 files kept as .night2)
for s in heldout mbm mpinets; do
  f=results/v2/raw/vamp_pass2a_$s.jsonl; [ -f $f ] && [ ! -f ${f%.jsonl}.night2.jsonl ] && mv $f ${f%.jsonl}.night2.jsonl
done
[ -f results/v2/pass2a_check.json ] && [ ! -f results/v2/pass2a_check.night2.json ] && git mv results/v2/pass2a_check.json results/v2/pass2a_check.night2.json
.venv/bin/python bench/v2_vamp_pass2a.py --freeze $H --deadline $DEADLINE; log "pass 2a rc=$?"
# (2) VAMP pick-place: needs the cells; if generation is still running, let it finish first (it is CPU, so it would
# contend with the timed run)
if [ -n "$G" ] && pgrep -f "$GEN" >/dev/null; then
  kill -CONT $G; log "waiting for pick-place cell generation"
  while pgrep -f "$GEN" >/dev/null; do sleep 30; [ $(date +%s) -gt $DEADLINE ] && break; done
fi
[ -f problems/v2_pickplace.pkl ] || { log "no pick-place cells; stopping slot"; exit 1; }
.venv/bin/python bench/v2_pickplace_vamp.py --freeze $H --seeds 5; log "VAMP pick-place rc=$?"
# cuRobo needs GPU memory out of the same RAM
if [ "$(memgb | cut -d. -f1)" -lt 4 ] && [ -n "$M" ]; then
  kill -TERM $M 2>/dev/null; kill -CONT $M 2>/dev/null; sleep 5; FEAS_KILLED=1
  log "MemAvailable under 4 GB with my jobs suspended: stopped the reference runner (resumes after the slot); now $(memgb) GB"
fi
( while [ $(date +%s) -lt $KILL_AT ]; do sleep 30; done; pkill -TERM -f '^[^ ]*python[0-9.]* bench/v2_pickplace_curobo\.py' && echo "$(date '+%H:%M') watchdog killed cuRobo run" ) &
WD=$!
export PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only
C1=~/projects/curobo_berkeley/venv/bin/python; C2=~/projects/clean_env/venv/bin/python
SM=$(mktemp -d)
for v in v1 v2; do
  [ $(date +%s) -gt $DEADLINE ] && { log "deadline reached before $v"; break; }
  if [ $v = v1 ]; then CP=$C1; export PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only; else CP=$C2; export PYTHONPATH=bench:.vamp_only; fi
  $CP bench/v2_pickplace_curobo.py --version $v --freeze $H --seeds 1 --problems results/v2/raw/smoke_cell.pkl --out $SM/smoke_$v.jsonl > $SM/smoke_$v.log 2>&1; rc=$?
  log "$v smoke rc=$rc"; tail -3 $SM/smoke_$v.log | sed "s/^/  /"
  if ! [ -s $SM/smoke_$v.jsonl ]; then log "$v SMOKE PRODUCED NO RECORD; skipping $v"; tail -30 $SM/smoke_$v.log | sed 's/^/  /'; continue; fi
  cp $SM/smoke_$v.jsonl results/v2/raw/smoke_pickplace_$v.jsonl
  python3 bench/powerlog.py results/v2/raw/curobo${v#v}_pickplace.power.json -- $CP bench/v2_pickplace_curobo.py --version $v --freeze $H --seeds 5 --deadline $DEADLINE
  log "$v pick-place rc=$? (0 complete, 3 partial)"
done
kill $WD 2>/dev/null
log "SLOT DONE"
