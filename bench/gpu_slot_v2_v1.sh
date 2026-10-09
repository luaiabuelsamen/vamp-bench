#!/usr/bin/env bash
# GPU slot (night of 2026-10-08): benchmark v2, cuRobo v1 full run under amendment A2 (T = 30 s).
# Gate: no python train_local.py process (matched on the interpreter at the start of the command line, never a
# shell's command line) AND GPU idle (GR3D <= 5%) for 10 consecutive minutes.
# While the slot runs, the VAMP CPU step is suspended (SIGSTOP) so its timing never shares the CPU with cuRobo;
# it is resumed on exit, and the one VAMP run caught mid-plan is re-run afterwards (bench/v2_vamp_fix_stopped.py).
# Deadline: no new query after 07:50; a watchdog kills the run at 07:58; ~/projects/_ops/GPU_RELEASED is touched
# on exit (only if the slot started). Partial results are kept in results/v2/raw/*.jsonl and resumed next slot.
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
VPID=$(pgrep -f '^\.venv/bin/python bench/v2_vamp\.py' | head -1)
if [ -n "$VPID" ]; then PL=$(ps -o ppid= -p $VPID | tr -d ' '); LOOP=$(ps -o ppid= -p $PL | tr -d ' '); fi
cleanup(){
  [ -n "$VPID" ] && kill -CONT $VPID $LOOP 2>/dev/null && log "resumed VAMP CPU step ($VPID, loop $LOOP)"
  touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"
}
trap cleanup EXIT
if [ -n "$VPID" ]; then kill -STOP $LOOP $VPID; log "suspended VAMP CPU step ($VPID, loop $LOOP) at $(date +%s)"; echo "$(date +%s)" > results/v2/vamp_suspended_at.txt; fi
log "GPU idle 10 min; slot starts. Deadline $(date -d @$DEADLINE '+%H:%M'), watchdog $(date -d @$KILL_AT '+%H:%M')"
pgrep -af "collect_local[.]py" | head -1 | sed 's/^/other job running: /'
unset PYTHONPATH
CP=~/projects/curobo_berkeley/venv/bin/python
export PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only
# smoke test: one query, one seed, scratch output
SM=$(mktemp -d)
$CP bench/v2_curobo.py --version v1 --freeze $H --set heldout --limit 1 --seeds 1 --out $SM/smoke.json; rc=$?
log "smoke test rc=$rc (3 = partial is expected? no: 1 query 1 seed must complete -> 0)"
[ $rc -ne 0 ] && { log "SMOKE FAILED; stopping slot"; exit 1; }
( while [ $(date +%s) -lt $KILL_AT ]; do sleep 30; done; pkill -TERM -f '^[^ ]*python[0-9.]* bench/v2_curobo\.py' && echo "$(date '+%H:%M') watchdog killed v1 run" ) &
WD=$!
for s in heldout mbm mpinets; do
  [ $(date +%s) -gt $DEADLINE ] && { log "deadline reached before $s"; break; }
  python3 bench/powerlog.py results/v2/raw/curobo1_$s.power.json -- $CP bench/v2_curobo.py --version v1 --freeze $H --set $s --deadline $DEADLINE
  log "v1 $s rc=$? (0 complete, 3 partial)"
done
kill $WD 2>/dev/null
log "SLOT DONE"
