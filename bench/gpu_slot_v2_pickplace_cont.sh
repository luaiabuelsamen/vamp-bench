#!/usr/bin/env bash
# Continuation of the cuRobo pick-place slot (started 2026-10-11 22:58:48) after amendment A5 regenerated the cells.
# Same clock as bench/gpu_slot_v2_pickplace.sh: no new cell after 04:38:48, its watchdog kills runners at 04:48:48.
# Smoke tests of both versions on the A5 smoke cell already passed in this slot (records present). GPU_RELEASED on exit.
cd "$(dirname "$0")/.."
log(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
H=6aea6eecf76a173d7ebdf548104aaec854b6b6c7
DEADLINE=$(date -d '2026-10-12 04:38:48' +%s)
FEAS='^\.venv/bin/python bench/v2_feasibility_par\.py'
cleanup(){
  M=$(pgrep -f "$FEAS" | tr '\n' ' ')
  [ -n "$M" ] && kill -CONT $M 2>/dev/null && log "resumed my CPU jobs: $M"
  touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"
}
trap cleanup EXIT
until [ -f problems/v2_pickplace.pkl ] && ! pgrep -f '^\.venv/bin/python bench/v2_pickplace_gen' >/dev/null; do sleep 20; done
log "A5 cells ready; cuRobo pick-place continues"
unset PYTHONPATH
for v in v1 v2; do
  [ $(date +%s) -gt $DEADLINE ] && { log "deadline reached before $v"; break; }
  if [ $v = v1 ]; then CP=~/projects/curobo_berkeley/venv/bin/python; export PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only
  else CP=~/projects/clean_env/venv/bin/python; export PYTHONPATH=bench:.vamp_only; fi
  python3 bench/powerlog.py results/v2/raw/curobo${v#v}_pickplace.power.json -- $CP bench/v2_pickplace_curobo.py --version $v --freeze $H --seeds 5 --deadline $DEADLINE
  log "$v pick-place rc=$? (0 complete, 3 partial)"
done
log "SLOT DONE"
