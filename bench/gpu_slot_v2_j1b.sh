#!/usr/bin/env bash
# Continuation of the 2026-10-12 night session: waits for the running v1 heldout_a6 run to exit, then A7 J1b
# (v1 pick-place, max_jerk 200). No new pick after 08:40, kill at 08:50 (watchdog already running). Then resumes the
# reference runner and touches GPU_RELEASED.
cd "$(dirname "$0")/.."
log(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
H=6aea6eecf76a173d7ebdf548104aaec854b6b6c7; DEADLINE=$(date -d 'tomorrow 08:40' +%s)
FEAS='^\.venv/bin/python bench/v2_feasibility_par\.py'
cleanup(){ M=$(pgrep -f "$FEAS" | tr '\n' ' '); [ -n "$M" ] && kill -CONT $M && log "resumed reference runner: $M"
  touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"; }
trap cleanup EXIT
while pgrep -f '^[^ ]*python[0-9.]* bench/v2_curobo\.py --version v1 .*heldout_a6' >/dev/null; do sleep 60; done
log "v1 heldout_a6 exited; $(tail -1 results/v2/raw/curobo1_heldout_a6.jsonl | cut -c1-60)"
[ $(date +%s) -gt $DEADLINE ] && { log "deadline reached before J1b"; exit 0; }
unset PYTHONPATH; C1=~/projects/curobo_berkeley/venv/bin/python
export PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only
python3 bench/powerlog.py results/v2/raw/curobo1_pickplace.A6.jerk200.power.json -- $C1 bench/v2_pickplace_curobo.py --version v1 --freeze $H --seeds 5 --max-jerk 200 \
  --out results/v2/raw/curobo1_pickplace.A6.jerk200.jsonl --deadline $DEADLINE
log "A7 J1b rc=$? (0 complete, 3 partial)"; log "SESSION DONE"
