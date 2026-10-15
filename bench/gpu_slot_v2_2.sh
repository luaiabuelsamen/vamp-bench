#!/bin/bash
# v2.2 GPU slot (docs/V2_2_MOTION_TYPES_PREREGISTRATION.md, Size and cost; amendments V1, V2, V2a; P0 done on dev cells).
# Order: Cartesian pipeline timing (quiet CPU) -> cuRobo v1 as configured (10 seeds) -> v1 max_jerk 200 (10 seeds)
# -> cuRoboV2 (3 seeds) -> PyRoki (1 run). Every runner is resumable and seed-major; a deadline stops cleanly.
# Hard stop 2026-10-17 12:00 local (flagship takes the GPU that evening); GPU_RELEASED touched on exit.
cd ~/projects/vamp-bench
H=6aea6eecf76a173d7ebdf548104aaec854b6b6c7
DEADLINE=$(date -d "2026-10-17 11:30" +%s); HARD=$(date -d "2026-10-17 12:00" +%s)
LOG=logs/gpu_slot_v2_2.log
log(){ echo "$(date '+%F %T') $*" >> $LOG; }
cleanup(){ kill $WD 2>/dev/null; touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"; }
trap cleanup EXIT
rm -f ~/projects/_ops/GPU_RELEASED
# watchdog: at the hard stop, kill this slot's python processes (matched on the python binary, never a shell line)
( while [ $(date +%s) -lt $HARD ]; do sleep 30; done; log "HARD STOP"
  pkill -f '^[^ ]*/python[0-9.]* bench/v2_2_' ) & WD=$!
log "slot start; load $(cut -d' ' -f1 /proc/loadavg); MemAvailable $(awk '/MemAvailable/{print $2}' /proc/meminfo) kB"
pgrep -af '^\.venv/bin/python bench/v2_2_gen' >/dev/null && { log "generator still running: not starting"; exit 1; }
[ -f problems/v2_2_programs.pkl ] || { log "no problems/v2_2_programs.pkl: not starting"; exit 1; }
unset PYTHONPATH
log "cartesian (quiet CPU, 10 repeats)"
.venv/bin/python bench/v2_2_cartesian.py --repeats 10 >> logs/v2_2_cartesian.log 2>&1; log "cartesian rc $?"
C1=~/projects/curobo_berkeley/venv/bin/python; C2=~/projects/clean_env/venv/bin/python
log "curobo v1 as configured"
PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only $C1 bench/v2_2_curobo.py --version v1 --freeze $H --seeds 10 --deadline $DEADLINE >> logs/v2_2_curobo1.log 2>&1; log "v1 rc $?"
log "curobo v1 max_jerk 200"
PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only $C1 bench/v2_2_curobo.py --version v1 --freeze $H --seeds 10 --max-jerk 200 --deadline $DEADLINE >> logs/v2_2_curobo1_j200.log 2>&1; log "v1 j200 rc $?"
log "curoboV2"
PYTHONPATH=bench:.vamp_only $C2 bench/v2_2_curobo.py --version v2 --freeze $H --seeds 3 --deadline $DEADLINE >> logs/v2_2_curobo2.log 2>&1; log "v2 rc $?"
log "pyroki"
PYROKI_MARGIN=0.005 bench/pyroki_env.sh python bench/v2_2_pyroki.py --deadline $DEADLINE >> logs/v2_2_pyroki.log 2>&1; log "pyroki rc $?"
log "SLOT DONE"
