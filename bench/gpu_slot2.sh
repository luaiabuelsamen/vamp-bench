#!/usr/bin/env bash
# GPU slot 2 (orchestrator review, 2026-10-07):
#   1. verify the cuRoboV2 between-waypoint collision finding (bench/verify_v2_collisions.py)
#   2. cuRobo v1 paper scenes (slot-1 results lost to a save-path bug)
#   3. UR5 MBM, cuRobo v1 + V2, with the real UR5 velocity limit
#   4. VAMP cross-check of every cuRobo success (same criterion for v1 and V2), then analysis
# Waits for the flagship to pause (no train_local.py); starts when GR3D <= 5% for 10 min, or 30 min after the pause.
# ~/projects/_ops/GPU_RELEASED is touched on exit only if the slot actually started.
cd "$(dirname "$0")/.."
log(){ echo "$(date +%H:%M) $*"; }
# START_NOW=1: orchestrator has paused the flagship queue; start immediately (GPU state is logged, not waited on).
# Without it: wait for no python train_local.py process (match the interpreter, not shells that mention the name).
if [ -z "$START_NOW" ]; then
  while pgrep -f "python[^ ]* scripts/train_local[.]py" >/dev/null; do sleep 60; done
fi
paused=$(date +%s); log "flagship paused"
idle=0
[ -n "$START_NOW" ] && idle=10 && log "START_NOW: orchestrator go; GPU state at start is logged above"
while [ $idle -lt 10 ] && [ $(( $(date +%s) - paused )) -lt 1800 ]; do
  g=$(timeout 5 tegrastats --interval 1000 | head -3 | grep -oE "GR3D_FREQ [0-9]+" | awk '{s+=$2} END {print int(s/NR)}')
  if [ "${g:-100}" -le 5 ]; then idle=$((idle+1)); else idle=0; fi
  log "GR3D=${g}% idle_min=$idle"; sleep 55
done
trap 'touch ~/projects/_ops/GPU_RELEASED; echo "$(date +%H:%M) touched GPU_RELEASED"' EXIT
if [ -n "$START_NOW" ]; then log "starting slot 2 on orchestrator go"; elif [ $idle -ge 10 ]; then log "GPU idle 10 min; starting slot 2"; else log "30 min after pause; starting slot 2 (GPU not confirmed idle)"; fi
pgrep -af "collect_local[.]py" | head -1 | sed 's/^/other job running: /'
unset PYTHONPATH
OUT=results/raw; PL="python3 bench/powerlog.py"; CP=~/projects/curobo_berkeley/venv/bin/python; C2=~/projects/clean_env/venv/bin/python; VP=.venv/bin/python

[ -z "$SKIP_VERIFY" ] && { PYTHONPATH=bench:.vamp_only $C2 bench/verify_v2_collisions.py 5 || log "FAILED verify"; }

for s in gantry_pillars:ur5e_rail:ur5erail gantry_free:ur5e_rail:ur5erail ur5e_pillars:ur5e:ur5e; do
  IFS=: read -r sc c m <<<"$s"
  PYTHONPATH=third_party/curobo_v1/src:bench $PL $OUT/curobo_${sc}_$c.power.json -- $CP bench/curobo_mbm.py --robot $c \
    --robot-cfg configs/${c}_shared.yml --problems problems/$sc.pkl --tag $sc --trials 10 --out $OUT/curobo_${sc}_$c.json || log "FAILED v1 $sc"
  $VP bench/crossval.py $OUT/curobo_${sc}_$c.json $m problems/$sc.pkl || true
done
PYTHONPATH=third_party/curobo_v1/src:bench $PL $OUT/curobo_mbm_ur5.power.json -- $CP bench/curobo_mbm.py --robot ur5 --out $OUT/curobo_mbm_ur5.json || log "FAILED v1 ur5"
$VP bench/crossval.py $OUT/curobo_mbm_ur5.json ur5 || true
PYTHONPATH=bench $PL $OUT/curobo2_mbm_ur5.power.json -- $C2 bench/curobo2_mbm.py --robot ur5 --stride 10 --max-attempts 20 --budget-s 1500 --out $OUT/curobo2_mbm_ur5.json || log "FAILED v2 ur5"
$VP bench/crossval.py $OUT/curobo2_mbm_ur5.json ur5 || true
$VP bench/analyze.py; log "SLOT DONE"
