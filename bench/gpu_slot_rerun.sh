#!/usr/bin/env bash
# Second GPU slot: v1 paper scenes (first run's results lost to a save bug) and UR5 v1 + v2 (velocity-limit fix).
# Fires only when the flagship is fully idle (no train_local.py, no collect_local.py) and GR3D <= 5% for 10 min.
# Touches ~/projects/_ops/GPU_RELEASED on exit.
cd "$(dirname "$0")/.."
trap 'touch ~/projects/_ops/GPU_RELEASED; echo "$(date +%H:%M) touched GPU_RELEASED"' EXIT
log(){ echo "$(date +%H:%M) $*"; }
idle=0
while [ $idle -lt 10 ]; do
  g=$(timeout 5 tegrastats --interval 1000 | head -3 | grep -oE "GR3D_FREQ [0-9]+" | awk '{s+=$2} END {print int(s/NR)}')
  if [ "${g:-100}" -le 5 ] && ! pgrep -f "train_local[.]py|collect_local[.]py" >/dev/null; then idle=$((idle+1)); else idle=0; fi
  [ $idle -gt 0 ] && log "GR3D=${g}% idle_min=$idle"; sleep 55
done
log "flagship idle 10 min; starting rerun"
unset PYTHONPATH
OUT=results/raw; PL="python3 bench/powerlog.py"; CP=~/projects/curobo_berkeley/venv/bin/python; C2=~/projects/clean_env/venv/bin/python; VP=.venv/bin/python
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
