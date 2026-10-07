#!/usr/bin/env bash
# Full benchmark: VAMP then cuRobo on all problem sets, each wrapped in tegrastats logging.
# usage: bench/run_all.sh [--vamp-only|--curobo-only] [OUTDIR=results/raw]
# Run the cuRobo half only when the GPU is idle (tegrastats GR3D_FREQ ~0%).
set -euo pipefail
cd "$(dirname "$0")/.."
MODE=all; case "${1:-}" in --vamp-only) MODE=vamp; shift;; --curobo-only) MODE=curobo; shift;; esac
OUT=${1:-results/raw}; mkdir -p "$OUT"
unset PYTHONPATH
CP=~/projects/curobo_berkeley/venv/bin/python
VP=.venv/bin/python
PL="python3 bench/powerlog.py"
if [ $MODE != curobo ]; then
for r in panda ur5; do
  $PL $OUT/vamp_mbm_$r.power.json -- $VP bench/vamp_mbm.py --robot $r --out $OUT/vamp_mbm_$r.json
done
for s in gantry_pillars:ur5erail gantry_free:ur5erail ur5e_pillars:ur5e; do
  sc=${s%%:*}; m=${s##*:}
  $PL $OUT/vamp_${sc}_$m.power.json -- $VP bench/vamp_mbm.py --robot $m --problems problems/$sc.pkl --tag $sc --trials 10 --out $OUT/vamp_${sc}_$m.json
done
fi
if [ $MODE != vamp ]; then
SETS="gantry_pillars:ur5e_rail:ur5erail gantry_free:ur5e_rail:ur5erail ur5e_pillars:ur5e:ur5e"
# cuRobo v1 (the version the paper used)
export PYTHONPATH=third_party/curobo_v1/src:bench
for r in panda ur5; do
  $PL $OUT/curobo_mbm_$r.power.json -- $CP bench/curobo_mbm.py --robot $r --out $OUT/curobo_mbm_$r.json || echo "FAILED v1 $r"
done
for s in $SETS; do
  IFS=: read -r sc c m <<<"$s"
  $PL $OUT/curobo_${sc}_$c.power.json -- $CP bench/curobo_mbm.py --robot $c --robot-cfg configs/${c}_shared.yml \
    --problems problems/$sc.pkl --tag $sc --trials 10 --out $OUT/curobo_${sc}_$c.json || echo "FAILED v1 $sc"
done
# cuRoboV2 (current; existing build in clean_env venv, editable -> curobo_berkeley/curobo ac5f931)
C2=~/projects/clean_env/venv/bin/python
export PYTHONPATH=bench
for r in panda ur5; do
  $PL $OUT/curobo2_mbm_$r.power.json -- $C2 bench/curobo2_mbm.py --robot $r --out $OUT/curobo2_mbm_$r.json || echo "FAILED v2 $r"
done
for s in $SETS; do
  IFS=: read -r sc c m <<<"$s"
  $PL $OUT/curobo2_${sc}_$c.power.json -- $C2 bench/curobo2_mbm.py --robot $c --robot-cfg configs/${c}_shared_v2.yml \
    --problems problems/$sc.pkl --tag $sc --trials 10 --out $OUT/curobo2_${sc}_$c.json || echo "FAILED v2 $sc"
done
# every cuRobo success re-checked with VAMP's collision checker
export PYTHONPATH=
for v in curobo curobo2; do
  for r in panda ur5; do [ -f $OUT/${v}_mbm_$r.json ] && $VP bench/crossval.py $OUT/${v}_mbm_$r.json $r || true; done
  for s in $SETS; do
    IFS=: read -r sc c m <<<"$s"
    [ -f $OUT/${v}_${sc}_$c.json ] && $VP bench/crossval.py $OUT/${v}_${sc}_$c.json $m problems/$sc.pkl || true
  done
done
fi
echo ALL DONE
