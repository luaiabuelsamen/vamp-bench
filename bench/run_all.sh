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
export PYTHONPATH=third_party/curobo_v1/src:bench
for r in panda ur5; do
  $PL $OUT/curobo_mbm_$r.power.json -- $CP bench/curobo_mbm.py --robot $r --out $OUT/curobo_mbm_$r.json
done
for s in gantry_pillars:ur5e_rail gantry_free:ur5e_rail ur5e_pillars:ur5e; do
  sc=${s%%:*}; c=${s##*:}
  $PL $OUT/curobo_${sc}_$c.power.json -- $CP bench/curobo_mbm.py --robot $c --robot-cfg configs/${c}_shared.yml --problems problems/$sc.pkl --tag $sc --trials 10 --out $OUT/curobo_${sc}_$c.json
done
export PYTHONPATH=
for f in $OUT/curobo_mbm_panda.json:panda: $OUT/curobo_mbm_ur5.json:ur5: \
         $OUT/curobo_gantry_pillars_ur5e_rail.json:ur5erail:problems/gantry_pillars.pkl \
         $OUT/curobo_gantry_free_ur5e_rail.json:ur5erail:problems/gantry_free.pkl \
         $OUT/curobo_ur5e_pillars_ur5e.json:ur5e:problems/ur5e_pillars.pkl; do
  IFS=: read -r j m p <<<"$f"; $VP bench/crossval.py $j $m $p
done
fi
echo ALL DONE
