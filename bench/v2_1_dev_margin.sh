#!/usr/bin/env bash
# v2.1 amendment 7d: choose PyRoki's world-collision margin on the gantry dev set (problems/gantry_pillars.pkl, not in
# v2), every 5th query (21), S1 seeds, GPU. Rule (fixed before running): the margin with the most R1 successes; ties
# go to the larger margin (PyRoki's example uses 0.05 m).
cd "$(dirname "$0")/.."
for m in 0.0 0.005 0.01 0.02 0.05; do
  echo "$(date '+%H:%M:%S') margin $m"
  PYROKI_MARGIN=$m bench/pyroki_env.sh python bench/v2_1_pyroki.py trajopt --set gantry --seeds s1 --stride 5 --tag _m$m 2>&1 | grep -vE "INFO|^\s" | tail -25
done
echo "DEV DONE"
