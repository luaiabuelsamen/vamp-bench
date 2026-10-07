#!/usr/bin/env bash
# Start the clean cuRobo run only after the GPU has been idle (GR3D_FREQ <= 5%) for 10 consecutive minutes
# and no training job is running. Then cross-check and re-analyze.
cd "$(dirname "$0")/.."
idle=0
while [ $idle -lt 10 ]; do
  g=$(timeout 5 tegrastats --interval 1000 | head -3 | grep -oE "GR3D_FREQ [0-9]+" | awk '{s+=$2} END {print int(s/NR)}')
  if [ "${g:-100}" -le 5 ] && ! pgrep -f "train_local.py|train.py" >/dev/null; then idle=$((idle+1)); else idle=0; fi
  echo "$(date +%H:%M) GR3D=${g}% idle_min=$idle"
  sleep 55
done
echo "GPU idle since $(date -d '-10 min' +%H:%M); starting clean cuRobo run"
bench/run_all.sh --curobo-only results/raw && unset PYTHONPATH && .venv/bin/python bench/analyze.py
echo WATCHER EXIT $?
