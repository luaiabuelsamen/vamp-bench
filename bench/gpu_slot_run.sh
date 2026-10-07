#!/usr/bin/env bash
# Orchestrator GPU schedule (2026-10-07): wait for the flagship to pause (no train_local.py), then start the clean
# cuRobo run once the GPU has been idle 10 consecutive minutes, or unconditionally 30 min after the pause.
# On any exit, touch ~/projects/_ops/GPU_RELEASED so the flagship resumes.
cd "$(dirname "$0")/.."
trap 'touch ~/projects/_ops/GPU_RELEASED; echo "$(date +%H:%M) touched GPU_RELEASED"' EXIT
log(){ echo "$(date +%H:%M) $*"; }
while pgrep -f "train_local[.]py" >/dev/null; do sleep 60; done
paused=$(date +%s); log "flagship paused (no train_local.py)"
idle=0
while [ $idle -lt 10 ] && [ $(( $(date +%s) - paused )) -lt 1800 ]; do
  g=$(timeout 5 tegrastats --interval 1000 | head -3 | grep -oE "GR3D_FREQ [0-9]+" | awk '{s+=$2} END {print int(s/NR)}')
  if [ "${g:-100}" -le 5 ]; then idle=$((idle+1)); else idle=0; fi
  log "GR3D=${g}% idle_min=$idle"; sleep 55
done
if [ $idle -ge 10 ]; then log "GPU idle 10 min; starting clean cuRobo run"
else log "30 min after pause; starting cuRobo run (GPU not confirmed idle)"; fi
bench/run_all.sh --curobo-only results/raw; log "run_all rc=$?"
unset PYTHONPATH; .venv/bin/python bench/analyze.py; log "analyze rc=$?"
log "SLOT DONE"
