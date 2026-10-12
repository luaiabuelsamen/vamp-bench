#!/usr/bin/env bash
# GPU slot for cuRobo pick-place on the A6 cells (v1, then V2), after flagship's Franka run (orchestrator 2026-10-12).
# Trigger: a PAUSED line in a flagship queue log (contact-insertion outputs/queue*.log, analysis/logs/queue*.log)
# written after this watcher was armed, with no RESUMED after it; plus no python train_local.py process.
# Cap: 8 h from slot start. No new cell after start+7h40; runners killed at start+7h50 (checkpoints keep partials);
# GPU_RELEASED touched on exit once the slot has started. No CPU benchmark work runs in this slot.
cd "$(dirname "$0")/.."
log(){ echo "$(date '+%m-%d %H:%M:%S') $*"; }
memgb(){ awk '/MemAvailable/ {printf "%.1f", $2/1048576}' /proc/meminfo; }
H=6aea6eecf76a173d7ebdf548104aaec854b6b6c7
CI=~/projects/contact-insertion
ARMED=$(date '+%F %T'); log "armed; waiting for a flagship PAUSED line after $ARMED"
TRAIN='^[^ ]*python[0-9.]* [^ ]*train_local\.py'
FEAS='^\.venv/bin/python bench/v2_feasibility_par\.py'
paused(){  # newest PAUSED/RESUMED line across the queue logs is a PAUSED written after ARMED
  last=$(grep -hE '^(PAUSED|RESUMED) ' $CI/outputs/queue*.log $CI/analysis/logs/queue*.log 2>/dev/null \
         | sed -E 's/^(PAUSED|RESUMED) ([0-9-]+ [0-9:]+[0-9]).*/\2 \1/' | sort | tail -1)
  [ "${last##* }" = PAUSED ] && [[ "${last% *}" > "$ARMED" ]]
}
n=0
until paused && ! pgrep -f "$TRAIN" >/dev/null; do sleep 60; n=$((n+1)); [ $((n % 60)) = 0 ] && log "still waiting"; done
START=$(date +%s); DEADLINE=$((START + 7*3600 + 40*60)); KILL_AT=$((START + 7*3600 + 50*60))
FEAS_KILLED=0
cleanup(){
  M=$(pgrep -f "$FEAS" | tr '\n' ' ')
  [ -n "$M" ] && kill -CONT $M 2>/dev/null && log "resumed my CPU jobs: $M"
  if [ $FEAS_KILLED = 1 ]; then
    nohup .venv/bin/python bench/v2_feasibility_par.py --freeze $H --workers 2 --max-workers 6 >> logs/v2_feas_par.log 2>&1 &
    log "restarted reference runner (pid $!)"
  fi
  touch ~/projects/_ops/GPU_RELEASED; log "touched GPU_RELEASED"
}
trap cleanup EXIT
M=$(pgrep -f "$FEAS" | tr '\n' ' ')
[ -n "$M" ] && kill -STOP $M && log "suspended reference runner: [$M]"
log "slot starts (flagship PAUSED seen); no new cell after $(date -d @$DEADLINE '+%a %H:%M'), kill at $(date -d @$KILL_AT '+%H:%M'); MemAvailable $(memgb) GB"
ps -eo pcpu,rss,args --sort=-pcpu | head -6 | cut -c1-150 | sed 's/^/load: /'
if [ "$(memgb | cut -d. -f1)" -lt 4 ] && [ -n "$M" ]; then
  kill -TERM $M 2>/dev/null; kill -CONT $M 2>/dev/null; sleep 5; FEAS_KILLED=1
  log "MemAvailable under 4 GB with the reference runner suspended: stopped it (restarts after the slot); now $(memgb) GB"
fi
( while [ $(date +%s) -lt $KILL_AT ]; do sleep 30; done; pkill -TERM -f '^[^ ]*python[0-9.]* bench/v2_pickplace_curobo\.py' && echo "$(date '+%H:%M') watchdog killed cuRobo run" ) &
WD=$!
unset PYTHONPATH
C1=~/projects/curobo_berkeley/venv/bin/python; C2=~/projects/clean_env/venv/bin/python
SM=$(mktemp -d)
for v in v1 v2; do
  [ $(date +%s) -gt $DEADLINE ] && { log "deadline reached before $v"; break; }
  if [ $v = v1 ]; then CP=$C1; export PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only; else CP=$C2; export PYTHONPATH=bench:.vamp_only; fi
  $CP bench/v2_pickplace_curobo.py --version $v --freeze $H --seeds 1 --limit 1 --out $SM/smoke_$v.jsonl > $SM/smoke_$v.log 2>&1; rc=$?
  log "$v smoke rc=$rc"; tail -3 $SM/smoke_$v.log | sed "s/^/  /"
  if ! [ -s $SM/smoke_$v.jsonl ]; then log "$v SMOKE PRODUCED NO RECORD; skipping $v"; tail -30 $SM/smoke_$v.log | sed 's/^/  /'; continue; fi
  cp $SM/smoke_$v.jsonl results/v2/raw/smoke_pickplace_A6_$v.jsonl
  grep -o '"diag_[a-z_]*": [^,}]*' $SM/smoke_$v.jsonl | sort | uniq -c | sort -rn | head -8 | sed 's/^/  diag: /'
  python3 bench/powerlog.py results/v2/raw/curobo${v#v}_pickplace.power.json -- $CP bench/v2_pickplace_curobo.py --version $v --freeze $H --seeds 5 --deadline $DEADLINE
  log "$v pick-place rc=$? (0 complete, 3 partial)"
done
kill $WD 2>/dev/null
log "SLOT DONE"
