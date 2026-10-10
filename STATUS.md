# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-8 — all planner runs of the main v2 benchmark DONE: VAMP (pass a + 2a), cuRobo v1 (9000 runs), cuRoboV2 subsample (900 runs; held-out chained after a config-key crash, same slot, GPU_RELEASED 05:16). Now: pass 2b validation (CPU, 2 cores), then A3 reference runs, then analysis
Last result: TOPP-RA/Ruckig cost measured before freeze: Ruckig P1 0.36 ms p50, TOPP-RA P2 17.6 ms p50 per path (results/v2/timing_cost.json)
Blocked: next GPU slot needed for the pick-place step: cuRobo v1 ~1 GPU-h, cuRoboV2 ~1-3 GPU-h (T = 30 s per segment), plus ~15 min at slot start for VAMP's timed pick-place segments on a quiet CPU.
  Optional: a truly idle-CPU re-timing of VAMP successes (~15 min) if flagship CPU jobs can also be paused at slot start (night-2 re-timing ran with ~4 flagship cores busy).
  CPU (2 cores, saturated machine): pass 2b ~40 min; A3 reference runs for queries no planner solved: ~20-25 min each on this CPU.
Next: pass 2b -> A3 references -> analysis (summary + success-vs-budget curves) -> pick-place generation + VAMP run -> GPU slot for cuRobo pick-place
Updated: 2026-10-10 05:18
