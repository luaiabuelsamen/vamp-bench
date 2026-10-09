# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-2 — VAMP CPU step running (held-out 26/300 at 18:24; ETA ~14 h CPU because VAMP fails ~43% of held-out runs at its 1M-iteration default, ~55 s each; then MBM + MpiNets ~1 h). Pick-place step pre-registered.
Last result: TOPP-RA/Ruckig cost measured before freeze: Ruckig P1 0.36 ms p50, TOPP-RA P2 17.6 ms p50 per path (results/v2/timing_cost.json)
Blocked: GPU slot needed (estimates from clean-slot per-query costs; pre-registered settings):
  - cuRobo v1, 900 queries x 10 seeds (held-out ~3 s/q, Panda ~2 s/q, + warmups + 1 kHz validation): ~6.5 GPU-h (range 5-9)
  - cuRoboV2, 300-query stratified subsample x 3 seeds at max_attempts=100 (NVIDIA benchmark value): ~8.7 GPU-h (range 5-20; a failed V2 Panda query costs 200-750 s at 100 attempts)
      option: amend to max_attempts=20 (same as v1) BEFORE the run -> ~2.9 GPU-h; would be logged as amendment A2 and every V2 success rate labelled "lower bound"
  - pick-place step: cuRobo v1 ~1 GPU-h; V2 1-7 GPU-h (optional)
  Total: ~16 GPU-h (V2 at 100 attempts) or ~10.5 GPU-h (V2 at 20). Preferred: two overnight slots. Decision needed: V2 max_attempts 100 vs 20.
Next: VAMP CPU step on held-out/MBM/MpiNets × 10 seeds -> exact GPU-hours under Blocked -> GPU slot (cuRobo v1 full, V2 subsample) -> pick-place MoveL sequence
Updated: 2026-10-08 18:25
