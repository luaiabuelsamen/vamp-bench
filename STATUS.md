# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-2 — VAMP CPU step running (held-out 26/300 at 18:24; ETA ~14 h CPU because VAMP fails ~43% of held-out runs at its 1M-iteration default, ~55 s each; then MBM + MpiNets ~1 h). Pick-place step pre-registered.
Last result: TOPP-RA/Ruckig cost measured before freeze: Ruckig P1 0.36 ms p50, TOPP-RA P2 17.6 ms p50 per path (results/v2/timing_cost.json)
Blocked: GPU slot needed (amendment A2: uniform per-query budget T = 30 s for every planner; V2 keeps max_attempts=100):
  - cuRobo v1, 900 queries x 10 seeds: ~6.5 GPU-h expected (5-9)
  - cuRoboV2, 300-query subsample x 3 seeds: 3.0 GPU-h expected, 4.2 pessimistic, 7.65 hard ceiling (every run times out)  [results/v2/budget_choice.json]
  - pick-place step: v1 ~1 GPU-h; V2 ~1-3 GPU-h with T = 30 s (optional)
  Total ~11 GPU-h expected, ~14 pessimistic, ~17 hard ceiling. Preferred: two overnight slots (v1 first, then V2 + pick-place).
Next: VAMP CPU step on held-out/MBM/MpiNets × 10 seeds -> exact GPU-hours under Blocked -> GPU slot (cuRobo v1 full, V2 subsample) -> pick-place MoveL sequence
Updated: 2026-10-08 20:45
