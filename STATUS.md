# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-4 — cuRobo v1 full run DONE in tonight's slot (22:32-05:27, GPU_RELEASED 05:26): planner success held-out 521/3000, MBM 2701/3000, MpiNets 2927/3000; 0 gate rejections. VAMP CPU step resumed. V2 + pick-place tomorrow night
Last result: TOPP-RA/Ruckig cost measured before freeze: Ruckig P1 0.36 ms p50, TOPP-RA P2 17.6 ms p50 per path (results/v2/timing_cost.json)
Blocked: GPU slot needed (amendment A2: uniform per-query budget T = 30 s for every planner; V2 keeps max_attempts=100):
  - cuRobo v1: DONE (2026-10-08/09 slot, 6.9 h)
  - cuRoboV2, 300-query subsample x 3 seeds: 3.0 GPU-h expected, 4.2 pessimistic, 7.65 hard ceiling (every run times out)  [results/v2/budget_choice.json]
  - pick-place step: v1 ~1 GPU-h; V2 ~1-3 GPU-h with T = 30 s (optional)
  Total ~11 GPU-h expected, ~14 pessimistic, ~17 hard ceiling. Preferred: two overnight slots (v1 first, then V2 + pick-place).
Next: VAMP CPU step on held-out/MBM/MpiNets × 10 seeds -> exact GPU-hours under Blocked -> GPU slot (cuRobo v1 full, V2 subsample) -> pick-place MoveL sequence
Updated: 2026-10-09 05:27
