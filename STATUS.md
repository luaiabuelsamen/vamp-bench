# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-10 — cuRobo pick-place slot done (v1 complete, V2 partial 556/600); VAMP pick-place (A5 cells) running on CPU; A3 references running (memory-gated)
Last result: pick-place success, unanalysed raw counts: cuRobo v1 0/600 (first failure: free-segment planner 417; MoveL rejected for jerk only, 183), cuRoboV2 120/556 (all failures are free-segment planner failures; all 798 MoveL segments valid, median TCP deviation 0.65 mm). VAMP pending. Preliminary v2 (unchanged): success VAMP/v1/V2 held-out 49/17/37%, MBM 81/90/83%, MpiNets 88/98/97%
Blocked: none on GPU (pick-place slot ran 2026-10-11 22:38 to 2026-10-12 04:40, GPU_RELEASED touched 04:40). CPU only from here.
  Memory (standing rule 2026-10-11): during a flagship SmolVLA eval (train_local.py --skip-train ... smolvla) the A3 runner runs at most 1 worker and starts nothing unless MemAvailable > 2.5 GB (built into bench/v2_feasibility_par.py); otherwise 2 workers, >2 only after 10 min sustained > 4 GB. PRM references grow to ~1.5 GB, so under the eval most are floor-killed and requeued (12 so far): the A3 runs effectively wait for the eval to end.
  CPU: VAMP pick-place (A5) started 04:43, about 6 h, contended, load logged. A3 references: 92 held-out queries left (+ MBM/MpiNets), 20-60 min each per worker.
Next: VAMP pick-place -> pick-place analysis (P1-P3, post-hoc no-jerk gate and TCP speed vs spec, labelled) -> re-merge pass 2b with the pass 2a timings -> final v2 analysis when A3 finishes (per-bin tables, curves, H1-H5, README/EMAIL_NOTES v2, held-out description fix, check_consistency)
Updated: 2026-10-12 04:50
