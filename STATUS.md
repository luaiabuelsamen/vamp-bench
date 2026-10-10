# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-9 — main v2 runs done; preliminary analysis committed (results/v2/summary.json). Now: A3 reference runs (memory-gated) + pick-place prepared (cells generating, VAMP runner tested, cuRobo runner written); night-3 slot watcher armed
Last result: preliminary v2: success VAMP/v1/V2 held-out 49/17/37%, MBM 81/90/83%, MpiNets 88/98/97%; VAMP T_ready 2-5 ms vs v1 0.24-1.2 s vs V2 2-10 s; every VAMP gate rejection is a collision between its discrete motion checks (<=3 mm sampled)
Blocked: night-3 GPU slot (bench/gpu_slot_v2_night3.sh, watcher armed; gate: no python train_local, GPU idle + MemAvailable >= 4 GB for 10 min; GPU_RELEASED on exit, deadline Sun 2026-10-11 07:50): pass 2a quiet repeat ~15 min, VAMP pick-place (CPU) ~0.5-1.5 h, cuRobo v1 pick-place ~0.5-1 GPU-h, cuRoboV2 pick-place 1-7 GPU-h (partial if the slot ends).
  Memory (2026-10-10 09:45): A3 runner capped at 2 workers (--max-workers 2) until flagship finishes the SmolVLA compliant eval (~3 h); gate uses /proc/meminfo MemAvailable; above 2 workers only after 10 min sustained > 4 GB, shed below 1.5 GB; workers oom_score_adj 1000.
  CPU: A3 references, 133 held-out + 1 MpiNets queries, ~20-60 min each per worker (PRM at 6e6 samples): ~1-2 days at 2 workers, faster at 6.
Next: slot -> re-merge pass 2b with quiet timings -> pick-place analysis -> final v2 analysis when A3 finishes (per-bin tables, curves, H1-H5 write-up, README/EMAIL_NOTES v2, check_consistency)
Updated: 2026-10-10 09:45
