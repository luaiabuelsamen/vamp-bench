# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-6 — pass (a) DONE (held-out 1540/3000, MBM 3000/3000, MpiNets 2990/3000 VAMP successes); A3 reference runs started (145 held-out queries unresolved by any timed run; 2 workers, ~27 h, ~9 h on 6); night-2 slot armed: pass 2a re-timing then cuRoboV2 subsample
Last result: TOPP-RA/Ruckig cost measured before freeze: Ruckig P1 0.36 ms p50, TOPP-RA P2 17.6 ms p50 per path (results/v2/timing_cost.json)
Blocked: (1) flagship to signal end of the 256 px re-render -> raise pass (a) to 6 workers (restart resumes from checkpoint).
  (2) A3 reference runs: Orin only (Luai declined the Mac); start after pass (a) on free cores (bench/v2_feasibility_par.py). Cost depends on how many queries no timed run solved; ~20-25 min per query on the contended CPU.
  (3) Tonight's slot armed (bench/gpu_slot_v2_night2.sh): pass 2a (~15 min, quiet CPU, my reference runs suspended) then cuRoboV2 subsample (3.0 h expected, 4.2 pessimistic, 7.65 ceiling). Pick-place NOT ready tonight (implementation pending) -> needs a later slot (~1 GPU-h v1, 1-3 h V2).
Next: VAMP CPU step on held-out/MBM/MpiNets × 10 seeds -> exact GPU-hours under Blocked -> GPU slot (cuRobo v1 full, V2 subsample) -> pick-place MoveL sequence
Updated: 2026-10-09 20:18
