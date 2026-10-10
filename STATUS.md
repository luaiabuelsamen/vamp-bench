# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-7 — night-2 slot running (opened 01:34): pass 2a DONE (7530/7530 identical paths; 0/20 killed runs solve within T alone; CPU NOT idle at slot start: flagship workers ~4 cores); V2 held-out crashed on a stale config key (fixed: USD keys dropped, no geometry/limit change) and is chained after mpinets in the same slot; V2 mbm running
Last result: TOPP-RA/Ruckig cost measured before freeze: Ruckig P1 0.36 ms p50, TOPP-RA P2 17.6 ms p50 per path (results/v2/timing_cost.json)
Blocked: (1) flagship to signal end of the 256 px re-render -> raise pass (a) to 6 workers (restart resumes from checkpoint).
  (2) A3 reference runs: 8/145 held-out done (0 paths found); runner died when the slot suspended it (checkpoint intact) -> restart after the slot.
  (3) Tonight's slot armed (bench/gpu_slot_v2_night2.sh): pass 2a (~15 min, quiet CPU, my reference runs suspended) then cuRoboV2 subsample (3.0 h expected, 4.2 pessimistic, 7.65 ceiling). Pick-place NOT ready tonight (implementation pending) -> needs a later slot (~1 GPU-h v1, 1-3 h V2).
Next: VAMP CPU step on held-out/MBM/MpiNets × 10 seeds -> exact GPU-hours under Blocked -> GPU slot (cuRobo v1 full, V2 subsample) -> pick-place MoveL sequence
Updated: 2026-10-10 01:51
