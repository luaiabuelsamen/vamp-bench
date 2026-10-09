# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-4 — cuRobo v1 full run DONE in tonight's slot (22:32-05:27, GPU_RELEASED 05:26): planner success held-out 521/3000, MBM 2701/3000, MpiNets 2927/3000; 0 gate rejections. VAMP CPU step resumed. V2 + pick-place tomorrow night
Last result: TOPP-RA/Ruckig cost measured before freeze: Ruckig P1 0.36 ms p50, TOPP-RA P2 17.6 ms p50 per path (results/v2/timing_cost.json)
Blocked: VAMP CPU step throughput + plan approval (A4) needed.
  Profile (results/v2/profile.json): 98.1% of CPU time is FAILED planning (54.7 s each: VAMP runs to its 1M-iteration default; results censored at T=30 s); successful plans 8 ms; 1 kHz validation 1.6%; TOPP-RA 0.1%. CPU is also saturated by other jobs (~7/8 cores), so current VAMP timings are contended.
  Option 0 (keep current run): ~13.5 h more (held-out 151/300 at 09:32, then Panda sets), timings contended -> successes need re-timing anyway.
  Option A4 (proposed, two-pass): pass 1 parallel classification with a 60 s kill per run, pass 2a sequential re-timing of successes on a quiet machine, pass 2b parallel validation.
     ~5 h wall with 6 free cores (pass 1 ~4.3 h, 2a <=0.5 h quiet, 2b ~0.2 h); only feasible once the other CPU jobs end (with ~1 free core: ~28 h).
     Pass 2a needs a quiet CPU window of ~30 min (e.g. start of tonight's GPU slot, before V2 starts).
  A3 (logged): long-budget reference runs (VAMP rrtc 300 s + PRM 300 s, untimed) for queries no timed run solved: est. ~100-130 held-out queries x <=600 s = ~20 core-h -> ~3.5 h on 6 cores. Sample of 15 running now.
  Mac (not started; needs approval): only UNTIMED work can move (A3 reference runs, pass 2b validation). Timed VAMP planning must stay on this Orin (same-machine rule).
Next: VAMP CPU step on held-out/MBM/MpiNets × 10 seeds -> exact GPU-hours under Blocked -> GPU slot (cuRobo v1 full, V2 subsample) -> pick-place MoveL sequence
Updated: 2026-10-09 09:32
