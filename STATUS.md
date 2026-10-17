# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks.
Finish line: results/*.json + one figure + README headline numbers + speedup-number (62x/50x/12x) reconciliation, all committed.
Deadline: none set
Milestone: V2-21 — technical report PDF report/vamp_bench_report.pdf (13 pages; LaTeX in report/, every number generated from the result JSONs by report/make_numbers.py; check_consistency covers the PDF). v2.2 done (0258f37)
Last result: v2.2 (100 programs per type). E: cuRobo v1/V2 hold no programmed line natively (0 of 583 lines along a holdable axis; 468/483 W legs and 99/100 T lines orientation-only), W stops at every via point, MoveC and the speed profile not expressible, V2 per-timestep targets rejected (P0). Success R1, W/C/T: Cartesian pipeline 100/100/100; cuRobo v1 0/-/29 of 1000, v1 max_jerk 200 10/-/319 of 1000, V2 3/-/120 of 300; PyRoki 1/3/84 of 100 (R2 19/88/99). cuRobo T lines run 4.4-4.9x programmed speed and miss the 5 mm path limit on most runs; PyRoki follows the T trapezoid (dev 0.21 mm, ramp err 6.8%) but W/C exceed jerk as returned and W plans in 41.0 s median (79/100 over 30 s); Cartesian pipeline halves speed on 76/100 W (blend jerk). Program set: 100 of 817 candidates (12%), skewed to 3-via W and sweeps < 180 deg
Blocked: nothing. URSim (controller reference + servoj check) waits on a platform decision: URSim is amd64-only and this board is aarch64.
Next: one-command planner-bench spin-out. Then the v2.1 write-up (leads with the 7g correction: 373/480, not 6; results/v2_1/summary.json now holds its numbers, provisional). URSim when a platform is approved
Updated: 2026-10-17
