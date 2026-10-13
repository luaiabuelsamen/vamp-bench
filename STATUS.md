# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-14 — pick-place analysis (docs/V2_PICKPLACE_RESULTS.md, results/v2/pickplace_jerk.png) and held-out A6 primary results (docs/V2_RESULTS.md, results/v2/tables.md, per_bin_*.png) committed 2026-10-13. GPU: none needed (flagship has it)
Last result Pick-place: VAMP 587/600, cuRoboV2 585/600, v1 as configured 0/600 (MoveL jerk 2.0x the 500 rad/s^3 ur5e.yml limit at segment ends), v1 positive control max_jerk 200: 572/600. Held-out A6: success VAMP 93.1%, v1 56.9% (V2 71.0% on subsample); H1-H3 hold on all three sets; H4 differs by set; on runs both solve, v1's cycle is shorter on held-out A6 (2.36 vs 2.67 s)
Blocked: nothing. CPU: flagship (EC2C) has priority — feasibility runner at 1 worker; no new CPU batch until MemAvailable > 4 GB. Held-out A6 + MpiNets references (3 queries) queued after the current runner exits
  Memory (standing rule 2026-10-11): during a flagship SmolVLA eval (train_local.py --skip-train ... smolvla) the A3 runner runs at most 1 worker and starts nothing unless MemAvailable > 2.5 GB (built into bench/v2_feasibility_par.py); otherwise 2 workers, >2 only after 10 min sustained > 4 GB. PRM references grow to ~1.5 GB, so under the eval most are floor-killed and requeued (12 so far): the A3 runs effectively wait for the eval to end.
  CPU: A3 references for the original held-out (appendix) running, 2 workers, memory-gated, ~90 queries left. heldout_a6: 3 queries VAMP never solved; references after the cuRobo held-out A6 runs.
Next: references finish -> rerun bench/v2_analyze.py + v2_report.py -> README / EMAIL_NOTES v2, held-out description fix (tool-down claim), check_consistency, CHANGELOG -> v2 final (no email before)
Updated: 2026-10-13 11:14
