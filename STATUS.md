# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: M8 — all cells filled (VAMP, cuRobo v1, cuRoboV2 × 5 problem sets); README/figure/EMAIL_NOTES consistent (bench/check_consistency.py passes)
Last result: same Orin NX — VAMP 0.55–2.02 ms median; cuRobo v1 145–849 ms (169–483x VAMP); cuRoboV2 0.38–6.55 s; between-waypoint mm-scale penetrations in BOTH cuRobo versions (provisional, out of email facts) (results/summary.json)
Blocked: none. Optional: ~5 min GPU to measure self-collision depth of 35 v1 UR5 flags; 62x/50x/12x source not on jetson3 (Luai)
Next: orchestrator/Luai review of EMAIL_NOTES; nothing pushed
Updated: 2026-10-08 00:39
