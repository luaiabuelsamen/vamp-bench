# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: M3 — VAMP builds on aarch64 (+UR5e, UR5e-rail via cricket); shared collision model verified 100% (results/collision_model_agreement.json); full run in progress under GPU contention
Last result: VAMP Panda MBM median ~1.8 ms wall vs cuRobo ~0.4–2 s on same Orin NX (smoke); speedup audit docs/SPEEDUP_CLAIMS.md (62x/50x/12x unsupported)
Blocked: clean timing needs an idle GPU; flagship ACT chain (~7 h from ~23:00) holds GR3D at 99%. Source of 62x/50x/12x not on jetson3 (Overleaf disconnected) — Luai to point to it
Next: finish contended run -> analyze -> README/EMAIL_NOTES draft; rerun clean (bench/run_all.sh) when GPU idle and replace numbers
Updated: 2026-10-07 00:05
