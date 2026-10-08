# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: M7 — GPU slot 2 running since 22:24 (orchestrator go): v1 paper scenes, then UR5 v1+V2; V2 collision check done (docs/V2_COLLISION_CHECK.md)
Last result: V2 between-waypoint finding: not a model artifact (V2 own checker confirms 24/24, agrees 12,000/12,000 random configs), but penetration only median 1.0 mm / max 3.5 mm on waypoint chords; continuous-spline check inconclusive -> stays provisional, out of EMAIL_NOTES facts
Blocked: none while slot 2 runs (~1.5–2 h); GPU_RELEASED is touched when it exits. 62x/50x/12x source: not on jetson3
Next: slot 2 done -> fill v1 paper + UR5 cells, run check_consistency, final commit
Updated: 2026-10-07 22:25
