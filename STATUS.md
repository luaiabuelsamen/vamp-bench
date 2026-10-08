# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: M6 — GPU slot 1 done (17:42–21:06, GPU_RELEASED touched). Panda final for VAMP/v1/V2; V2 paper scenes final; README/figure/EMAIL_NOTES updated
Last result: Panda MBM, this Orin: VAMP 1.76 ms (699/699), cuRobo v1 849 ms (638/699, 483x), cuRoboV2 6.55 s (55/69 subset); V2 4–16% of reported successes collide between waypoints (results/summary.json)
Blocked: waiting for GPU slot 2 (flagship pauses after its current run). bench/gpu_slot2.sh armed: verification of the V2 collision finding first, then v1 paper scenes, UR5 v1+V2; GPU_RELEASED touched on exit only if the slot started. (A spurious GPU_RELEASED at 21:26 from killing the old watcher was removed at 21:26.) V2 collision finding is PROVISIONAL and kept out of EMAIL_NOTES until verified
Next: after rerun -> fill UR5 rows + v1 paper rows in README/EMAIL_NOTES; final commit
Updated: 2026-10-07 21:27
