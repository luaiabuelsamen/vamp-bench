# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: M6 — GPU slot 1 done (17:42–21:06, GPU_RELEASED touched). Panda final for VAMP/v1/V2; V2 paper scenes final; README/figure/EMAIL_NOTES updated
Last result: one definition of solved (planner-reported) across README/figure/EMAIL_NOTES; bench/check_consistency.py passes (every count/time traces to results/summary.json). Panda: VAMP 699/699 1.76 ms, v1 638/699 849 ms, V2 59/69 6.55 s (max_attempts 20)
Blocked: waiting for GPU slot 2 (flagship pauses after its current run). bench/gpu_slot2.sh armed: verification of the V2 collision finding first, then v1 paper scenes, UR5 v1+V2; GPU_RELEASED touched on exit only if the slot started. (A spurious GPU_RELEASED at 21:26 from killing the old watcher was removed at 21:26.) V2 collision finding is PROVISIONAL and kept out of EMAIL_NOTES until verified
Next: after rerun -> fill UR5 rows + v1 paper rows in README/EMAIL_NOTES; final commit
Updated: 2026-10-07 22:00
