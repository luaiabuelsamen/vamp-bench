# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: M4 — VAMP full benchmark done (CPU); README/EMAIL_NOTES/figure with cuRobo cells pending
Last result: VAMP medians 1.76 ms Panda MBM (699/699), 2.02 ms UR5 MBM (689/689), 0.55–0.97 ms paper scenes (results/summary.json)
Blocked: clean cuRobo run waits for idle GPU (flagship has priority; watcher bench/wait_idle_then_curobo.sh starts it after 10 idle min). Source of 62x/50x/12x not on jetson3 — Luai to point to it
Next: on idle GPU, cuRobo run (~2–3 h) -> crossval -> analyze -> fill README/EMAIL_NOTES pending cells
Updated: 2026-10-07 03:48
