# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: M5 — adding cuRoboV2 (existing build: ~/projects/clean_env/venv, editable -> curobo_berkeley/curobo ac5f931) alongside v1 and VAMP
Last result: VAMP medians 1.76 ms Panda MBM (699/699), 2.02 ms UR5 MBM (689/689), 0.55–0.97 ms paper scenes (results/summary.json)
Blocked: GPU SLOT IS ~2 h LONGER than first planned (now ~4–5 h: cuRobo v1 then cuRoboV2 on all 5 sets, + crossval + analysis). Starts after flagship pause (idle 10 min, or forced at pause+30 min); ~/projects/_ops/GPU_RELEASED is touched on exit, whatever happens. Source of 62x/50x/12x not on jetson3 — Luai to confirm
Next: slot run (bench/gpu_slot_run.sh -> run_all --curobo-only) -> fill README/EMAIL_NOTES v1+v2 cells; docs line on vega_curobo 12 s vs v2 ms-scale if applicable
Updated: 2026-10-07 15:39
