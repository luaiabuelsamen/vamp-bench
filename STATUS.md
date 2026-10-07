# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: M5 — adding cuRoboV2 (existing build: ~/projects/clean_env/venv, editable -> curobo_berkeley/curobo ac5f931) alongside v1 and VAMP
Last result: cuRoboV2 smoke on this Orin (GPU shared with flagship): 5–24 s per solved plan with NVIDIA benchmark preset and with library defaults alike (cf. vega_curobo ~12 s); v2 build reused from ~/projects/clean_env/venv, not rebuilt
Blocked: GPU SLOT ≈ 3.5–4 h total: cuRobo v1 full (~1.5 h) + cuRoboV2 subset under hard budgets (≤ 2 h: MBM every 10th problem, paper scenes 1 trial, max_attempts 20) + crossval/analysis. Starts after flagship pause (idle 10 min, or forced at pause+30 min); ~/projects/_ops/GPU_RELEASED touched on exit regardless. Source of 62x/50x/12x not on jetson3 — Luai to confirm
Next: slot run (bench/gpu_slot_run.sh -> run_all --curobo-only) -> fill README/EMAIL_NOTES v1+v2 cells; docs line on vega_curobo 12 s vs v2 ms-scale if applicable
Updated: 2026-10-07 16:06
