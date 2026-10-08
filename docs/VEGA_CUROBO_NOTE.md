# vega_curobo vs this benchmark's cuRoboV2 config (same Orin NX, same v2 build)

Our cuRoboV2 numbers on this board are not fast either: medians are 0.38–1.06 s per query on the paper scenes and 6.55 s on Panda MBM (`results/summary.json`). That is the same order of magnitude as vega_curobo's ~12 s, so neither config is the outlier.

What vega_curobo's config does differently (`~/projects/vega_curobo/vega_curobo/solvers.py`):
- **Optimizer settings.** It uses `MotionPlannerCfg.create` defaults: L-BFGS-only IK and L-BFGS B-spline trajopt, collision activation 0.01. We use NVIDIA's `benchmark/motion_plan_benchmark.py` settings: particle + L-BFGS for both, activation 0.0025.
- **Goal type.** It calls `plan_pose`, which includes IK. We call `plan_cspace`, which takes a joint goal and has no IK.
- **Robot.** It plans for a 10-active-DoF Vega arm with 28 collision links. We plan for 6–7 DoF arms.

**Evidence from this board.** A 7-query Panda smoke test ran both settings on the same v2 build, and per-plan time was about the same: 5–24 s for each (scratchpad runs, 2026-10-07). The optimizer settings are therefore not what makes vega_curobo take ~12 s.
- **Caveat:** that smoke test ran while the flagship's training job was also on the GPU, so it is not a clean timing.
