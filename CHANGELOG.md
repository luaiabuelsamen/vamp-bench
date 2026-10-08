# Changelog

## 2026-10-08
- **Segment check: joint-limit tolerance.** `bench/crossval.py` flagged 39 cuRobo v1 UR5 trajectories. 35 of these came from float32 rounding at the ±π joint limit: each was 2.4e-7 rad outside VAMP's bounds, and cuRobo v1's own checker found no world or self collision on them (`results/selfcollision_depth_v1_ur5.json`). The check now allows 1e-5 rad of rounding and records larger violations. The v1 UR5 flagged count is 4 of 367. No other set changed.
- **Added `docs/PAPER_CLAIMS_AUDIT.md`.**

## 2026-10-07
- **One definition of "solved".** "Solved" is now the planner-reported success everywhere. An earlier draft reported cuRoboV2 Panda as 55/69, which counted only successes that passed VAMP's segment check (59 − 4).
- **UR5 velocity limit.** The cuRobo UR5 runs were redone with the real UR5 velocity limit (π rad/s). VAMP's URDF caps joints at 0.5 rad/s. The earlier runs are in `results/raw/superseded/`.
- **v1 paper scenes rerun.** The first run's results file was never written because of a path bug in `bench/curobo_mbm.py`.
- **VAMP timing fix.** The per-trial sampler skip was moved outside the timed region; it had added about 24 ms per trial to the wall time.
