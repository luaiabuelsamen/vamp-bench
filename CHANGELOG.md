# Changelog

## 2026-10-14
- **Benchmark v2 final.** All feasibility reference runs are complete:
  - held-out A6: 299 of 300 queries certified, 1 with no path found;
  - MπNets: 299 and 1;
  - held-out with original goals: 156 and 144.
  - Hypotheses H1–H5 are unchanged from the partial analysis.
  - The README and `docs/EMAIL_NOTES.md` now carry the v2 results. `bench/check_consistency.py` checks their v2 sections against the v2 result files.
- **Held-out set description.** `docs/V2_RESULTS.md` and the README now describe the held-out queries as station-to-station configuration pairs, not tool-down picks (see 2026-10-11).
- **PyRoki MoveL (v2.1, not yet published).** The pre-registered PyRoki MoveL run succeeded on 6 of 480 segments. That figure reflects our harness, not PyRoki: our endpoint handling differed from PyRoki's own trajopt recipe. With PyRoki's recipe, 373 of 480 succeed under the same rules (amendment 7g, `docs/BENCHMARK_V2.md`). The v2.1 write-up reports 373 first and the 6 as the harness error.

## 2026-10-11
- **Held-out and pick-place IK orientations.** The station IK in `bench/gen_v2.py` used an orientation error that is zero at a 180° rotation as well as at the target.
  - **Held-out set:** in the v2 held-out set, 498 of 600 start/goal configurations do not have the intended tool orientation (tool down at bin, conveyor and fixture stations; horizontal at shelves). Positions are as intended. The held-out comparison stands, because every planner receives the same collision-free joint-space start and goal. But these queries are station-to-station configuration pairs, not tool-down picks, and they are described that way.
  - **Pick-place cells:** they were regenerated with a log-map orientation error and per-endpoint checks (amendment A5 in `docs/V2_PICKPLACE_PREREGISTRATION.md`). The first pick-place generation and its VAMP run are void.

## 2026-10-08
- **Segment check: joint-limit tolerance.** `bench/crossval.py` flagged 39 cuRobo v1 UR5 trajectories. 35 of these came from float32 rounding at the ±π joint limit: each was 2.4e-7 rad outside VAMP's bounds, and cuRobo v1's own checker found no world or self collision on them (`results/selfcollision_depth_v1_ur5.json`). The check now allows 1e-5 rad of rounding and records larger violations. The v1 UR5 flagged count is 4 of 367. No other set changed.
- **Added `docs/PAPER_CLAIMS_AUDIT.md`.**

## 2026-10-07
- **One definition of "solved".** "Solved" is now the planner-reported success everywhere. An earlier draft reported cuRoboV2 Panda as 55/69, which counted only successes that passed VAMP's segment check (59 − 4).
- **UR5 velocity limit.** The cuRobo UR5 runs were redone with the real UR5 velocity limit (π rad/s). VAMP's URDF caps joints at 0.5 rad/s. The earlier runs are in `results/raw/superseded/`.
- **v1 paper scenes rerun.** The first run's results file was never written because of a path bug in `bench/curobo_mbm.py`.
- **VAMP timing fix.** The per-trial sampler skip was moved outside the timed region; it had added about 24 ms per trial to the wall time.
