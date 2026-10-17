# Changelog

## 2026-10-17 (licence; internal notes removed)
- **Licence:** MIT (`LICENSE`) for this repository's own code and documents. Third-party material keeps its own licence (`THIRD_PARTY.md`).
- **`docs/EMAIL_NOTES.md` removed** from the repository. It held drafting notes for correspondence and is now kept outside it. It is still in the history. `bench/check_consistency.py` checks README.md by default, plus any Markdown files given as arguments.
- **`STATUS.md`** no longer describes the machine's operating setup.
- **Report date** is now fixed at the date of the report's content (2026-10-15) instead of the last commit's date, which made the report fail the consistency check after any later commit. The PDF was rebuilt, and its text is unchanged.

## 2026-10-17 (public release preparation)
- **No home-directory paths in the repository.** Robot configs now give `urdf_path` and `asset_root_path` relative to the repository root. `bench/robot_cfg.py` makes them absolute at load time, and every script that loads a robot config uses it. The loaded configs are identical to before for all 12 files. The config writers (`bench/v2_curobo_configs.py`, `bench/curobo_to_spherized.py`) write relative paths. In committed logs and result files, the home-directory prefix is replaced by `~`. This affects only path strings, such as process lists and tracebacks, never a number. `bench/powerlog.py` and `bench/v2_vamp_pass2a.py` now record process lists that way. Two watchdogs match the PyRoki process by `$PWD` instead of a fixed path. Older commits still contain the original paths. History is not rewritten, because the freeze commit `6aea6eec` is cited by hash and seeds the runs.
- **Licences of third-party material.** Added `THIRD_PARTY.md` and `licenses/` (cuRobo's NVIDIA License, VAMP's Apache 2.0, MIT for robometrics and Cricket). The cuRobo-derived UR5e configs carry NVIDIA's notice as a header comment. Comments only, so the loaded configs are unchanged.

## 2026-10-15 (technical report)
- **Technical report PDF:** `report/vamp_bench_report.pdf`, built from `report/vamp_bench_report.tex`. No new runs.
  - Every number comes from the committed result files through `report/make_numbers.py`, which writes `report/generated/`.
  - `bench/check_consistency.py` now also checks the report: the generated files must match the result files, the source must contain no typed numbers, and every number in the PDF must come from the generated files or the document structure.
- **v2.1 summary:** `bench/v2_1_summary.py` writes `results/v2_1/summary.json` from the stored v2.1 runs. It covers MoveL as pre-registered and with PyRoki's endpoint recipe, IK, trajectory optimisation S1/S2 and the margin sensitivity, and is provisional until the v2.1 write-up.
- **v2.2 summary additions:** `results/v2_2/summary.json` gains the generator's acceptance and set shape (`program_set`) and the cuRobo line classes (`expressibility_lines`), so the report reads them from JSON.

## 2026-10-15
- **Benchmark v2.2 (industrial motion types) results.** `docs/V2_2_RESULTS.md`, `results/v2_2/summary.json` and `results/v2_2/motion_types.png`; README and `docs/EMAIL_NOTES.md` sections added. `bench/check_consistency.py` now also checks them against `results/v2_2/summary.json`.
- **Pre-registration text, jerk-limited corners.** `docs/V2_2_MOTION_TYPES_PREREGISTRATION.md` gives the corner durations as 25 ms for W and C and 100 ms for T. The time law (a / j with j = 10 m/s³) gives the reverse: 100 ms for W and C (a = 1 m/s²) and 25 ms for T (a = 0.25 m/s²). The code used a / j throughout, so no result is affected.
- **Analysis additions, after the run:**
  - `bench/v2_2_analyze.py` now records failure reasons, success counts and the Cartesian pipeline's speed halvings.
  - It also reports the pre-registered bins. Path length is split at each type's median, since the pre-registration set no bin edges.

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
