# Benchmark v2, step 4: pick-place sequence with MoveL segments (pre-registration)

**Status:** pre-registered and committed before any run. Freeze `6aea6eecf76a173d7ebdf548104aaec854b6b6c7`. New seed labels: `pickplace_cells`, `pickplace_seed0`…`pickplace_seed4`.

## Task
**Cells.** 30 cells from `bench/gen_v2.py`'s cell builder, seed `pickplace_cells`. They use the same families and geometry ranges as the held-out set.

**Picks.** Each cell has 4 picks. Each pick goes from a bin, conveyor or fixture station (tool pointing down) to another down-pointing station, with these segments:
1. **free:** home (or the previous retract) → pre-grasp, 0.10 m above the grasp along −tool z
2. **MoveL:** pre-grasp → grasp, 0.10 m straight down, TCP speed ≤ 0.25 m/s
3. **MoveL:** grasp → post-grasp, 0.10 m straight up
4. **free:** post-grasp → pre-place
5. **MoveL:** pre-place → place, down
6. **MoveL:** place → post-place, up

Home is a fixed configuration per cell.
- **Gripper:** grasp and release dwell is 0 s; no gripper is modelled.
- **Shelf stations** are excluded, because their approach is horizontal and their windows would test reachability rather than motion.

## How each planner expresses each segment

| Segment | VAMP | cuRobo v1 | cuRoboV2 |
|---|---|---|---|
| free | joint-space plan (as in the main v2 run) + P1 | `plan_single_js` | `plan_cspace` |
| MoveL | **post-processing:** TCP line sampled every 5 mm → IK per sample, seeded from the previous one (reject if any joint step > 0.05 rad) → VAMP `validate_motion` per step → time-scaling: 1-D Ruckig on the path parameter, with v = 0.25 m/s and a, j from TCP limits 1.0 m/s², 10 m/s³ | **native soft cost:** `plan_single` to the goal pose with `PoseCostMetric(hold_partial_pose=True, hold_vec_weight=[1,1,1,1,1,0])`, i.e. hold orientation and the two lateral axes in the goal frame | **native soft cost:** `plan_pose` with `ToolPoseCriteria.linear_motion(axis="z")` |
| time-parameterization of MoveL | as above. If joint limits are violated at 1 kHz, retry with v halved (up to 3 times); the retry count is recorded. | as returned | as returned |

## Metrics
- **Per pick:**
  - success: all 6 segments solved and valid under the same 1 kHz gate as v2;
  - T_cycle: the sum over segments of plan + parameterization + execution time.
- **MoveL segments:**
  - maximum TCP deviation from the ideal line (mm, 1 kHz, VAMP FK);
  - maximum orientation deviation (deg);
  - TCP speed during the cruise phase (the middle 60% of the segment's duration): mean and max |v − v̄| / v̄ in %.
- **Seeds:** 5 per pick (`pickplace_seed0`…`seed4`).

## How results will be read
- **P1, MoveL precision:** a planner "expresses MoveL" on this task if its median maximum TCP deviation is ≤ 1 mm and its 95th percentile is ≤ 5 mm.
  - VAMP's post-processed MoveL is a straight line by construction (5 mm samples), so its deviation measures interpolation error only.
  - cuRobo's deviation measures how well a soft cost approximates a line.
- **P2, cycle time:** the planner with the lower median T_cycle per pick is faster for this cell type. It is reported together with the split between planning and execution time.
  - If execution dominates, as expected at a 0.25 m/s TCP speed, a planning-speed advantage will matter little. That is stated as such.
- **P3, robustness:** the share of picks that fail on some seed, per planner.

## Cost
- **CPU, VAMP:** 30 × 4 × 5 = 600 pick sequences; estimated < 30 min.
- **GPU:** cuRobo v1 600 sequences × 6 calls × ~0.5–1 s ≈ 0.5–1 h. cuRoboV2: the same count at ~1–7 s per call ≈ 1–7 h. V2 is run only if the GPU slot allows; otherwise it is reported as not run.

## Execution notes (logged 2026-10-10, before any run)
- **Frame:** every planner targets `tool0`; VAMP's `eefk` equals the URDF's tool0 to 1e-5 (checked on random configurations).
- **Segment chaining:**
  - cuRobo segments start from the last sample of the previous segment's own trajectory.
  - VAMP's MoveL-up segment is the reversed MoveL-down line, so it ends at the shared pre-grasp/pre-place configuration.
  - A failed segment ends that pick; its remaining segments are recorded as not attempted. The next pick starts from that pick's pre-place configuration, so one failure does not cascade.
- **Budget:** T = 30 s applies per planner call (segment). cuRoboV2's attempt loop is unrolled with a clock check before each attempt, as in the main run.
- **Timing scope:**
  - The per-cell world update (cuRobo) and environment construction (VAMP) are not counted in T_cycle; cuRobo's world update is recorded separately.
  - VAMP's MoveL time includes line IK, spline, Ruckig and the 1 kHz limit check that drives the speed-halving retry. That check runs the full validation, collision included, so it slightly overstates VAMP's MoveL time.
- **Speed:** the TCP cruise speed is computed from FK of the 1 kHz trajectory (5 ms finite differences) for every planner.
  - With v = 0.25 m/s, a = 1 m/s² and j = 10 m/s³ over 0.10 m, VAMP's 1-D profile barely reaches cruise, so the "middle 60%" includes ramps.
  - The speed deviation is reported as defined, with this caveat.
- **Order in the GPU slot:**
  1. pass 2a repeat (quiet CPU);
  2. VAMP pick-place, timed (quiet CPU);
  3. cuRobo v1 pick-place;
  4. cuRoboV2 pick-place until the slot deadline (seed-major order, so partial V2 results cover all cells for the first seeds and are reported as partial).
- **As run (2026-10-11/12):** the order above was changed; VAMP no longer runs inside the slot.
  - Slot 2026-10-11 22:38 to 2026-10-12 04:40 (6 h cap), A5 cells. Both cuRobo versions ran on a quiet GPU. The A3 reference runner was stopped; other jobs on the CPU kept running.
  - cuRobo v1 pick-place: complete, 600 picks (30 cells x 4 picks x 5 seeds).
  - cuRoboV2 pick-place: partial at the slot deadline, 556 of 600 picks. Seeds 0-3 are complete; seed 4 has 76 of 120 picks. V2 is reported on 556 picks, and on seeds 0-3 where all planners are compared on the same picks.
  - VAMP pick-place (A5 cells) runs on CPU after the slot, from 2026-10-12 04:43, while other jobs share the board (1-min load average about 19 on 8 cores at start). The load is logged every 5 min next to the results (`results/v2/raw/vamp_pickplace.load.log`), and VAMP's timings are reported as contended.

## Amendment A5 (logged 2026-10-11 23:40, before any pick-place result was analysed and before any valid cuRobo pick-place run)
- **Defect:**
  - The cell generator used `gen_v2.pose_err`, whose orientation term ½ Σᵢ xᵢ × rᵢ is zero at a 180° rotation as well as at the target.
  - In the first generation, 430 of 480 endpoint configurations had a tool orientation 180° from the intended pose, about arbitrary axes, so many tools did not point down.
  - The "MoveL" lines also drifted in orientation by up to 21.6° (207 of 240 lines by more than 0.5°).
  - It was found when cuRobo v1 rejected a MoveL query whose start and goal orientations differed by 180°.
- **Fix:**
  - The orientation error becomes the log map, i.e. the rotation vector of R_target·Rᵀ, used for both IK and line IK.
  - Every endpoint is checked: < 0.3° from the intended R at both ends of the line, tool z pointing down (z·ẑ < −0.99), position < 1 mm.
  - Cells are seeded per cell, `seed_from(H, "pickplace_cells_A5_<k>")`; the first 30 valid k in order are kept, so they can be built in parallel.
- **Consequences:**
  - The first-generation VAMP pick-place run (600 picks) is void; VAMP is rerun on the new cells.
  - The 6 cuRobo v1 records produced before the defect was found are discarded.
  - The first-generation file is kept as `results/v2/raw/v2_pickplace.pre_A5.pkl`.
- **Main v2 held-out set:** it used the same IK for its goal configurations.
  - Its comparison is unaffected, since all planners receive the same joint-space start and goal.
  - Any description of its goals as "tool pointing down" must be checked against the actual FK; this is reported in CHANGELOG.

## Amendment A6 (logged 2026-10-12, after the A5 runs and before any pick-place analysis; approved by the orchestrator)
- **Defect:** the A5 cells' IK configurations are not on the 2π-equivalent branch nearest the configuration the robot comes from.
  - 86% of free segments need some arm joint to turn more than 4.5 rad (median 6.7 rad, up to 13).
  - Below 4.5 rad, cuRobo v1 solved 93–100% of free segments and cuRoboV2 100%. Above it, v1 solved 23%, V2 41% and VAMP 43% (partial run).
- **Rule:** the same wrap as `docs/V2_PREREGISTRATION.md` A6, applied along each cell's pick order. Revolute joints with ±2π limits are shifted by multiples of 2π, staying within ±2π:
  - pick i's pre-grasp `q_above` goes to the branch nearest the pick's start (home for pick 0, otherwise pick i−1's wrapped pre-place);
  - the grasp configuration `q` gets the same shift, so the MoveL line is unchanged;
  - the pre-place `q_above` goes to the branch nearest the wrapped pre-grasp, and the place `q` gets the same shift;
  - home and all poses are unchanged.
  - Geometry is identical; `bench/v2_wrap_goals.py` asserts FK and validity for every configuration.
- **Files:**
  - Problems: `problems/v2_pickplace.pkl` becomes the A6 cells; the A5 cells are kept as `problems/v2_pickplace_A5_unwrapped.pkl`.
  - A5 results are kept as `*_pickplace.A5_unwrapped.jsonl` and reported only as a labelled appendix.
  - The partial A5 VAMP rerun (86 picks) was stopped and is void.
- **Reruns:**
  - VAMP on CPU now: contended, with the load log.
  - cuRobo v1, then cuRoboV2, in one overnight GPU slot.
- **Diagnostics in that slot (labelled, not part of the results):**
  - the cuRobo status of each failed free segment;
  - for MoveL segments, the jerk of cuRobo's optimizer-level trajectory next to the 1 kHz interpolated one, to tell a real jerk excess from an interpolation artefact.
  - The pre-registered gate (jerk included) is unchanged. The no-jerk gate stays a labelled post-hoc result.
