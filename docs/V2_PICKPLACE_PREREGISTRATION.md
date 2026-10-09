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
