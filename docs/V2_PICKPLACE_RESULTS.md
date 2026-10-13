# Benchmark v2: pick-place with MoveL (results)

Pre-registration and amendments: `docs/V2_PICKPLACE_PREREGISTRATION.md`. Numbers: `results/v2/pickplace_summary.json`, produced by `bench/v2_pickplace_analyze.py` from `results/v2/raw/`.

## Setup
- **Task:**
  - 30 UR5e-on-rail cells (bin, conveyor and fixture stations), 4 picks per cell, 5 seeds: 600 picks per planner.
  - Each pick has 6 segments: free → pre-grasp, MoveL down 0.10 m, MoveL up, free → pre-place, MoveL down, MoveL up.
  - Goals are on the 2π branch nearest the configuration the robot comes from (A6).
- **Success:** every segment passes the same gate for every planner:
  - planning within T = 30 s;
  - at 1 kHz: no collision, joint bounds respected, and peak velocity, acceleration and jerk each ≤ 1.01 × limit.
- **Jerk limit:** 500 rad/s³ per arm joint. This is the value in cuRobo's shipped `ur5e.yml`; Universal Robots publishes no jerk limit for the UR5e.
- **Hardware:** one Jetson Orin NX 16 GB.
  - cuRobo v1 and cuRoboV2 ran on its GPU, with no other GPU work.
  - VAMP ran on its CPU while other jobs shared the board (1-min load average 13.6–15.6 on 8 cores), so VAMP's planning times are upper bounds.
- **MoveL per planner:**
  - **VAMP:** a straight TCP line by construction (IK every 5 mm, then time-scaled at 0.25 m/s TCP).
  - **cuRobo v1 and cuRoboV2:** their native soft line costs, with trajectories used as returned.

## Results

| | VAMP | cuRobo v1 (as configured) | cuRobo v1, max_jerk 200 (positive control) | cuRoboV2 |
|---|---|---|---|---|
| Picks completed | 587/600 | 0/600 | 572/600 | 585/600 |
| Pick slots failing on some seed (P3) | 8/120 | 120/120 | 11/120 | 3/120 |
| Pick slots failing on every seed | 0 | 120 | 3 | 3 |
| Median cycle time per completed pick | 6.08 s | — | 8.22 s | 18.92 s |
| … planning | 0.35 s | — | 2.76 s | 13.71 s |
| … execution | 5.72 s | — | 5.47 s | 5.34 s |
| MoveL max TCP deviation, median / p95 (P1) | 0.04 / 0.07 mm | 0.04 / 0.49 mm | 0.03 / 0.34 mm | 0.65 / 0.92 mm |
| MoveL max orientation deviation, p95 | 0.03° | 1.89° | 1.51° | 0.05° |
| First failure on failed picks | 12 free segments collide at 1 kHz; 1 free segment not solved | 575 jerk only; 25 free segments not planned | 27 free segments not planned; 1 jerk only | 15 free segments not planned |

- **P1, MoveL precision:** all three planners meet the pre-registered criterion (median ≤ 1 mm, p95 ≤ 5 mm).
  - VAMP's line is straight by construction, so its deviation is interpolation error only.
  - cuRobo v1 holds the line in position but has the largest orientation drift (p95 1.9°).
  - cuRoboV2 deviates about 0.65 mm in position and holds orientation within 0.05°.
- **P2, cycle time:** VAMP has the lowest median cycle time per completed pick.
  - **Paired picks:** on the 576 picks both complete, VAMP takes 6.07 s and cuRoboV2 18.88 s, a median ratio of 0.32. On the 566 picks VAMP and the v1 positive control both complete, VAMP takes 6.06 s and v1 8.22 s (ratio 0.74).
  - **Where the difference comes from:** execution times are within 0.4 s of each other, so the difference is planning time.
  - **v1 as configured:** it completes no pick, so it has no cycle time; the v1 cycle times are from the positive control.
- **P3, robustness:** cuRoboV2 has the fewest pick slots failing on some seed (3/120), VAMP 8/120, and the v1 positive control 11/120.
  - V2's 3 failing slots fail on every seed, all in free segments.
  - VAMP fails no slot on every seed. Its failures are mostly free segments whose time-parameterized trajectory touches an obstacle at 1 kHz, although the path itself is collision-free.

## The jerk gate

![MoveL executed jerk per planner](../results/v2/pickplace_jerk.png)

| Run (A6 cells) | MoveL segments within 500 rad/s³ | MoveL peak, median / max (× limit) | Free segments within |
|---|---|---|---|
| VAMP | 2360/2360 | 0.11 / 0.52 | 1192/1192 |
| cuRobo v1 as configured (max_jerk 500) | 67/632 | 2.00 / 2.33 | 582/592 |
| cuRobo v1, max_jerk 250 | 74/640 | 1.10 / 1.79 | 588/598 |
| cuRobo v1, max_jerk 200 | 2302/2303 | 0.89 / 1.22 | 1152/1152 |
| cuRoboV2 | 2350/2350 | 0.93 / 1.00 | 1175/1175 |

- **Result:** cuRobo v1's MoveL trajectories exceed the configured 500 rad/s³ jerk limit, executed at 1 kHz, by a median factor of 2.0. cuRoboV2's and VAMP's stay within it.
- **Where the excess is:** in one cell's dumped trajectories (8 MoveL segments, max_jerk 500 and 250), every over-limit sample lies in the first or last 30 ms of the segment; between the ends, jerk stays at or below 0.84 × 500.
  - The peak at the ends is 1.9–2.3 × v1's own `max_jerk`, which is why halving that setting halves the excess rather than removing it.
  - At the start, this matches v1's kernel: its jerk term is a central difference over optimizer steps, set to zero at the first step. The mechanism at the end is not traced.
- **Positive control:** with `max_jerk` = 200, v1 meets the gate on 2302 of 2303 MoveL segments and completes 572/600 picks.
  - This meets the pre-registered criterion (≥ 80%).
  - v1 therefore plans these cells; as configured, it fails them on jerk (at the segment ends, in the cell examined).
  - Its remaining failures are 27 free segments it does not plan, and one MoveL segment over the jerk limit.
- **Post hoc (not a success measure):**
  - **v1 without the jerk check:** if the jerk check alone were ignored, v1 as configured would complete 573/600 picks.
  - **TCP speed:** the task specifies 0.25 m/s TCP speed on MoveL. VAMP's peak stays at or below it. cuRobo has no TCP speed limit, and its MoveL peaks exceed 0.25 m/s on 85% (v1) and 100% (V2) of segments (p95 peak 0.47 m/s for v1, 0.42 m/s for V2), which shortens cuRobo's execution times.

## Appendix: goals not wrapped to the nearest branch (A5 cells)
- **The A5 cells:** the same cells before A6, where goals were not wrapped. 81% of free segments need some arm joint to turn more than 4.5 rad.
- **cuRobo results on them:** v1 completed 0/600 picks and cuRoboV2 120/556 (a partial run).
- **VAMP:** the A5 VAMP run is void.
- **Comparison with the main results:** on long joint moves, both cuRobo versions fail many free segments that they plan once the goals are wrapped.
