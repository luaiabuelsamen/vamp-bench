# Benchmark v2: joint-space planning results

Pre-registration and amendments: `docs/V2_PREREGISTRATION.md`.
- **Numbers:** `results/v2/summary.json` (`bench/v2_analyze.py`) and `results/v2/hypotheses.json` (`bench/v2_report.py`).
- **Full tables:** `results/v2/tables.md`.
- **Per-bin figures:** `results/v2/per_bin_<set>.png`.
- **Pick-place with MoveL:** `docs/V2_PICKPLACE_RESULTS.md`.

## Setup
- **Sets:**
  - **Held-out A6 (primary):** UR5e on a rail, 300 queries in industrial cells generated after a frozen commit. Goals are on the 2π branch nearest the start.
  - **MotionBenchMaker (MBM)** and **MπNets:** Panda, 300 queries each.
  - **Held-out with the original, unwrapped goals:** reported as an appendix.
- **Runs:**
  - VAMP and cuRobo v1: 10 seeds per query.
  - cuRoboV2: a stratified subsample of 100 queries per set, 3 seeds, compared with the others on the same queries and seeds.
- **Success:** a trajectory that passes the 1 kHz gate (no collision; joint bounds; velocity, acceleration and jerk ≤ 1.01 × the shared limits), with planning plus time-parameterization (T_ready) within T = 30 s.
  - T_cycle = T_ready + execution time.
  - Failures count at T = 30 s where a statistic includes them.
- **Hardware:** one Jetson Orin NX 16 GB. VAMP runs on its CPU and cuRobo on its GPU.
  - VAMP's timings come from a separate timing pass (A4). cuRobo ran with no other GPU work.

## Held-out A6 (primary)

| | VAMP | cuRobo v1 | cuRoboV2 (100-query subsample) |
|---|---|---|---|
| Success (runs) | 93.1% | 56.9% | 71.0% (VAMP 95.0%, v1 64.7% on the same runs) |
| Queries failing on some seed / every seed | 100 / 3 | 160 / 95 | 29 of 100 / not computed |
| Median T_ready (solved runs) | 3.5 ms | 210 ms | 1505 ms |
| Median T_exec (solved runs) | 3.03 s | 2.06 s | 2.16 s |
| Median T_cycle (median over queries of per-query medians, failures at 30 s) | 3.09 s | 3.76 s | 4.06 s |

- **Feasibility:** 298 of 300 queries are certified feasible by some planner run; reference runs for the other 2 are pending.
- **cuRobo v1's failures:** 1289 of its 1292 failed runs end in its trajectory optimizer (`TRAJOPT_FAIL`); none fail the 1 kHz gate.

## Hypotheses (pre-registered)

| | Held-out A6 | MBM | MπNets |
|---|---|---|---|
| **H1:** VAMP's T_ready lower than cuRobo v1's in every bin | yes; 95% CIs disjoint in every bin | yes; disjoint | yes; disjoint |
| T_ready ratio, cuRobo v1 / VAMP (medians, solved runs) | 59× | 478× | 159× |
| **H2:** relative T_cycle gap smaller than the T_ready gap | yes (T_cycle ratio 1.22) | yes (1.14) | yes (0.99) |
| Median T_cycle on runs both solve, VAMP / cuRobo v1 | 2.67 / 2.36 s | 2.53 / 3.10 s | 1.65 / 1.81 s |
| **H3:** cuRobo v1 lower jerk and shorter T_exec than VAMP + Ruckig (P1) | yes / yes | yes / yes | yes / yes |
| VAMP + TOPP-RA (P2): T_exec, and share of planned paths valid | 1.79 s, 48% | 1.55 s, 56% | 1.14 s, 71% |
| **H4:** fewer queries failing on some seed | VAMP (100 vs 160) | cuRobo v1 (55 vs 231) | cuRobo v1 (17 vs 128) |

- **H1:**
  - VAMP returns an executable trajectory 59–478× sooner than cuRobo v1 in median, in every bin of every set.
  - These medians are over solved runs. On held-out A6, cuRobo v1 solves 57% of runs and VAMP 93%.
- **H2:**
  - **Cycle-time gap:** relative to its planning-time gap, the cycle-time gap is small on every set, because execution takes seconds.
  - **Held-out A6, runs both planners solve:** cuRobo v1's cycle is shorter (2.36 vs 2.67 s), because its execution is 0.97 s shorter while VAMP plans only 0.21 s faster. By the pre-registered reading, cuRobo v1 wins on cycle time for the runs it solves.
  - **Held-out A6, counting failures:** VAMP's median cycle time is lower (3.09 vs 3.76 s).
  - **MBM and MπNets:** VAMP's cycle is shorter on the runs both solve.
- **H3:**
  - **Against VAMP + Ruckig (P1):** cuRobo v1's trajectories are smoother and shorter to execute on every set. P1 stops at every waypoint.
  - **VAMP + TOPP-RA (P2):** this is faster to execute than cuRobo v1, but only 48–71% of its trajectories pass the 1 kHz check (jerk not checked, since TOPP-RA bounds only velocity and acceleration).
  - **Clearance:** cuRobo v1 keeps more clearance (median minimum 9–16 mm, against 2–3 mm for VAMP).
- **H4:** VAMP is more robust on the held-out set, and cuRobo v1 on MBM and MπNets.
- **H5, generality:** the signs of H1, H2 and H3 are the same on all three sets, so those claims generalize across them. H4's sign differs between the held-out set and the Panda sets, so robustness is reported per set only.
- **cuRoboV2 vs cuRobo v1 (same queries and seeds):**
  - **Planning time:** V2's median T_ready is 1.5 s (held-out A6), 10.2 s (MBM) and 4.1 s (MπNets), 7–15× v1's (213 ms, 1.2 s and 0.28 s).
  - **Success:** V2's success is within 7 points of v1's: higher on held-out A6, lower on MBM and MπNets.

## Appendix: held-out with unwrapped goals (long joint moves)
- **The set:** the same 300 queries with the original goals, which are not on the 2π branch nearest the start. In 95% of queries some arm joint must turn more than 4.5 rad.
- **Comparison with A6:** each query is paired with itself on the A6 set, over the same seeds.

| Planner | Queries | Success, original goals | Success, A6 | Difference (95% CI) |
|---|---|---|---|---|
| VAMP | 300 | 49.3% | 93.1% | +43.8 (+38.2, +49.7) points |
| cuRobo v1 | 300 | 17.4% | 56.9% | +39.6 (+34.2, +45.3) |
| cuRoboV2 | 100 | 37.0% | 71.0% | +34.0 (+24.0, +44.0) |

- **Sensitivity:** every planner is sensitive to long joint moves; success rises from the original goals to A6 beyond the 95% CI.
- **The gap between planners:** the pre-registered rule says the original-set gap is "explained by move length" only if it halves on A6. The two available measures of the gap disagree:
  - **Success gap (VAMP minus cuRobo):** it does not shrink. It goes from 31.9 to 36.1 points against cuRobo v1, and from 15.0 to 24.0 against cuRoboV2 (on the subsample).
  - **Median T_cycle gap:** it shrinks from 20.0 s to 0.7 s against cuRobo v1, and from 22.5 s to 1.1 s against cuRoboV2. On the original set, more than half of cuRobo's runs fail, so this measure sits at the 30 s cap.
  - **Conclusion:** on success rate, VAMP's lead over cuRobo is not an artefact of move length.
- **Feasibility on the original set:**
  - 155 queries are certified feasible and 127 have no path found by the reference planners.
  - Reference runs for 18 queries are still running.
