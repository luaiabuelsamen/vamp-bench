# Facts for writing to Prof. Kavraki
These are facts to use, not a draft. Each one traces to `results/summary.json` or `docs/`. **Solved** = planner-reported success, the same definition as the README and figure. All cells are filled (GPU slot 2, 2026-10-07 22:24 – 2026-10-08 00:14).

1. **Setup.** We compared VAMP (`27cb9b6`), cuRobo v1 (the version our paper used) and the current cuRoboV2, all on a single Jetson Orin NX 16 GB. VAMP ran on one Cortex-A78AE core with NEON, and both cuRobo versions on the Orin NX's GPU. All three checked the same sphere models against the same obstacles, and their collision checkers agreed on all 16,000 random configurations we tested.
2. **Panda and UR5 results.** On VAMP's MotionBenchMaker Panda problems, VAMP solved 699/699 with a median of 1.76 ms per query (wall clock, including Python). cuRobo v1 solved 638/699 with a median of 849 ms, about 480x slower. On a 69-problem subset, cuRoboV2 solved 59 with a median of 6.55 s (with `max_attempts=20`; NVIDIA's own benchmark uses 100, so V2's success rate is a lower bound). On the UR5 problems, VAMP solved 689/689 (median 2.02 ms), cuRobo v1 367/689 (827 ms) and cuRoboV2 35/64 (4.52 s). The cuRobo UR5 runs use the real UR5 velocity limit; VAMP's UR5 robot file caps joints at 0.5 rad/s, which cuRobo cannot plan under.
3. **Paper scenes.** We used cricket to generate VAMP models for the two robots in our paper: the UR5e, and the UR5e on a 7th-axis rail. On the paper's three pick-and-place scenes:
   - **VAMP:** median 0.55–0.97 ms per query; solved 1020/1030, 1030/1030 and 1030/1030.
   - **cuRobo v1:** median 145–377 ms; solved 823/1030 (rail with pillars), 1030/1030 (rail, no obstacles) and 985/1030 (UR5e with pillars). That is 169–388x VAMP's median.
   - **cuRoboV2:** median 0.38–1.06 s, with `max_attempts=20`, not NVIDIA's 100.
4. **Availability.** The code, the problem sets and the per-query results are in one repository, so her group can reproduce or extend the comparison (for example, to an x86 CPU or a desktop GPU).

## Benchmark v2 facts (time to an executable trajectory)
Each fact traces to `results/v2/summary.json`, `results/v2/hypotheses.json` or `results/v2/pickplace_summary.json`. Success means the trajectory passes one 1 kHz check (no collision; joint bounds; velocity, acceleration and jerk ≤ 1.01 × the shared limits) within 30 s. The design was pre-registered before any run (`docs/V2_PREREGISTRATION.md`).

5. **Time to an executable trajectory.** VAMP (with Ruckig time-parameterization) reaches an executable trajectory 59× (held-out UR5e-on-rail cells), 478× (MBM) and 159× (MπNets) sooner than cuRobo v1 in median, in every bin of every set.
6. **Cycle time is close.** On held-out runs both solve, cuRobo v1's cycle is shorter: 2.36 s vs 2.67 s. Its trajectories execute 0.97 s faster, which outweighs VAMP's 0.21 s planning advantage. Counting failures at 30 s, VAMP's median cycle is lower: 3.09 s vs 3.76 s.
   - The held-out queries are station-to-station configuration pairs, not tool-down picks.
7. **Success and robustness.** On the held-out set VAMP succeeds on 93.1% of runs and cuRobo v1 on 56.9%. On MBM and MπNets, cuRobo v1 has fewer queries failing on some seed.
8. **cuRoboV2.** On the same queries and seeds, its median planning time is 7–15× cuRobo v1's.
9. **Pick-place with MoveL** (600 picks per planner):
   - VAMP completes 587/600, median cycle 6.08 s.
   - cuRoboV2 completes 585/600, median cycle 18.92 s.
   - cuRobo v1 as configured completes 0/600: its MoveL trajectories exceed its own 500 rad/s³ jerk setting at the segment ends. With `max_jerk` 200 it completes 572/600 at 8.22 s.
   - VAMP's pick-place planning ran on a loaded CPU, so its planning times are upper bounds.

## Benchmark v2.2 facts (industrial motion types)
Each fact traces to `results/v2_2/summary.json`. Pre-registered in `docs/V2_2_MOTION_TYPES_PREREGISTRATION.md`; results in `docs/V2_2_RESULTS.md`. 100 UR5e programs per type: waypoint paths with blends (W), MoveC (C), and a straight line with a trapezoidal TCP profile (T).

10. **Expressibility is the main cuRobo result.**
    - **Straight lines:** cuRobo (v1 and V2) holds a line by holding axes of the goal or base frame, and none of the 583 programmed lines runs along such an axis.
    - **Other motions:** waypoint paths run as one line per leg, stopping at every via point. MoveC and a programmed speed profile are not expressible.
    - **cuRoboV2's per-timestep pose targets:** they are rejected by its trajectory optimiser (the goal buffer has horizon 1).
11. **T line, success over runs:**
    - cuRoboV2: 120/300.
    - cuRobo v1: 319/1000 with `max_jerk` 200, and 29/1000 as configured.
    - Most failures exceed 5 mm path deviation. cuRobo runs the line at about 4.4–4.9 times the programmed speed.
12. **PyRoki:** in its own trajectory-optimiser category, it follows the T line and its trapezoid (84/100; median deviation 0.21 mm).
    - **W and C as returned:** they mostly exceed the joint jerk limit. A uniform slow-down gives 19/100 and 88/100, below the programmed speed.
    - **Planning time on W:** a median of 41.0 s.
13. **The programmed motions are executable:** our Cartesian pipeline (VAMP as collision check) completes all 300. The circular blends force half speed on 76 of 100 waypoint paths. This is a reference for the programs, not a VAMP planning result.
    - No UR controller (URSim) has run these programs yet.

## Before writing
- She read arXiv 2508.04146. `docs/PAPER_CLAIMS_AUDIT.md` lists which of its numbers a committed experiment supports. The 60× is cited from NVIDIA; the 15–25× vs OMPL, 8× vs TrajOpt, CHOMP 92% vs 78% and ABB 40% have no supporting experiment. The 3× matches only a cycle-time ratio against MoveIt/OMPL (3.02 s vs 9.90 s, different poses). The facts above come from this repo's same-board benchmark, not from the paper.

## Provisional: do not use until verified
- **Millimetre penetrations between waypoints, both cuRobo versions (provisional).** VAMP's straight-line check between returned waypoints flags some successes of both versions:
  - **Counts:** v1 0/638 Panda, 4/367 UR5, 44/823 rail-with-pillars, 3/985 UR5e-with-pillars. V2 4/59, 6/35, 16/100, 4/102.
  - **Not an artifact of the collision model:** V2's own checker confirms its flagged trajectories at the same samples, and agrees with VAMP on 12,000/12,000 random configurations.
  - **Size:** median 0.12–2.30 mm per set, maximum 5.08 mm, never at a waypoint.
  - **Why it stays out of the email:** it is millimetre-scale, it affects both versions (so it is not a V2-specific defect), and the continuous trajectories are unverified.
  - Details: `docs/V2_COLLISION_CHECK.md`, `results/penetration_depth.json`.
