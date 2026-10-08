# Facts for writing to Prof. Kavraki
These are facts to use, not a draft. Each one traces to `results/summary.json` or `docs/`. **Solved** = planner-reported success, the same definition as the README and figure. All cells are filled (GPU slot 2, 2026-10-07 22:24 – 2026-10-08 00:14).

1. **Setup.** We compared VAMP (`27cb9b6`), cuRobo v1 (the version our paper used) and the current cuRoboV2, all on a single Jetson Orin NX 16 GB. VAMP ran on one Cortex-A78AE core with NEON, and both cuRobo versions on the Orin NX's GPU. All three checked the same sphere models against the same obstacles, and their collision checkers agreed on all 16,000 random configurations we tested.
2. **Panda and UR5 results.** On VAMP's MotionBenchMaker Panda problems, VAMP solved 699/699 with a median of 1.76 ms per query (wall clock, including Python). cuRobo v1 solved 638/699 with a median of 849 ms, about 480x slower. On a 69-problem subset, cuRoboV2 solved 59 with a median of 6.55 s (with `max_attempts=20`; NVIDIA's own benchmark uses 100, so V2's success rate is a lower bound). On the UR5 problems, VAMP solved 689/689 (median 2.02 ms), cuRobo v1 367/689 (827 ms) and cuRoboV2 35/64 (4.52 s). The cuRobo UR5 runs use the real UR5 velocity limit; VAMP's UR5 robot file caps joints at 0.5 rad/s, which cuRobo cannot plan under.
3. **Paper scenes.** We used cricket to generate VAMP models for the two robots in our paper: the UR5e, and the UR5e on a 7th-axis rail. On the paper's three pick-and-place scenes:
   - **VAMP:** median 0.55–0.97 ms per query; solved 1020/1030, 1030/1030 and 1030/1030.
   - **cuRobo v1:** median 145–377 ms; solved 823/1030 (rail with pillars), 1030/1030 (rail, no obstacles) and 985/1030 (UR5e with pillars). That is 169–388x VAMP's median.
   - **cuRoboV2:** median 0.38–1.06 s, with `max_attempts=20`, not NVIDIA's 100.
4. **Availability.** The code, the problem sets and the per-query results are in one repository, so her group can reproduce or extend the comparison (for example, to an x86 CPU or a desktop GPU).

## Provisional: do not use until verified
- **Millimetre penetrations between waypoints, both cuRobo versions (provisional).** VAMP's straight-line check between returned waypoints flags some successes of both versions:
  - **Counts:** v1 0/638 Panda, 39/367 UR5, 44/823 rail-with-pillars, 3/985 UR5e-with-pillars. V2 4/59, 6/35, 16/100, 4/102.
  - **Not an artifact of the collision model:** V2's own checker confirms its flagged trajectories at the same samples, and agrees with VAMP on 12,000/12,000 random configurations.
  - **Size:** median 0.12–2.30 mm per set, maximum 5.08 mm, never at a waypoint. 35 of v1's 39 UR5 flags are self-collision, with depth not yet measured.
  - **Why it stays out of the email:** it is millimetre-scale, it affects both versions (so it is not a V2-specific defect), and the continuous trajectories are unverified.
  - Details: `docs/V2_COLLISION_CHECK.md`, `results/penetration_depth.json`.
