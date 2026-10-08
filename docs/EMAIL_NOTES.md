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
- **Small penetrations between cuRoboV2 waypoints (provisional).** VAMP's segment check flags 4/59 Panda, 16/100 rail-with-pillars and 4/102 UR5e-with-pillars cuRoboV2 successes.
  - **Not an artifact of the collision model:** V2's own checker confirms all 24 at the same samples, and agrees with VAMP on 12,000/12,000 random configurations.
  - **Size:** a median of 1.0 mm and a maximum of 3.5 mm, on the straight line between returned waypoints, never at a waypoint.
  - **Why it stays out of the email:** whether V2's continuous spline itself penetrates is unverified. A millimetre-scale graze is not a meaningful claim to make to her.
  - If subtracted, V2's Panda count would be 55/69 (59 − 4).
  - Details: `docs/V2_COLLISION_CHECK.md`.
