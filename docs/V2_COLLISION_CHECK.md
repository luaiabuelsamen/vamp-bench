# Is the cuRoboV2 between-waypoint collision finding real? (status: PENDING, runs first in GPU slot 2)

**The finding (provisional).** `bench/crossval.py` checks each cuRobo success with VAMP's `validate_motion` on every consecutive pair of the returned interpolated waypoints (dt = 0.025 s). It flagged 4/59 V2 successes on Panda, 16/100 on rail/pillars and 4/102 on UR5e/pillars. cuRobo v1 had 0/638 flagged on Panda. In every flagged trajectory, all waypoints are collision-free and exactly one 0.05–0.09 rad step fails.

**Why it could be an artifact:**
- VAMP checks the straight joint-space line between two waypoints, but V2's trajectory between them is a B-spline.
- V2 might use a different sphere set or radii than VAMP.
- V2's 0.0025 m activation distance or some padding might be counted differently by the two checkers.

**Checks** (`bench/verify_v2_collisions.py` → `results/v2_collision_check.json`):
- **Reproduce.** Re-plan each flagged query, plus 5 unflagged controls per scene, with the same settings and `reset_seed()`. Record whether the stored trajectory is reproduced.
- **(a) Dense re-sampling.** Re-interpolate V2's own optimized B-spline at dt/10 with V2's own interpolator, and confirm that every 10th dense sample equals the dt = 0.025 output. Then check every dense sample. This tests V2's own continuous trajectory, not a straight line.
- **(b) Identical model and margins.**
  - Check the dense samples with V2's own `RobotCollisionChecker` at activation 0 (true contact) and at 0.0025 (the planner's margin), alongside VAMP's point check.
  - Also check that V2's checker and VAMP agree on 4,000 random configurations per robot, i.e. that the spheres and radii are the same. The v1 equivalent is 16,000/16,000 in `results/collision_model_agreement.json`.
  - Both configs use zero sphere buffer and zero self-collision buffer (`configs/*_v2.yml`).
- **(c) v1 counted the same way.** All v1 sets, including the reruns, go through the same `bench/crossval.py` segment check.

**Verdict rule:**
- **Ruled in:** V2's own checker at activation 0 finds penetration in its own dense trajectory for the flagged queries, and the trajectory reproduces.
- **Ruled out:** the dense samples are collision-free under V2's checker, or the models disagree.
