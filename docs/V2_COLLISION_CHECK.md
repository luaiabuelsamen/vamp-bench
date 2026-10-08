# Is the cuRoboV2 between-waypoint collision finding real?

**Verdict (2026-10-07).** Partly settled:
- **Not a model artifact.** cuRoboV2's own collision checker confirms the penetration.
- **It is small:** a median of 1.0 mm and a maximum of 3.5 mm, on the straight line between V2's returned 25 ms waypoints.
- **Still open:** whether V2's actual continuous B-spline penetrates. Our dense re-sampling of V2's spline failed its self-check, so we can't say.

The finding stays out of the email facts. Its wording changes from "cuts through an obstacle" to "grazes obstacles by up to 3.5 mm between returned waypoints".

All numbers are from `results/v2_collision_check.json`, summarized in `results/summary.json` under `v2_collision_check`. The script is `bench/verify_v2_collisions.py`, run on 2026-10-07 at 22:1x in GPU slot 2.

## What was flagged
`bench/crossval.py` runs VAMP's `validate_motion` on each consecutive pair of a cuRobo success's interpolated waypoints (dt = 0.025 s). For cuRoboV2 it flagged:
- 4/59 successes on Panda
- 16/100 on rail/pillars
- 4/102 on UR5e/pillars

That is 24 in total.

## Checks and results
**(b) Same spheres, radii and margins: model artifact ruled out.**
- **Random configurations:** V2's own `RobotCollisionChecker` at activation 0 agrees with VAMP on 4000/4000 random configurations for each of the three robots. Both configs use 0 sphere buffer and 0 self-collision buffer.
- **Flagged trajectories:** on all 24 of the stored flagged trajectories, V2's own checker at activation 0 flags exactly the same samples VAMP does. We sampled each straight segment between waypoints at 1/200 steps.
- **Waypoints:** V2's checker finds 0 collisions at the waypoints themselves.
- **Controls:** 15 unflagged controls (5 per scene) are clean under both checkers.
- **Activation distance:** V2 plans with activation distance 0.0025 m, a cost margin that pushes waypoints at least about 2.5 mm away from obstacles. It does not check between waypoints.

**Size of the penetration.** This is V2's own signed distance, as the maximum over spheres per flagged trajectory, n = 24:

| Statistic | Penetration |
|---|---|
| median | 1.0 mm |
| p95 | 3.0 mm |
| max | 3.5 mm |
| over 1 mm | 12 of 24 |
| over 2.5 mm | 3 of 24 |

The robot spheres are themselves a conservative approximation of the robot's geometry.

**(a) Dense re-sampling of V2's own trajectory: inconclusive.**
- **Re-planning doesn't reproduce:** re-planning a query with the same settings and `reset_seed()` reproduced none of the stored trajectories (0 of 21 successful re-plans). The planner keeps state between queries, such as its graph, so the stored trajectories can't be regenerated exactly.
- **Dense re-sampling failed its self-check:** we re-interpolated each fresh plan's optimized B-spline at dt/10 with V2's own interpolator. Every 10th dense sample should equal V2's normal output, but they differ by up to 0.59 rad. So our call does not reproduce V2's interpolation, and the dense samples were not used.
- **The flag recurs on fresh plans:** VAMP's segment check flags 4 of 21 fresh plans again.

So the evidence covers the straight line between V2's returned waypoints, not V2's continuous spline. A controller that linearly interpolates the returned waypoints would follow those straight lines.

**(c) v1 counted the same way.** The same `bench/crossval.py` check ran on cuRobo v1:
- 0 of its 638 Panda successes are flagged.
- The v1 paper scenes and UR5 are being checked identically in GPU slot 2. Results go to `results/summary.json` under `success_flagged_by_vamp_segment_check_PROVISIONAL`.

## What it would take to close (a)
Either of these would settle it:
- A deterministic V2 replay of the stored queries (fresh planner per query).
- V2's continuous-collision or "speed metric" check, if it has one, run on the stored trajectories.

Until then the claim is limited to the waypoint chords.
