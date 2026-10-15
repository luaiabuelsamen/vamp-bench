# Benchmark v2.2: industrial motion types (pre-registration)

**Status:** pre-registered and committed before any code for it runs on a measured program.
- **Freeze:** the v2 freeze `6aea6eecf76a173d7ebdf548104aaec854b6b6c7` (H) for the cell builder, the shared limits and the gate.
- **New seed labels:** `v22_cells_<k>`, `v22_dev_<k>`, `v22_seed0`…`v22_seed9`.
- **Changes after this commit:** logged as amendments before they are applied.

## Why
v2 covers one industrial motion type, MoveL. The other motions a cell integrator programs are:
- waypoint paths with blends (UR `movep`/`movel` with blend radius r > 0);
- circular moves (MoveC);
- moves with a programmed constant-acceleration (trapezoidal) speed profile.

v2.2 adds these. For each planner it records how the motion is expressed:
- natively;
- by our post-processing;
- not at all.

A motion a planner cannot express is reported as a result. It is not forced through a workaround.

## Programs
- **Cells:** 100 cells from `bench/gen_v2.py`'s cell builder (bin, conveyor and fixture stations; the same families and geometry ranges as the held-out set).
  - Each cell is seeded with `seed_from(H, "v22_cells_<k>")`; the first 100 valid k are kept.
  - Dev cells (`v22_dev_<k>`, 10 cells) are used for smoke tests and the capability probe only, never for results.
- **Programs:** each cell gets one program of each type, giving 300 programs (100 per type).
- **Common to all programs:**
  - The TCP is `tool0`.
  - Orientation is constant along the path: tool pointing down, the orientation of the start station.
  - The path starts 0.10 m above a station, at a collision-free start configuration on the A6 branch rule (nearest 2π branch to the cell's home). This configuration is given to every planner.
  - Speeds and accelerations are TCP values along the path's arc length.

### W: waypoint path with blends
- **Geometry:** start above station A and end above station B, through 3–6 via points (uniform count).
  - Via points are drawn in the free volume above the cell: heights 0.10–0.40 m above the station tops, lateral spread within the cell footprint.
  - Each leg is ≥ 0.10 m long.
  - The turn angle at each via point is between 30° and 150°.
- **Blends:** at each via point, a circular arc tangent to both legs, starting and ending at distance r from the via point along each leg.
  - r = min(0.05 m, 0.4 × the shorter adjacent leg).
  - This is the programmed blend geometry. A UR controller's blend is a similar but not identical curve, so the blend-zone deviation is reported separately from the line deviation.
- **Speed:** constant cruise speed v = 0.25 m/s through the blends (movep semantics). Ramps happen only at the start and end, with a = 1.0 m/s².

### C: circular arc (MoveC)
- **Geometry:** start above a station; an arc in the horizontal plane through a via point to an end point.
  - Radius 0.10–0.30 m; central angle 90°–270°.
  - The via point is at mid-angle.
  - The arc is fully defined by start, via and end, as in UR `movec`.
- **Speed:** v = 0.25 m/s, a = 1.0 m/s².

### T: straight line with a programmed trapezoidal profile
- **Geometry:** a straight line 0.30–0.60 m long between points above two stations.
- **Profile:** a = 0.25 m/s², cruise v = 0.25 m/s.
  - Each ramp covers 0.125 m, so every program has a cruise phase of ≥ 0.05 m.

### All program types
- **Time law:** the programmed time law is a 1-D jerk-limited profile on arc length (Ruckig), with the program's v and a and TCP jerk 10 m/s³.
  - The jerk-limited corners last 25 ms (W, C) or 100 ms (T, since a/j = 0.025 s per corner). Outside them, acceleration is constant at ±a or 0.
- **Validity (generator):** a program is kept only if both hold:
  - its path has a continuous collision-free IK solution from the start configuration: samples every 2 mm, IK seeded from the previous sample, every joint step ≤ 0.05 rad, VAMP `validate_motion` between samples, inside joint bounds;
  - every path point is at least 1 cm from the obstacles (sphere-model signed distance).
- **Programmed time law and joint limits:** the programmed time law is not required to fit the joint limits. A planner may have to slow down, and the metrics show it.

## How each planner expresses each motion

| Motion | Cartesian pipeline (ours) + VAMP check | cuRobo v1 | cuRoboV2 | PyRoki (own category, as in v2.1) |
|---|---|---|---|---|
| **W** waypoints + blends | **post-processing:** the programmed path, IK every 2 mm (seeded), VAMP `validate_motion`, joint spline, the programmed time law; if a joint limit is exceeded at 1 kHz, v halves (up to 3 times, recorded) | **not expressible** as one blended motion. v1's pose cost takes one goal pose (or goal set) per problem; `PoseCostMetric` can hold axes or add one approach offset, not several via points. **Run as the closest native motion:** one MoveL per leg (hold-line metric as in v2), each stopping at its via point. Reported as "stops at via points", not as a blend. | **native candidate:** per-timestep tool-pose targets (`GoalToolPose` with horizon > 1, `track_position_and_orientation`), targets = the programmed path sampled at its time law. Subject to probe P0; if P0 fails, as for v1 with `linear_motion` per leg. | **native costs, our harness:** a pose cost per knot toward the programmed path at its time law, endpoints held (v2.1 7g recipe), fixed dt = programmed duration / knots |
| **C** MoveC | post-processing as for W | **not expressible** (no arc or per-timestep pose target; chaining short MoveLs would be our approximation, not v1's motion). Not run. | as for W, if P0 passes; otherwise **not expressible**, not run | as for W |
| **T** trapezoidal MoveL | post-processing; the programmed profile is applied exactly | **path native, profile not expressible:** v1 MoveL as in v2. Its timing is v1's own time-optimal parameterization; there is no TCP speed or acceleration input. Run, and the profile metrics show the gap. | per-timestep targets if P0 passes (they encode the profile); otherwise `linear_motion` (path only, as for v1) | per-knot targets at the programmed timing |

- **The Cartesian pipeline column:** VAMP is a free-space planner. In this column it only checks collisions; the Cartesian path, IK and time law are ours.
  - Its geometric success is guaranteed by the generator's validity check (the same IK method), so its success measures the time law against the 1 kHz gate, and its T_ready measures our pipeline's speed.
  - It is the reference for what the programmed motion costs on this robot. It is not evidence about VAMP's planning.
- **cuRobo v1 configurations:** as configured (`max_jerk` 500, as in v2) and the A7 positive control (`max_jerk` 200). Both are reported.
- **Start and end:** every planner gets the start configuration and the programmed path. The end is a pose (the path end); no end configuration is given.
  - PyRoki's endpoint constraint uses its own IK on the end pose, timed as part of T_ready.
  - PyRoki is initialised by its own IK on each knot target, seeded from the previous knot, also in T_ready. It is never given the generator's IK path.
- **Probe P0 (cuRoboV2 per-timestep targets), on the 10 dev cells, before any measured run:**
  - **Setup:** trajopt with horizon-H targets, dt fixed to programmed duration / H (dt fine-tuning off if the API allows; otherwise recorded), `non_terminal_scale` 1.0.
  - **Pass:** the API accepts it, and on ≥ 6 of the 10 dev W programs the result passes the gate and keeps the max deviation outside blends ≤ 5 mm.
  - **Settings:** at most 3 settings (horizon and weight) are tried on dev cells. The one chosen is logged before the measured run.
  - **On failure:** the fallbacks above apply, and the probe's output is reported.
- **cuRobo v1 code check:** if the implementation finds a per-timestep pose target in v1 after all, this is logged as an amendment before any run and v1 is probed like V2.

## Metrics
All metrics are computed at 1 kHz with VAMP's FK.
- **Path deviation:** the maximum distance from the TCP to the programmed path (the path densely sampled at 0.5 mm, nearest point).
  - Reported separately in blend zones (the nearest path point on a blend arc) and outside them.
  - Also reported: maximum orientation deviation, and end pose error.
- **Speed constancy over the cruise:** the cruise is the time span where the programmed speed equals v.
  - Mapping: each planner's TCP speed is mapped to the programmed path by arc-length progress, so a planner that runs the path at a different overall pace is compared over its own cruise. The cruise is taken as the middle span between the points where its progress reaches the programmed ramp-end and ramp-start arc lengths.
  - Reported: mean and max |v − v_prog| / v_prog, and the achieved cruise speed over the programmed v.
- **Blends (W):** per via point, the minimum TCP speed within the blend zone, over v_prog.
  - **Stops at via point:** if that minimum is ≤ 0.05 (≤ 12.5 mm/s).
  - **Holds speed:** if it is ≥ 0.9.
- **Profile (T):** in each programmed ramp, the median |a_tcp − a_prog| / a_prog, and the ramp duration over the programmed one. Arc length is used as for the cruise.
- **Times:**
  - T_ready: planning plus parameterization (all IK and seeding included).
  - T_exec: the trajectory's duration.
  - T_cycle = T_ready + T_exec.
- **Success per run:** all of the following.
  - It passes the v2 1 kHz gate: no collision; joint bounds; velocity, acceleration and jerk ≤ 1.01 × the shared limits, with jerk 500 rad/s³ per arm joint.
  - T_ready ≤ 30 s.
  - End pose ≤ 1 mm and 1°.
  - Max path deviation outside blend zones ≤ 5 mm, and orientation ≤ 1°.
  - Speed and blend behaviour are reported, not part of success, since some planners have no speed input.
- **R2 (uniform slow-down, as in v2.1 7f, pre-registered here for every planner):**
  - Each failed-only-on-velocity/acceleration/jerk trajectory is slowed by s = max(1, rv, √ra, ∛rj) and reported in its own column with its cycle-time cost. R1, as returned, is primary.
  - R2 changes the speed, so R2 speed metrics are reported against the programmed v as well.

## Seeds, timing and statistics (as in v2)
- **Runs:**
  - cuRobo v1: 10 seeds per program (`v22_seed0`…`9`), both configurations.
  - cuRoboV2: 3 seeds per program.
  - Cartesian pipeline: deterministic, one geometric run; T_ready is the median of 10 timed repeats on a quiet CPU.
  - PyRoki: deterministic given its init, one run (stated as such).
- **Budget:** T = 30 s per planner call. For the per-leg fallback, the sum over legs counts against 30 s × number of legs, and T_cycle sums the legs.
- **Statistics:**
  - Per program type: success over runs; per-program success fractions; medians with bootstrap 95% CIs (2000 resamples over programs).
  - Paired comparisons only over programs both planners solve.
  - Bins: path length (W, C), number of vias (W), min path clearance (1–2 cm, 2–5 cm, > 5 cm). Bins with < 30 programs are flagged underpowered.
- **Hardware:** the GPU must have no other work during GPU runs. The Cartesian pipeline is timed on a quiet CPU at the start of the slot, as v2's pass 2a was.

## How results will be read
- **E, expressibility:** the table above, with the P0 outcome, is a result by itself. For each planner and motion type it is one of: native, our post-processing, closest native motion (stops at via points / path only), or not expressible.
- **Q1, path fidelity:** a planner holds a motion type's path if, on its successful runs, the max deviation outside blends has median ≤ 1 mm and p95 ≤ 5 mm. Blend-zone deviation is reported with its median and p95; no pass mark, since blend geometry differs between controllers.
- **Q2, constant speed:** a planner holds constant TCP speed on a motion type if the median over runs of the max cruise deviation is ≤ 10% and its p95 ≤ 25%.
- **Q3, blends:** a planner blends if the median per-via minimum speed is ≥ 0.9 v_prog. It stops at via points if the median is ≤ 0.05.
- **Q4, trapezoid:** a planner follows the programmed profile if the median ramp acceleration error is ≤ 20% and the ramp durations are within 20%.
- **Q5, cycle time:** median T_cycle per program type, paired over programs both planners solve, with the planning/execution split.
  - The per-leg fallback for W includes its stops; this is stated wherever it is compared.
  - If execution dominates, a planning-time advantage is reported as mattering little.
- **Q6, robustness:** the share of programs failing on some seed, and on every seed.

## Size and cost

| Item | Count | Estimate (from v2 per-call medians on this board) |
|---|---|---|
| Programs | 100 W + 100 C + 100 T | |
| Cartesian pipeline (CPU) | 300 programs × (1 run + 10 timed repeats) | ~0.1–0.3 s per program, < 15 min |
| cuRobo v1, 2 configurations × 10 seeds | W: ~4.5 legs × 0.62 s; T: 0.62 s; C not run | ~65 min per configuration, ~2.2 h |
| cuRoboV2, 3 seeds | if P0 passes: 300 calls × ~2.3–5 s; else W 4.5 legs + T, ~2.3 s each | ~0.6–1.5 h |
| PyRoki, 1 run | 300 programs, ~4–10 s each, plus one compile per knot count | ~0.5–1 h |
| Probe P0 + smoke on dev cells | | ~0.5 h |
| **GPU total** | | **~4–5.5 h**, one overnight slot |

- **Window:** the GPU is free until the evening of 2026-10-17.
- **Order in the slot:**
  1. Cartesian pipeline timing (quiet CPU);
  2. P0;
  3. cuRobo v1 as configured;
  4. v1 `max_jerk` 200;
  5. cuRoboV2;
  6. PyRoki.
- **Slot rules:** seed-major order, so a cut-off leaves complete seeds. Hard stop and `GPU_RELEASED` by 2026-10-17 12:00.

## UR controller in the loop (URSim e-series): roles, pending a platform
- **Role 1, reference:** for every W, C and T program, the UR controller's own motion (`movep`/`movel` with blend radius r, `movec`, `movel` with a and v) is run in URSim. Its TCP is recorded at 500 Hz over RTDE.
  - It is the baseline a UR programmer would use: a planner's blend, arc or profile is compared with the controller's on the same metrics (path deviation inside and outside blends, cruise speed, blend speed, ramps, execution time).
- **Role 2, execution check (subset):** each planner's successful trajectories on a subset of programs (size set once the platform's throughput is measured) are streamed through `servoj` at the controller's cycle. Recorded: protective stops, speed scaling, and tracking deviation from the commanded trajectory.
- **Limit of the evidence:** URSim is kinematic. It checks the controller's trajectory generator and its limits, not dynamics, payload or real tracking error. This is stated wherever its results appear.
- **Platform (feasibility, 2026-10-14):**
  - URSim e-series is published only for amd64: every tag of `universalrobots/ursim_e-series` on Docker Hub (5.17–5.26 and `latest`, 0.98 GB) is amd64-only. The Jetson is aarch64.
  - Running it here needs x86 emulation (qemu-user-static, not installed). Whether emulated URControl keeps its real-time cycle on this board is untested.
  - The alternative is an amd64 or Rosetta host. That needs the owner's approval for that machine.
- **PolyScope X simulator (`universalrobots/ursim_polyscopex:10.14.0`), tested 2026-10-14:**
  - **What the arm64 tag is:** a Docker-in-Docker wrapper (4.18 GB unpacked; its inner image store adds about 6 GB). The six components it starts are all amd64 images, including the UR controller (`controller-artifact` 99.4.49): `bootstrapper`, `citadel`, `controller`, `nginx`, `urauth`, `urservice`.
  - **Result:** on the Jetson, the first amd64 service (`citadel`) exits with code 255 and the stack stops. It does not run natively on aarch64 either.
  - **Board-specific fixes needed to get that far:** the board's kernel has no iptables `raw` table (`CONFIG_IP_NF_RAW` not set), so port publishing needs `DOCKER_INSECURE_NO_IPTABLES_RAW=1`; the inner dockerd needs `DOCKER_IPTABLES_LEGACY=1`.
  - **Licence:** it is a beta SDK image under Universal Robots' SDK licence terms; any use of it here is stated with that.
  - The URSim part runs only after a platform is approved. Its protocol details (subset size, servoj gain and lookahead) are logged as an amendment before it runs. The planner runs above do not depend on it.

## Amendment V1 (logged 2026-10-15, before any program set was generated for results and before any planner run)
Found on the dev cells (`v22_dev_*`) while building the generator. No measured cell was generated and no planner was run.
- **Problem.** As written, almost no programs were valid. Of the first candidates, 99% of W, 88% of T and about half of C failed the path-IK check, for three reasons:
  - **Start inside the station:** the start point 0.10 m above the sampled grasp point can lie inside a bin's walls.
  - **Paths over the robot:** paths toward a station on the other side of the rail run over the robot's own base, where the folded arm collides with itself.
  - **Elbow singularity:** high via points and wide arcs reach the stretched-elbow singularity. With the tool pointing down, the UR5e's usable TCP reach is well below its 0.85 m nominal reach.
- **Rules, applied to every program type:**
  - **Start and end points:** "0.10 m above a station" means 0.10 m above the station's highest box, directly over the sampled target (or 0.10 m above the target if that is higher). The start configuration is the IK at the target, followed by a straight vertical line up to this point.
  - **End station:** W ends above, and T heads toward, a station on the same side of the rail as the start station. Cells without such a pair are skipped.
  - **Workspace bounds:** every path point must be at least 0.35 m horizontally from the base axis (the rail carriage's vertical axis) and within 0.70 m of the shoulder. The two bounds were set on the dev cells: 0.30 and 0.80 m still left elbow jumps and arm collisions. W via points are drawn within the pre-registered ranges and redrawn (up to 20 times) until they meet these bounds.
  - **Rail:** fixed along every path. Each program is a fixed-base UR5e motion, which is also what a UR controller (URSim) runs.
  - **Speed:** the log-map orientation error is computed in closed form instead of with scipy. The values are identical to 2e-10 rad on 2000 random poses.
- **Consequence:** the programs are W, C and T motions in the comfortable part of the arm's workspace, about 1 cell in 6 is accepted, and the acceptance rate is reported.
- **Generation:** k up to 1500, 4 workers. Memory and disk rules apply: MemAvailable has stayed above 4 GB for hours with no flagship job running, so more than 2 workers is allowed.

## Amendment V2 (logged 2026-10-15, from reading the code, before any cuRobo or PyRoki run for v2.2)
- **cuRobo straight lines are axis holds.** In both versions, the native line constraint holds selected axes of the goal pose:
  - **v1:** `PoseCostMetric(hold_partial_pose=True, hold_vec_weight=[rx, ry, rz, px, py, pz], project_to_goal_frame=...)`.
  - **V2:** `ToolPoseCriteria` with non-terminal axis weights, e.g. `linear_motion(axis)`, and `project_distance_to_goal`.
  - **What a held axis means:** for every timestep, the distance to the goal pose along that axis (in the goal frame, or in the base frame without projection) is penalised.
  - **Consequence:** a straight line is held natively only when it is parallel to an axis of the goal frame (the tool axes, since the goal orientation is the programmed R) or of the base frame. The v2 pick-place MoveL lines were vertical, along the tool z axis.
- **Expression rule for every v2.2 line (T, and each W leg):**
  - **Native MoveL:** the line is within 1° of a goal-frame or base-frame axis. The other two position axes and all three rotation axes are held.
  - **Partial (plane):** the line is off-axis, but a frame axis is perpendicular to it within 1° (e.g. a horizontal line, base z). Orientation and that axis are held.
  - **Partial (orientation only):** otherwise only orientation is held.
  - **Recording:** the class is recorded per line. Success and the path metrics are measured as pre-registered in every case.
- **cuRoboV2 per-timestep targets (P0):**
  - **Code finding:** `GoalToolPose` documents horizon > 1 as per-timestep targets, but `ToolPoseCost.forward` passes `goal_tool_poses.position.squeeze(1)` to a kernel that indexes goals by (problem, link, goal-set entry) only. In this build (`ac5f931`), the cost does not read a target per timestep.
  - **Expectation:** P0 is expected to fail. It is still run as pre-registered, on the dev cells, to confirm the code reading empirically.
  - **If it fails:** cuRoboV2 W runs as one line per leg (the expression rule above, stopping at via points), C is not expressible, and T is the line rule above (path only).
- **cuRobo v1:** no per-timestep pose target was found. Its only multi-point mechanism is one approach offset (`create_grasp_approach_metric`), so W runs as one line per leg and C is not expressible, as pre-registered.
- **Consequence:**
  - For the tool-down programs, most cuRobo lines fall in the partial classes. The v2.2 result for cuRobo is then mainly expressibility, which the pre-registration counts as a result.
  - The programs are not changed to suit an axis. Lines in arbitrary directions are what a cell programmer writes.

## Amendment V2a (logged 2026-10-15, from the dev-cell smoke test, before any measured cuRobo run)
- **Problem:** cuRobo v1 refuses a partial hold whose start and goal differ by more than 5 mm on a held axis (`motion_gen.py`, "Partial position between start and goal is not equal"). V2's 1° angle tolerance allows up to 8.7 mm on a 0.5 m leg, and one dev leg was refused for this reason.
- **New rule:** a position axis of the goal or base frame can be held if the line's start-to-end displacement along it is ≤ 1 mm.
  - **Native:** two axes of one frame can be held, so the line runs along the third.
  - **Plane:** one axis can be held.
  - **Orientation only:** none can be held.
  - **Ties:** the base frame wins.
- **Dev smoke test (3 cells, v1, seed 0):** every T line and almost every W leg was in the orientation-only class. In those runs the path is not held: max deviation 2–27 mm, against the pre-registered 5 mm success limit.

## Probe P0 result (2026-10-15, dev cells, before any measured run)
- **Outcome: fails on all 10 dev W programs under all 3 settings.** The settings were `non_terminal_scale` 1 and 10 with dt fixed, and 1 with the solver's own dt.
  - **Where it fails:** cuRoboV2's trajopt (`trajopt_solver.solve_pose`, action horizon 16) rejects a goal of horizon 16.
  - **Why:** the goal buffer is allocated with horizon 1: `manager_goal.update_goal_buffer` gives "output with shape [1, 1, 1, 1, 3] doesn't match the broadcast shape [1, 16, 1, 1, 3]".
  - Records: `results/v2_2/p0.jsonl`.
- **Consequence for cuRoboV2** (as pre-registered for a P0 failure):
  - W runs as one line per leg, stopping at the via points.
  - C is not expressible and is not run.
  - T uses the line rule (V2a).

## Execution notes (2026-10-15, before the measured runs)
- **Dev smoke tests:** every harness ran end to end on dev cells (`results/v2_2/dev/`). These are dev records only, not results.
- **Deviation metric:** fixed on dev data before any measured run.
  - Deviation is the exact distance to the programmed path: the polyline through 0.5 mm samples, with segment refinement and the global nearest point.
  - Progress for the speed metrics uses a forward-moving window.
  - The first version measured to the nearest sample, which put a 0.25 mm floor on every deviation.
- **Cartesian pipeline on dev W programs:** the circular blends make the programmed speed infeasible at 0.25 m/s on 9 of 10 dev programs.
  - The cause is the curvature step at blend entry, which makes the joint jerk spike.
  - The pre-registered retry halves v for the whole program, and the speed metrics show it against the programmed 0.25 m/s.
- **PyRoki knot counts:** set by the pre-registered rule (spacing ≤ 40 ms for the longest program of each type), computed on the measured set. On dev, W needs about 24 s of T_ready per program, against the 30 s budget.
- **Slot:** `bench/gpu_slot_v2_2.sh`, which starts once the program generator has finished.
