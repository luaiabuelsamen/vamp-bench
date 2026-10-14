# Benchmark v2: design (no runs yet)

**Why a v2.** v1 (this repo's `results/`) shows one thing reliably: on the same Jetson, with identical collision models, VAMP returns a collision-free *geometric path* 2–4 orders of magnitude sooner than cuRobo returns a *time-parameterized trajectory*. It does not support general claims, for three reasons:
- **Home-turf problem sets.** MotionBenchMaker (MBM) is VAMP's home benchmark, and the paper scenes are 103 jittered pick→place transfers.
- **One metric.** Planning wall-clock is the only metric.
- **Different products.** That metric compares an untimed path with an executable trajectory.

v2 fixes these by:
- choosing problems neither planner was developed on;
- measuring time to an *executable* trajectory under shared limits;
- reporting motion quality and variance;
- adding the motion types industrial cells actually use.

## 1. Problems: who chooses them, coverage, stratification

**Three sources, so no single planner's home set decides the result:**

| Set | Chosen by | Why it's included |
|---|---|---|
| MBM (Panda, UR5) | Chamzas et al.; VAMP's home set | Continuity with v1; VAMP's best case |
| MπNets subset (Panda) via robometrics | Fishman et al.; one of cuRobo's own benchmark sets (`curobo_benchmark.py` reads `mpinets_raw`) | cuRobo's best case. VAMP reads it with `resources/robometrics_to_pkl_json.py`. |
| **Held-out industrial generator** (UR5e, UR5e + rail) | us, frozen *before* any run | Neither planner has seen it |

**Held-out generator** (`bench/gen_v2.py`, to write):
- **Scene families:** parametric industrial cells. Shelving, bins, a conveyor, pillars/guards, a gantry rail, and a fixture table with tight slots.
- **Geometry and queries:**
  - Obstacles are cuboids, so both planners use identical analytic geometry and there are no mesh-conversion effects.
  - Queries are pick/place pairs from cell semantics (bin → conveyor, shelf → fixture), not jitter around one pose.
- **Freezing protocol:**
  - The generator code and its parameter ranges are committed and hashed before any run.
  - Problem seeds are drawn after the freeze, from a published seed (e.g. the hash of the freeze commit).
- **No tuning on the set:**
  - Planner settings are fixed beforehand to each library's own benchmark defaults: v1 `curobo_benchmark.py`, V2 `motion_plan_benchmark.py`, VAMP `evaluate_mbm.py`.
  - Any change to settings after seeing held-out results invalidates the run.
- **Validity:** a query is kept only if both collision checkers accept its start and goal. The same 100%-agreement check as v1 is run on each scene family.

**Stratification:** every result is reported per bin, never only pooled. Bins are computed *before* planning, from geometry alone:

| Axis | Measure | Bins |
|---|---|---|
| Reach | TCP distance from base ÷ max reach, at start and goal | <0.5 / 0.5–0.8 / >0.8 |
| Clearance | min robot–obstacle distance at start and goal (shared sphere model) | <1 cm / 1–5 cm / >5 cm |
| Narrow passage | straight joint-space line collides? and passage width ÷ largest arm sphere diameter (generator-controlled) | free line / blocked, wide / blocked, <2× |
| Joint-limit proximity | min margin to any joint limit at start and goal | <0.2 rad / ≥0.2 rad |

Target is ≥30 queries per occupied bin, so medians and 95% bootstrap CIs per bin are meaningful.

## 2. Fair end-to-end metric: time to an executable trajectory

**Shared limits.** Both planners get the same per-joint velocity, acceleration and jerk limits, taken from the URDF or the manufacturer: UR5e and Panda datasheets. v1 already showed that a wrong velocity cap silently decides results.

**What gets timed, per planner:**

| Stage | VAMP | cuRobo v1 / V2 |
|---|---|---|
| Plan | RRT-Connect + simplify (measured) | `plan_single_js` / `plan_cspace` (measured) |
| Time-parameterize | **TOPP-RA** (time-optimal under vel/acc limits) on the simplified path, then **Ruckig** jerk-limited smoothing between the TOPP-RA samples; time measured | none needed (B-spline already respects limits); verify the limits independently |
| Validate | VAMP check on the *timed, sampled* trajectory at ≥1 kHz | same check, same rate |

**Reported per query:**
- **T_ready** = plan + parameterization (+ validation, reported separately). This is the latency until the robot can move.
- **T_exec** = trajectory duration.
- **T_cycle** = T_ready + T_exec.

On an industrial cell T_exec dominates, and cuRobo's optimized trajectories may be shorter in duration, so T_cycle is the number that can reverse v1's conclusion.

Also report:
- **Limit violations** of the timed trajectory: count and maximum.
- **Collision verdict** of the *timed* trajectory at 1 kHz. This also replaces v1's waypoint-chord check, which was inconclusive about the continuous spline.

## 3. Motion quality

All quality metrics are computed on the timed trajectory sampled at 1 kHz, identically for every planner:
- **Path length:** joint-space L2 (rad) and TCP Cartesian length (m).
- **Smoothness:** RMS and peak joint jerk, and integrated squared jerk.
- **Clearance:** minimum over time of robot–obstacle signed distance (shared model, `bench/penetration_depth.py`), and the time spent within 1 cm.
- **Joint-limit margins:** the minimum margin over time, per joint.
- **TCP orientation drift:** angle between start and goal orientation along the path, for pick-place where the tool should stay level.

## 4. Randomness and determinism
- **Seeds:** N = 10 seeds per query for every planner. Seeds are recorded: VAMP Halton skip / RNG seed, cuRobo `random_seed` + `reset_seed()`.
- **Per query:** success fraction over seeds, median, p95 and worst-case T_cycle, and the coefficient of variation.
- **Per bin:** median over queries of the per-query median, plus the worst query.
- **Determinism test:** each planner re-runs 20 queries twice with identical seeds in a fresh process, recording whether the output is bit-identical. v1 showed cuRoboV2 does not reproduce stored trajectories after `reset_seed()` because state carries over between queries, so V2 gets a fresh planner per seed in v2.
- **Failure accounting:** a failure counts as a failure in success rate, and as T_cycle = timeout in worst-case tables. It is never dropped silently.

## 5. Industrial motion types

| Motion | VAMP | cuRobo v1 / V2 | Metric |
|---|---|---|---|
| **MoveL** (straight TCP line, fixed orientation) | not native. Post-processing: Cartesian interpolation → IK per sample (continuity check) → VAMP motion check per segment → TOPP-RA with a TCP speed constraint | partial native: pose-cost terms along the trajectory (`hold_vec_weight`, used in the paper for fixed orientation); a straight-line TCP constraint is not a hard constraint in the API we used (to verify for V2) | max TCP deviation from the line (mm), TCP speed deviation from commanded (%), success |
| **Multi-waypoint with blends/zones** | not native: sequence of joint goals → TOPP-RA through via points (zone = blend radius) | not native in the calls used: chained single plans; V2 goal sets are for alternative goals, not via points (to verify) | path-through error at each via point, T_exec |
| **Constant-TCP-speed segment** | post-processing (TOPP-RA path-velocity constraint) | post-processing (re-time cuRobo's path with TOPP-RA) | TCP speed variance on the segment |
| **Acceleration-limited segments** | TOPP-RA / Ruckig | native (limits in the optimizer) | violations, T_exec |
| **Realistic pick-place sequence**: home → approach → MoveL down → grasp → MoveL up → transfer → approach → MoveL down → release → retract | composed from the above | composed from the above | total T_cycle per pick-place, success over seeds |

**Native vs post-processed.** This table is itself a result. "Needs post-processing" counts toward that planner's T_ready, and the post-processing code is identical for both planners wherever both need it.

## 6. What each result would support, and what it costs

**Claims each result would support:**
- **T_ready per bin, all three sources:** a claim like "VAMP is ready to execute X× sooner across problem families, including cuRobo's own benchmark set and a held-out industrial generator". Or the result shows where that isn't true.
- **T_cycle on the pick-place sequence:** a cell-level claim, e.g. "including execution, the difference is Y s per pick". This is the claim an industrial reader cares about, and it can favour either planner.
- **Quality + variance:** whether either planner's speed costs smoothness, clearance or worst-case reliability.
- **MoveL / blends table:** what each planner can express, and what extra code it needs, at what time cost.
- **Not supported:** anything about desktop GPUs or x86. Every number is this Orin NX, plus the Mac for VAMP only.

**Cost estimates.** These scale from v1's measured medians on this Orin:
- VAMP ≈ 1–2 ms per query;
- cuRobo v1 ≈ 0.15–0.85 s per query;
- cuRoboV2 ≈ 0.4–6.6 s per query, plus ~5 min warmup per robot.

TOPP-RA and Ruckig cost is not yet measured (an assumption to check first).

| Minimal v2 | Queries | Seeds | Orin NX | Luai's Mac (Apple Silicon) |
|---|---|---|---|---|
| VAMP (+ TOPP-RA/Ruckig, quality metrics) | ~1,000 | 10 | **< 1 h CPU**, dominated by post-processing and 1 kHz validation | same, CPU only; VAMP builds on arm64 (NEON) |
| cuRobo v1 | ~1,000 | 10 | **~3–6 h GPU** (10k × ~0.2–0.85 s, plus retries on failures) | not possible: cuRobo needs CUDA |
| cuRoboV2 | ~300 (stratified subsample) | 3 | **~2–5 h GPU** (fresh planner per seed adds warmup) | not possible |
| Pick-place sequences (all planners) | 30 cells × 10 picks | 5 | ~1 h GPU + minutes CPU | VAMP only |

That is about 7–12 GPU-hours, which needs one overnight slot plus one evening slot, coordinated with the flagship. Cost: $0.

## Recommended minimal v2 (actually informative)
1. **Freeze first.** Commit the held-out generator, the bins, the shared limits and the planner settings.
2. **Problems:**
   - **Held-out industrial set:** UR5e + rail, ~300 queries, stratified so each bin has ≥30.
   - **Each planner's home set:** 300 MBM Panda and 300 MπNets Panda queries. The two home sets cancel each other's home advantage.
3. **Primary metric:** T_cycle = plan + parameterization + execution, with 10 seeds and per-bin CIs.
4. **Secondary metrics:** limit and collision verdict on the 1 kHz timed trajectory, plus the quality metrics.
5. **Industrial sequence:** one realistic pick-place sequence (§5) with MoveL segments, 30 cells. This is the headline for an industrial audience.
6. **Order of work:** VAMP + TOPP-RA/Ruckig on the CPU first (hours of work, minutes of compute). Then cuRobo v1. Then cuRoboV2 on the stratified subsample.

Each step is useful on its own, and the CPU step can run on the Mac.

## 7. v2.1: PyRoki as a trajectory optimiser (design, no runs yet)

v2's results stand as published; v2.1 adds a third method and changes nothing in v2.

**What PyRoki is and how it is reported.**
- PyRoki (Kim et al., Berkeley; JAX, runs on CPU and GPU) is a kinematics and trajectory-optimisation library. It has no global planner: its trajectory optimiser refines a seed trajectory locally.
- It is therefore reported in its own category ("trajectory optimiser"), in its own tables. It is not ranked against VAMP or cuRobo on the global-planning sets. Where the same quantity is measured for the other planners (IK, MoveL, trajopt from cuRobo's seeds), their numbers are shown alongside for reference.

**Versions.**
- PyRoki `388e43e`, jaxls `50a58be`, JAX/jaxlib 0.6.2 (the JetPack 6 version in jetson-containers).
- Python 3.10, in its own venv (`.venv-pyroki`).

**Hardware.**
- One Jetson Orin NX 16 GB, as for v2.
- GPU runs are primary, with no other GPU work, as for cuRobo.
- CPU runs (`JAX_PLATFORMS=cpu`) are reported separately, with the board's load at the time.

**Robot, world and gate (identical to v2).**
- **Robot:** the same URDF (`configs/ur5e_rail_spherized.urdf`), the same collision spheres and self-collision ignore pairs (`configs/ur5e_rail_shared_v2.yml`), and the shared limits (`configs/v2_limits.yaml`).
- **World:** obstacles are passed to PyRoki as its analytic `Box` primitives, with the same poses and extents.
- **What decides validity:** PyRoki's own collision terms only guide its optimiser. Validity is decided by the v2 1 kHz gate: the shared VAMP collision check, joint bounds, and velocity, acceleration and jerk ≤ 1.01 × the shared limits, with jerk 500 rad/s³ for the UR5e arm joints (cuRobo's `ur5e.yml`; UR publishes none).
- **Budget:** T = 30 s, including retries.
- **Endpoints:** PyRoki holds its endpoints by augmented-Lagrangian constraints, which converge only to a tolerance, so a PyRoki run must also start and end within 1e-3 rad (1e-3 m on the rail) of the given configurations. v2's gate has no endpoint check: VAMP's path ends at the goal exactly, and cuRobo reports success only within its own goal tolerance.

**Settings, fixed before any benchmark run.**
- PyRoki's own example defaults are used: `01_basic_ik` for IK, and `07_trajopt` for trajectory optimisation (50 knots; smoothness and acceleration costs; self-collision margin 0.02 m; world-collision margin 0.05 m; joint-limit and velocity-limit constraints).
- **Three changes, all from v2's fairness rules:**
  1. Endpoints are joint configurations (the A6 start and goal), constrained exactly, because the other planners get joint goals.
  2. The shared acceleration and jerk limits are added as constraints, using PyRoki's own `limit_acceleration_residual` and `limit_jerk_residual`. Its examples constrain velocity only, and cuRobo is given the same three limits.
  3. Obstacles are padded to a fixed count per set with inert boxes far from the robot, so JAX compiles once per set rather than once per scene.
- **No tuning on benchmark sets:**
  - Any further change is decided on the v1 gantry problems (`problems/gantry_pillars.pkl`), which are not in v2.
  - The CPU smoke test used held-out A6 query 0 to check the plumbing only. No setting was changed after it.

**Horizon (the trajectory duration is fixed per solve).**
- **Duration:** D = 1.5 × the time-optimal duration of the straight joint-space line under the shared velocity, acceleration and jerk limits (1-D Ruckig on the line parameter), computed from start and goal only. Knots are spaced D/49 apart.
- **Retry:** if the 1 kHz gate fails, the solve is repeated with D × 1.5, until the 30 s budget is spent. Retries count toward T_ready and are recorded.

**From knots to the 1 kHz trajectory.**
- **R1 (as returned, primary):** the knots are interpolated with a C² cubic spline with zero end velocities, sampled at 1 kHz, and gated. Nothing else is done to them.
- **R2 (post-processed, secondary):** if R1 fails only on velocity, acceleration or jerk, the same path is uniformly slowed by the smallest factor s that brings the 1 kHz peaks within limits (velocity scales as 1/s, acceleration as 1/s², jerk as 1/s³), then gated again. The time for this counts toward T_ready, and R2 is labelled as post-processed.

**Timing.**
- **Compile time:** JAX traces and compiles once per input shape. Compile time is reported separately, as the first call's time minus the warm median for that shape.
- **Warm T_ready:** measured after one warm-up call per shape, on a query outside the measured set.
- **Cold:** the first-query time (compile included) is reported too, since a cell controller pays it once per start-up.
- **Timer:** wall clock around the solve, with `block_until_ready`, so asynchronous dispatch is not mistaken for speed.

**Measurements.**

| | Inputs | Seeds | Success | Shown alongside |
|---|---|---|---|---|
| **IK** | TCP (`tool0`) pose of each held-out A6 goal, 300 poses | (a) PyRoki's default initial configuration; (b) as many random initial configurations as cuRobo v1's IK solver is given (64 in this repo), solved batched, best returned | position error ≤ 1 mm, rotation error ≤ 1°, within joint bounds, collision-free under the shared check | cuRobo v1 `IKSolver` on the same poses with the same seed count (GPU) |
| **MoveL** | the 480 MoveL segments of the 120 A6 pick slots (pre-grasp → grasp, grasp → pre-grasp, pre-place → place, place → pre-place), from the configurations in `problems/v2_pickplace.pkl` | a straight joint-space line between start and the IK end configuration | the v2 pick-place gate and P1 criterion (TCP deviation median ≤ 1 mm, p95 ≤ 5 mm), the jerk gate per segment | VAMP, cuRobo v1 as configured and v1 at max_jerk 200, cuRoboV2 (from the v2 pick-place runs, same segments) |
| **Free-space trajopt** | held-out A6, 300 queries (MBM and MπNets subsamples if time allows) | (S1) straight joint-space line, PyRoki's own recipe; (S2) the seed trajectories cuRobo v1 gives its trajectory optimiser, attempt by attempt, dumped during a v1 rerun at the same random seed (see amendment 7a), tried in v1's order until one passes the gate or the budget runs out | the v2 gate within 30 s | VAMP and cuRobo v1 success and T_ready per bin |

- **MoveL horizon:** knots follow the TCP profile VAMP's MoveL used (1-D Ruckig on arc length: 0.25 m/s, 1.0 m/s², 10 m/s³), with a pose cost at each knot toward the matching point on the line.
- **Determinism:** PyRoki's solver is deterministic given its seed. It is run once per input, with a re-run of 20 inputs to confirm bit-identical output, so "seeds" means seed trajectories, not random seeds.

**What each result would mean (stated before running).**
- **IK solve rate and warm time vs cuRobo v1:** these answer whether a JAX IK on this Jetson is a usable replacement for cuRobo's IK in these cells.
- **MoveL:**
  - Meeting P1 and the jerk gate on most segments would mean a trajectory optimiser with an exact per-knot pose cost expresses MoveL natively, which cuRobo v1's soft line cost did not do within the jerk limit.
  - Failing the gate under R1 but passing under R2 would mean the knot-to-trajectory step, not the optimiser, decides the outcome.
- **Free-space trajopt:**
  - **S1 success** measures how often local optimisation from a straight line suffices in these cells. It is expected to be lowest in the blocked-passage bins, and that is why PyRoki is not ranked against the global planners.
  - **S2 vs S1** measures how much cuRobo's seeds, rather than its optimiser, explain cuRobo's success.
  - **S2 vs cuRobo v1 on the same seeds** compares the two optimisers directly.
- **Not supported:** anything about global planning with PyRoki, or about hardware other than this Orin NX.

**Amendments (logged before any PyRoki benchmark run).**
- **7a, S2 seeds (2026-10-13):**
  - **What the seed dump showed** (`bench/v2_curobo_seeds.py`): for joint goals, cuRobo v1's `plan_single_js` runs its trajectory optimiser with one seed per attempt, not 12. `num_trajopt_seeds` = 12 applies to pose goals; the joint-space optimiser uses its own count of 1.
  - That seed is the straight joint-space line, except on attempts where v1's graph planner has found a path, which then becomes the seed.
  - **New S2 definition:** the sequence of seeds v1 used, attempt by attempt, solved in the same order. It differs from S1 only on queries where v1's graph planner found a path.
- **7b, GPU build (2026-10-13):**
  - **Build used:** the GPU runs use the PyPI `jax-cuda12-plugin` 0.6.2, with cuDNN 9.8 from pip inside the venv (JetPack 6.2 ships 9.3) and XLA autotuning off. No source build was needed.
  - **Kernel workarounds:** the plugin's precompiled kernels have no sm_87 image. XLA compiles everything else for the device. Two operations therefore run as pure XLA (`bench/pyroki_site/sitecustomize.py`):
    - LU uses JAX's own partial-pivoting lowering instead of cuSOLVER;
    - a batched triangular solve runs as a loop of unbatched cuBLAS solves.
  - **Effect:** the numbers are unaffected to float32 precision (batched-solve residual 3e-7), and the speed may be lower than with a native build.
  - **CPU vs GPU:** IK reaches the same pose on both (error 0.1 µm), with joint solutions up to 0.03 rad apart, since the 7-joint robot is redundant for a 6-D pose. Trajopt outputs can differ widely when the solve does not converge. CPU and GPU results are therefore reported separately, never pooled.
- **7a, seed dump result:** across the 300 held-out A6 queries, v1's outcome at this random seed reproduces the stored v2 record on all 300, and v1 solves 168. On 111 queries at least one of its seeds is a graph-planner path.
- **7c, implementation details (2026-10-13, decided from the inputs, before any PyRoki benchmark run):**
  - **World-collision margin 0 instead of 0.05 m** (a hard constraint on the swept spheres between knots, distance ≥ 0, which is what the gate tests).
    - **Why:** 29% of the MoveL segment endpoints in `problems/v2_pickplace.pkl` are closer than 0.05 m to an obstacle, and so are the held-out A6 endpoints in the clearance bins below 5 cm. With fixed endpoints, a 0.05 m hard margin would make those problems infeasible by construction.
    - The self-collision term stays a soft cost with PyRoki's 0.02 m margin.
  - **MoveL:**
    - the pose cost per knot uses `01_basic_ik`'s weights (position 50, orientation 10);
    - the start is constrained to the given configuration;
    - the end must reach the target TCP pose within 1 mm and 1°;
    - the seed is the straight joint-space line to the generator's end configuration.
  - **S2 resampling and order:**
    - v1's seeds have 28 knots and are resampled to 50 by linear interpolation in time.
    - Consecutive identical seeds are tried once, since PyRoki is deterministic.
    - Each seed's duration is D = 1.5 × max(time-optimal straight-line duration, max over joints of the seed's joint path length / v_max).
    - After the sequence, it repeats with D × 1.5 until the budget is spent.
  - **IK:**
    - (b) returns the lowest-pose-error solution of its 64 random starts (uniform within joint limits, seeded from the freeze hash); joint bounds and collision are then checked with the shared VAMP check.
    - Warm-up targets are pick-place grasp poses, outside the measured set.
  - **Validation:**
    - runs in the PyRoki venv with the same VAMP build (`vamp-planner` 0.6.4 copied from the main venv) and the same `bench/v2_validate.py`;
    - spline derivatives are passed as the sampler's own velocity and acceleration;
    - validation time is reported separately from T_ready, as for VAMP.
  - **Obstacle padding:** 40 boxes for held-out A6 (maximum 38) and 25 for pick-place (maximum 25).
  - **CPU load:** the feasibility runner (1 worker) is not paused during PyRoki runs, so the v2 references are not delayed. The 1-minute load average is recorded with each result.
- **7d, before any held-out run (2026-10-13). Plumbing tests (5 IK targets, 4 MoveL segments, 2 trajopt queries, all discarded) showed four problems:**
  - **Endpoint holds:**
    - **Problem:** with one knot held at each end, the knots next to the endpoints oscillate, because PyRoki's acceleration and jerk stencils do not reach them. The result was 45,000 rad/s³ in the spline.
    - **Fix:** endpoints are now held on two knots each, as in `07_trajopt`. For MoveL, the last two knots are constrained equal.
  - **World-collision margin 0:**
    - **Problem:** a margin of 0 puts the optimum on contact, and the gate's 1 kHz check finds grazing penetrations of about 0.1 mm.
    - **Fix:** the margin is now chosen on the v1 gantry problems (`problems/gantry_pillars.pkl`, not in v2; every 5th query, 21 queries; S1 seeds; GPU) from {0, 5, 10, 20, 50} mm.
    - **Selection rule:** the margin with the most successes under R1; ties go to the larger margin. It is used for trajopt, MoveL and IK (c) (`bench/v2_1_dev_margin.sh`).
  - **IK without collision terms:** PyRoki's basic IK reaches the poses (error around 0.1 µm), but its solutions often collide, while cuRobo's IK is collision-aware.
    - **Added variant (c):** PyRoki's own collision-aware IK (`04_ik_with_coll`: pose weights 5/1, rest 0.01, self-collision 0.02 m soft, world collision hard at the chosen margin), from the same 64 random starts. It returns the lowest-pose-error start among those within tolerance.
    - (a) and (b) are kept as pre-registered.
  - **Slow batched solves:**
    - **Problem:** batched Cholesky also needs the batch-pointer kernel, and a loop of single solves took 5 s per 64-start IK.
    - **Fix:** batched Cholesky and batched triangular solves now use plain-XLA Cholesky–Banachiewicz and substitution, vectorised over the batch. All eight triangular-solve variants and batched Cholesky match NumPy to within 6e-7.
- **7d result, margin choice (gantry dev set):** R1 successes out of 21 were 7 at 0 mm, **15 at 5 mm**, 14 at 10 mm, 5 at 20 mm and 1 at 50 mm (`results/v2_1/dev_margin.json`). The rule picks 5 mm, which is used for every held-out run.
- **7e, MoveL end criterion: unchanged.** The plumbing test missed the 1 mm end criterion by 1.6–2.7 mm, but the criterion is kept: in v2, every other planner's MoveL ends within 0.2 mm of the target.
- **7f, post-hoc (2026-10-13, after the PyRoki MoveL run): R2 applied to every planner's MoveL segments.** Not in the pre-registration. Requested by the orchestrator as a fairness check: R2 was defined for PyRoki only, so PyRoki got a rescue the others did not.
  - **What was known when this was written:**
    - PyRoki's MoveL result: R1 passes the gate on 0 of 480 segments, R2 on 473 of 479.
    - The v2 pick-place jerk table (`docs/V2_PICKPLACE_RESULTS.md`): every MoveL segment of VAMP and cuRoboV2 passes as returned, and every failed MoveL segment of cuRobo v1 as configured fails on jerk alone.
  - **R2 for VAMP, cuRobo v1 and cuRoboV2:**
    - **Rule:** the same as for PyRoki. A segment that fails the 1 kHz gate only on velocity, acceleration or jerk is slowed uniformly by s = max(1, r_v, √r_a, ∛r_j), where the r are the 1 kHz peak ratios to the limits. It then passes if it has no colliding sample and no bound violation.
    - **No re-run:** the v2 runs stored the peak ratios and the collision and bound result of every segment, not the trajectories. A uniform time scaling leaves the path unchanged and divides the peaks by s, s² and s³. So R2 is computed from the stored records.
    - **Check of this shortcut, before it is used:**
      - **cuRobo v1:** the 8 cuRobo v1 MoveL trajectories dumped at max_jerk 500 (`results/v2/diag_jerk/dump_jerk500`) are rescaled, resampled at 1 kHz (qdd interpolated linearly) and run through the full gate, including collision.
      - **PyRoki:** the predicted s is compared with the s recorded for PyRoki's R2.
      - **Failure condition:** if any rescaled dump fails the gate, or the peak after scaling exceeds 1.01, the shortcut is not used. cuRobo v1's MoveL is then re-run with trajectories dumped, in a later GPU slot.
  - **Source runs:**
    - **cuRobo v1 as configured:** the jerk-continue run (`curobo1_pickplace.A6.jerkcontinue.jsonl`), which continued past jerk-only failures. Each segment therefore starts where the slowed-down trajectory would end, since the path is unchanged.
    - **VAMP and cuRoboV2:** their A6 pick-place runs.
  - **Time cost:**
    - **Rescale time:** measured on the dumps and added to T_ready.
    - **Execution time:** grows by (s − 1) × T_exec.
  - **Reported:**
    - **Per planner, in the same columns:** MoveL gate pass and success under R1 and R2, the median and p95 slowdown factor, and the median added execution time per MoveL segment.
    - **Per pick, for the three v2 planners:** picks completed and median cycle time under R1 and R2, with R2 applied to every segment. PyRoki ran MoveL segments only, from the stored start configurations, so it has no per-pick numbers.
  - **Readings:**
    - **v1 completes picks under R2:** the v2 headline "cuRobo v1 as configured completes 0/600 picks" describes its trajectories as returned, and is reported next to the R2 count.
    - **R2 changes nothing for VAMP and cuRoboV2:** s = 1 on every segment.

**Order of work.**
1. CPU smoke test (done: `bench/pyroki_smoke.py`, `results/v2_1/smoke_pyroki.json`).
   - **What it ran:** PyRoki's example recipe with joint-space endpoints and world margin 0, without the acceleration and jerk constraints. It checks the plumbing, not the design.
   - **FK:** PyRoki's FK matches an independent URDF parser (yourdfpy) to 2e-7.
   - **IK:** the goal pose is reached to 0.1 µm. The first call (compile) takes 5.4 s and warm calls 1.6 ms.
   - **Trajopt (64 knots):** compile takes 61 s and a warm solve 16.7 s on the CPU, so CPU runs may hit the 30 s budget. The knots ended 5–7 mm from the endpoints and penetrated an obstacle by 52 mm.
2. Freeze this section and the scripts.
3. IK and MoveL on the CPU.
4. A GPU slot when the GPU is released: the S2 seed dump from cuRobo v1, then PyRoki IK, MoveL and trajopt on the GPU.
