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
