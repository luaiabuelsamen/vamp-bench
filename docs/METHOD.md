# Method

## Hardware and software (both planners, same machine)
- **Board:** NVIDIA Jetson Orin NX 16 GB Engineering Reference Developer Kit (`jetson3`). L4T R36.4.3, JetPack 6, CUDA 12.6, nvpmodel mode 0 (MAXN).
- **CPU:** 8× Arm Cortex-A78AE at up to 1.984 GHz, with NEON (128-bit). **GPU:** 1024-core Ampere (Orin NX).
- **VAMP:** KavrakiLab/vamp `27cb9b6` (v0.6.4), built from source with GCC 11.4 using `-O3 -mcpu=native` in an isolated venv (`.venv`, Python 3.10). VAMP is single-threaded, so it runs on one core.
  - The two extra robot modules (`ur5e`, `ur5erail`) were generated with CoMMALab/cricket `fb2d081` (see below).
- **cuRobo:** NVlabs/curobo `ebb7170` (v0.7.7-5, the version the paper used), torch `2.5.0a0+872d972e41.nv24.08`. It uses CUDA graphs and is warmed up before timing.
  - How it was restored on jetson3: `docs/CUROBO_JETSON_FIX.md`.
- **Shared machine:** the board is shared with other jobs. Every run is wrapped in `bench/powerlog.py`, which samples `tegrastats` (GPU load, board power, temperatures) and records the top processes at start. These logs are in `results/raw/*.power.json`, and any contention is stated in the README.

## Same problem for both planners
- **Same robot geometry.** Both planners check collisions against exactly the same set of spheres, with zero padding, and ignore exactly the same self-collision pairs.
  - **MBM robots (Panda, UR5):** `bench/spherized_to_curobo.py` turns VAMP's `*_spherized.urdf` and SRDF into a cuRobo config.
  - **Paper robots (UR5e, UR5e on a rail):** `bench/curobo_to_spherized.py` turns the paper's cuRobo sphere configs (`curobo_berkeley/config/curobo/self_collision/ur5e_x.yml`, and cuRobo's stock `ur5e.yml`) into a spherized URDF + SRDF. Cricket then generates a VAMP robot from those files.
  - Link pairs on the same rigid body are excluded for both planners, because cricket/pinocchio never checks them.
- **Same obstacles** (`bench/mbm_world.py`).
  - Cuboids are the same in both planners.
  - MBM cylinders are capsules in VAMP (`vamp.problem_dict_to_vamp`), so they are capsule meshes in cuRobo. The exception is the MBM `box` scene, where VAMP uses cuboids for them, and so does cuRobo.
  - cuRobo v1 has no analytic capsule obstacle, so capsules go through its mesh checker as triangle meshes.
- **Same query.** Both planners get a joint-space start and a joint-space goal.
  - In the paper scenes, the goal comes from cuRobo's collision-free IK of a pick/place pose (`bench/make_paper_problems.py`, seed 0). IK time is not counted for either planner.
- **Cross-check.** Every cuRobo success is re-checked segment by segment with VAMP's `validate_motion` on the shared model (`bench/crossval.py`).

## Planner settings
- **VAMP:** `rrtc` with its default settings, the Halton sampler, then its default `simplify`. These are the settings `scripts/evaluate_mbm.py` uses.
- **cuRobo:** `MotionGen.plan_single_js`, MESH collision checker, CUDA graphs, 12 trajopt/graph seeds. Plan config: `max_attempts=20`, graph enabled from attempt 1, `timeout=60 s`, finetune on. This matches cuRobo's own `benchmark/curobo_benchmark.py`.
  - Joint limits are left as given; `position_limit_clip` is set to 0.

## Metrics
- **Success:** the planner reports a solution. cuRobo successes are additionally re-checked with VAMP's checker.
- **Time:** wall-clock per query, measured from Python on the same board.
  - VAMP: environment build + plan + simplify.
  - cuRobo: `update_world` + `plan_single_js`, with CUDA synchronization before and after.
  - Each planner's own internal timer is also kept (`internal_ms`).
- **Path length:** the sum of joint-space L2 lengths of the returned piecewise-linear path. For VAMP this is the simplified path; for cuRobo it is the interpolated trajectory.
  - The outputs differ in kind: cuRobo returns a time-parameterized, smooth, jerk-limited trajectory, while VAMP returns a geometric path that still needs time parameterization.

## Paper scenes and deviations
- **`gantry_pillars`** (`mujoco_scripts/profiling_script.py`):
  - Robot: `ur5e_x` (UR5e on a prismatic `base_x` rail).
  - Rail limit: ±1.0 m, matching the MuJoCo/real rail (`range="-1 1"`). The cuRobo config's ±10 m was not used.
  - Obstacles: the floor plus pillars `col_1`/`col_2` (0.14 × 0.14 × 0.7 m at y = ±0.5).
  - The mesh obstacles `bs_link`/`linear_rail` are left out because the original script never actually loaded them. Its meshes go into the dict after `WorldConfig` is built, and its `mesh_paths` is a list, so the membership test always fails.
  - **Pick pose moved up 3 cm:** the paper's pick pose z = −0.03 m has no collision-free IK under the paper's own sphere model (the tool spheres hit the floor top at z = −0.04 m). It is reachable from z = 0.0, so the "exact" query uses z = 0.0.
- **`gantry_free`** (`src/vention_curobo/vention_curobo/pick_and_place.py`):
  - Same robot, no obstacles; this matches the paper's committed timing logs.
  - Those logs used the 2F-140 gripper variant. We use the arm-only `ur5e_x`.
- **`ur5e_pillars`** (`config/demos.yml: ur5e_with_suction`, `assets/object/collision.xml`):
  - Fixed-base UR5e, table, pillars at y = ±0.5.
  - The paper does not fix pick/place poses for this scene, so we chose (−0.45, −0.45, 0.10) and (0.45, 0.45, 0.20), facing down. The paper's gantry poses (y = ±0.7) are out of reach without the rail.
- **Each scene has two kinds of query:**
  - 3 exact queries (home → pick, pick → place, place → pick).
  - 100 randomized pick → place transfers: ±0.10–0.15 m position jitter and ±90° yaw around the paper's poses.
