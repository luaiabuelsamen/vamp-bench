# Benchmark v2: pre-registration

This document is committed in the **freeze commit**, together with:
- the generator `bench/gen_v2.py` and the bins in `bench/v2_common.py`;
- the shared limits `configs/v2_limits.yaml`;
- the planner and post-processing settings `configs/v2_planners.yaml`;
- the time-parameterization code `bench/v2_timing.py`.

All seeds are derived from that commit's hash `H` with `seed_from(H, label)` (sha256):
- held-out problems: label `gen_v2`;
- subset selection: `select_mbm`, `select_mpinets`;
- planner seeds: `seed0`…`seed9`.

The hash is recorded in `docs/V2_FREEZE.md` in the next commit. Nothing listed above may change after the freeze. A change voids the affected results, and the change is logged in CHANGELOG.

## Problem sets
| Set | Size | Selection |
|---|---|---|
| Held-out industrial cells, UR5e + rail | 300 | `gen_v2.py --freeze H` |
| MotionBenchMaker Panda (VAMP's home set) | 300 | 7 families × 43 (300 kept), uniform within family, seed `select_mbm`, from problems valid under the shared model |
| MπNets Panda (robometrics `mpinets_set.yaml`; cuRobo's home set) | 300 | 12 families × 25, seed `select_mpinets`. The goal is the first `goal_ik` entry that is valid under the shared model; start and goal must be valid. |

- **Shared model:** VAMP's spheres, as in v1 (16,000/16,000 agreement with cuRobo v1 and V2).
- **Generator tuning before the freeze:** only on geometry statistics from the development seed 12345, never on planner outputs. Log: slots moved inward, IK fixes the rail near the target, higher targets, more and narrower windows, passage labels merged. Each change was made to populate the pre-defined bins. The dev-seed occupancy at n = 60 after tuning was: reach <0.5: 1, 0.5–0.8: 41, >0.8: 18; clearance <1 cm: 33, 1–5 cm: 22, >5 cm: 5; passage free: 3, blocked-wide: 55, blocked-narrow: 2; limits <0.2 rad: 31, ≥0.2 rad: 29. Further balancing (quota sampling) was not done because of its cost (~3.6 CPU-h for the rarest bin). Rare bins are reported as underpowered.

## Metrics: primary and secondary
- **Primary: T_cycle = T_ready + T_exec**
  - **T_ready** = plan wall time + time-parameterization time.
  - **T_exec** = trajectory duration.
  - For VAMP, the parameterization is **P1** (Ruckig, phase-synchronized, rest-to-rest per segment).
  - For cuRobo, it is the trajectory exactly as returned.
- **Validity gate:** a run counts as solved only if its timed trajectory passes 1 kHz validation:
  - collision-free under the shared model;
  - joint limits respected (1e-5 rad tolerance);
  - velocity, acceleration and jerk within the shared limits (1% tolerance).
- **Secondary metrics:**
  - T_ready alone, and T_exec alone;
  - P2 (TOPP-RA) T_exec and its validity rate;
  - joint L2 and TCP path length;
  - RMS and peak jerk, and integrated squared jerk;
  - minimum clearance (100 Hz) and time within 1 cm;
  - minimum joint-limit margin;
  - TCP tilt from vertical during held-out transfers.
- **Statistics:**
  - 10 seeds per query; failures count as failures, at T_cycle = 60 s in worst-case tables;
  - per query: median, p95 and worst over seeds;
  - per bin: median of per-query medians, with a 95% bootstrap CI (2000 resamples over queries);
  - bins with fewer than 30 queries are reported but marked *underpowered*.

- **The 1 kHz gate applies to every planner.** VAMP validates motions at a discrete resolution, so a VAMP+P1 run can fail it, just as cuRobo can between its samples.
- **VAMP failure cost:** VAMP's default cap of 1M iterations means a failed plan takes up to about 55 s here. It is kept, being close to the 60 s worst-case timeout.
- **Pinned inputs:** `third_party/robometrics/mpinets_set.yaml` (sha256 `9189186d…`), `third_party/vamp/resources/panda/problems.pkl` (sha256 `737640df…`). Both are checked by `bench/v2_select.py`.
- **Code in the freeze:** `bench/gen_v2.py`, `v2_common.py`, `v2_timing.py`, `v2_validate.py`, `v2_select.py`, `v2_vamp.py`, `v2_cost_probe.py`, plus the configs and this document.

## Hypotheses and how each result will be read
1. **H1, latency:** VAMP's T_ready is lower than cuRobo v1's in every bin of every set.
   - **Read as** "VAMP is faster to an executable trajectory" only if it holds for the held-out and MπNets sets, not just MBM.
   - A bin where cuRobo is faster is reported as such.
2. **H2, cycle:** the T_cycle difference is smaller in relative terms than the T_ready difference, because T_exec (seconds) dominates both.
   - **Read as:** if cuRobo's T_exec is shorter by more than VAMP's T_ready advantage, cuRobo wins on cycle time. That is an industrially relevant result and is stated as such.
3. **H3, quality:** cuRobo's trajectories have lower jerk and shorter T_exec than VAMP+P1, because P1 stops at every waypoint.
   - If VAMP+P2 closes the gap, that is reported, together with P2's validity rate.
   - Clearance differences are reported, with no prediction made.
4. **H4, robustness:** the worst case over seeds is reported per planner.
   - The planner with fewer "fails on some seeds" queries is called more robust **on that set**.
5. **H5, generality:** a claim generalizes only if its sign is the same on all three sets.
   - If MBM and MπNets disagree, the README states which home set favours which planner.
6. **V2 vs v1:** cuRoboV2 runs a stratified subsample of 300 queries × 3 seeds with `max_attempts=100`. The subsample is 100 per set, round-robin over occupied (reach, clearance, passage, limits) bin combinations, seed `select_v2`.
   - Its result is compared with cuRobo v1 on the same queries only.

## Compute (GPU parts run only in scheduled slots)
- **CPU (now):** VAMP, P1, P2 and validation on 900 queries × 10 seeds.
- **GPU:** cuRobo v1 on 900 × 10, then cuRoboV2 on 300 × 3. The exact GPU-hour figure goes in STATUS.md under Blocked once the CPU step has measured the query mix.
- **Afterwards:** the pick-place sequence with MoveL segments (separate pre-registration, before it is run).

## Amendments (logged before the affected data was used by any planner)
- **A1 (2026-10-08, MπNets data preparation):** every MπNets problem was invalid under the shared model, because VAMP's `panda_link0` spheres intersect the robot stand (`cube_robot_stand`) and the table piece under the base in every configuration.
  - Obstacles whose axis-aligned box overlaps the fixed base region (|x|, |y| < 0.15 m, z < 0.05 m) are removed, for all planners, before validity filtering (`bench/v2_select.py: touches_fixed_base`).
  - The removed names are recorded per problem (`removed_base_obstacles`).
  - On the first 40 problems per family, 27–40 of 40 become valid. Tabletop families are lowest, consistent with VAMP's note on finger separation.
- **A2 (2026-10-08, decided by the orchestrator, before any GPU run): one uniform per-query wall budget T = 30 s for every planner.**
  - Success means a valid trajectory with T_ready ≤ T; a timeout is a failure and enters worst-case tables at T, replacing the 60 s above.
  - cuRoboV2 keeps NVIDIA's `max_attempts=100`. Its attempt loop is unrolled so the budget can be checked before each attempt. cuRobo v1 uses its native `timeout = T`. VAMP is censored at T, which is exact because its runs within T are unaffected.
  - T was chosen from measured slot data so the V2 subsample fits ~5 GPU-h pessimistically: 4.2 h, with 3.0 h expected and 7.65 h if every run timed out. T is above the slowest observed V2 success (19.7 s). Details: `results/v2/budget_choice.json`, `bench/v2_budget.py`.
  - **Added metric:** success-vs-budget curves, the share of runs solved with T_ready ≤ t for t from 1 ms to T, per planner and set. These remove the dependence on any single T.
- **A3 (2026-10-09, requested by the orchestrator; logged before any feasibility rule was applied to results): infeasible queries.**
  - **Problem:** start/goal validity only shows the endpoints are collision-free, not that a path exists. Held-out success is low for every planner so far (VAMP ~52 percent of runs; cuRobo v1 solves 57/300 queries in at least 1 seed).
  - **Rule, applied identically to all planners and all sets:**
    - **Certified feasible:** a valid trajectory was found by any timed v2 run of any planner (any seed, any time), or by a long-budget reference run. The reference is VAMP rrtc with `max_iterations = max_samples = 6e6` (~300 s), then VAMP PRM with the same caps, seed0, untimed.
    - **Unresolved (presumed infeasible):** all of the above failed.
  - **Reporting:**
    - Primary success rates use certified-feasible queries as the denominator.
    - Raw rates over all queries are also reported.
    - The count of unresolved queries is reported per set and per bin.
  - **Status of reference runs:** they run for every query that no timed run solved, once every planner's v2 runs are complete, so a query is never removed because one planner failed it. A sample is already running (`bench/v2_feasibility.py`).
- **A4 (proposed and APPROVED by the orchestrator 2026-10-09, logged before it was applied): two-pass VAMP execution.**
  - **Profile** (`results/v2/profile.json`, 20 held-out queries x 2 seeds):
    - 98.1 percent of CPU time is failed planning (mean 54.7 s per failure: VAMP runs to its 1M-iteration default although results are censored at T = 30 s);
    - successful plans take 8 ms;
    - validation is 1.6 percent;
    - TOPP-RA is 0.1 percent.
    - The current run is also timed under heavy CPU contention from other jobs (~7 of 8 cores busy).
  - **Pass 1, classification (parallel):** every run in a worker process killed at 2T = 60 s; records which runs succeed and their paths. Deterministic: a fixed Halton skip gives the same path.
  - **Pass 2a, timing (sequential, one process, quiet machine):**
    - Every run that succeeded in pass 1 is re-run and timed. Its timed result is what counts, so success still means T_ready <= T measured without contention.
    - A run that hit the 60 s kill in pass 1 is a failure. Under the measured contention slowdown, it would also exceed 30 s alone.
    - The slowdown is verified on a sample (each run alone vs in parallel); if it exceeds 2x, the kill is raised accordingly.
  - **Pass 2b, validation and metrics (parallel, untimed).**
  - **A4 execution notes:**
    - The serial run was stopped at held-out query ~151/300. Its records lived only in memory (no checkpointing), and ptrace is restricted on this machine, so they could not be recovered. Because the runs are seeded and deterministic, pass (a) redoes them with identical outcomes; the loss is compute time only.
    - The pass (a) runner checkpoints every run (`results/v2/raw/vamp_pass1_<set>.jsonl`).
    - Cores: 2 until the other CPU jobs end, then 6.
    - Pass 2a re-timing runs at the start of the GPU slot, on a quiet CPU, before cuRoboV2.
