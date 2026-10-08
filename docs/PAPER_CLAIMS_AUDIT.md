# Audit: quantitative claims in arXiv 2508.04146 against committed experiments

**Scope.** This audit covers the paper text in `docs/arxiv_2508.04146_v1.txt`. Note that the file's first line reads `arXiv:2508.04146v2 [cs.RO] 11 Aug 2025`. For each claim it asks whether a committed experiment backs the number, searching:
- `~/projects/curobo_berkeley`: working tree, untracked files and full git history, including deleted files;
- `~/projects/vention_curobo2`: not a git repo; it holds demos and scratch scripts, with no logs or result files.

Line numbers refer to the text file. The paper itself is not modified. Audit date: 2026-10-08.

**Verdicts:**
- **supported:** a committed file contains or reproduces the number.
- **partly supported:** committed data exists, but it differs in value, spread or measurement method, as stated.
- **cited from NVIDIA:** the number comes from the cuRobo paper (Sundaralingam et al., ref. [2]), not from this work.
- **unsupported:** no code, log, plot or commit measures it.

**Data that exists.** The comparison rests on these files:
- **`logs/results_*.json`:** 78 cuRobo MotionGen results, April 2025. These are gitignored, so they exist only locally.
- **Two cycle-time plots and one statistics file:** both plots were committed in `9cebb2f` (2025-05-24). The statistics file has two committed versions.
  - `mujoco_scripts/plots/cycle_times.png`: cuRobo, 20 cycles, average 3.02 s.
  - `src/vention_curobo/vention_curobo/data/cycle_times.png`: MoveIt/OMPL RRTConnect, 20 cycles, average 9.90 s.
  - `.../data/performance_statistics.txt`: MoveIt statistics, at commits `9bbbbb7` and `9cebb2f`.

Nothing in either repo measures TrajOpt, CHOMP, STOMP, ABB RobotStudio, a desktop GPU, power, GPU utilization, or a deployment.

## The six claims requested

| # | Claim (quote, line) | Evidence in the repos | Verdict |
|---|---|---|---|
| 1 | "NVIDIA's cuRobo library achieves an average of 60× faster trajectory generation under 100 milliseconds compared to leading CPU-based methods [2]" (L95–97). Also "cuRobo's 60× performance gains" (L179) | None; cited. See note 1. | **cited from NVIDIA** |
| 2 | "vs. OMPL-RRT variants: 15-25× faster planning with higher success rates" (L512–513) | No committed cuRobo-vs-OMPL planning-time comparison under matched conditions. See note 2. | **unsupported** |
| 3 | "vs. TrajOpt: 8× speedup with comparable trajectory quality" (L514–515) | No TrajOpt code or results anywhere; "trajopt" appears only as cuRobo's internal stage names. | **unsupported** |
| 4 | "vs. CHOMP: Superior convergence in cluttered environments (92% vs 78%)" (L516) | No CHOMP code or results anywhere | **unsupported** |
| 5 | "vs. Commercial solutions (ABB RobotStudio): 40% faster cycle times with better adaptability" (L517–518) | No ABB RobotStudio code or results anywhere | **unsupported** |
| 6 | "This 3× performance improvement" (L361); "The 3× improvement in cycle time compared to traditional planners" (L476–477); "systematic cost function optimization yielding 3× performance improvements over traditional planners" (L527) | The committed cycle-time plots average cuRobo 3.02 s and MoveIt 9.90 s; 9.90 / 3.02 = 3.3. See note 3. | **partly supported** as a cycle-time ratio against MoveIt/OMPL. **Unsupported** as an effect of cost-function optimization (L527). |

**Note 1, the 60× figure.** The cited paper is Sundaralingam et al., arXiv 2310.17274v2. It states "within 50ms on average, 60× faster than state-of-the-art (SOTA) trajectory optimization methods".
- **Baseline:** Tesseract, i.e. OMPL RRTConnect plus TrajOpt.
- **Hardware:** an RTX 4090 with a Ryzen 9 7950X.
- **On a Jetson AGX Orin,** the same paper reports 28× (60 W) and 21× (15 W).

Two wording differences: the paper here says "under 100 milliseconds" and "leading CPU-based methods" where the source says 50 ms and SOTA trajectory optimization. The Jetson figures are the ones that apply to the Jetson setting this paper describes.

**Note 2, OMPL.** The only data that relates cuRobo to OMPL is described below. The ratios it gives (13–29×, depending on which cuRobo statistic is used) fall roughly in the 15–25× range. But they compare different poses, obstacle-free cuRobo runs, and a MoveIt timer that includes execution. No success-rate comparison exists.
- **MoveIt side:** `performance_statistics.txt` at `9bbbbb7` says "Average Planning Time: 10.8650 ± 13.2778 s" (20 cycles). The timer ran from sending the goal to receiving the result, so it includes execution.
- **The current version** (`9cebb2f`) says 0.0031 ± 0.0020 s, which times only goal acceptance.
- **cuRobo side:** the logs (local only) have a median `total_time` of 0.683 s on an obstacle-free scene, with different poses.

For context, the matched same-machine benchmark in this repo (`results/summary.json`) measures cuRobo v1 against VAMP, not OMPL.

**Note 3, the 3× ratio.**
- **cuRobo plot** (`profiling_script.py` scene): average 3.02 s, bars about 2.7–4.2 s. The paper reports 3.1 ± 0.15 s (L348). The ± value is tighter than the plotted spread.
- **MoveIt plot and file:** average 9.90 s; the file says 9.8986 ± 4.3701 s. The paper reports 9.9 ± 2.3 s (L351). The mean matches, the spread does not.
- **Failures in the MoveIt data:** failed plans are recorded as 0 s, and cycles 19–20 plot at about 0.9 s and 0 s. Including failures as zeros lowers the MoveIt average. No file counts successes.
- **Method:** the two scripts use different start/goal poses.
- **Provenance of the cuRobo plot:** the line that would fill `cycle_times` in the committed `profiling_script.py` is commented out, so that plot came from an earlier, uncommitted version of the script.
- **L527:** no committed experiment links cost-function weights to cycle time.

## Other quantitative claims

| Claim (line) | Evidence | Verdict |
|---|---|---|
| Parallel RRT "5-10× speedups" (L179) | none | unsupported (no citation given) |
| "512 parallel trajectories with 64 timesteps each at 500Hz"; CPU "10-50 trajectory evaluations per second" (L193–195); planner "operating at 500Hz" (L315–317) | no such configuration; scripts use `interpolation_dt=0.01` (100 Hz commands) | unsupported |
| "approximately 60-70% GPU utilization", "2-6GB" memory (L235–239) | no GPU or memory measurement in any script | unsupported |
| Jetson "75% of desktop performance while consuming 25W versus 400W"; "8-hour continuous operation" (L240–244) | no desktop-GPU, power or long-run data | unsupported |
| Circular buffer "reducing allocation overhead by 40%" (L253–254) | no buffer implementation found | unsupported |
| "sub-5ms communication latency" via shared memory (L255–258) | communication is plain ROS 2 topics; no latency measurement | unsupported |
| Automated sphere packing, "25%" fewer spheres, "99.9%" accuracy (L259–262) | sphere sets are hand-written YAML (52 / 156 / 352 `radius` entries across three configs); no packing algorithm or accuracy test | unsupported |
| Integrated gantry model "15% faster planning", world-obstacle model "30% reduced memory" (L282–285) | both configurations exist; no timing or memory comparison | unsupported |
| cuRobo "3.1 ± 0.15 seconds", "98.5% success"; MoveIt "9.9 ± 2.3 seconds", "4 to 16 seconds", "87.2%" (L348–352) | means: see note 3. The MoveIt range of about 0.9–16.4 s is consistent with "4 to 16" if the failed cycles are excluded. No success counts exist. | partly supported (means); unsupported (success rates, spreads) |
| Planning time "cuRobo: 45 ± 8ms, MoveIt: 1,200 ± 400ms" (L354–355) | committed cuRobo plot: pick 0.63 s, place 0.62 s average (`result.total_time`, gantry scene). MoveIt file: 10.865 s (`9bbbbb7`) or 0.0031 s (`9cebb2f`); neither is planning-only. This repo's same-board benchmark measures cuRobo v1 medians of 145–849 ms. | unsupported; committed data shows different values |
| Peak jerk "2.1 rad/s³" vs "5.8" (L356–357) | jerk arrays are stored in the local logs; no peak-jerk computation or MoveIt jerk data | unsupported |
| "cuRobo paths 12% shorter on average" (L358) | no path-length analysis | unsupported |
| "> 50mm clearance", MoveIt "20-80mm" (L359–360) | no clearance analysis | unsupported |
| Table II weight sweep: position 2,000–50,000, optimum 12,000–25,000; orientation 1,600–2,000; "Rotation Error = 8.53E-07" and "6.48E-06" (L365–431) | the local logs record no weight values (`util/weights_compare.py` plots errors against file time only). Both quoted errors appear in `logs/results_20250413_164113.json`, `home_to_pick`: `position_error` 8.534e-07 and `rotation_error` 6.479e-06. The first is a position error, not a rotation error, and identical values recur across the April 13–15 runs. | unsupported (sweep); the two error values exist in an uncommitted log, with one labelled differently |
| Deployment on three manufacturing lines: "28-35%" cycle time, "95%" success at ±10 mm, "60%" setup time, "Zero ... failures over 240 hours", scrap "18%" (L433–449) | no deployment data in either repo | unsupported |
| "planning times under 100 milliseconds even with the additional DOF", "up to 12 DOF tested (dual-arm + gantry configurations)" (L456–459) | only 7-DOF configs exist. Committed and logged cuRobo planning times are about 0.4–0.9 s. | unsupported; committed data shows higher values |
| Narrow passages "3% of complex scenarios", "0.5 m/s", "¿500 collision objects" near "8GB" (L466–472) | none | unsupported |
| MPC "replanning latencies under 50 milliseconds" (L506) | `src/curobo_api/curobo_api/curobo.py` collects an `mpc_time` list (commit `6736df0`) but never prints or saves it | unsupported |
| Hardware "1024 CUDA cores with 8GB unified memory" (L233–234) | hardware description, not an experiment. jetson3 is an Orin NX **16 GB** (`/proc/device-tree/model`, `free`). | not an experimental claim; the memory size differs from this board |

## What can be said with committed data
- **Same-board comparisons from this repo** (`results/summary.json`): cuRobo v1 against VAMP and cuRoboV2 on the paper's gantry and UR5e scenes and on MotionBenchMaker. Identical collision models, per-query records, reproducible.
- **From `curobo_berkeley`'s committed plots:** a 20-cycle MuJoCo pick-and-place with cuRobo averaging 3.02 s per cycle, against MoveIt/OMPL RRTConnect averaging 9.90 s. Different poses, and failed MoveIt cycles counted as 0 s.
