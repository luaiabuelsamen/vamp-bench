# cuRobo speedup numbers: which one is supported?

Short answer: **none of 62x, 50x or 12x is supported by a committed results file.** No file on jetson3 states any of them as a speedup for arXiv 2508.04146 (searched: `~/projects/**`, `~/luai`, the website, the survey and its git history, VS Code history). The only cuRobo-vs-baseline numbers that can be computed from data on disk give a ratio of about 13x to 29x, and that comparison is not like-for-like (below). The published paper itself claims different numbers (15–25x, 8x), and those aren't backed by any file either. **Update 2026-10-08:** the paper text is now in `docs/arxiv_2508.04146_v1.txt`. It says 60× (L95, L179, cited to NVIDIA [2]); it does not contain 62×, 50× or 12×. The per-claim audit is in `docs/PAPER_CLAIMS_AUDIT.md`.

## What the published paper says (arXiv 2508.04146 v1 and v2, pdftotext)
| Claim in the PDF | Supporting file on disk |
|---|---|
| "vs. OMPL-RRT variants: 15-25× faster planning" | none |
| "vs. TrajOpt: 8× speedup" | none |
| "98.5% success … (87.2% due to planning failures in cluttered scenes)" | none |
| "cuRobo paths 12% shorter on average" | none (the logs have no path-length field) |
| "15% faster planning … 30% reduced memory", "28-35% cycle-time reduction", "60% reduction in programming time" | none |
| "Jetson platform achieves 75% of desktop performance" | none (no desktop run is recorded) |

## Numbers that do appear, and where they come from
| Number | Where | What it actually is |
|---|---|---|
| 60x, 50 ms | Sundaralingam et al. 2023, cuRobo paper (arXiv 2310.17274) abstract | NVIDIA's own claim: cuRobo vs. state-of-the-art trajectory optimization on desktop GPUs. It is not a result of this paper. |
| 50x | `~/projects/luai/MotionPlanning/BRUTAL_TRUTH.txt:28`, `src/movel_optimizer_gpu.hpp:7`, `tests/test.cpp:276` ("Expected speedup: 50-100x") | An unbenchmarked expectation in a different project (a C++ GPU IK prototype). |
| 12x | `~/projects/luai/MotionPlanning/BUILD_SUCCESS.md:107` ("12x smaller than full cuRobo") | Binary size, not speed. |
| 61x, 12x, 10x | cuRoboV2 abstract (NVIDIA, arXiv 2603.05493) | "up to 61x speedup" is for V2's whole-body kernels (kinematics, inverse dynamics, self-collision), "12x lower cross-seed variance than GMR" is about retargeting, and "10x faster" is about ESDF perception. None of them is a planning-time speedup over a planner baseline. |
| 62x | not found anywhere on jetson3 | Probably a conflation of cuRobo v1's "60x" or V2's "61x"; no file says 62x. |

## The ratios the on-disk data can support
- **cuRobo:** `~/projects/curobo_berkeley/logs/results_*.json`, 77 runs × 2 plans, gantry UR5e + 2F-140, **no obstacles** (`world_config=None` in `pick_and_place.py`). Success: 154/154. Median `solve_time` 0.372 s, median `total_time` 0.683 s, mean `total_time` 0.824 s.
  - These logs are **not committed**: `logs` is in `curobo_berkeley/.gitignore`.
- **MoveIt/OMPL RRTConnect:** `src/vention_curobo/vention_curobo/data/performance_statistics.txt` at commit `9bbbbb7` (2025-04-24): "Average Planning Time: 10.8650 ± 13.2778 s", 20 cycles.
  - The timer runs from sending the goal to getting the result, and execution was not disabled.
  - The current version of that file (`9cebb2f`) says 0.0031 s, which only times goal acceptance. That number is a measurement error.

| cuRobo statistic | Ratio |
|---|---|
| mean total_time | 10.865 / 0.824 = **13.2x** |
| median total_time | 10.865 / 0.683 = **15.9x** |
| mean solve_time | 10.865 / 0.4905 = **22.2x** |
| median solve_time | 10.865 / 0.3725 = **29.2x** |

**Caveats:**
- The poses differ between the two scripts.
- The cuRobo runs have no obstacles.
- The MoveIt time includes execution.
- The cuRobo side is not committed.

**Recommendation:** if a cuRobo-vs-OMPL number is needed, use the measured VAMP comparison in this repo (`results/`). VAMP is a stronger and more relevant CPU baseline than MoveIt's default RRTConnect. Retire 62x/50x/12x and the PDF's 15–25x/8x claims.
