# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: M10 — benchmark GIFs (5 sets + VAMP vs cuRobo v1 side-by-side) in results/media/, embedded in README
Last result: of the paper's comparison claims, 60x is cited from NVIDIA (desktop RTX 4090; NVIDIA reports 21–28x on Jetson AGX Orin), 15–25x OMPL / 8x TrajOpt / CHOMP 92 vs 78% / ABB 40% have no supporting experiment, 3x matches only a MoveIt cycle-time ratio (3.02 s vs 9.90 s). Between-waypoint flags now v1 4/367 UR5 (was 39)
Blocked: none. 62x/50x/12x: not in the paper (it says 60x, cited); origin of the brief's 62/50/12 unknown
Next: Luai decides on BENCHMARK_V2 minimal plan (~7–12 GPU-h on Orin; VAMP part CPU-only, runs on Mac); EMAIL_NOTES review; nothing pushed
Updated: 2026-10-08 15:38
