# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: M9 — paper claims audited (docs/PAPER_CLAIMS_AUDIT.md); v1 UR5 self-collision check done (35 flags were a crossval bounds-rounding artifact, fixed)
Last result: of the paper's comparison claims, 60x is cited from NVIDIA (desktop RTX 4090; NVIDIA reports 21–28x on Jetson AGX Orin), 15–25x OMPL / 8x TrajOpt / CHOMP 92 vs 78% / ABB 40% have no supporting experiment, 3x matches only a MoveIt cycle-time ratio (3.02 s vs 9.90 s). Between-waypoint flags now v1 4/367 UR5 (was 39)
Blocked: none. 62x/50x/12x: not in the paper (it says 60x, cited); origin of the brief's 62/50/12 unknown
Next: Luai reviews PAPER_CLAIMS_AUDIT + EMAIL_NOTES before writing to Kavraki; nothing pushed
Updated: 2026-10-08 00:53
