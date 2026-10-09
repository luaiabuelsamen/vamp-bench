# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-5 — A4 approved+logged; serial VAMP run stopped (in-memory results unrecoverable: ptrace restricted; deterministic, redone in pass a); pass (a) running on 2 workers: MBM 3000/3000 solved, MpiNets running, held-out next (~12 h on 2 workers, ~4 h once 6 cores free)
Last result: TOPP-RA/Ruckig cost measured before freeze: Ruckig P1 0.36 ms p50, TOPP-RA P2 17.6 ms p50 per path (results/v2/timing_cost.json)
Blocked: (1) flagship to signal end of the 256 px re-render -> raise pass (a) to 6 workers (restart resumes from checkpoint).
  (2) Luai: may the A3 reference runs (untimed, ~20+ core-h; one rrtc+PRM pair took >20 min each on the contended CPU) run on his Mac? If not, they run on the Orin after pass (a).
  (3) Tonight's GPU slot: start with pass 2a quiet re-timing of VAMP successes (<=30 min), then cuRoboV2 subsample (3.0 h expected, 4.2 pessimistic, 7.65 ceiling) and pick-place.
Next: VAMP CPU step on held-out/MBM/MpiNets × 10 seeds -> exact GPU-hours under Blocked -> GPU slot (cuRobo v1 full, V2 subsample) -> pick-place MoveL sequence
Updated: 2026-10-09 09:46
