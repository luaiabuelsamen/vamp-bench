# STATUS — vamp (VAMP vs cuRobo benchmark)
Goal: Honest, reproducible VAMP vs cuRobo comparison on the cuRobo paper's problems (UR5e, 7th-axis gantry) plus VAMP's standard benchmarks; discharges the 2025-09-03 promise to Prof. Kavraki.
Finish line: results/*.json + one figure + README headline numbers + docs/EMAIL_NOTES.md + speedup-number (62x/50x/12x) reconciliation, all committed locally.
Deadline: none set (brief: $0, highest relationship value)
Milestone: V2-15 — v2 finalisation waits on 21 reference runs; v2.1 PyRoki held-out A6 GPU session running (IK a/b/c, MoveL 480, trajopt S1/S2; margin 5 mm chosen on the gantry dev set; amendments 7a-7e logged). GPU: held until the session ends (deadline 08:40, GPU_RELEASED on exit)
Last result v2.1 setup: PyRoki runs on the Jetson GPU via the PyPI JAX plugin with workarounds (no source build); cuRobo v1 seed dump reproduces all 300 stored outcomes, and shows v1 js planning uses 1 trajopt seed per attempt. v2 results unchanged (see docs/V2_RESULTS.md)
Blocked: nothing. CPU: feasibility runner at 1 worker; MemAvailable gate 4 GB for new CPU batches while flagship trains (not training now). Disk: keep >= 20 GB free (32 GB now)
  Memory (standing rule 2026-10-11): during a flagship SmolVLA eval (train_local.py --skip-train ... smolvla) the A3 runner runs at most 1 worker and starts nothing unless MemAvailable > 2.5 GB (built into bench/v2_feasibility_par.py); otherwise 2 workers, >2 only after 10 min sustained > 4 GB. PRM references grow to ~1.5 GB, so under the eval most are floor-killed and requeued (12 so far): the A3 runs effectively wait for the eval to end.
  CPU: A3 references for the original held-out (appendix): 18 queries left, 1 worker. Held-out A6 (2) + MpiNets (1) references queued after it
Next: v2 first when references finish (rerun v2_analyze + v2_report -> README / EMAIL_NOTES v2, held-out description fix, check_consistency, CHANGELOG; no email). v2.1: seed dump -> freeze section 7 + PyRoki run scripts -> IK, MoveL, trajopt S1/S2 on GPU (CPU subsample separately) -> touch GPU_RELEASED by 09:00
Updated: 2026-10-13 16:58
