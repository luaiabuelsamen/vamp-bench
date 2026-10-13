"""Pick-place analysis (docs/V2_PICKPLACE_PREREGISTRATION.md: P1-P3, A6 primary, A7 jerk runs).

Runs (all on the A6 cells, 30 cells x 4 picks x 5 seeds = 600 picks):
  vamp      VAMP (CPU, contended; load log next to the results)
  curobo1   cuRobo v1 as configured (max_jerk 500), primary
  curobo1_J1b  v1 with max_jerk 200: the pre-registered positive control (A7 addendum)
  curobo1_J1   v1 with max_jerk 250: sensitivity (A7)
  curobo1_J2   v1 as configured, picks continue past jerk-only failures: post-hoc no-jerk count (A7)
  curobo2   cuRoboV2 as configured
Appendix: the A5 (unwrapped goals) cuRobo runs, counts only.
Gate (every planner, every segment): no collision at 1 kHz, joint bounds, |qd|, |qdd|, |jerk| <= 1.01 x limit, plan time
<= T = 30 s. Jerk limit 500 rad/s^3 per arm joint: cuRobo's shipped ur5e.yml value (configs/v2_limits.yaml); Universal
Robots publishes no jerk limit for the UR5e. VAMP's per-segment jerk ratios come from the untimed replay
(bench/v2_pickplace_vamp_jerk.py), which reproduces every timed segment.
Writes results/v2/pickplace_summary.json and results/v2/pickplace_jerk.png.
"""
import json
from collections import Counter
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]; RAW = ROOT / "results/v2/raw"
T_BUDGET = 30.0; TOL = 1.01; JERK_LIMIT = 500.0; TCP_V_SPEC = 0.25
RUNS = {"vamp": "vamp_pickplace.A6.jsonl", "curobo1": "curobo1_pickplace.A6.jsonl",
        "curobo1_J1b": "curobo1_pickplace.A6.jerk200.jsonl", "curobo1_J1": "curobo1_pickplace.A6.jerk250.jsonl",
        "curobo1_J2": "curobo1_pickplace.A6.jerkcontinue.jsonl", "curobo2": "curobo2_pickplace.A6.jsonl"}
ROLE = {"vamp": "primary", "curobo1": "primary (as configured, max_jerk 500)",
        "curobo1_J1b": "pre-registered positive control (max_jerk 200)", "curobo1_J1": "sensitivity (max_jerk 250)",
        "curobo1_J2": "post-hoc no-jerk count (as configured; picks continue past jerk-only failures)", "curobo2": "primary (as configured)"}
APPENDIX = {"curobo1": "curobo1_pickplace.A5_unwrapped.jsonl", "curobo2": "curobo2_pickplace.A5_unwrapped.jsonl"}


def load(f):
    return [json.loads(l) for l in open(RAW / f)]


def q(x, p):
    return float(np.percentile(x, p)) if len(x) else None


def med(x):
    return float(np.median(x)) if len(x) else None


def attach_vamp_jerk(recs):
    """Copy the replayed 1 kHz validation values onto the VAMP segments (every timed segment must be reproduced)."""
    rep = {(r["cell"], r["pick"], r["seed_k"], r["seg"]): r for r in load("vamp_pickplace.A6.jerk.jsonl")}
    n = 0
    for p in recs:
        for i, s in enumerate(p["segments"]):
            if s["kind"] == "not_attempted":
                continue
            r = rep[(p["cell"], p["pick"], p["seed_k"], i)]
            assert r["replay_matches"], (p["cell"], p["pick"], p["seed_k"], i)
            if r["max_jerk_ratio"] is not None:
                for k in ("colliding_samples", "bound_violation_rad", "max_vel_ratio", "max_acc_ratio", "max_jerk_ratio"):
                    s.setdefault(k, r[k])
            n += 1
    return n


def failure(s):
    """First reason a segment fails the gate, in a fixed order."""
    if s.get("reason") in ("planner failed", "not solved", "line IK failed", "ruckig"):
        return "no trajectory (" + s["reason"] + ")"
    if s.get("t_ready_s", 0) > T_BUDGET:
        return "over budget"
    if s.get("colliding_samples"):
        return "collision at 1 kHz"
    if s.get("bound_violation_rad", 0) > 1e-5:
        return "joint bounds"
    if max(s.get("max_vel_ratio", 0), s.get("max_acc_ratio", 0)) > TOL:
        return "velocity/acceleration"
    if s.get("max_jerk_ratio", 0) > TOL:
        return "jerk only"
    return "other"


def jerk_stats(segs):
    j = np.array([s["max_jerk_ratio"] for s in segs if s.get("max_jerk_ratio") is not None])
    return {"segments": int(len(j)), "within_limit_share": float((j <= TOL).mean()) if len(j) else None,
            "ratio_median": med(j), "ratio_p95": q(j, 95), "ratio_max": float(j.max()) if len(j) else None}


def run_summary(name, recs):
    picks = len(recs); succ = [p for p in recs if p["success"]]
    first_fail = Counter()
    for p in recs:
        for s in p["segments"]:
            if s["kind"] != "not_attempted" and not s["ok"]:
                first_fail[f"{s['kind']}: {failure(s)}"] += 1; break
    segs = {k: [s for p in recs for s in p["segments"] if s["kind"] == k and "t_exec_s" in s] for k in ("free", "moveL")}
    ml = segs["moveL"]
    by_pick = {}
    for p in recs:
        by_pick.setdefault((p["cell"], p["pick"]), []).append(p["success"])
    S = {"role": ROLE[name], "picks": picks, "success": len(succ), "success_share": len(succ) / picks,
         "first_failure": dict(first_fail.most_common()),
         # P3: share of the 120 (cell, pick) pairs that fail on at least one of the 5 seeds
         "P3_picks_failing_some_seed": sum(not all(v) for v in by_pick.values()), "P3_pick_pairs": len(by_pick),
         "P3_picks_failing_all_seeds": sum(not any(v) for v in by_pick.values()),
         # P2: cycle time per successful pick, with the planning / execution split
         "P2_t_cycle_s_median": med([p["t_cycle_s"] for p in succ]), "P2_t_cycle_s_p95": q([p["t_cycle_s"] for p in succ], 95),
         "P2_t_ready_s_median": med([p["t_ready_s"] for p in succ]), "P2_t_exec_s_median": med([p["t_exec_s"] for p in succ]),
         # P1: MoveL precision over every MoveL segment that returned a trajectory
         "P1_moveL_segments": len(ml),
         "P1_tcp_dev_mm_median": med([s["max_tcp_dev_mm"] for s in ml]), "P1_tcp_dev_mm_p95": q([s["max_tcp_dev_mm"] for s in ml], 95),
         "P1_tcp_dev_mm_max": max([s["max_tcp_dev_mm"] for s in ml], default=None),
         "P1_orient_dev_deg_median": med([s["max_orient_dev_deg"] for s in ml]), "P1_orient_dev_deg_p95": q([s["max_orient_dev_deg"] for s in ml], 95),
         "P1_final_pos_err_mm_p95": q([s["final_pos_err_mm"] for s in ml], 95),
         "jerk_gate": {k: jerk_stats(v) for k, v in segs.items()},
         # post hoc: TCP cruise speed against the 0.25 m/s task spec (VAMP enforces it; cuRobo has no TCP speed limit)
         "posthoc_tcp_cruise_speed_mps_median": med([s["cruise_speed_mean_mps"] for s in ml]),
         "posthoc_tcp_cruise_speed_max_mps_p95": q([s["cruise_speed_max_mps"] for s in ml], 95),
         "posthoc_moveL_share_peak_over_spec": float(np.mean([s["cruise_speed_max_mps"] > TCP_V_SPEC * TOL for s in ml])) if ml else None}
    if S["P1_tcp_dev_mm_median"] is not None:
        S["P1_expresses_moveL"] = bool(S["P1_tcp_dev_mm_median"] <= 1.0 and S["P1_tcp_dev_mm_p95"] <= 5.0)
    if "success_nojerk" in recs[0]:
        S["posthoc_success_nojerk"] = sum(bool(p.get("success_nojerk")) for p in recs)
    return S


def paired_p2(R, a, b):
    """P2 on the picks both runs complete (same cell, pick, seed)."""
    A = {(p["cell"], p["pick"], p["seed_k"]): p for p in R[a] if p["success"]}
    B = {(p["cell"], p["pick"], p["seed_k"]): p for p in R[b] if p["success"]}
    k = sorted(A.keys() & B.keys())
    if not k:
        return {"picks": 0}
    d = lambda X, f: med([X[i][f] for i in k])
    return {"picks": len(k), **{f"{x}_{f}_median": d(X, f) for x, X in ((a, A), (b, B)) for f in ("t_cycle_s", "t_ready_s", "t_exec_s")},
            "ratio_t_cycle_median": med([A[i]["t_cycle_s"] / B[i]["t_cycle_s"] for i in k])}


def figure(R, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rows = [("vamp", "VAMP"), ("curobo1", "cuRobo v1\n(as configured)"), ("curobo1_J1", "cuRobo v1\nmax_jerk 250"),
            ("curobo1_J1b", "cuRobo v1\nmax_jerk 200"), ("curobo2", "cuRoboV2")]
    col = {"vamp": "#2a78b5", "curobo1": "#b8442a", "curobo1_J1": "#d9744f", "curobo1_J1b": "#e0956f", "curobo2": "#4a9a5b"}
    fig, ax = plt.subplots(figsize=(8, 4.2), dpi=150)
    rng = np.random.default_rng(0)
    for i, (k, lab) in enumerate(rows):
        j = np.array([s["max_jerk_ratio"] for p in R[k] for s in p["segments"] if s["kind"] == "moveL" and s.get("max_jerk_ratio") is not None])
        y = len(rows) - 1 - i
        ax.scatter(j, y + rng.uniform(-0.28, 0.28, len(j)), s=4, alpha=0.35, color=col[k], linewidths=0, rasterized=True)
        ax.boxplot(j, positions=[y], vert=False, widths=0.6, showfliers=False, medianprops={"color": "#222"},
                   boxprops={"color": "#444"}, whiskerprops={"color": "#444"}, capprops={"color": "#444"})
        ax.text(2.62, y, f"{int((j <= TOL).sum())}/{len(j)}\nwithin", va="center", fontsize=8, color="#333")
    ax.axvline(1.0, color="#222", lw=1.2, ls="--")
    ax.text(1.02, len(rows) - 0.45, "limit: 500 rad/s³\n(cuRobo ur5e.yml)", fontsize=8, va="top")
    ax.set_yticks(range(len(rows))); ax.set_yticklabels([lab for _, lab in rows][::-1], fontsize=8.5)
    ax.set_xlim(0, 3.1); ax.set_xlabel("peak joint jerk on each MoveL segment, executed at 1 kHz  (÷ 500 rad/s³)")
    ax.set_title("UR5e pick-place, MoveL segments: executed jerk against the configured limit", fontsize=10)
    fig.text(0.01, 0.01, "Within = peak \u2264 1.01 \u00d7 limit (the gate used for every planner). UR publishes no UR5e jerk limit.", fontsize=7, color="#555")
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    fig.tight_layout(rect=(0, 0.03, 1, 1)); fig.savefig(path); plt.close(fig)


def main():
    R = {k: load(f) for k, f in RUNS.items()}
    n_rep = attach_vamp_jerk(R["vamp"])
    out = {"cells": 30, "picks_per_run": 600, "budget_T_s": T_BUDGET, "gate_tolerance": TOL,
           "jerk_limit_rad_s3": JERK_LIMIT,
           "jerk_limit_source": "cuRobo's shipped ur5e.yml (configs/v2_limits.yaml); Universal Robots publishes no UR5e jerk limit",
           "timing_conditions": {"vamp": "CPU (Jetson Orin NX 16 GB), contended (1-min load average 13.6-15.6 on 8 cores); load log results/v2/raw/vamp_pickplace.A6.load.log",
                                 "curobo": "GPU (same board), quiet; reference runner stopped"},
           "vamp_segments_replayed_for_jerk": n_rep, "runs": {k: run_summary(k, v) for k, v in R.items()}}
    out["P2_paired"] = {"vamp_vs_curobo2": paired_p2(R, "vamp", "curobo2"), "vamp_vs_curobo1_J1b": paired_p2(R, "vamp", "curobo1_J1b"),
                        "curobo1_J1b_vs_curobo2": paired_p2(R, "curobo1_J1b", "curobo2")}
    out["appendix_A5_unwrapped"] = {k: {"picks": len(v), "success": sum(p["success"] for p in v)}
                                    for k, f in APPENDIX.items() for v in [load(f)]}
    json.dump(out, open(ROOT / "results/v2/pickplace_summary.json", "w"), indent=1)
    figure(R, ROOT / "results/v2/pickplace_jerk.png")
    for k, s in out["runs"].items():
        print(f"{k:12s} {s['success']:3d}/600  P3 {s['P3_picks_failing_some_seed']:3d}/120  T_cycle {s['P2_t_cycle_s_median']}  "
              f"ready {s['P2_t_ready_s_median']}  exec {s['P2_t_exec_s_median']}  dev {s['P1_tcp_dev_mm_median']}/{s['P1_tcp_dev_mm_p95']}  "
              f"jerk MoveL {s['jerk_gate']['moveL']['within_limit_share']} free {s['jerk_gate']['free']['within_limit_share']}  {s['first_failure']}")


if __name__ == "__main__":
    main()
