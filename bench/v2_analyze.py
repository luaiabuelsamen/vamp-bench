"""Benchmark v2 analysis (pre-registered statistics, docs/V2_PREREGISTRATION.md).

Success = valid trajectory with T_ready <= T (amendment A2, T from configs/v2_planners.yaml), for every planner.
Per planner and set: per query over its seeds -> success fraction, median / p95 / worst T_cycle (failures = T),
success-vs-budget curve (share of runs solved with T_ready <= t, t from 1 ms to T),
median T_ready, T_exec. Per bin (each of the 4 axes): median of per-query medians with a 95% bootstrap CI over
queries (2000 resamples, seed 0); bins with < 30 queries marked underpowered. Quality metrics: medians over solved runs.
Writes results/v2/summary.json.
"""
import json, glob
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
import yaml
TIMEOUT = float(yaml.safe_load(open(ROOT / "configs/v2_planners.yaml"))["budget"]["T_s"])  # A2: uniform budget T
CURVE_T = np.logspace(-3, np.log10(TIMEOUT), 61)
AXES = ("reach", "clearance", "passage", "limits")
QUAL = ("joint_length_rad", "tcp_length_m", "rms_jerk_norm", "peak_jerk_norm", "integrated_sq_jerk", "min_clearance_m",
        "time_within_1cm_s", "min_limit_margin_rad", "max_tcp_tilt_deg")


def boot_ci(x, rng, n=2000):
    x = np.asarray(x, float)
    if len(x) == 0:
        return None
    m = [np.median(rng.choice(x, len(x))) for _ in range(n)]
    return [float(np.percentile(m, 2.5)), float(np.percentile(m, 97.5))]


def per_query(recs, key_traj):
    by = {}
    for r in recs:
        by.setdefault(r["q"], []).append(r)
    out = {}
    for q, rs in by.items():
        cyc = [r["t_cycle_s"] if r["solved"] else TIMEOUT for r in rs]
        rdy = [r["t_ready_ms"] for r in rs if r["solved"]]
        ex = [r[key_traj]["t_exec_s"] for r in rs if r["solved"]]
        out[q] = {"bins": rs[0]["bins"], "n": len(rs), "success": sum(r["solved"] for r in rs) / len(rs),
                  "t_cycle_med": float(np.median(cyc)), "t_cycle_p95": float(np.percentile(cyc, 95)), "t_cycle_worst": float(max(cyc)),
                  "t_ready_med_ms": float(np.median(rdy)) if rdy else None, "t_exec_med_s": float(np.median(ex)) if ex else None,
                  "cv_t_cycle": float(np.std(cyc) / np.mean(cyc)) if np.mean(cyc) > 0 else None}
    return out


def censor(recs):
    """A2: success = valid trajectory with T_ready <= T, identically for every planner."""
    for r in recs:
        valid = r.get("solved_any_time", r["solved"])
        r["solved"] = bool(valid and r.get("t_ready_ms", np.inf) <= TIMEOUT * 1e3)
    return recs


def curve(recs):
    t = np.array([r["t_ready_ms"] / 1e3 if r["solved"] else np.inf for r in recs])
    return {"t_s": CURVE_T.round(6).tolist(), "success": [float((t <= x).mean()) for x in CURVE_T]}


def summarize(path, key_traj):
    D = json.load(open(path)); recs = censor(D["records"]); pq = per_query(recs, key_traj); rng = np.random.default_rng(0)
    solved = [r for r in recs if r["solved"]]
    s = {"source": str(Path(path).relative_to(ROOT)), "runs": len(recs), "queries": len(pq),
         "success_rate_runs": sum(r["solved"] for r in recs) / len(recs),
         "queries_failing_some_seed": sum(v["success"] < 1 for v in pq.values()),
         "queries_failing_all_seeds": sum(v["success"] == 0 for v in pq.values()),
         "t_cycle_med_of_query_meds": float(np.median([v["t_cycle_med"] for v in pq.values()])),
         "t_cycle_worst_over_queries": float(max(v["t_cycle_worst"] for v in pq.values())),
         "t_ready_ms_median_runs": float(np.median([r["t_ready_ms"] for r in solved])) if solved else None,
         "t_exec_s_median_runs": float(np.median([r[key_traj]["t_exec_s"] for r in solved])) if solved else None,
         "quality_median": {m: float(np.median([r[key_traj][m] for r in solved])) for m in QUAL} if solved else {},
         "budget_T_s": TIMEOUT, "success_vs_budget": curve(recs),
         "per_bin": {}}
    if key_traj == "p1":  # VAMP secondary: P2 (TOPP-RA)
        p2 = [r for r in recs if r.get("p2", {}).get("valid")]
        s["p2_validity_rate_of_planned"] = len(p2) / max(1, sum(r["plan_success"] for r in recs))
        s["p2_t_exec_s_median"] = float(np.median([r["p2"]["t_exec_s"] for r in p2])) if p2 else None
        s["p1_t_exec_s_median_same_runs"] = float(np.median([r["p1"]["t_exec_s"] for r in p2 if r["p1"].get("valid")])) if p2 else None
    for ax in AXES:
        s["per_bin"][ax] = {}
        for b in sorted({v["bins"][ax] for v in pq.values()}):
            vs = [v for v in pq.values() if v["bins"][ax] == b]
            med = [v["t_cycle_med"] for v in vs]
            rd = [v["t_ready_med_ms"] for v in vs if v["t_ready_med_ms"] is not None]
            s["per_bin"][ax][b] = {"queries": len(vs), "underpowered": len(vs) < 30, "success_mean": float(np.mean([v["success"] for v in vs])),
                                   "t_cycle_med": float(np.median(med)), "t_cycle_ci95": boot_ci(med, rng),
                                   "t_ready_med_ms": float(np.median(rd)) if rd else None, "t_ready_ci95": boot_ci(rd, rng)}
    return s


summary = {}
for f in sorted(glob.glob(str(ROOT / "results/v2/raw/*.json"))):
    name = Path(f).stem
    summary[name] = summarize(f, "p1" if name.startswith("vamp") else "traj")
    print(name, {k: summary[name][k] for k in ("queries", "success_rate_runs", "t_cycle_med_of_query_meds", "t_ready_ms_median_runs", "t_exec_s_median_runs")})
json.dump(summary, open(ROOT / "results/v2/summary.json", "w"), indent=1)
