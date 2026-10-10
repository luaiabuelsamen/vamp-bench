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
    D = json.load(open(path)); recs = D["records"]; pq = per_query(recs, key_traj); rng = np.random.default_rng(0)
    solved = [r for r in recs if r["solved"]]
    s = {"runs": len(recs), "queries": len(pq),
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


def feasibility(s):
    """A3: certified feasible = any valid trajectory from any planner run (any time) or a reference run path."""
    cert = set()
    for pref in ("vamp", "curobo1", "curobo2"):
        f = ROOT / f"results/v2/raw/{pref}_{s}.json"
        if f.exists():
            cert |= {r["q"] for r in json.load(open(f))["records"] if r.get("solved_any_time")}
    ref = {}
    f = ROOT / f"results/v2/raw/feasibility_{s}.jsonl"
    if f.exists():
        for r in map(json.loads, open(f)):
            ref[r["q"]] = r["path_exists"]
    cert |= {q for q, e in ref.items() if e}
    unresolved = {q for q, e in ref.items() if not e and q not in cert}
    return cert, unresolved, set(ref)


def restrict(recs, qs=None, seeds=None):
    return [r for r in recs if (qs is None or r["q"] in qs) and (seeds is None or r["seed_k"] in seeds)]


summary = {"budget_T_s": TIMEOUT, "sets": {}}
subset = json.load(open(ROOT / "results/v2/v2_subsample.json"))
NQ = {"heldout": 300, "mbm": 300, "mpinets": 300}
for s in ("heldout", "mbm", "mpinets"):
    cert, unres, refd = feasibility(s)
    pending = set(range(NQ[s])) - cert - unres
    S = {"feasibility": {"certified": len(cert), "unresolved": len(unres), "pending_reference": len(pending),
                         "status": "complete" if not pending else "partial (reference runs still running)"}, "planners": {}}
    for pref, key in (("vamp", "p1"), ("curobo1", "traj"), ("curobo2", "traj")):
        f = ROOT / f"results/v2/raw/{pref}_{s}.json"
        if not f.exists():
            continue
        D = json.load(open(f)); recs = censor(D["records"])
        out = {}
        for label, sel in (("all_queries", None), ("certified_feasible", cert)):
            tmp = Path("/tmp") / f"_v2_{pref}_{s}_{label}.json"
            json.dump({"records": restrict(recs, sel)}, open(tmp, "w"))
            out[label] = summarize(tmp, key); out[label]["source"] = str(f.relative_to(ROOT))
        if pref != "curobo2":  # same queries and seeds as the V2 subsample
            tmp = Path("/tmp") / f"_v2_{pref}_{s}_v2sub.json"
            json.dump({"records": restrict(recs, set(subset[s]), {0, 1, 2})}, open(tmp, "w"))
            out["v2_subsample_seeds012"] = summarize(tmp, key); out["v2_subsample_seeds012"]["source"] = str(f.relative_to(ROOT))
        S["planners"][pref] = out
        a_, c_ = out["all_queries"], out["certified_feasible"]
        print(f"{s:8s} {pref:8s} runs {a_['runs']:5d} success all {a_['success_rate_runs']:.3f} certified {c_['success_rate_runs']:.3f} "
              f"T_ready med {a_['t_ready_ms_median_runs']} ms  T_exec med {a_['t_exec_s_median_runs']} s  T_cycle med-of-q {a_['t_cycle_med_of_query_meds']:.2f} s")
    summary["sets"][s] = S
json.dump(summary, open(ROOT / "results/v2/summary.json", "w"), indent=1)
print("feasibility:", {s: summary["sets"][s]["feasibility"] for s in summary["sets"]})
