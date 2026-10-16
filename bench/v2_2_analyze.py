"""Benchmark v2.2 analysis (docs/V2_2_MOTION_TYPES_PREREGISTRATION.md, Metrics / How results will be read).

Inputs: results/v2_2/raw/{cartesian, curobo1, curobo1_jerk200, curobo2, pyroki_gpu}.jsonl; problems/v2_2_programs.pkl.
Per planner and program type (W, C, T):
  E   expressibility: the line classes cuRobo used (amendments V2/V2a), or "post-processing" / "per-knot costs"
  success over runs (R1, as returned), per-program success fraction, programs failing on some seed / every seed (Q6)
  R2  uniform slow-down (pre-registered; v2.1 7f shortcut: s = max(1, rv, sqrt(ra), cbrt(rj)) divides the velocity,
      acceleration and jerk peaks by s, s^2, s^3; geometry, hence path metrics, unchanged; speeds divided by s)
  Q1  path fidelity on successful runs: max deviation outside blends (median, p95), blend-zone deviation (median, p95)
  Q2  cruise speed: median and p95 over runs of the max |v - v_prog| / v_prog; achieved cruise speed / v_prog
  Q3  blends (W): median per-via minimum speed / v_prog; share of via points stopped (<= 0.05) and held (>= 0.9)
  Q4  ramps (T): median ramp acceleration error and ramp duration ratios
  Q5  T_ready, T_exec, T_cycle medians (bootstrap 95% CI over programs) and paired T_cycle vs the Cartesian pipeline
Q1-Q4 are computed on the runs that have a trajectory; "_success" variants restrict to successful runs.
Also: failure reasons (non-exclusive) over failed runs; success by the pre-registered bins (path length: below / at or
above the type's median, fixed after the run since the pre-registration gave no edges; number of vias (W); min path
clearance 1-2 / 2-5 / > 5 cm), bins with < 30 programs flagged; the Cartesian pipeline's speed halvings.
Writes results/v2_2/summary.json.
"""
import json, pickle
from collections import defaultdict
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
import os
RAW = Path(os.environ.get("V22_RAW", ROOT / "results/v2_2/raw"))
SUFFIX = os.environ.get("V22_SUFFIX", "")
RUNS = {k: v.replace(".jsonl", SUFFIX + ".jsonl") for k, v in {"cartesian": "cartesian.jsonl", "curobo1": "curobo1.jsonl",
        "curobo1_jerk200": "curobo1_jerk200.jsonl", "curobo2": "curobo2.jsonl", "pyroki": "pyroki_gpu.jsonl"}.items()}
rng = np.random.default_rng(0)


def ci(x, f=np.median, n=2000):
    x = np.asarray([v for v in x if v is not None], float)
    if len(x) == 0:
        return None
    b = [f(rng.choice(x, len(x))) for _ in range(n)]
    return {"median" if f is np.median else "value": float(f(x)), "ci95": [float(np.percentile(b, 2.5)), float(np.percentile(b, 97.5))], "n": int(len(x))}


def q(x, p):
    x = [v for v in x if v is not None]
    return float(np.percentile(x, p)) if x else None


def path_ok(m):
    return m["end_pos_err_mm"] <= 1 and m["end_rot_err_deg"] <= 1 and m["max_dev_outside_mm"] <= 5 and m["max_orient_dev_deg"] <= 1


def normalise(planner, r):
    """One record shape for every planner: success (R1), metrics, gate, times, R2 success and scale."""
    if planner == "pyroki":
        m = r["R1"]["metrics"]; g = r["R1"]["gate"]; t_exec = r["R1"]["t_exec_s"]
        r2 = r.get("R2")
        return {"cell": r["cell"], "type": r["type"], "seed_k": 0, "success": r["R1"]["success"], "metrics": m, "gate": g,
                "t_ready_s": r["t_ready_s"], "t_exec_s": t_exec, "classes": ["per-knot costs"],
                "r2_success": bool(r2["success"]) if r2 else r["R1"]["success"], "r2_scale": r2["scale"] if r2 else 1.0,
                "r2_t_exec_s": r2["t_exec_s"] if r2 else t_exec, "start_ok": r["start_ok"], "speed_retries": None}
    m, g = r.get("metrics"), r.get("gate")
    out = {"cell": r["cell"], "type": r["type"], "seed_k": r.get("seed_k", 0), "success": r["success"], "metrics": m, "gate": g,
           "t_ready_s": r["t_ready_s"], "t_exec_s": r.get("t_exec_s"),
           "classes": r.get("classes", ["post-processing"]) if planner != "cartesian" else ["post-processing"]}
    out["r2_success"], out["r2_scale"], out["r2_t_exec_s"] = out["success"], 1.0, out["t_exec_s"]
    if not out["success"] and m is not None and g is not None and g["colliding_samples"] == 0 and g.get("bound_violation_rad", 0) <= 1e-5:
        s = max(1.0, g["max_vel_ratio"], np.sqrt(g["max_acc_ratio"]), np.cbrt(g["max_jerk_ratio"]))
        out["r2_scale"] = float(s); out["r2_t_exec_s"] = out["t_exec_s"] * s
        out["r2_success"] = bool(path_ok(m) and r["t_ready_s"] <= 30)
    out["start_ok"] = r.get("start_ok", True); out["speed_retries"] = r.get("speed_retries")
    return out


def reasons(r):
    """Why a run failed (non-exclusive)."""
    m, g = r["metrics"], r["gate"]
    if m is None:
        return ["no_trajectory"] + (["time"] if r["t_ready_s"] > 30 else [])
    f = {"collision": g["colliding_samples"] > 0, "bounds": g.get("bound_violation_rad", 0) > 1e-5,
         "velocity": g["max_vel_ratio"] > 1.01, "acceleration": g["max_acc_ratio"] > 1.01, "jerk": g["max_jerk_ratio"] > 1.01,
         "path_dev_outside": m["max_dev_outside_mm"] > 5, "orientation": m["max_orient_dev_deg"] > 1,
         "end_pose": m["end_pos_err_mm"] > 1 or m["end_rot_err_deg"] > 1, "time": r["t_ready_s"] > 30, "start": not r["start_ok"]}
    out = [k for k, v in f.items() if v]
    if out and set(out) <= {"velocity", "acceleration", "jerk"}:
        out.append("dynamics_only")
    return out


def bins(recs, progs, t):
    """Success over runs per pre-registered bin."""
    def rate(sel):
        rs = [r for r in recs if sel(progs[r["cell"]]["programs"][t])]
        n = len({r["cell"] for r in rs})
        return {"programs": n, "success_rate_runs": float(np.mean([r["success"] for r in rs])) if rs else None,
                "r2_success_rate_runs": float(np.mean([r["r2_success"] for r in rs])) if rs else None, "underpowered": n < 30}
    med = float(np.median([c["programs"][t]["path_length_m"] for c in progs.values()]))
    out = {"path_length_median_m": med, "path_length": {"short": rate(lambda p: p["path_length_m"] < med),
                                                         "long": rate(lambda p: p["path_length_m"] >= med)},
           "clearance": {k: rate(lambda p, lo=lo, hi=hi: lo <= p["min_clearance_m"] < hi)
                         for k, lo, hi in (("1-2cm", 0.0, 0.02), ("2-5cm", 0.02, 0.05), (">5cm", 0.05, 9.0))}}
    if t == "W":
        out["vias"] = {str(n): rate(lambda p, n=n: len(p["via_points"]) == n) for n in (3, 4, 5, 6)}
    return out


def summarise(recs, cart_by_prog, progs=None, t=None):
    by_prog = defaultdict(list)
    for r in recs:
        by_prog[r["cell"]].append(r)
    traj = [r for r in recs if r["metrics"] is not None]; succ = [r for r in traj if r["success"]]
    M = lambda rs, k: [r["metrics"][k] for r in rs]
    S = {"runs": len(recs), "programs": len(by_prog), "success_rate_runs": float(np.mean([r["success"] for r in recs])) if recs else None,
         "r2_success_rate_runs": float(np.mean([r["r2_success"] for r in recs])) if recs else None,
         "successes_runs": sum(r["success"] for r in recs), "r2_successes_runs": sum(r["r2_success"] for r in recs),
         "programs_failing_some_seed": sum(1 for v in by_prog.values() if not all(r["success"] for r in v)),
         "programs_failing_every_seed": sum(1 for v in by_prog.values() if not any(r["success"] for r in v)),
         "classes": {c: sum(c in r["classes"] for r in recs) for c in sorted({c for r in recs for c in r["classes"]})},
         "runs_with_trajectory": len(traj), "gate_valid_runs": sum(1 for r in traj if r["gate"]["valid"])}
    for tag, rs in (("", traj), ("_success", succ)):
        S["Q1" + tag] = {"dev_outside_mm_median": q(M(rs, "max_dev_outside_mm"), 50), "dev_outside_mm_p95": q(M(rs, "max_dev_outside_mm"), 95),
                         "dev_blend_mm_median": q(M(rs, "max_dev_blend_mm"), 50), "dev_blend_mm_p95": q(M(rs, "max_dev_blend_mm"), 95),
                         "orient_dev_deg_p95": q(M(rs, "max_orient_dev_deg"), 95)}
        S["Q2" + tag] = {"cruise_dev_max_median": q(M(rs, "cruise_dev_max"), 50), "cruise_dev_max_p95": q(M(rs, "cruise_dev_max"), 95),
                         "cruise_speed_ratio_median": q(M(rs, "cruise_speed_ratio"), 50)}
        bl = [x for r in rs for x in r["metrics"]["blend_min_speed_ratio"] if x is not None]
        S["Q3" + tag] = {"vias": len(bl), "min_speed_ratio_median": q(bl, 50),
                         "share_stopped": float(np.mean([x <= 0.05 for x in bl])) if bl else None,
                         "share_held": float(np.mean([x >= 0.9 for x in bl])) if bl else None}
        ra = [rp for r in rs for rp in r["metrics"]["ramps"]]
        S["Q4" + tag] = {"ramp_acc_err_median": q([x["acc_err_median"] for x in ra], 50),
                         "ramp_duration_ratio_median": q([x["duration_ratio"] for x in ra], 50),
                         "ramp_duration_ratio_p5_p95": [q([x["duration_ratio"] for x in ra], 5), q([x["duration_ratio"] for x in ra], 95)]}
    per_prog = lambda rs, k: [float(np.median([r[k] for r in v])) for v in group(rs).values()]
    S["Q5"] = {"t_ready_s": ci(per_prog(succ, "t_ready_s")) if succ else None, "t_exec_s": ci(per_prog(succ, "t_exec_s")) if succ else None,
               "t_cycle_s": ci(per_prog(succ, "t_cycle")) if succ else None}
    r2s = [r for r in recs if r["r2_success"]]
    S["R2"] = {"success_rate_runs": S["r2_success_rate_runs"], "scale_median": q([r["r2_scale"] for r in r2s], 50),
               "scale_p95": q([r["r2_scale"] for r in r2s], 95), "t_cycle_s": ci(per_prog(r2s, "r2_t_cycle")) if r2s else None}
    # paired with the Cartesian pipeline, programs both solve (R1)
    mine = {c: np.median([r["t_cycle"] for r in v]) for c, v in group(succ).items()}
    both = sorted(set(mine) & set(cart_by_prog))
    S["Q5_paired_vs_cartesian"] = {"programs": len(both), "t_cycle_s_median": float(np.median([mine[c] for c in both])) if both else None,
                                   "cartesian_t_cycle_s_median": float(np.median([cart_by_prog[c] for c in both])) if both else None}
    fails = [r for r in recs if not r["success"]]
    S["fail_reasons"] = {"failed_runs": len(fails)}
    for r in fails:
        for k in reasons(r):
            S["fail_reasons"][k] = S["fail_reasons"].get(k, 0) + 1
    S["t_ready_s_all"] = {"median": q([r["t_ready_s"] for r in recs], 50), "p95": q([r["t_ready_s"] for r in recs], 95),
                          "over_30s": sum(r["t_ready_s"] > 30 for r in recs)}
    if progs is not None:
        S["bins"] = bins(recs, progs, t)
    if recs and recs[0]["speed_retries"] is not None:
        S["speed_halvings"] = {str(k): sum(r["speed_retries"] == k for r in recs) for k in sorted({r["speed_retries"] for r in recs})}
    return S


def group(rs):
    g = defaultdict(list)
    for r in rs:
        g[r["cell"]].append(r)
    return g


def program_set(progs):
    """Generator acceptance and the shape of the kept set (from the program file)."""
    W = [c["programs"]["W"] for c in progs]; C = [c["programs"]["C"]["prims"][0] for c in progs]
    L = lambda t: [c["programs"][t]["path_length_m"] for c in progs]
    return {"cells": len(progs), "candidates": max(c["k"] for c in progs) + 1,
            "acceptance": len(progs) / (max(c["k"] for c in progs) + 1),
            "w_vias": {str(n): sum(len(w["via_points"]) == n for w in W) for n in range(3, 7)},
            "c_sweep_deg": {"min": float(np.degrees(min(p[7] for p in C))), "median": float(np.degrees(np.median([p[7] for p in C]))),
                            "max": float(np.degrees(max(p[7] for p in C)))},
            "c_radius_m_median": float(np.median([p[5] for p in C])),
            "path_length_m": {t: {"min": min(L(t)), "median": float(np.median(L(t))), "max": max(L(t))} for t in "WCT"},
            "v_mps": {t: progs[0]["programs"][t]["v"] for t in "WCT"}, "a_mps2": {t: progs[0]["programs"][t]["a"] for t in "WCT"},
            "clearance_below_5cm": {t: sum(c["programs"][t]["min_clearance_m"] < 0.05 for c in progs) for t in "WCT"}}


def line_classes():
    """cuRobo line classes (amendment V2a) per W leg and T line; geometric, so identical for v1 and V2 (seed 0 of v1)."""
    fp = RAW / RUNS["curobo1"]
    if not fp.exists():
        return None
    out = {t: {"lines": 0, "native": 0, "plane": 0, "orient": 0} for t in "WT"}
    for l in open(fp):
        r = json.loads(l)
        if r.get("seed_k", 0) != 0:
            continue
        for g in r["legs"]:
            out[r["type"]]["lines"] += 1; out[r["type"]][g["class"]] += 1
    out["all_lines"] = out["W"]["lines"] + out["T"]["lines"]
    out["all_native"] = out["W"]["native"] + out["T"]["native"]
    return out


def main():
    progs = pickle.load(open(os.environ.get("V22_PROBLEMS", ROOT / "problems/v2_2_programs.pkl"), "rb"))["cells"]
    by_index = {c["index"]: c for c in progs}
    data = {}
    for p, f in RUNS.items():
        fp = RAW / f
        if fp.exists():
            data[p] = [normalise(p, json.loads(l)) for l in open(fp)]
            for r in data[p]:
                r["t_cycle"] = (r["t_ready_s"] or 0) + (r["t_exec_s"] or 0)
                r["r2_t_cycle"] = (r["t_ready_s"] or 0) + (r["r2_t_exec_s"] or 0)
    out = {"programs": {t: len(progs) for t in "WCT"}, "planners": {}, "program_set": program_set(progs),
           "expressibility_lines": line_classes()}
    for t in "WCT":
        cart = {r["cell"]: r["t_cycle"] for r in data.get("cartesian", []) if r["type"] == t and r["success"]}
        for p, recs in data.items():
            rs = [r for r in recs if r["type"] == t]
            out["planners"].setdefault(p, {})[t] = summarise(rs, cart, by_index, t) if rs else {"runs": 0, "note": "not run (not expressible)"}
    json.dump(out, open(os.environ.get("V22_OUT", ROOT / "results/v2_2/summary.json"), "w"), indent=1)
    for p, v in out["planners"].items():
        print(p, {t: (s.get("runs"), None if s.get("success_rate_runs") is None else round(s["success_rate_runs"], 3),
                      None if s.get("r2_success_rate_runs") is None else round(s["r2_success_rate_runs"], 3)) for t, s in v.items()})


if __name__ == "__main__":
    main()
