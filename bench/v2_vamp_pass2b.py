"""Amendment A4 pass 2b: untimed 1 kHz validation + quality metrics for VAMP successes, merged with pass 2a timings
into final per-run records (same schema as bench/v2_vamp.py) -> results/v2/raw/vamp_<set>.json.

Per run: pass (a) gives success/failure and the path (identical to pass 2a, verified); pass 2a gives the quiet
timings (t_plan_ms, t_p1_ms, t_p2_ms, t_ready_ms); this pass time-parameterizes the path with P1 (Ruckig) and P2
(TOPP-RA), validates both at 1 kHz and computes the quality metrics. Failures and kills from pass (a) are failures.
Usage: python bench/v2_vamp_pass2b.py --workers N
"""
import argparse, json, multiprocessing as mp, pickle, sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, limits
from v2_timing import ruckig_p1, toppra_p2
from v2_validate import validate
from v2_vamp_pass1 import SETS

PROBS = {}


def work(args):
    s, r, t = args
    vm, pk, key = SETS[s]
    p = PROBS[s][r["q"]]; vmax, amax, jmax = limits(vm)
    rec = {k: r[k] for k in ("q", "index", "family", "seed_k", "seed", "bins")}
    rec.update(plan_success=bool(r["plan_success"]), pass1_killed="killed_at_s" in r)
    if not r["plan_success"]:
        rec.update(solved_any_time=False, solved=False, t_plan_ms=r.get("killed_at_s", 0) * 1e3 or r.get("contended_t_plan_ms"))
        return rec
    rec.update({k: t[k] for k in ("t_plan_ms", "plan_internal_ms", "t_p1_ms", "t_p2_ms", "t_ready_ms") if k in t})
    rec["path_identical_pass1_pass2a"] = t["path_identical_to_pass1"]
    path = np.asarray(r["path"], float); rec["n_waypoints"] = int(len(path))
    for name, T in (("p1", ruckig_p1(path, vmax, amax, jmax)), ("p2", toppra_p2(path, vmax, amax))):
        if T is None:
            rec[name] = {"valid": False, "reason": "parameterization failed"}; continue
        q, qd, qdd = T.sample(np.arange(0, T.duration + 1e-9, 1e-3))
        v = validate(vm, p, q, qd, qdd, check_jerk=(name == "p1")); v["t_exec_s"] = float(T.duration); rec[name] = v
    rec["solved_any_time"] = bool(rec["p1"]["valid"])
    if rec["solved_any_time"]:
        rec["t_cycle_s"] = rec["t_ready_ms"] / 1e3 + rec["p1"]["t_exec_s"]
    rec["solved"] = rec["solved_any_time"]  # A2 censoring at T applied in bench/v2_analyze.py
    return rec


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--workers", type=int, default=2); ap.add_argument("--freeze", default="6aea6eecf76a173d7ebdf548104aaec854b6b6c7")
    a = ap.parse_args()
    for s, (vm, pk, key) in SETS.items():
        PROBS[s] = pickle.load(open(ROOT / pk, "rb"))["problems"][key]
    for s in SETS:
        p1 = [json.loads(l) for l in open(ROOT / f"results/v2/raw/vamp_pass1_{s}.jsonl")]
        p2a = {(r["q"], r["seed_k"]): r for r in map(json.loads, open(ROOT / f"results/v2/raw/vamp_pass2a_{s}.jsonl"))}
        missing = [(r["q"], r["seed_k"]) for r in p1 if r["plan_success"] and (r["q"], r["seed_k"]) not in p2a]
        assert not missing, f"{s}: {len(missing)} successes lack pass 2a timing"
        jobs = [(s, r, p2a.get((r["q"], r["seed_k"]))) for r in sorted(p1, key=lambda x: (x["q"], x["seed_k"]))]
        with mp.get_context("fork").Pool(a.workers) as pool:
            recs = pool.map(work, jobs, chunksize=20)
        json.dump({"freeze": a.freeze, "set": s, "robot": SETS[s][0], "planner": "VAMP rrtc + simplify, P1 Ruckig / P2 TOPP-RA (A4 two-pass)",
                   "records": recs}, open(ROOT / f"results/v2/raw/vamp_{s}.json", "w"))
        print(s, len(recs), "runs; valid P1", sum(r["solved_any_time"] for r in recs), flush=True)


if __name__ == "__main__":
    main()
