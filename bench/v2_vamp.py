"""Benchmark v2, CPU step: VAMP + P1 (Ruckig) + P2 (TOPP-RA), 1 kHz validation and quality metrics, 10 seeds.

Timed (single thread, nothing else of ours running): env build + rrtc + simplify (T_plan), P1 and P2 parameterization.
Untimed: 1 kHz sampling, validation, metrics.
Usage: python bench/v2_vamp.py --freeze <hash> --set heldout|mbm|mpinets [--limit N]
Writes results/v2/raw/vamp_<set>.json.
"""
import argparse, json, pickle, platform, sys, time
from pathlib import Path

import numpy as np
import vamp

sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, limits, seed_from
from v2_timing import ruckig_p1, toppra_p2
from v2_validate import validate

SETS = {"heldout": ("ur5erail", "problems/v2_heldout.pkl", "v2_heldout"),
        "mbm": ("panda", "problems/v2_mbm_panda.pkl", "v2_mbm"),
        "mpinets": ("panda", "problems/v2_mpinets_panda.pkl", "v2_mpinets")}
ap = argparse.ArgumentParser()
ap.add_argument("--freeze", required=True); ap.add_argument("--set", required=True, choices=list(SETS))
ap.add_argument("--seeds", type=int, default=10); ap.add_argument("--problems", default=None); ap.add_argument("--limit", type=int, default=0); ap.add_argument("--out", default=None)
a = ap.parse_args()
vm, pk, key = SETS[a.set]
probs = pickle.load(open(a.problems or ROOT / pk, "rb"))["problems"][key][: a.limit or None]
vmod, planner, ps, ss = vamp.configure_robot_and_planner_with_kwargs(vm, "rrtc")
sampler = vmod.halton()
vmax, amax, jmax = limits(vm)
seeds = [seed_from(a.freeze, f"seed{k}") for k in range(a.seeds)]

recs = []
for qi, p in enumerate(probs):
    for k, sd in enumerate(seeds):
        sampler.reset(); sampler.skip(sd % 100000)
        t0 = time.perf_counter_ns(); env = vamp.problem_dict_to_vamp(p)
        res = planner(p["start"], p["goals"], env, ps, sampler)
        rec = {"q": qi, "index": p["index"], "family": p["problem"], "seed_k": k, "seed": sd, "bins": {x: p["bins"][x] for x in ("reach", "clearance", "passage", "limits")},
               "plan_success": bool(res.solved)}
        if res.solved:
            simple = vmod.simplify(res.path, env, ss, sampler); t1 = time.perf_counter_ns()
            path = np.array(simple.path.numpy(), float)
            t2 = time.perf_counter_ns(); T1 = ruckig_p1(path, vmax, amax, jmax); t3 = time.perf_counter_ns()
            T2 = toppra_p2(path, vmax, amax); t4 = time.perf_counter_ns()
            rec.update(t_plan_ms=(t1 - t0) / 1e6, plan_internal_ms=(res.nanoseconds + simple.nanoseconds) / 1e6,
                       t_p1_ms=(t3 - t2) / 1e6, t_p2_ms=(t4 - t3) / 1e6, n_waypoints=int(len(path)), path=path.round(6).tolist())
            for name, T in (("p1", T1), ("p2", T2)):
                if T is None:
                    rec[name] = {"valid": False, "reason": "parameterization failed"}; continue
                ts = np.arange(0, T.duration + 1e-9, 1e-3)
                q, qd, qdd = T.sample(ts)
                v = validate(vm, p, q, qd, qdd, check_jerk=(name == "p1")); v["t_exec_s"] = float(T.duration)
                rec[name] = v
            if rec["p1"]["valid"]:
                rec["t_ready_ms"] = rec["t_plan_ms"] + rec["t_p1_ms"]
                rec["t_cycle_s"] = rec["t_ready_ms"] / 1e3 + rec["p1"]["t_exec_s"]
        else:
            rec["t_plan_ms"] = (time.perf_counter_ns() - t0) / 1e6
        rec["solved"] = bool(res.solved and rec.get("p1", {}).get("valid", False))
        recs.append(rec)
    if qi % 25 == 0:
        print(f"{a.set} {qi + 1}/{len(probs)} solved {sum(r['solved'] for r in recs)}/{len(recs)}", flush=True)

out = Path(a.out or ROOT / f"results/v2/raw/vamp_{a.set}.json"); out.parent.mkdir(parents=True, exist_ok=True)
json.dump({"freeze": a.freeze, "set": a.set, "robot": vm, "seeds": seeds, "planner": "VAMP rrtc + simplify, P1 Ruckig / P2 TOPP-RA",
           "host": platform.node(), "records": recs}, open(out, "w"))
print("wrote", out, "solved", sum(r["solved"] for r in recs), "/", len(recs))
