"""Re-run VAMP v2 runs whose timing spans the GPU-slot suspension (SIGSTOP of the CPU step).

A suspended run shows a planning time far longer than any real VAMP run, whose ceiling is ~55 s at the 1M-iteration
default. Every run with t_plan_ms > 600 s is re-run with the identical procedure and seed: VAMP with a fixed Halton
skip is deterministic, so only the timing changes. The record is replaced and marked rerun_after_suspension=True.
Usage: python bench/v2_vamp_fix_stopped.py results/v2/raw/vamp_<set>.json
"""
import json, pickle, sys, time
from pathlib import Path

import numpy as np
import vamp

sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, limits
from v2_timing import ruckig_p1, toppra_p2
from v2_validate import validate

f = Path(sys.argv[1]); D = json.load(open(f))
SETS = {"heldout": ("problems/v2_heldout.pkl", "v2_heldout"), "mbm": ("problems/v2_mbm_panda.pkl", "v2_mbm"),
        "mpinets": ("problems/v2_mpinets_panda.pkl", "v2_mpinets")}
pk, key = SETS[D["set"]]; vm = D["robot"]
probs = pickle.load(open(ROOT / pk, "rb"))["problems"][key]
vmod, planner, ps, ss = vamp.configure_robot_and_planner_with_kwargs(vm, "rrtc"); sampler = vmod.halton()
vmax, amax, jmax = limits(vm)
bad = [i for i, r in enumerate(D["records"]) if r["t_plan_ms"] > 600e3]
print("suspended runs:", [(D["records"][i]["q"], D["records"][i]["seed_k"], round(D["records"][i]["t_plan_ms"] / 1e3)) for i in bad])
for i in bad:
    old = D["records"][i]; p = probs[old["q"]]
    sampler.reset(); sampler.skip(old["seed"] % 100000)
    t0 = time.perf_counter_ns(); env = vamp.problem_dict_to_vamp(p)
    res = planner(p["start"], p["goals"], env, ps, sampler)
    rec = {k: old[k] for k in ("q", "index", "family", "seed_k", "seed", "bins")}
    rec.update(plan_success=bool(res.solved), rerun_after_suspension=True, suspended_t_plan_ms=old["t_plan_ms"])
    if res.solved:
        simple = vmod.simplify(res.path, env, ss, sampler); t1 = time.perf_counter_ns()
        path = np.array(simple.path.numpy(), float)
        t2 = time.perf_counter_ns(); T1 = ruckig_p1(path, vmax, amax, jmax); t3 = time.perf_counter_ns(); T2 = toppra_p2(path, vmax, amax); t4 = time.perf_counter_ns()
        rec.update(t_plan_ms=(t1 - t0) / 1e6, plan_internal_ms=(res.nanoseconds + simple.nanoseconds) / 1e6, t_p1_ms=(t3 - t2) / 1e6,
                   t_p2_ms=(t4 - t3) / 1e6, n_waypoints=int(len(path)), path=path.round(6).tolist())
        for name, T in (("p1", T1), ("p2", T2)):
            if T is None:
                rec[name] = {"valid": False, "reason": "parameterization failed"}; continue
            q, qd, qdd = T.sample(np.arange(0, T.duration + 1e-9, 1e-3))
            v = validate(vm, p, q, qd, qdd, check_jerk=(name == "p1")); v["t_exec_s"] = float(T.duration); rec[name] = v
        if rec["p1"]["valid"]:
            rec["t_ready_ms"] = rec["t_plan_ms"] + rec["t_p1_ms"]; rec["t_cycle_s"] = rec["t_ready_ms"] / 1e3 + rec["p1"]["t_exec_s"]
    else:
        rec["t_plan_ms"] = (time.perf_counter_ns() - t0) / 1e6
    rec["solved_any_time"] = bool(res.solved and rec.get("p1", {}).get("valid", False)); rec["solved"] = rec["solved_any_time"]
    same = bool(old.get("plan_success") == rec["plan_success"] and old.get("path") == rec.get("path"))
    rec["identical_result_to_suspended_run"] = same
    D["records"][i] = rec
    print("rerun", rec["q"], rec["seed_k"], "t_plan_ms", round(rec["t_plan_ms"], 2), "identical result:", same)
json.dump(D, open(f, "w"))
