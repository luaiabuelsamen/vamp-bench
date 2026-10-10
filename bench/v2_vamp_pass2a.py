"""Amendment A4 pass 2a: quiet-CPU sequential re-timing of every VAMP success from pass (a).

For each success (all three sets): env build + rrtc + simplify timed (T_plan, as bench/v2_vamp.py), then P1 (Ruckig)
and P2 (TOPP-RA) timed. The re-run must reproduce pass (a)'s path exactly (deterministic Halton seed); mismatches are
recorded. These quiet timings are the ones that count (success = T_ready <= T).
A4 check: (i) contention slowdown = pass (a) contended time / quiet time, over successes; (ii) a sample of 20 runs
killed at 60 s in pass (a) is re-run alone with a 30 s kill to confirm that they also fail within T alone.
CPU load (top processes) is recorded at start. Writes results/v2/raw/vamp_pass2a_<set>.jsonl and
results/v2/pass2a_check.json. Usage: python bench/v2_vamp_pass2a.py --freeze H
"""
import argparse, json, multiprocessing as mp, os, pickle, signal, subprocess, sys, time
from pathlib import Path

import numpy as np
import vamp

sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, limits
from v2_timing import ruckig_p1, toppra_p2
from v2_vamp_pass1 import SETS, run_one

ap = argparse.ArgumentParser(); ap.add_argument("--freeze", required=True); ap.add_argument("--deadline", type=float, default=0)
a = ap.parse_args()
load = subprocess.run(["ps", "-eo", "pcpu,args", "--sort=-pcpu"], capture_output=True, text=True).stdout.splitlines()[1:8]
check = {"top_processes_at_start": load, "slowdown": {}, "killed_sample": []}
for s in ("heldout", "mbm", "mpinets"):
    vm, pk, key = SETS[s]; probs = pickle.load(open(ROOT / pk, "rb"))["problems"][key]
    vmod, planner, ps, ss = vamp.configure_robot_and_planner_with_kwargs(vm, "rrtc"); sampler = vmod.halton()
    vmax, amax, jmax = limits(vm)
    p1 = [json.loads(l) for l in open(ROOT / f"results/v2/raw/vamp_pass1_{s}.jsonl")]
    out = ROOT / f"results/v2/raw/vamp_pass2a_{s}.jsonl"
    done = {(r["q"], r["seed_k"]) for r in map(json.loads, open(out))} if out.exists() else set()
    ratios = []
    for r in p1:
        if not r["plan_success"] or (r["q"], r["seed_k"]) in done:
            continue
        if a.deadline and time.time() > a.deadline:
            break
        p = probs[r["q"]]
        sampler.reset(); sampler.skip(r["seed"] % 100000)
        t0 = time.perf_counter_ns(); env = vamp.problem_dict_to_vamp(p)
        res = planner(p["start"], p["goals"], env, ps, sampler)
        rec = {k: r[k] for k in ("q", "index", "family", "seed_k", "seed", "set", "bins")}
        rec["plan_success"] = bool(res.solved)
        if res.solved:
            simple = vmod.simplify(res.path, env, ss, sampler); t1 = time.perf_counter_ns()
            path = np.array(simple.path.numpy(), float)
            t2 = time.perf_counter_ns(); ruckig_p1(path, vmax, amax, jmax); t3 = time.perf_counter_ns(); toppra_p2(path, vmax, amax); t4 = time.perf_counter_ns()
            rec.update(t_plan_ms=(t1 - t0) / 1e6, plan_internal_ms=(res.nanoseconds + simple.nanoseconds) / 1e6,
                       t_p1_ms=(t3 - t2) / 1e6, t_p2_ms=(t4 - t3) / 1e6, t_ready_ms=(t1 - t0) / 1e6 + (t3 - t2) / 1e6,
                       path_identical_to_pass1=bool(path.round(6).tolist() == r["path"]))
            ratios.append(r["contended_t_plan_ms"] / rec["t_plan_ms"])
        else:
            rec.update(t_plan_ms=(time.perf_counter_ns() - t0) / 1e6, path_identical_to_pass1=False)
        with open(out, "a") as f:
            f.write(json.dumps(rec) + "\n")
    if ratios:
        check["slowdown"][s] = {"n": len(ratios), "median": float(np.median(ratios)), "p95": float(np.percentile(ratios, 95)), "max": float(max(ratios))}
    print(s, "re-timed", len(ratios), check["slowdown"].get(s), flush=True)

# (ii) killed-at-60 s sample re-run alone with a 30 s kill (A4 assumption: they fail within T alone too)
vm, pk, key = SETS["heldout"]; probs = pickle.load(open(ROOT / pk, "rb"))["problems"][key]
killed = [r for r in map(json.loads, open(ROOT / "results/v2/raw/vamp_pass1_heldout.jsonl")) if "killed_at_s" in r]
rng = np.random.default_rng(0); sample = [killed[i] for i in rng.choice(len(killed), size=min(20, len(killed)), replace=False)]
ctx = mp.get_context("fork")
for r in sample:
    if a.deadline and time.time() > a.deadline:
        break
    rd, wr = ctx.Pipe(duplex=False); pr = ctx.Process(target=run_one, args=(vm, probs[r["q"]], r["seed"], wr)); pr.start(); wr.close()
    t0 = time.time(); res = None
    while time.time() - t0 < 30.0:
        if rd.poll(0.05):
            res = rd.recv(); break
    if res is None:
        os.kill(pr.pid, signal.SIGKILL)
    pr.join()
    ok = bool(res and res["plan_success"] and res["contended_t_plan_ms"] <= 30e3)
    check["killed_sample"].append({"q": r["q"], "seed_k": r["seed_k"], "solved_alone_within_T": ok,
                                   "alone_t_plan_ms": res["contended_t_plan_ms"] if res else None})
    print("killed sample", r["q"], r["seed_k"], "solved alone within T:", ok, flush=True)
check["killed_sample_solved_within_T"] = sum(x["solved_alone_within_T"] for x in check["killed_sample"])
json.dump(check, open(ROOT / "results/v2/pass2a_check.json", "w"), indent=1)
print("wrote results/v2/pass2a_check.json", check["slowdown"], "killed sample solved within T:", check["killed_sample_solved_within_T"])
