"""Amendment A3 reference runs, parallel and checkpointed (Orin only; Luai declined the Mac 2026-10-09).

Queries: every query of a set that no timed run has solved yet (VAMP pass (a) checkpoint + cuRobo v1 raw results;
cuRoboV2 successes certify feasibility on their own when they arrive, so they need no reference run).
Reference (untimed, deterministic): VAMP rrtc with max_iterations = max_samples = 6e6, then VAMP PRM with the same
caps, seed0. Caps are iteration counts, not wall time, so CPU contention changes duration but never the verdict.
Checkpoint: results/v2/raw/feasibility_<set>.jsonl (resume skips finished queries).
Usage: python bench/v2_feasibility_par.py --freeze H --workers N [--sets heldout mbm mpinets]
"""
import argparse, json, multiprocessing as mp, pickle, sys, time
from pathlib import Path

import vamp

sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, seed_from

SETS = {"heldout": ("ur5erail", "problems/v2_heldout.pkl", "v2_heldout"),
        "mbm": ("panda", "problems/v2_mbm_panda.pkl", "v2_mbm"),
        "mpinets": ("panda", "problems/v2_mpinets_panda.pkl", "v2_mpinets")}
CAP = 6_000_000


def reference(args):
    s, q, vm, p, seed = args
    env = vamp.problem_dict_to_vamp(p); rec = {"set": s, "q": q}
    for name in ("rrtc", "prm"):
        vmod, planner, ps, ss = vamp.configure_robot_and_planner_with_kwargs(vm, name)
        ps.max_iterations = CAP; ps.max_samples = CAP
        sampler = vmod.halton(); sampler.reset(); sampler.skip(seed % 100000)
        t = time.perf_counter(); r = planner(p["start"], p["goals"], env, ps, sampler)
        rec[name] = {"solved": bool(r.solved), "s": time.perf_counter() - t}
        if r.solved:
            break
    rec["path_exists"] = any(rec.get(n, {}).get("solved") for n in ("rrtc", "prm"))
    return rec


def solved_by_timed_runs(s):
    ok = set()
    f = ROOT / f"results/v2/raw/vamp_pass1_{s}.jsonl"
    if f.exists():
        ok |= {r["q"] for r in map(json.loads, open(f)) if r["plan_success"]}
    f = ROOT / f"results/v2/raw/curobo1_{s}.json"
    if f.exists():
        ok |= {r["q"] for r in json.load(open(f))["records"] if r.get("solved_any_time")}
    return ok


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--freeze", required=True); ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--sets", nargs="+", default=["heldout", "mbm", "mpinets"]); a = ap.parse_args()
    seed = seed_from(a.freeze, "seed0")
    for s in a.sets:
        vm, pk, key = SETS[s]; probs = pickle.load(open(ROOT / pk, "rb"))["problems"][key]
        pass1 = ROOT / f"results/v2/raw/vamp_pass1_{s}.jsonl"
        n_pass1 = sum(1 for _ in open(pass1)) if pass1.exists() else 0
        assert n_pass1 == 10 * len(probs), f"{s}: pass (a) incomplete ({n_pass1} runs); run reference runs after it"
        ck = ROOT / f"results/v2/raw/feasibility_{s}.jsonl"
        done = {r["q"] for r in map(json.loads, open(ck))} if ck.exists() else set()
        ok = solved_by_timed_runs(s)
        todo = [q for q in range(len(probs)) if q not in ok and q not in done]
        print(f"{s}: {len(ok)} queries already solved by a timed run, {len(done)} references done, {len(todo)} to run", flush=True)
        with mp.get_context("fork").Pool(a.workers) as pool:
            for rec in pool.imap_unordered(reference, [(s, q, vm, probs[q], seed) for q in todo]):
                with open(ck, "a") as f:
                    f.write(json.dumps(rec) + "\n")
                print(rec, flush=True)


if __name__ == "__main__":
    main()
