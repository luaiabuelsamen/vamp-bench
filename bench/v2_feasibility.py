"""Long-budget feasibility check for v2 queries (orchestrator request; feeds amendment A3).
For each query given, VAMP rrtc with max_iterations and max_samples raised to ~300 s of planning (seed0), plus
VAMP PRM with the same budget as a second, structurally different planner. Solved by either => a path exists.
Untimed (feasibility only). Usage: python bench/v2_feasibility.py <set> <q> [<q> ...]  ->  results/v2/feasibility_<set>.jsonl"""
import json, pickle, sys, time
from pathlib import Path
import vamp
sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, seed_from
SETS = {"heldout": ("ur5erail", "problems/v2_heldout.pkl", "v2_heldout"), "mbm": ("panda", "problems/v2_mbm_panda.pkl", "v2_mbm"),
        "mpinets": ("panda", "problems/v2_mpinets_panda.pkl", "v2_mpinets")}
s = sys.argv[1]; vm, pk, key = SETS[s]; qs = [int(x) for x in sys.argv[2:]]
probs = pickle.load(open(ROOT / pk, "rb"))["problems"][key]
H = "6aea6eecf76a173d7ebdf548104aaec854b6b6c7"
out = ROOT / f"results/v2/feasibility_{s}.jsonl"
for q in qs:
    p = probs[q]; env = vamp.problem_dict_to_vamp(p); rec = {"set": s, "q": q}
    for name in ("rrtc", "prm"):
        vmod, planner, ps, ss = vamp.configure_robot_and_planner_with_kwargs(vm, name)
        for obj in (ps, getattr(ps, "rrtc", None)):
            if obj is not None and hasattr(obj, "max_iterations"):
                obj.max_iterations = 6_000_000; obj.max_samples = 6_000_000
        sampler = vmod.halton(); sampler.reset(); sampler.skip(seed_from(H, "seed0") % 100000)
        t = time.perf_counter(); r = planner(p["start"], p["goals"], env, ps, sampler); dt = time.perf_counter() - t
        rec[name] = {"solved": bool(r.solved), "s": dt}
        if r.solved:
            break
    rec["path_exists"] = any(rec.get(n, {}).get("solved") for n in ("rrtc", "prm"))
    print(rec, flush=True)
    with open(out, "a") as f:
        f.write(json.dumps(rec) + "\n")
