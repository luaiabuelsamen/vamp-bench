"""Profile the v2 VAMP CPU step: per-stage wall time on a sample (held-out queries 0..N-1, seeds 0..S-1).
Stages: env build, rrtc (split solved / failed), simplify, P1 (Ruckig), P2 (TOPP-RA), 1 kHz sampling P1/P2,
validation P1/P2 (collision + limits), quality metrics. Writes results/v2/profile.json."""
import json, pickle, sys, time
from pathlib import Path
import numpy as np
import vamp
sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, limits, seed_from
from v2_timing import ruckig_p1, toppra_p2
import v2_validate

N, S = int(sys.argv[1]), int(sys.argv[2])
H = "6aea6eecf76a173d7ebdf548104aaec854b6b6c7"
probs = pickle.load(open(ROOT / "problems/v2_heldout.pkl", "rb"))["problems"]["v2_heldout"][:N]
vm = "ur5erail"; vmod, planner, ps, ss = vamp.configure_robot_and_planner_with_kwargs(vm, "rrtc"); sampler = vmod.halton()
vmax, amax, jmax = limits(vm)
T = {k: [] for k in ("env", "plan_solved", "plan_failed", "simplify", "p1", "p2", "sample_p1", "sample_p2", "validate_p1", "validate_p2")}
runs = []
clk = time.perf_counter
for qi, p in enumerate(probs):
    for k in range(S):
        sampler.reset(); sampler.skip(seed_from(H, f"seed{k}") % 100000)
        a = clk(); env = vamp.problem_dict_to_vamp(p); b = clk(); T["env"].append(b - a)
        res = planner(p["start"], p["goals"], env, ps, sampler); c = clk()
        T["plan_solved" if res.solved else "plan_failed"].append(c - b)
        runs.append({"q": qi, "seed_k": k, "solved": bool(res.solved), "plan_s": c - b})
        if not res.solved:
            continue
        simple = vmod.simplify(res.path, env, ss, sampler); d = clk(); T["simplify"].append(d - c)
        path = np.array(simple.path.numpy(), float)
        e = clk(); T1 = ruckig_p1(path, vmax, amax, jmax); f = clk(); T2 = toppra_p2(path, vmax, amax); g = clk()
        T["p1"].append(f - e); T["p2"].append(g - f)
        for name, TT in (("p1", T1), ("p2", T2)):
            if TT is None:
                continue
            h = clk(); q, qd, qdd = TT.sample(np.arange(0, TT.duration + 1e-9, 1e-3)); i = clk()
            v2_validate.validate(vm, p, q, qd, qdd, check_jerk=(name == "p1")); j = clk()
            T[f"sample_{name}"].append(i - h); T[f"validate_{name}"].append(j - i)
tot = {k: float(np.sum(v)) for k, v in T.items()}; grand = sum(tot.values())
out = {"sample": f"held-out q0..{N - 1} x seeds 0..{S - 1}", "runs": len(runs), "solved": sum(r["solved"] for r in runs),
       "stage_total_s": tot, "stage_share": {k: v / grand for k, v in tot.items()},
       "stage_mean_s": {k: float(np.mean(v)) if v else None for k, v in T.items()}, "runs_detail": runs}
json.dump(out, open(ROOT / "results/v2/profile.json", "w"), indent=1)
for k in tot:
    print(f"{k:12s} total {tot[k]:8.1f} s  share {tot[k] / grand:6.1%}  mean {out['stage_mean_s'][k] or 0:8.3f} s  n {len(T[k])}")
print("runs", len(runs), "solved", out["solved"], "total", round(grand, 1), "s")
