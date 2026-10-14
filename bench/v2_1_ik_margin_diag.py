"""v2.1 IK harness check (diagnostic, not a result): does the 5 mm hard world margin cause PyRoki IK (c)'s failures?

Re-solves IK variant (c) (04_ik_with_coll recipe, same 64 random starts, same selection) at the world margin given by
PYROKI_MARGIN on: the 15 held-out A6 poses where (c) failed in the run, plus every 10th pose (30) as a control.
Usage: PYROKI_MARGIN=0.0 bench/pyroki_env.sh python bench/v2_1_ik_margin_diag.py
Writes results/v2_1/ik_margin_diag_m<margin>.jsonl.
"""
import json, os, pickle, sys, time
from pathlib import Path

import jax, jax.numpy as jnp, jaxlie
import numpy as np
import vamp

sys.path.insert(0, str(Path(__file__).parent))
sys.argv = sys.argv[:1]
import v2_1_pyroki as V

probs = pickle.load(open(V.ROOT / "problems/v2_heldout_a6.pkl", "rb"))["problems"]["v2_heldout_a6"]
run = {json.loads(l)["q"]: json.loads(l) for l in open(V.ROOT / "results/v2_1/raw/pyroki_ik_gpu.jsonl")}
failed = sorted(q for q, r in run.items() if not r["coll_rand64"]["success"])
qs = sorted(set(failed) | set(range(0, 300, 10)))
rng = np.random.default_rng(V.seed_from("6aea6eecf76a173d7ebdf548104aaec854b6b6c7", "pyroki_ik"))
lo, hi = np.asarray(V.ROBOT.joints.lower_limits), np.asarray(V.ROBOT.joints.upper_limits)
inits = jnp.asarray(rng.uniform(lo, hi, size=(64, 7)))  # same draw as the run
out = open(V.ROOT / f"results/v2_1/ik_margin_diag_m{V.MARGIN}.jsonl", "w")
first = True
for qi in qs:
    p = probs[qi]; env = vamp.problem_dict_to_vamp(p); T = V.tcp(V.VM, p["goals"][0])
    arg = (jnp.asarray(jaxlie.SO3.from_matrix(jnp.asarray(T[:3, :3])).wxyz), jnp.asarray(T[:3, 3]))
    wc = V.padded_world(p, 40)
    f = lambda: V.ik_coll_batch(V.ROBOT, V.COLL, wc, jnp.array(V.LINK), *arg, inits)
    if first:
        jax.block_until_ready(f()); first = False
    t0 = time.perf_counter(); cand = np.asarray(jax.block_until_ready(f())); t = time.perf_counter() - t0
    errs = [V.pose_err(c, T) for c in cand]
    within = [e[0] <= 1.0 and e[1] <= 1.0 for e in errs]
    k = min((i for i in range(len(cand)) if within[i]), key=lambda i: errs[i][0] + errs[i][1]) if any(within) \
        else int(np.argmin([e[0] + e[1] for e in errs]))
    qb = cand[k]; dp, dr = errs[k]
    bounds = bool(np.all(qb >= np.array(vamp.ur5erail.lower_bounds()) - 1e-5) and np.all(qb <= np.array(vamp.ur5erail.upper_bounds()) + 1e-5))
    free = bool(vamp.ur5erail.validate(np.clip(qb, vamp.ur5erail.lower_bounds(), vamp.ur5erail.upper_bounds()).astype(np.float32).tolist(), env))
    rec = {"q": qi, "margin_m": V.MARGIN, "failed_in_run": qi in failed, "clearance_bin": p["bins"]["clearance"], "t_s": t,
           "pos_err_mm": dp, "rot_err_deg": dr, "collision_free": free, "in_bounds": bounds, "n_within": int(sum(within)),
           "success": bool(dp <= 1.0 and dr <= 1.0 and bounds and free), "run_success": run[qi]["coll_rand64"]["success"]}
    out.write(json.dumps(rec) + "\n"); out.flush()
    print(qi, rec["failed_in_run"], rec["success"], f"{dp:.2f}mm", free, f"{t:.2f}s", flush=True)
