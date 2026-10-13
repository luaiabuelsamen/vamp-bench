"""Pick-place jerk-gate result for VAMP (docs/V2_PICKPLACE_PREREGISTRATION.md A7: jerk gate reported per planner).

The VAMP pick-place records store each segment's gate verdict but not its jerk ratio. This replays every attempted
segment of results/v2/raw/vamp_pickplace.A6.jsonl untimed (same planner, sampler seed, simplification, P1 Ruckig and
MoveL time-scaling; all deterministic) and records the 1 kHz validation output. Each replayed segment is checked against
the timed run (execution time to 1 us and the same verdict); mismatches are counted and reported, never dropped.
Writes results/v2/raw/vamp_pickplace.A6.jerk.jsonl (one row per attempted segment). CPU only.
"""
import json, pickle, sys
from pathlib import Path

import numpy as np
import vamp

sys.path.insert(0, str(Path(__file__).parent))
import v2_pickplace_vamp as V
from v2_common import ROOT, seed_from
from v2_pickplace_gen import LIFT

LASTV = {}
_validate = V.validate
def capture(*args, **kw):
    out = _validate(*args, **kw); LASTV.clear(); LASTV.update(out); return out
V.validate = capture

H = "6aea6eecf76a173d7ebdf548104aaec854b6b6c7"
recs = [json.loads(l) for l in open(ROOT / "results/v2/raw/vamp_pickplace.A6.jsonl")]
D = pickle.load(open(ROOT / "problems/v2_pickplace.pkl", "rb")); assert D["amendment"] == "A6"
cells = {c["index"]: c for c in D["cells"]}
_, planner, ps, ss = vamp.configure_robot_and_planner_with_kwargs(V.VM, "rrtc"); sampler = V.vmod.halton()
out = ROOT / "results/v2/raw/vamp_pickplace.A6.jerk.jsonl"
done = {(r["cell"], r["pick"], r["seed_k"], r["seg"]) for r in map(json.loads, open(out))} if out.exists() else set()
mism = 0
for r in sorted(recs, key=lambda x: (x["seed_k"], x["cell"], x["pick"])):
    c = cells[r["cell"]]; pk = c["picks"][r["pick"]]; g, pl = pk["grasp"], pk["place"]
    if all((r["cell"], r["pick"], r["seed_k"], i) in done for i, s in enumerate(r["segments"]) if s["kind"] != "not_attempted"):
        continue
    env = vamp.problem_dict_to_vamp(c); seed = seed_from(H, f"pickplace_seed{r['seed_k']}")
    cur = np.asarray(c["home"]) if r["pick"] == 0 else np.asarray(c["picks"][r["pick"] - 1]["place"]["q_above"])
    above_g = np.asarray(g["pos"]) + [0, 0, LIFT]; above_p = np.asarray(pl["pos"]) + [0, 0, LIFT]; lines = []
    steps = [lambda: V.free_segment(c, env, cur, g["q_above"], sampler, planner, ps, ss, seed),
             lambda: V.moveL_segment(c, env, g["q_above"], above_g, g["pos"], g["R"]),
             lambda: V.moveL_segment(c, env, None, g["pos"], above_g, g["R"], reuse=lines[-1][::-1]),
             lambda: V.free_segment(c, env, g["q_above"], pl["q_above"], sampler, planner, ps, ss, seed),
             lambda: V.moveL_segment(c, env, pl["q_above"], above_p, pl["pos"], pl["R"]),
             lambda: V.moveL_segment(c, env, None, pl["pos"], above_p, pl["R"], reuse=lines[-1][::-1])]
    rows = []
    for i, (step, s0) in enumerate(zip(steps, r["segments"])):
        if s0["kind"] == "not_attempted":
            break
        LASTV.clear(); res = step(); seg, line = res if isinstance(res, tuple) else (res, None)
        if line is not None:
            lines.append(line)
        match = seg["ok"] == s0["ok"] and abs(seg.get("t_exec_s", -1) - s0.get("t_exec_s", -1)) < 1e-6
        mism += not match
        rows.append({"cell": r["cell"], "pick": r["pick"], "seed_k": r["seed_k"], "seg": i, "kind": s0["kind"], "ok": s0["ok"],
                     "replay_matches": bool(match), "reason": s0.get("reason"),
                     **{k: LASTV.get(k) for k in ("valid", "colliding_samples", "bound_violation_rad", "max_vel_ratio", "max_acc_ratio", "max_jerk_ratio")}})
    with open(out, "a") as f:
        for row in rows:
            f.write(json.dumps(row) + "\n")
print("replay mismatches", mism, flush=True)
