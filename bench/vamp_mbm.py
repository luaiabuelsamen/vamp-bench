"""VAMP RRT-Connect (+ default simplification) on its MotionBenchMaker problems; per-problem JSON.

Run:  .venv/bin/python bench/vamp_mbm.py --robot panda
Times: VAMP's internal C++ timers (planning, simplification) and Python wall-clock including
building the collision environment, so it can be compared with cuRobo's wall-clock (which includes update_world).
"""
import argparse, json, pickle, platform, time
from pathlib import Path

import numpy as np
import vamp

ROOT = Path(__file__).resolve().parents[1]
ap = argparse.ArgumentParser()
ap.add_argument("--robot", default="panda")
ap.add_argument("--planner", default="rrtc")
ap.add_argument("--trials", type=int, default=1, help="seeds per problem (RNG skip offsets)")
ap.add_argument("--out", default=None)
ap.add_argument("--problems", default=None)
ap.add_argument("--tag", default="mbm")
args = ap.parse_args()

data = pickle.load(open(args.problems or ROOT / f"third_party/vamp/resources/{args.robot}/problems.pkl", "rb"))
vmod, planner, plan_settings, simp_settings = vamp.configure_robot_and_planner_with_kwargs(args.robot, args.planner)
sampler = vmod.halton()

records = []
for scene, pset in data["problems"].items():
    for i, p in enumerate(pset):
        if not p["valid"]:
            records.append({"scene": scene, "index": i, "valid": False}); continue
        for trial in range(args.trials):
            sampler.reset(); sampler.skip(trial * 100000)  # per-trial seed; outside the timed region
            t0 = time.perf_counter_ns()
            env = vamp.problem_dict_to_vamp(p)
            t1 = time.perf_counter_ns()
            res = planner(p["start"], p["goals"], env, plan_settings, sampler)
            rec = {"scene": scene, "index": i, "trial": trial, "valid": True, "success": bool(res.solved),
                   "env_build_us": (t1 - t0) / 1e3, "plan_us": res.nanoseconds / 1e3}
            if res.solved:
                simple = vmod.simplify(res.path, env, simp_settings, sampler)
                t2 = time.perf_counter_ns()
                q = np.array(simple.path.numpy())
                rec.update(simplify_us=simple.nanoseconds / 1e3, total_us=(res.nanoseconds + simple.nanoseconds) / 1e3,
                           wall_us=(t2 - t0) / 1e3, path_length_l2=float(simple.path.cost()),
                           initial_path_length_l2=float(res.path.cost()), n_waypoints=int(q.shape[0]),
                           path=q.round(6).tolist())
            records.append(rec)
    done = [r for r in records if r["scene"] == scene and r["valid"]]
    print(scene, f"{sum(r['success'] for r in done)}/{len(done)}",
          "median total us", np.median([r.get("total_us", np.nan) for r in done]), flush=True)

out = Path(args.out or ROOT / f"results/raw/vamp_{args.tag}_{args.robot}.json")
out.parent.mkdir(parents=True, exist_ok=True)
json.dump({"planner": f"VAMP {args.planner} + simplify (KavrakiLab/vamp 27cb9b6, v0.6.4), halton sampler, defaults",
           "robot": args.robot, "plan_settings": str(plan_settings), "host": platform.node(), "records": records},
          open(out, "w"))
print("wrote", out)
