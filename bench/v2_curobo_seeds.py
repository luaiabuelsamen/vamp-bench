"""v2.1 S2 seeds: the seed trajectories cuRobo v1 gives its joint-space trajectory optimiser, per attempt, on held-out A6.

Same settings, robot config, budget and random seed (seed_k 0) as the v2 run (bench/v2_curobo.py --version v1).
MotionGen.plan_single_js runs unchanged; js_trajopt_solver.get_seed_set is wrapped to record what it returns
(num_trajopt_seeds x action_horizon x dof) on every attempt, together with whether graph-planner seeds were passed in.
The plan's own outcome is recorded too and compared with the stored v2 record for seed_k 0 (a reproducibility check).
Usage: PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only ~/projects/curobo_berkeley/venv/bin/python \
         bench/v2_curobo_seeds.py --freeze H [--limit N] [--deadline T]
Writes results/v2_1/raw/curobo1_seeds_heldout_a6.npz (arrays per query) and .jsonl (per-query metadata).
"""
import argparse, json, pickle, sys, time
from pathlib import Path

import numpy as np
import torch
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from mbm_world import problem_to_world
from v2_common import ROOT, seed_from

ap = argparse.ArgumentParser()
ap.add_argument("--freeze", required=True); ap.add_argument("--limit", type=int, default=0)
ap.add_argument("--deadline", type=float, default=0)
a = ap.parse_args()
P = yaml.safe_load(open(ROOT / "configs/v2_planners.yaml"))
probs = pickle.load(open(ROOT / "problems/v2_heldout_a6.pkl", "rb"))["problems"]["v2_heldout_a6"]
idx = list(range(len(probs)))[: a.limit or None]
robot = load_robot_cfg("configs/v2/ur5e_rail_v1.yml")
seed = seed_from(a.freeze, "seed0")
OUT = ROOT / "results/v2_1/raw"; OUT.mkdir(parents=True, exist_ok=True)
meta_f = OUT / "curobo1_seeds_heldout_a6.jsonl"; npz_dir = OUT / "curobo1_seeds_heldout_a6"; npz_dir.mkdir(exist_ok=True)
stored = {r["q"]: r for r in map(json.loads, open(ROOT / "results/v2/raw/curobo1_heldout_a6.jsonl")) if r["seed_k"] == 0}
done = {json.loads(l)["q"] for l in open(meta_f)} if meta_f.exists() else set()

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.types.robot import JointState
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
from robot_cfg import load_robot_cfg

S = P["curobo_v1"]["settings"]
cfg = MotionGenConfig.load_from_robot_config(
    robot, problem_to_world(probs[0]), collision_checker_type=CollisionCheckerType.MESH,
    collision_cache={"obb": 60, "mesh": 30}, use_cuda_graph=S["use_cuda_graph"],
    interpolation_dt=S["interpolation_dt"], interpolation_steps=S["interpolation_steps"],
    num_trajopt_seeds=S["num_trajopt_seeds"], num_graph_seeds=S["num_graph_seeds"], ik_seed=seed % (2 ** 31), graph_seed=seed % (2 ** 31))
mg = MotionGen(cfg); mg.warmup(enable_graph=True, warmup_js_trajopt=True)
pc = dict(P["curobo_v1"]["plan_config"]); pc["timeout"] = float(P["budget"]["T_s"])
pcfg = MotionGenPlanConfig(**pc)
js = mg.js_trajopt_solver
CAP = []
_orig = js.get_seed_set


def _rec(goal, seed_traj=None, *args, **kw):
    out = _orig(goal, seed_traj, *args, **kw)
    CAP.append((out.detach().float().cpu().numpy().copy(), seed_traj is not None))
    return out


js.get_seed_set = _rec
info = {"action_horizon": int(js.action_horizon), "trajopt_dt": float(js.solver_dt.view(-1)[0].item()) if hasattr(js, "solver_dt") and js.solver_dt is not None else None,
        "seed_ratio": dict(js.seed_ratio), "num_seeds": int(js.num_seeds)}
print(json.dumps(info), flush=True)

with open(meta_f, "a") as mf:
    for qi in idx:
        if qi in done:
            continue
        if a.deadline and time.time() > a.deadline:
            print("deadline: stopping", flush=True); break
        p = probs[qi]; CAP.clear()
        mg.reset(reset_seed=True); mg.world_coll_checker.clear_cache(); mg.update_world(problem_to_world(p))
        st = JointState.from_position(mg.tensor_args.to_device([p["start"]])); gl = JointState.from_position(mg.tensor_args.to_device([p["goals"][0]]))
        torch.cuda.synchronize(); t0 = time.perf_counter()
        res = mg.plan_single_js(st, gl, pcfg.clone()); torch.cuda.synchronize(); t = time.perf_counter() - t0
        ok = bool(res.success.item())
        np.savez_compressed(npz_dir / f"q{qi:03d}.npz", seeds=np.stack([c[0] for c in CAP]) if CAP else np.zeros((0,)),
                            from_graph=np.array([c[1] for c in CAP]))
        lin = [bool(np.allclose(c[0], c[0][:1], atol=1e-6)) for c in CAP]
        r = {"q": qi, "seed": seed, "attempts_with_seeds": len(CAP), "from_graph": [c[1] for c in CAP], "all_seeds_identical": lin,
             "plan_success": ok, "status": str(res.status), "t_plan_s": t,
             "stored_v2": {"plan_success": stored.get(qi, {}).get("plan_success"), "status": stored.get(qi, {}).get("status")}, **info}
        mf.write(json.dumps(r) + "\n"); mf.flush()
        print(qi, ok, len(CAP), sum(r["from_graph"]), f"{t:.2f}s", "stored:", r["stored_v2"]["plan_success"], flush=True)
