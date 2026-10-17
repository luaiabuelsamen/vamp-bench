"""v2.1 IK comparator: cuRobo v1 IKSolver on the 300 held-out A6 goal TCP poses (docs/BENCHMARK_V2.md section 7).

Same poses (tool0 of each A6 goal), same seed count (64) and the same success criterion as PyRoki's IK
(position error <= 1 mm, rotation error <= 1 deg, within joint bounds, collision-free under the shared VAMP check).
Robot config, world conversion and collision checker as in the v2 cuRobo v1 runs (configs/v2/ur5e_rail_v1.yml,
mbm_world.problem_to_world, MESH checker). Two configurations:
  default:        IKSolverConfig defaults as MotionGen builds them (position 5 mm, rotation 0.05), 64 seeds
  high_precision: position 1 mm, rotation 0.05 (cuRobo switches to high_precision at <= 1 mm), 64 seeds
cuRobo's returned solution is judged by the shared criterion; its own success flag is recorded too.
Warm-up on 3 pick-place targets outside the measured set, as for PyRoki; first-call time recorded separately.
Usage: PYTHONPATH=third_party/curobo_v1/src:bench:.vamp_only ~/projects/curobo_berkeley/venv/bin/python \
         bench/v2_1_curobo_ik.py --freeze H [--limit N] [--deadline T]
Writes results/v2_1/raw/curobo1_ik_heldout_a6.jsonl and .meta.json.
"""
import argparse, json, os, pickle, sys, time
from pathlib import Path

import numpy as np
import torch
import yaml
import vamp

sys.path.insert(0, str(Path(__file__).parent))
from mbm_world import problem_to_world
from v2_common import ROOT, seed_from, tcp

from curobo.geom.sdf.world import CollisionCheckerType
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig
from robot_cfg import load_robot_cfg

ap = argparse.ArgumentParser()
ap.add_argument("--freeze", required=True); ap.add_argument("--limit", type=int, default=0)
ap.add_argument("--deadline", type=float, default=0)
a = ap.parse_args()
VM = "ur5erail"; N_SEEDS = 64
probs = pickle.load(open(ROOT / "problems/v2_heldout_a6.pkl", "rb"))["problems"]["v2_heldout_a6"]
pp = pickle.load(open(ROOT / "problems/v2_pickplace.pkl", "rb"))["cells"]
robot = RobotConfig.from_dict(load_robot_cfg("configs/v2/ur5e_rail_v1.yml"))
seed = seed_from(a.freeze, "seed0") % (2 ** 31)
ta = TensorDeviceType()
OUT = ROOT / "results/v2_1/raw"; OUT.mkdir(parents=True, exist_ok=True)
out_f = OUT / "curobo1_ik_heldout_a6.jsonl"
done = {json.loads(l)["q"] for l in open(out_f)} if out_f.exists() else set()
lo, hi = np.array(vamp.ur5erail.lower_bounds()), np.array(vamp.ur5erail.upper_bounds())
CFGS = {"default": dict(position_threshold=0.005, rotation_threshold=0.05),
        "high_precision": dict(position_threshold=0.001, rotation_threshold=0.05, high_precision=True)}


def build(kw):
    cfg = IKSolverConfig.load_from_robot_config(robot, problem_to_world(probs[0]), num_seeds=N_SEEDS, self_collision_check=True,
                                                self_collision_opt=True, use_cuda_graph=True, tensor_args=ta,
                                                collision_checker_type=CollisionCheckerType.MESH,
                                                collision_cache={"obb": 60, "mesh": 30}, seed=seed, **kw)
    return IKSolver(cfg)


def goal(T):
    from scipy.spatial.transform import Rotation
    x, y, z, w = Rotation.from_matrix(T[:3, :3]).as_quat()
    return Pose(position=ta.to_device([T[:3, 3].tolist()]), quaternion=ta.to_device([[w, x, y, z]]))


def solve(ik, T):
    torch.cuda.synchronize(); t0 = time.perf_counter(); r = ik.solve_single(goal(T)); torch.cuda.synchronize()
    return r, time.perf_counter() - t0


def pose_err(q, T):
    Tq = tcp(VM, q); dp = float(np.linalg.norm(Tq[:3, 3] - T[:3, 3]) * 1e3)
    dr = float(np.degrees(np.arccos(np.clip((np.trace(T[:3, :3].T @ Tq[:3, :3]) - 1) / 2, -1, 1))))
    return dp, dr


solvers, meta = {}, {"seed": seed, "n_seeds": N_SEEDS, "configs": CFGS, "torch": torch.__version__,
                     "device": torch.cuda.get_device_name(0), "compile": {}}
for name, kw in CFGS.items():
    tb = time.perf_counter(); ik = build(kw); t_build = time.perf_counter() - tb
    Tw = [tcp(VM, pp[0]["picks"][k]["grasp"]["q"]) for k in range(3)]
    ik.update_world(problem_to_world(pp[0]))
    _, c = solve(ik, Tw[0]); _, w1 = solve(ik, Tw[1]); _, w2 = solve(ik, Tw[2])
    meta["compile"][name] = {"build_s": t_build, "first_call_s": c, "warm_s": [w1, w2]}
    solvers[name] = ik
print(json.dumps(meta), flush=True)

with open(out_f, "a") as f:
    for qi in [i for i in range(len(probs))[: a.limit or None] if i not in done]:
        if a.deadline and time.time() > a.deadline:
            print("deadline: stopping", flush=True); break
        p = probs[qi]; env = vamp.problem_dict_to_vamp(p); T = tcp(VM, p["goals"][0])
        rec = {"q": qi, "bins": {x: p["bins"][x] for x in ("reach", "clearance", "passage", "limits")}, "load1": os.getloadavg()[0]}
        for name, ik in solvers.items():
            tw = time.perf_counter(); ik.update_world(problem_to_world(p)); torch.cuda.synchronize(); t_world = time.perf_counter() - tw
            r, t = solve(ik, T)
            q = r.solution.view(-1, 7)[0].cpu().numpy().astype(float)
            dp, dr = pose_err(q, T)
            bounds = bool(np.all(q >= lo - 1e-5) and np.all(q <= hi + 1e-5))
            free = bool(vamp.ur5erail.validate(np.clip(q, lo, hi).astype(np.float32).tolist(), env))
            rec[name] = {"t_s": t, "t_world_s": t_world, "curobo_success": bool(r.success.view(-1)[0].item()),
                         "pos_err_mm": dp, "rot_err_deg": dr, "in_bounds": bounds, "collision_free": free,
                         "success": bool(dp <= 1.0 and dr <= 1.0 and bounds and free), "q": q.tolist(),
                         "dist_to_a6_goal_rad": float(np.abs(q - np.asarray(p["goals"][0])).max())}
        f.write(json.dumps(rec) + "\n"); f.flush()
        print(qi, *[(n, rec[n]["success"], rec[n]["curobo_success"], round(rec[n]["t_s"] * 1e3, 1)) for n in solvers], flush=True)
json.dump(meta, open(OUT / "curobo1_ik_heldout_a6.meta.json", "w"), indent=1)
