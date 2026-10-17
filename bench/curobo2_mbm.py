"""cuRoboV2 (NVlabs/curobo ac5f931, v0.8.0.post1.dev33) on the same problem sets, joint-space goals, shared sphere model.

Settings follow cuRoboV2's own benchmark/motion_plan_benchmark.py: particle + L-BFGS optimizers for IK and
trajopt, 4 trajopt seeds, collision activation distance 0.0025, CUDA graphs, warmup(enable_graph=True),
reset_seed() before every query, graph seeding from attempt 1. Its benchmark uses max_attempts=100 for pose
goals; we keep that for plan_cspace.

Run with the existing v2 build (editable install in ~/projects/clean_env/venv -> curobo_berkeley/curobo):
  PYTHONPATH=bench ~/projects/clean_env/venv/bin/python bench/curobo2_mbm.py --robot panda
"""
import argparse, json, pickle, platform, time
from pathlib import Path

import numpy as np
import torch
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import JointState
import curobo._version as cv

from mbm_world import problem_to_world

ROOT = Path(__file__).resolve().parents[1]
ap = argparse.ArgumentParser()
ap.add_argument("--robot", default="panda")
ap.add_argument("--robot-cfg", default=None)
ap.add_argument("--problems", default=None)
ap.add_argument("--tag", default="mbm")
ap.add_argument("--trials", type=int, default=1)
ap.add_argument("--limit", type=int, default=0)
ap.add_argument("--out", default=None)
ap.add_argument("--preset", default="benchmark", choices=["benchmark", "default"],
                help="benchmark = NVIDIA motion_plan_benchmark.py; default = MotionPlannerCfg.create defaults (vega_curobo)")
ap.add_argument("--max-attempts", type=int, default=100)
ap.add_argument("--stride", type=int, default=1, help="run problems with index %% stride == 0 (deterministic subset)")
ap.add_argument("--budget-s", type=float, default=0, help="stop starting new queries after this many seconds")
args = ap.parse_args()

data = pickle.load(open(args.problems or ROOT / f"third_party/vamp/resources/{args.robot}/problems.pkl", "rb"))
cfg_path = args.robot_cfg or str(ROOT / f"configs/{args.robot}_vamp_spheres_v2.yml")
import yaml
from robot_cfg import load_robot_cfg
robot = load_robot_cfg(cfg_path)

if args.preset == "benchmark":
    SETTINGS = dict(
        ik_optimizer_configs=["ik/particle_ik.yml", "ik/lbfgs_ik.yml"],
        trajopt_optimizer_configs=["trajopt/particle_trajopt.yml", "trajopt/lbfgs_bspline_trajopt.yml"],
        num_trajopt_seeds=4, optimizer_collision_activation_distance=0.0025, use_cuda_graph=True,
        collision_cache={"obb": 50, "mesh": 20},
    )
else:  # library defaults, as vega_curobo uses them (L-BFGS IK, L-BFGS B-spline trajopt, 4 seeds, 0.01 activation)
    SETTINGS = dict(use_cuda_graph=True, collision_cache={"obb": 50, "mesh": 20})
PLAN = dict(max_attempts=args.max_attempts, enable_graph_attempt=1)

first = next(p for ps in data["problems"].values() for p in ps if p["valid"])
mp = MotionPlanner(MotionPlannerCfg.create(robot=robot, scene_model=problem_to_world(first), **SETTINGS))
t = time.perf_counter(); mp.warmup(enable_graph=True); warmup_s = time.perf_counter() - t
dev = mp.device_cfg if hasattr(mp, "device_cfg") else None
to_dev = lambda x: torch.as_tensor(np.asarray(x, dtype=np.float32), device="cuda:0")

# Queries are interleaved across scenes (round-robin by index) so a budget cut keeps the scenes balanced.
queue = []
for scene, pset in data["problems"].items():
    for i, p in enumerate(pset):
        if (args.limit and i >= args.limit) or i % args.stride:
            continue
        queue.append((i, list(data["problems"]).index(scene), scene, p))
queue.sort(key=lambda x: (x[0], x[1]))
records, t_start, cut = [], time.perf_counter(), False
for i, _, scene, p in queue:
    if not p["valid"]:
        records.append({"scene": scene, "index": i, "valid": False}); continue
    if args.budget_s and time.perf_counter() - t_start > args.budget_s:
        cut = True; break
    for trial in range(args.trials):
        rec = {"scene": scene, "index": i, "trial": trial, "valid": True}
        torch.cuda.synchronize(); t0 = time.perf_counter()
        mp.clear_scene_cache()
        mp.update_world(problem_to_world(p))
        mp.reset_seed()
        torch.cuda.synchronize(); t1 = time.perf_counter()
        start = JointState.from_position(to_dev([p["start"]]), joint_names=mp.joint_names)
        goal = JointState.from_position(to_dev([p["goals"][0]]), joint_names=mp.joint_names)
        res = mp.plan_cspace(goal, start, **PLAN)
        torch.cuda.synchronize(); t2 = time.perf_counter()
        ok = res is not None and bool(res.success.view(-1)[0].item())
        rec.update(success=ok, status="ok" if ok else ("no_result" if res is None else "fail"),
                   world_update_us=(t1 - t0) * 1e6, plan_wall_us=(t2 - t1) * 1e6,
                   solve_time_us=float(res.solve_time) * 1e6 if res is not None else None,
                   total_time_us=float(res.total_time) * 1e6 if res is not None else None)
        if ok:
            q = res.get_interpolated_plan().position.view(-1, len(p["start"])).cpu().numpy()
            rec["path_length_l2"] = float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum())
            rec["n_waypoints"] = int(q.shape[0])
            rec["path"] = q.round(6).tolist()
        records.append(rec)
        print(scene, i, trial, ok, round((t2 - t1) * 1e3), "ms", flush=True)
ran = [r for r in records if r["valid"]]
print(f"done: {sum(r['success'] for r in ran)}/{len(ran)} solved, budget cut: {cut}", flush=True)

out = Path(args.out or ROOT / f"results/raw/curobo2_{args.tag}_{args.robot}.json")
out.parent.mkdir(parents=True, exist_ok=True)
json.dump({"planner": f"cuRoboV2 {cv.__version__} (NVlabs/curobo ac5f931) MotionPlanner.plan_cspace",
           "robot": args.robot, "robot_model": str(Path(cfg_path).resolve().relative_to(ROOT)),
           "problems": str(args.problems or "vamp resources"), "settings": SETTINGS, "plan_config": PLAN,
           "preset": args.preset, "subset": {"stride": args.stride, "limit": args.limit, "budget_s": args.budget_s, "budget_cut": cut, "queries_run": len(ran), "queries_in_subset": sum(1 for q in queue if q[3]["valid"])}, "warmup_s": warmup_s, "host": platform.node(), "torch": torch.__version__,
           "gpu": torch.cuda.get_device_name(0), "records": records}, open(out, "w"))
print("wrote", out)
