"""cuRobo v1 on VAMP's MotionBenchMaker problems, joint-space goals, VAMP's sphere model.

Run with the cuRobo venv:  PYTHONPATH=third_party/curobo_v1/src:bench ~/projects/curobo_berkeley/venv/bin/python bench/curobo_mbm.py --robot panda
"""
import argparse, json, pickle, platform, time
from pathlib import Path

import numpy as np
import torch
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.types.robot import JointState
from curobo.util_file import load_yaml
from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig

from mbm_world import problem_to_world

ROOT = Path(__file__).resolve().parents[1]

ap = argparse.ArgumentParser()
ap.add_argument("--robot", default="panda")
ap.add_argument("--problems", default=None, help="pickle in VAMP MBM format (default: VAMP resources)")
ap.add_argument("--robot-cfg", default=None)
ap.add_argument("--tag", default="mbm")
ap.add_argument("--trials", type=int, default=1)
ap.add_argument("--limit", type=int, default=0, help="problems per scene (0 = all)")
ap.add_argument("--out", default=None)
args = ap.parse_args()

data = pickle.load(open(args.problems or ROOT / f"third_party/vamp/resources/{args.robot}/problems.pkl", "rb"))
robot_cfg_path = args.robot_cfg or str(ROOT / f"configs/{args.robot}_vamp_spheres.yml")
robot_cfg = load_yaml(robot_cfg_path)["robot_cfg"]

SETTINGS = dict(
    collision_checker_type="MESH", collision_cache={"obb": 50, "mesh": 20}, use_cuda_graph=True,
    interpolation_dt=0.025, num_trajopt_seeds=12, num_graph_seeds=12,
)
PLAN = dict(max_attempts=20, enable_graph_attempt=1, disable_graph_attempt=10, enable_finetune_trajopt=True, timeout=60.0)

first = next(p for ps in data["problems"].values() for p in ps if p["valid"])
cfg = MotionGenConfig.load_from_robot_config(
    robot_cfg, problem_to_world(first),
    collision_checker_type=CollisionCheckerType.MESH,
    **{k: v for k, v in SETTINGS.items() if k != "collision_checker_type"},
)
mg = MotionGen(cfg)
t = time.perf_counter(); mg.warmup(enable_graph=True, warmup_js_trajopt=True); warmup_s = time.perf_counter() - t
plan_cfg = MotionGenPlanConfig(**PLAN)
dev = mg.tensor_args

records = []
for scene, pset in data["problems"].items():
    for i, p in enumerate(pset):
        if args.limit and i >= args.limit:
            break
        if not p["valid"]:
            records.append({"scene": scene, "index": i, "valid": False})
            continue
        for trial in range(args.trials):
            rec = {"scene": scene, "index": i, "trial": trial, "valid": True}
            if trial:
                mg.reset(reset_seed=False)
            torch.cuda.synchronize(); t0 = time.perf_counter()
            mg.world_coll_checker.clear_cache()
            mg.update_world(problem_to_world(p))
            torch.cuda.synchronize(); t1 = time.perf_counter()
            start = JointState.from_position(dev.to_device([p["start"]]))
            goal = JointState.from_position(dev.to_device([p["goals"][0]]))
            res = mg.plan_single_js(start, goal, plan_cfg.clone())
            torch.cuda.synchronize(); t2 = time.perf_counter()
            ok = bool(res.success.item())
            rec.update(success=ok, status=str(res.status), world_update_us=(t1 - t0) * 1e6,
                       plan_wall_us=(t2 - t1) * 1e6, solve_time_us=float(res.solve_time) * 1e6,
                       attempts=int(res.attempts) if res.attempts is not None else None)
            if ok:
                q = res.get_interpolated_plan().position.cpu().numpy()
                rec["path_length_l2"] = float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum())
                rec["n_waypoints"] = int(q.shape[0])
                rec["motion_time_s"] = float(res.motion_time)
                rec["path"] = q.round(6).tolist()
            records.append(rec)
    done = [r for r in records if r["scene"] == scene and r["valid"]]
    print(scene, f"{sum(r['success'] for r in done)}/{len(done)}",
          "median plan ms", round(np.median([r["plan_wall_us"] for r in done]) / 1e3, 1), flush=True)

out = Path(args.out or ROOT / f"results/raw/curobo_{args.tag}_{args.robot}.json")
out.parent.mkdir(parents=True, exist_ok=True)
json.dump({
    "planner": "cuRobo v1 (NVlabs/curobo ebb7170, v0.7.7-5) MotionGen.plan_single_js",
    "robot": args.robot, "robot_model": str(Path(robot_cfg_path).resolve().relative_to(ROOT)), "problems": str(args.problems or "vamp resources"),
    "settings": SETTINGS, "plan_config": PLAN, "warmup_s": warmup_s,
    "host": platform.node(), "torch": torch.__version__, "gpu": torch.cuda.get_device_name(0),
    "records": records,
}, open(out, "w"))
print("wrote", out)
