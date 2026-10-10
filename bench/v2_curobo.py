"""Benchmark v2, GPU step: cuRobo v1 or V2 on the frozen v2 problem sets with the frozen settings and shared limits.

  --version v1   ~/projects/curobo_berkeley/venv + third_party/curobo_v1 (ebb7170); MotionGen.plan_single_js
  --version v2   ~/projects/clean_env/venv (ac5f931); MotionPlanner.plan_cspace
Settings: configs/v2_planners.yaml (v1: 12 seeds, 20 attempts; V2: NVIDIA benchmark, max_attempts 100 unless the
pre-registration is amended). Robot configs: configs/v2/<robot>_<version>.yml (shared spheres + datasheet limits).
Output trajectories are requested at interpolation_dt = 0.001 s and validated at 1 kHz on those samples with
bench/v2_validate.py (same gate as VAMP). Timed: update_world + plan (CUDA-synchronized wall clock).
Usage: PYTHONPATH=... <python> bench/v2_curobo.py --version v1|v2 --freeze H --set heldout|mbm|mpinets [--subset v2]
Writes results/v2/raw/curobo<1|2>_<set>.json
"""
import argparse, json, pickle, platform, sys, time
from pathlib import Path

import numpy as np
import torch
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from mbm_world import problem_to_world
from v2_common import ROOT, seed_from
from v2_validate import validate

ap = argparse.ArgumentParser()
ap.add_argument("--version", required=True, choices=["v1", "v2"]); ap.add_argument("--freeze", required=True)
ap.add_argument("--set", required=True, choices=["heldout", "mbm", "mpinets"]); ap.add_argument("--seeds", type=int, default=10)
ap.add_argument("--subset", default=None, help="json list of query indices (V2 stratified subsample)")
ap.add_argument("--limit", type=int, default=0); ap.add_argument("--max-attempts", type=int, default=None)
ap.add_argument("--out", default=None)
ap.add_argument("--deadline", type=float, default=0, help="unix time; stop starting new queries after it (partial results kept)")
ap.add_argument("--budget-s", type=float, default=None, help="uniform per-query wall budget T (amendment A2; default from configs/v2_planners.yaml)")
a = ap.parse_args()
VM, PK, KEY, RB = {"heldout": ("ur5erail", "problems/v2_heldout.pkl", "v2_heldout", "ur5e_rail"),
                   "mbm": ("panda", "problems/v2_mbm_panda.pkl", "v2_mbm", "panda"),
                   "mpinets": ("panda", "problems/v2_mpinets_panda.pkl", "v2_mpinets", "panda")}[a.set]
P = yaml.safe_load(open(ROOT / "configs/v2_planners.yaml"))
T_BUDGET = a.budget_s or float(P["budget"]["T_s"])
probs = pickle.load(open(ROOT / PK, "rb"))["problems"][KEY]
idx = json.load(open(a.subset))[a.set] if a.subset else list(range(len(probs)))
idx = idx[: a.limit or None]
robot = yaml.safe_load(open(ROOT / f"configs/v2/{RB}_{a.version}.yml"))["robot_cfg"]
seeds = [seed_from(a.freeze, f"seed{k}") for k in range(a.seeds)]
first = probs[idx[0]]
sync = torch.cuda.synchronize


def build_v1(seed):
    from curobo.geom.sdf.world import CollisionCheckerType
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
    S = P["curobo_v1"]["settings"]
    cfg = MotionGenConfig.load_from_robot_config(
        robot, problem_to_world(first), collision_checker_type=CollisionCheckerType.MESH,
        collision_cache={"obb": 60, "mesh": 30}, use_cuda_graph=S["use_cuda_graph"],
        interpolation_dt=S["interpolation_dt"], interpolation_steps=S["interpolation_steps"],
        num_trajopt_seeds=S["num_trajopt_seeds"], num_graph_seeds=S["num_graph_seeds"], ik_seed=seed % (2 ** 31), graph_seed=seed % (2 ** 31))
    mg = MotionGen(cfg); mg.warmup(enable_graph=True, warmup_js_trajopt=True)
    pc = dict(P["curobo_v1"]["plan_config"]); pc["timeout"] = T_BUDGET  # A2: v1's native per-query timeout = T
    if a.max_attempts:
        pc["max_attempts"] = a.max_attempts
    return mg, MotionGenPlanConfig(**pc), pc


def plan_v1(mg, pc, p):
    from curobo.types.robot import JointState
    mg.reset(reset_seed=True)
    sync(); t0 = time.perf_counter(); mg.world_coll_checker.clear_cache(); mg.update_world(problem_to_world(p))
    st = JointState.from_position(mg.tensor_args.to_device([p["start"]])); gl = JointState.from_position(mg.tensor_args.to_device([p["goals"][0]]))
    res = mg.plan_single_js(st, gl, pc.clone()); sync(); t1 = time.perf_counter()
    ok = bool(res.success.item())
    if not ok:
        return t1 - t0, False, str(res.status), None
    tr = res.get_interpolated_plan()
    return t1 - t0, True, "ok", (tr.position.cpu().numpy(), tr.velocity.cpu().numpy(), tr.acceleration.cpu().numpy(), float(res.interpolation_dt))


def build_v2(seed):
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
    S = P["curobo_v2"]["settings"]
    mp = MotionPlanner(MotionPlannerCfg.create(robot=robot, scene_model=problem_to_world(first), collision_cache={"obb": 60, "mesh": 30},
                                               random_seed=seed % (2 ** 31), **S))
    ts = mp.trajopt_solver; ts.config.interpolation_dt = 0.001; ts.config.interpolation_buffer_size = 20000
    ts._interpolation_dt_buffer = None; ts._interpolated_traj_buffer = None
    mp.warmup(enable_graph=True)
    pc = dict(P["curobo_v2"]["plan_config"])
    if a.max_attempts:
        pc["max_attempts"] = a.max_attempts
    return mp, pc, pc


def plan_v2(mp, pc, p):
    from curobo.types import JointState
    dev = lambda x: torch.as_tensor(np.asarray(x, np.float32), device="cuda:0")
    sync(); t0 = time.perf_counter(); mp.clear_scene_cache(); mp.update_world(problem_to_world(p)); mp.reset_seed()
    goal = JointState.from_position(dev([p["goals"][0]]), joint_names=mp.joint_names)
    start = JointState.from_position(dev([p["start"]]), joint_names=mp.joint_names)
    # A2: NVIDIA's plan_cspace loop (max_attempts, graph seeding from attempt enable_graph_attempt) unrolled one
    # attempt at a time so the uniform wall budget T can be checked before each attempt.
    res = None
    for att in range(pc["max_attempts"]):
        if time.perf_counter() - t0 > T_BUDGET:
            break
        res = mp.plan_cspace(goal, start, max_attempts=1, enable_graph_attempt=0 if att >= pc["enable_graph_attempt"] else 1)
        if res is not None and bool(res.success.view(-1)[0].item()):
            break
    sync(); t1 = time.perf_counter()
    ok = res is not None and bool(res.success.view(-1)[0].item())
    if not ok:
        return t1 - t0, False, "no_result" if res is None else "fail", None
    tr = res.get_interpolated_plan(); dof = len(p["start"])
    dt = getattr(tr, "dt", None)
    dt = float(dt.view(-1)[0].item()) if dt is not None and hasattr(dt, "view") else float(mp.trajopt_solver.config.interpolation_dt)
    return t1 - t0, True, "ok", (tr.position.view(-1, dof).cpu().numpy(), tr.velocity.view(-1, dof).cpu().numpy(),
                                 tr.acceleration.view(-1, dof).cpu().numpy(), dt)


build, plan = (build_v1, plan_v1) if a.version == "v1" else (build_v2, plan_v2)
out = Path(a.out or ROOT / f"results/v2/raw/curobo{a.version[1]}_{a.set}.json"); out.parent.mkdir(parents=True, exist_ok=True)
ckpt = out.with_suffix(".jsonl")  # one record per line; resume skips (seed_k, q) already present
recs = [json.loads(l) for l in open(ckpt)] if ckpt.exists() else []
done = {(r["seed_k"], r["q"]) for r in recs}
stopped = False
pcd = None
for k, sd in enumerate(seeds):
    todo = [qi for qi in idx if (k, qi) not in done]
    if not todo or stopped:
        continue
    if a.deadline and time.time() > a.deadline:
        stopped = True; break
    tb = time.perf_counter(); solver, pc, pcd = build(sd); warm = time.perf_counter() - tb
    for qi in todo:
        if a.deadline and time.time() > a.deadline:
            stopped = True; break
        p = probs[qi]
        t, ok, status, tr = plan(solver, pc, p)
        rec = {"q": qi, "index": p["index"], "family": p["problem"], "seed_k": k, "seed": sd, "status": status,
               "bins": {x: p["bins"][x] for x in ("reach", "clearance", "passage", "limits")}, "t_plan_ms": t * 1e3, "plan_success": ok}
        if ok:
            q, qd, qdd, dt = tr
            if abs(dt - 0.001) > 1e-9:
                raise SystemExit(f"interpolation dt {dt} != 0.001")
            v = validate(VM, p, q, qd, qdd); v["t_exec_s"] = float((len(q) - 1) * dt)
            rec["traj"] = v; rec["t_ready_ms"] = rec["t_plan_ms"]; rec["t_cycle_s"] = rec["t_plan_ms"] / 1e3 + v["t_exec_s"]
            rec["path_waypoints"] = q[:: max(1, len(q) // 50)].round(5).tolist()
        rec["solved_any_time"] = bool(ok and rec["traj"]["valid"])
        rec["solved"] = bool(rec["solved_any_time"] and rec["t_plan_ms"] <= T_BUDGET * 1e3)  # A2: success within T
        recs.append(rec)
        with open(ckpt, "a") as f:
            f.write(json.dumps(rec) + "\n")
    print(f"curobo{a.version} {a.set} seed {k}: solved {sum(r['solved'] for r in recs if r['seed_k'] == k)}/{len(idx)} warmup {warm:.0f}s", flush=True)
    del solver; torch.cuda.empty_cache()

complete = len({(r["seed_k"], r["q"]) for r in recs}) == len(seeds) * len(idx)
json.dump({"freeze": a.freeze, "complete": complete, "stopped_at_deadline": stopped, "set": a.set, "version": a.version, "plan_config": pcd, "settings": P[f"curobo_{a.version}"],
           "seeds": seeds, "subset": a.subset, "budget_T_s": T_BUDGET, "host": platform.node(), "gpu": torch.cuda.get_device_name(0), "records": recs}, open(out, "w"))
print("wrote", out, "complete" if complete else "PARTIAL", len(recs), "records")
sys.exit(0 if complete else 3)
