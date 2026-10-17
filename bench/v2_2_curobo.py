"""Benchmark v2.2, cuRobo v1 / cuRoboV2 (docs/V2_2_MOTION_TYPES_PREREGISTRATION.md, amendments V1, V2).

Expression (amendment V2): every straight line of a program (T: the line; W: each leg between consecutive points,
without the blends) is one planner call to the line's end pose (orientation R) from the previous call's last
configuration, with a native axis hold:
  native   the line's displacement along two axes of the goal or base frame is <= 1 mm (V2a): hold them and orientation
  plane    otherwise, along one axis: hold it and orientation
  orient   otherwise: hold orientation only
  v1: MotionGen.plan_single with PoseCostMetric(hold_partial_pose=True, hold_vec_weight=[1,1,1,px,py,pz],
      project_to_goal_frame=<goal frame?>) (plan_config frozen in v2, timeout T)
  V2: MotionPlanner.plan_pose with ToolPoseCriteria(non_terminal_pose_axes_weight_factor=[px,py,pz,1,1,1],
      project_distance_to_goal=<goal frame?>), NVIDIA's attempt loop unrolled with the budget check (as in v2)
Each call ends at rest, so a W program stops at every via point. C (MoveC) is not expressible and is not run, unless
--types includes C after probe P0 passes (V2 only; see bench/v2_2_p0.py).
The calls' 1 kHz trajectories are concatenated; the gate (bench/v2_validate.py) and the metrics (bench/v2_2_metrics.py)
are applied to the whole program against the programmed path and speed. Budget: 30 s per call (sum over legs <=
30 s x legs). T_ready = the sum of the calls (CUDA-synchronized); the per-cell update_world is timed separately.
Usage: PYTHONPATH=... <python> bench/v2_2_curobo.py --version v1|v2 --freeze H [--seeds 10] [--max-jerk 200]
       [--types W T] [--problems problems/v2_2_programs.pkl] [--deadline unix] [--out ...]
"""
import argparse, json, math, pickle, sys, time
from pathlib import Path

import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from mbm_world import problem_to_world
from v2_2_metrics import metrics
from v2_common import ROOT, seed_from
from v2_validate import validate
from robot_cfg import load_robot_cfg

ap = argparse.ArgumentParser()
ap.add_argument("--version", required=True, choices=["v1", "v2"]); ap.add_argument("--freeze", required=True)
ap.add_argument("--seeds", type=int, default=10); ap.add_argument("--limit", type=int, default=0)
ap.add_argument("--types", nargs="+", default=["W", "T"]); ap.add_argument("--out", default=None)
ap.add_argument("--deadline", type=float, default=0); ap.add_argument("--max-jerk", type=float, default=None)
ap.add_argument("--problems", default=str(ROOT / "problems/v2_2_programs.pkl"))
a = ap.parse_args()
VM, TOOL = "ur5erail", "tool0"
P = yaml.safe_load(open(ROOT / "configs/v2_planners.yaml")); T_BUDGET = float(P["budget"]["T_s"])
D = pickle.load(open(a.problems, "rb")); cells = D["cells"][: a.limit or None]
robot = load_robot_cfg(f"configs/v2/ur5e_rail_{a.version}.yml")
if a.max_jerk is not None:
    cs = robot["kinematics"]["cspace"]; cs["max_jerk"] = [a.max_jerk] * len(cs["max_jerk"]); print("planner max_jerk", cs["max_jerk"], flush=True)
sync = torch.cuda.synchronize
dev = lambda x: torch.as_tensor(np.asarray(x, np.float32), device="cuda:0")
HOLD_TOL = 0.001  # m, amendment V2a


def wxyz(R):
    x, y, z, w = Rotation.from_matrix(np.asarray(R)).as_quat()
    return [w, x, y, z]


def classify(p0, p1, R):
    """Amendment V2 rule, in displacement terms (V2a): a position axis of a frame can be held if the line's start-to-end
    displacement along it is <= 1 mm (v1 refuses a hold whose start and goal differ by > 5 mm on a held axis).
    native: two axes of one frame holdable (the line runs along the third); plane: one; orient: none.
    Returns (class, frame, position hold weights [px, py, pz])."""
    d = np.asarray(p1, float) - np.asarray(p0, float)
    frames = {"goal": np.asarray(R), "base": np.eye(3)}
    hold = {f: (np.abs(frames[f].T @ d) <= HOLD_TOL) for f in frames}
    f = max(("base", "goal"), key=lambda f: int(hold[f].sum()))  # ties: base frame
    n = int(hold[f].sum())
    if n >= 2:
        return "native", f, [float(x) for x in hold[f]]
    if n == 1:
        return "plane", f, [float(x) for x in hold[f]]
    return "orient", "base", [0.0, 0.0, 0.0]


def program_lines(prog):
    if "points" in prog:  # W: legs between consecutive points (no blends)
        pts = [np.asarray(x) for x in prog["points"]]
        return [(pts[i], pts[i + 1]) for i in range(len(pts) - 1)]
    pr = prog["prims"][0]
    return [(np.asarray(pr[2]), np.asarray(pr[3]))]


# ---------------------------------------------------------------- v1
def build_v1(seed):
    from curobo.geom.sdf.world import CollisionCheckerType
    from curobo.wrap.reacher.motion_gen import MotionGen, MotionGenConfig, MotionGenPlanConfig
    S = P["curobo_v1"]["settings"]
    cfg = MotionGenConfig.load_from_robot_config(
        robot, problem_to_world(cells[0]), collision_checker_type=CollisionCheckerType.MESH,
        collision_cache={"obb": 60, "mesh": 30}, use_cuda_graph=S["use_cuda_graph"],
        interpolation_dt=S["interpolation_dt"], interpolation_steps=S["interpolation_steps"],
        num_trajopt_seeds=S["num_trajopt_seeds"], num_graph_seeds=S["num_graph_seeds"], ik_seed=seed % (2 ** 31), graph_seed=seed % (2 ** 31))
    mg = MotionGen(cfg); mg.warmup(enable_graph=True, warmup_js_trajopt=True)
    pc = dict(P["curobo_v1"]["plan_config"]); pc["timeout"] = T_BUDGET
    return mg, MotionGenPlanConfig(**pc)


def world_v1(mg, c):
    mg.reset(reset_seed=True); mg.world_coll_checker.clear_cache(); mg.update_world(problem_to_world(c))


def line_v1(mg, pc, q0, pos, R, frame, w):
    from curobo.rollout.cost.pose_cost import PoseCostMetric
    from curobo.types.math import Pose
    from curobo.types.robot import JointState
    st = JointState.from_position(mg.tensor_args.to_device([list(q0)]))
    goal = Pose(position=mg.tensor_args.to_device([list(pos)]), quaternion=mg.tensor_args.to_device([wxyz(R)]))
    c = pc.clone()
    c.pose_cost_metric = PoseCostMetric(hold_partial_pose=True, hold_vec_weight=mg.tensor_args.to_device([1.0, 1.0, 1.0] + list(w)),
                                        project_to_goal_frame=(frame == "goal"))
    res = mg.plan_single(st, goal, c)
    if not bool(res.success.item()):
        return None, str(getattr(res.status, "name", res.status))
    tr = res.get_interpolated_plan(); dt = float(res.interpolation_dt)
    return (tr.position.cpu().numpy(), tr.velocity.cpu().numpy(), tr.acceleration.cpu().numpy(), dt), None


# ---------------------------------------------------------------- V2
def build_v2(seed):
    from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
    S = P["curobo_v2"]["settings"]
    mp = MotionPlanner(MotionPlannerCfg.create(robot=robot, scene_model=problem_to_world(cells[0]), collision_cache={"obb": 60, "mesh": 30},
                                               random_seed=seed % (2 ** 31), **S))
    ts = mp.trajopt_solver; ts.config.interpolation_dt = 0.001; ts.config.interpolation_buffer_size = 20000
    ts._interpolation_dt_buffer = None; ts._interpolated_traj_buffer = None
    mp.warmup(enable_graph=True)
    return mp, dict(P["curobo_v2"]["plan_config"])


def world_v2(mp, c):
    mp.clear_scene_cache(); mp.update_world(problem_to_world(c)); mp.reset_seed()


def line_v2(mp, pc, q0, pos, R, frame, w):
    from curobo.types import GoalToolPose, JointState, Pose, ToolPoseCriteria
    st = JointState.from_position(dev([q0]), joint_names=mp.joint_names)
    goal = GoalToolPose.from_poses({TOOL: Pose(position=dev([pos]), quaternion=dev([wxyz(R)]))}, ordered_tool_frames=[TOOL], num_goalset=1)
    mp.update_tool_pose_criteria({TOOL: ToolPoseCriteria(terminal_pose_axes_weight_factor=[1.0] * 6,
                                                         non_terminal_pose_axes_weight_factor=list(w) + [1.0, 1.0, 1.0],
                                                         project_distance_to_goal=(frame == "goal"))})
    try:
        t0 = time.perf_counter(); res = None
        for att in range(pc["max_attempts"]):
            if time.perf_counter() - t0 > T_BUDGET:
                break
            res = mp.plan_pose(goal, st, max_attempts=1, enable_graph_attempt=0 if att >= pc["enable_graph_attempt"] else 1)
            if res is not None and bool(res.success.view(-1)[0].item()):
                break
        if res is None or not bool(res.success.view(-1)[0].item()):
            return None, "not solved"
        tr = res.get_interpolated_plan()
        dt = getattr(tr, "dt", None)
        dt = float(dt.view(-1)[0].item()) if dt is not None and hasattr(dt, "view") else float(mp.trajopt_solver.config.interpolation_dt)
        return (tr.position.view(-1, 7).cpu().numpy(), tr.velocity.view(-1, 7).cpu().numpy(), tr.acceleration.view(-1, 7).cpu().numpy(), dt), None
    finally:
        mp.update_tool_pose_criteria({TOOL: ToolPoseCriteria()})


build, world, plan_line = {"v1": (build_v1, world_v1, line_v1), "v2": (build_v2, world_v2, line_v2)}[a.version]


def run_program(solver, pc, c, prog):
    R = np.asarray(c["R"]); q = np.asarray(c["q_start"]); legs, Q, QD, QDD = [], [], [], []
    for p0, p1 in program_lines(prog):
        cls, frame, w = classify(p0, p1, R)
        sync(); t0 = time.perf_counter(); tr, why = plan_line(solver, pc, q, p1, R, frame, w); sync(); t = time.perf_counter() - t0
        leg = {"class": cls, "frame": frame, "hold_pos": w, "t_s": t, "ok": tr is not None and t <= T_BUDGET}
        if tr is None:
            leg["reason"] = why; legs.append(leg); return legs, None
        qq, qd, qdd, dt = tr
        if abs(dt - 0.001) > 1e-9:
            raise SystemExit(f"interpolation dt {dt} != 0.001")
        leg["t_exec_s"] = float((len(qq) - 1) * dt); legs.append(leg)
        if not leg["ok"]:
            return legs, None
        Q.append(qq if not Q else qq[1:]); QD.append(qd if not QD else qd[1:]); QDD.append(qdd if not QDD else qdd[1:])
        q = qq[-1]
    return legs, (np.concatenate(Q), np.concatenate(QD), np.concatenate(QDD))


out = Path(a.out or ROOT / f"results/v2_2/raw/curobo{a.version[1]}{'' if a.max_jerk is None else f'_jerk{int(a.max_jerk)}'}.jsonl")
out.parent.mkdir(parents=True, exist_ok=True)
done = {(r["cell"], r["type"], r["seed_k"]) for r in map(json.loads, open(out))} if out.exists() else set()
stopped = False
for k in range(a.seeds):
    seed = seed_from(a.freeze, f"v22_seed{k}")
    if all((c["index"], t, k) in done for c in cells for t in a.types):
        continue
    if a.deadline and time.time() > a.deadline:
        stopped = True; break
    tb = time.perf_counter(); solver, pc = build(seed); warm = time.perf_counter() - tb
    for c in cells:
        if a.deadline and time.time() > a.deadline:
            stopped = True; break
        if all((c["index"], t, k) in done for t in a.types):
            continue
        sync(); tw = time.perf_counter(); world(solver, c); sync(); t_world = time.perf_counter() - tw
        for t in a.types:
            if (c["index"], t, k) in done:
                continue
            prog = c["programs"][t]
            legs, traj = run_program(solver, pc, c, prog)
            rec = {"cell": c["index"], "type": t, "seed_k": k, "seed": seed, "planner": f"curobo{a.version[1]}",
                   "planner_max_jerk": a.max_jerk or 500.0, "legs": legs, "t_world_s": t_world,
                   "t_ready_s": float(sum(l["t_s"] for l in legs)), "classes": sorted({l["class"] for l in legs})}
            if traj is None:
                rec.update(success=False, reason="leg failed")
            else:
                q, qd, qdd = traj
                v = validate(VM, c, q, qd, qdd)
                rec["gate"] = {kk: v[kk] for kk in ("valid", "colliding_samples", "bound_violation_rad", "max_vel_ratio", "max_acc_ratio", "max_jerk_ratio")}
                rec["t_exec_s"] = float((len(q) - 1) * 1e-3)
                m = metrics(q, prog, c["R"]); rec["metrics"] = m
                rec["success"] = bool(v["valid"] and m["end_pos_err_mm"] <= 1 and m["end_rot_err_deg"] <= 1
                                      and m["max_dev_outside_mm"] <= 5 and m["max_orient_dev_deg"] <= 1)
                rec["t_cycle_s"] = rec["t_ready_s"] + rec["t_exec_s"]
            with open(out, "a") as f:
                f.write(json.dumps(rec) + "\n")
            print(c["index"], t, k, rec["success"], rec["classes"], "%.2fs" % rec["t_ready_s"],
                  {kk: round(vv, 2) for kk, vv in rec.get("metrics", {}).items() if kk in ("max_dev_outside_mm", "cruise_dev_max") and vv is not None}, flush=True)
    print(f"seed {k} warmup {warm:.0f}s", flush=True)
    del solver; torch.cuda.empty_cache()
    if stopped:
        break
n = sum(1 for _ in open(out)) if out.exists() else 0
complete = n >= a.seeds * len(cells) * len(a.types)
print("wrote", out, n, "records", "complete" if complete else "PARTIAL", flush=True)
sys.exit(0 if complete else 3)
