"""Benchmark v2 step 4, cuRobo v1 / cuRoboV2: pick-place sequences with MoveL segments (docs/V2_PICKPLACE_PREREGISTRATION.md).

Same cells, picks, seeds, limits, per-segment budget T = 30 s (A2) and 1 kHz gate as bench/v2_pickplace_vamp.py.
  free   v1: MotionGen.plan_single_js (frozen v2 plan_config, timeout = T)
         V2: MotionPlanner.plan_cspace, attempt loop unrolled with a clock check before each attempt (as bench/v2_curobo.py)
  MoveL  v1: MotionGen.plan_single to the goal pose with PoseCostMetric(hold_partial_pose=True,
             hold_vec_weight=[1,1,1,1,1,0], project_to_goal_frame=True) (hold orientation + the two lateral axes)
         V2: plan_pose with ToolPoseCriteria.linear_motion(axis="z") on tool0, same unrolled attempt loop
Each segment starts from the last sample of the previous segment's own trajectory; a failed segment ends the pick and
the next pick starts from the failed pick's pre-place configuration (as for VAMP). Timed per segment: plan call(s),
CUDA-synchronized. The per-cell update_world is timed separately (t_world_s) and not counted in T_cycle, because
VAMP's environment construction is not timed either.
Usage: PYTHONPATH=... <python> bench/v2_pickplace_curobo.py --version v1|v2 --freeze H [--seeds 5] [--deadline unix]
Writes results/v2/raw/curobo<1|2>_pickplace.jsonl
"""
import argparse, json, pickle, sys, time
from pathlib import Path

import numpy as np
import torch
import yaml
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
from mbm_world import problem_to_world
from v2_common import ROOT, limits, seed_from
from v2_pickplace_metrics import moveL_metrics
from v2_validate import validate

ap = argparse.ArgumentParser()
ap.add_argument("--version", required=True, choices=["v1", "v2"]); ap.add_argument("--freeze", required=True)
ap.add_argument("--seeds", type=int, default=5); ap.add_argument("--limit", type=int, default=0)
ap.add_argument("--out", default=None); ap.add_argument("--deadline", type=float, default=0)
ap.add_argument("--problems", default=str(ROOT / "problems/v2_pickplace.pkl"))
ap.add_argument("--max-jerk", type=float, default=None, help="A7 J1: override the planner's max_jerk (rad/s^3); the gate is unchanged")
ap.add_argument("--jerk-continue", action="store_true", help="A7 J2: a segment failing only the jerk check does not end the pick")
a = ap.parse_args()
VM, LIFT, TOOL = "ur5erail", 0.10, "tool0"
P = yaml.safe_load(open(ROOT / "configs/v2_planners.yaml")); T_BUDGET = float(P["budget"]["T_s"])
D = pickle.load(open(a.problems, "rb")); cells = D["cells"][: a.limit or None]
assert abs(D["lift_m"] - LIFT) < 1e-12
robot = yaml.safe_load(open(ROOT / f"configs/v2/ur5e_rail_{a.version}.yml"))["robot_cfg"]
if a.max_jerk is not None:
    cs = robot["kinematics"]["cspace"]; cs["max_jerk"] = [a.max_jerk] * len(cs["max_jerk"]); print("A7: planner max_jerk", cs["max_jerk"], flush=True)
sync = torch.cuda.synchronize
dev = lambda x: torch.as_tensor(np.asarray(x, np.float32), device="cuda:0")


def wxyz(R):
    x, y, z, w = Rotation.from_matrix(np.asarray(R)).as_quat()
    return [w, x, y, z]


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


LAST = {}  # A6 diagnostics: the last planner result, read after timing


def diag(lim_j):
    """A6 diagnostics (labelled, not part of the results): planner status on failure (v1; V2's plan_cspace/plan_pose
    return None on failure), and on success the jerk of the optimizer-level trajectory (finite differences of its
    acceleration at its own dt) as a ratio to the limit, next to the 1 kHz interpolated value the gate uses."""
    out = {}
    try:
        res = LAST.get("res")
        if res is None:
            return {"diag_status": "no result object" if a.version == "v2" else None}
        st = getattr(res, "status", None)
        if st is not None:
            out["diag_status"] = str(getattr(st, "name", st))
        js = getattr(res, "optimized_plan", None) or getattr(res, "js_solution", None)
        dt = getattr(res, "optimized_dt", None)
        if dt is None and js is not None:
            dt = getattr(js, "dt", None)
        if js is not None and dt is not None and getattr(js, "acceleration", None) is not None:
            dtv = float(dt.reshape(-1)[0].item()) if hasattr(dt, "reshape") else float(dt)
            acc = js.acceleration.reshape(-1, js.acceleration.shape[-1])[:, :7].cpu().numpy()
            out["diag_opt_dt"] = dtv; out["diag_opt_n"] = int(len(acc))
            if len(acc) > 2:
                out["diag_opt_jerk_ratio_fd"] = float((np.abs(np.diff(acc, axis=0)) / dtv / lim_j).max())
            jk = getattr(js, "jerk", None)
            if jk is not None:
                out["diag_opt_jerk_ratio_reported"] = float((np.abs(jk.reshape(-1, jk.shape[-1])[:, :7].cpu().numpy()) / lim_j).max())
    except Exception as e:  # diagnostics must never break the run
        out["diag_error"] = repr(e)[:200]
    return out


def traj_v1(res):
    LAST["res"] = res
    if not bool(res.success.item()):
        return None
    tr = res.get_interpolated_plan(); dt = float(res.interpolation_dt)
    return tr.position.cpu().numpy(), tr.velocity.cpu().numpy(), tr.acceleration.cpu().numpy(), dt


def free_v1(mg, pc, q0, q1):
    from curobo.types.robot import JointState
    st = JointState.from_position(mg.tensor_args.to_device([list(q0)])); gl = JointState.from_position(mg.tensor_args.to_device([list(q1)]))
    return traj_v1(mg.plan_single_js(st, gl, pc.clone()))


def moveL_v1(mg, pc, q0, pos, R):
    from curobo.rollout.cost.pose_cost import PoseCostMetric
    from curobo.types.math import Pose
    from curobo.types.robot import JointState
    st = JointState.from_position(mg.tensor_args.to_device([list(q0)]))
    goal = Pose(position=mg.tensor_args.to_device([list(pos)]), quaternion=mg.tensor_args.to_device([wxyz(R)]))
    c = pc.clone()
    c.pose_cost_metric = PoseCostMetric(hold_partial_pose=True, hold_vec_weight=mg.tensor_args.to_device([1.0, 1.0, 1.0, 1.0, 1.0, 0.0]),
                                        project_to_goal_frame=True)
    return traj_v1(mg.plan_single(st, goal, c))


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


def unrolled_v2(mp, pc, call):
    """NVIDIA's attempt loop (max_attempts, graph seeding from attempt enable_graph_attempt), one attempt per call so
    the per-segment budget T is checked before each attempt (A2)."""
    t0 = time.perf_counter(); res = None
    for att in range(pc["max_attempts"]):
        if time.perf_counter() - t0 > T_BUDGET:
            break
        res = call(0 if att >= pc["enable_graph_attempt"] else 1); LAST["res"] = res
        if res is not None and bool(res.success.view(-1)[0].item()):
            break
    if res is None or not bool(res.success.view(-1)[0].item()):
        return None
    tr = res.get_interpolated_plan(); dof = 7
    dt = getattr(tr, "dt", None)
    dt = float(dt.view(-1)[0].item()) if dt is not None and hasattr(dt, "view") else float(mp.trajopt_solver.config.interpolation_dt)
    return tr.position.view(-1, dof).cpu().numpy(), tr.velocity.view(-1, dof).cpu().numpy(), tr.acceleration.view(-1, dof).cpu().numpy(), dt


def free_v2(mp, pc, q0, q1):
    from curobo.types import JointState
    st = JointState.from_position(dev([q0]), joint_names=mp.joint_names); gl = JointState.from_position(dev([q1]), joint_names=mp.joint_names)
    return unrolled_v2(mp, pc, lambda g: mp.plan_cspace(gl, st, max_attempts=1, enable_graph_attempt=g))


def moveL_v2(mp, pc, q0, pos, R):
    from curobo.types import GoalToolPose, JointState, Pose, ToolPoseCriteria
    st = JointState.from_position(dev([q0]), joint_names=mp.joint_names)
    goal = GoalToolPose.from_poses({TOOL: Pose(position=dev([pos]), quaternion=dev([wxyz(R)]))}, ordered_tool_frames=[TOOL], num_goalset=1)
    mp.update_tool_pose_criteria({TOOL: ToolPoseCriteria.linear_motion(axis="z")})
    try:
        return unrolled_v2(mp, pc, lambda g: mp.plan_pose(goal, st, max_attempts=1, enable_graph_attempt=g))
    finally:
        mp.update_tool_pose_criteria({TOOL: ToolPoseCriteria()})


build, world, free, moveL = {"v1": (build_v1, world_v1, free_v1, moveL_v1), "v2": (build_v2, world_v2, free_v2, moveL_v2)}[a.version]


def segment(kind, c, fn, line=None):
    """Run one planner call, time it, validate the returned trajectory at 1 kHz (same gate as VAMP)."""
    LAST.clear(); sync(); t0 = time.perf_counter(); tr = fn(); sync(); t = time.perf_counter() - t0
    rec = {"kind": kind, "t_ready_s": t}
    rec.update(diag(limits(VM)[2]))
    if tr is None:
        rec.update(ok=False, reason="planner failed"); return rec, None
    q, qd, qdd, dt = tr
    if abs(dt - 0.001) > 1e-9:
        raise SystemExit(f"interpolation dt {dt} != 0.001")
    v = validate(VM, c, q, qd, qdd)
    rec.update(t_exec_s=float((len(q) - 1) * dt), valid=bool(v["valid"]), colliding_samples=v["colliding_samples"],
               max_vel_ratio=v["max_vel_ratio"], max_acc_ratio=v["max_acc_ratio"], max_jerk_ratio=v["max_jerk_ratio"],
               end_speed=float(np.abs(qd[-1]).max()))
    if line is not None:
        rec.update(moveL_metrics(q, *line))
    rec["ok"] = bool(v["valid"] and t <= T_BUDGET)
    rec["ok_nojerk"] = bool(v["colliding_samples"] == 0 and v["bound_violation_rad"] <= 1e-5 and v["max_vel_ratio"] <= 1.01
                            and v["max_acc_ratio"] <= 1.01 and t <= T_BUDGET)
    return rec, q[-1]


out = Path(a.out or ROOT / f"results/v2/raw/curobo{a.version[1]}_pickplace.jsonl"); out.parent.mkdir(parents=True, exist_ok=True)
done = {(r["cell"], r["pick"], r["seed_k"]) for r in map(json.loads, open(out))} if out.exists() else set()
stopped = False
for k in range(a.seeds):
    seed = seed_from(a.freeze, f"pickplace_seed{k}")
    if all((c["index"], i, k) in done for c in cells for i in range(len(c["picks"]))):
        continue
    if a.deadline and time.time() > a.deadline:
        stopped = True; break
    tb = time.perf_counter(); solver, pc = build(seed); warm = time.perf_counter() - tb
    for c in cells:
        if a.deadline and time.time() > a.deadline:
            stopped = True; break
        if all((c["index"], i, k) in done for i in range(len(c["picks"]))):
            continue
        sync(); tw = time.perf_counter(); world(solver, c); sync(); t_world = time.perf_counter() - tw
        cur = np.asarray(c["home"])
        for i, pk in enumerate(c["picks"]):
            g, pl = pk["grasp"], pk["place"]
            if (c["index"], i, k) in done:
                cur = np.asarray(pl["q_above"]); continue
            above_g = np.asarray(g["pos"]) + [0, 0, LIFT]; above_p = np.asarray(pl["pos"]) + [0, 0, LIFT]
            plan = [("free", None, lambda q: free(solver, pc, q, g["q_above"])),
                    ("moveL", (above_g, g["pos"], g["R"]), lambda q: moveL(solver, pc, q, g["pos"], g["R"])),
                    ("moveL", (g["pos"], above_g, g["R"]), lambda q: moveL(solver, pc, q, above_g, g["R"])),
                    ("free", None, lambda q: free(solver, pc, q, pl["q_above"])),
                    ("moveL", (above_p, pl["pos"], pl["R"]), lambda q: moveL(solver, pc, q, pl["pos"], pl["R"])),
                    ("moveL", (pl["pos"], above_p, pl["R"]), lambda q: moveL(solver, pc, q, above_p, pl["R"]))]
            segs, q = [], cur
            for kind, line, fn in plan:
                seg, q_end = segment(kind, c, lambda: fn(q), line)
                segs.append(seg)
                if not seg["ok"] and not (a.jerk_continue and seg.get("ok_nojerk")):
                    break
                q = q_end
            segs += [{"kind": "not_attempted", "ok": False}] * (6 - len(segs))
            ok = all(s["ok"] for s in segs); ok_nj = all(s.get("ok_nojerk", False) for s in segs)
            rec = {"cell": c["index"], "pick": i, "seed_k": k, "seed": seed, "planner": f"curobo{a.version[1]}", "success": bool(ok),
                   "segments": segs, "t_world_s": t_world if i == 0 else 0.0,
                   "t_ready_s": float(sum(s.get("t_ready_s", 0) for s in segs)), "t_exec_s": float(sum(s.get("t_exec_s", 0) for s in segs))}
            rec["t_cycle_s"] = rec["t_ready_s"] + rec["t_exec_s"] if ok else None
            if a.jerk_continue or a.max_jerk is not None:
                rec.update(success_nojerk=bool(ok_nj), planner_max_jerk=a.max_jerk or 500.0, jerk_continue=a.jerk_continue)
            with open(out, "a") as f:
                f.write(json.dumps(rec) + "\n")
            cur = np.asarray(q if (ok or (a.jerk_continue and ok_nj)) else pl["q_above"])
        print(f"curobo{a.version} pickplace seed {k} cell {c['index']} done", flush=True)
    print(f"seed {k} warmup {warm:.0f}s", flush=True)
    del solver; torch.cuda.empty_cache()
    if stopped:
        break
n = sum(1 for _ in open(out)) if out.exists() else 0
complete = n >= a.seeds * sum(len(c["picks"]) for c in cells)
print("wrote", out, n, "records", "complete" if complete else "PARTIAL", flush=True)
sys.exit(0 if complete else 3)
