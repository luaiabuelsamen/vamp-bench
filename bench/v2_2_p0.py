"""Benchmark v2.2 probe P0 (docs/V2_2_MOTION_TYPES_PREREGISTRATION.md; amendment V2): can cuRoboV2 track a programmed
TCP path with per-timestep tool-pose targets?

On the 10 dev cells' W programs: the trajopt solver (MotionPlanner.trajopt_solver.solve_pose) gets a GoalToolPose
with horizon H = the solver's action horizon, the targets being the programmed path at its programmed time law
resampled to H steps, tool criteria track_position_and_orientation(non_terminal_scale), dt fixed to
duration / (H - 1) and no dt fine-tuning. Up to 3 settings (pre-registered limit):
  s1: non_terminal_scale 1.0, dt fixed     s2: non_terminal_scale 10.0, dt fixed     s3: non_terminal_scale 1.0, solver dt
Pass: the API accepts it and >= 6 of 10 results pass the 1 kHz gate with max path deviation outside blends <= 5 mm.
Writes results/v2_2/p0.jsonl. Usage: PYTHONPATH=... <clean_env python> bench/v2_2_p0.py --freeze H
"""
import argparse, json, pickle, sys, time, traceback
from pathlib import Path

import numpy as np
import torch
import yaml

sys.path.insert(0, str(Path(__file__).parent))
import v2_2_paths as G
from mbm_world import problem_to_world
from v2_2_metrics import metrics
from v2_common import ROOT, seed_from
from v2_validate import validate

ap = argparse.ArgumentParser(); ap.add_argument("--freeze", required=True); a = ap.parse_args()
VM, TOOL = "ur5erail", "tool0"
P = yaml.safe_load(open(ROOT / "configs/v2_planners.yaml"))
cells = pickle.load(open(ROOT / "problems/v2_2_dev.pkl", "rb"))["cells"]
robot = load_robot_cfg("configs/v2/ur5e_rail_v2.yml")
dev = lambda x: torch.as_tensor(np.asarray(x, np.float32), device="cuda:0")

from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import GoalToolPose, JointState, ToolPoseCriteria
from scipy.spatial.transform import Rotation
from robot_cfg import load_robot_cfg

S = P["curobo_v2"]["settings"]
mp = MotionPlanner(MotionPlannerCfg.create(robot=robot, scene_model=problem_to_world(cells[0]), collision_cache={"obb": 60, "mesh": 30},
                                           random_seed=seed_from(a.freeze, "v22_seed0") % (2 ** 31), **S))
ts = mp.trajopt_solver; ts.config.interpolation_dt = 0.001; ts.config.interpolation_buffer_size = 20000
ts._interpolation_dt_buffer = None; ts._interpolated_traj_buffer = None
mp.warmup(enable_graph=True)
H = int(ts.action_horizon); print("action horizon", H, flush=True)
out = open(ROOT / "results/v2_2/p0.jsonl", "w")
SETTINGS = [("s1", 1.0, True), ("s2", 10.0, True), ("s3", 1.0, False)]
for name, scale, fix_dt in SETTINGS:
    npass = 0
    for c in cells:
        prog = c["programs"]["W"]; R = np.asarray(c["R"])
        mp.clear_scene_cache(); mp.update_world(problem_to_world(c)); mp.reset_seed()
        prof = G.profile(G.length(prog["prims"]), prog["v"], prog["a"])
        tk = np.linspace(0, prof["duration"], H); sk = np.interp(tk, prof["t"], prof["s"])
        D, S_, _ = G.sample(prog["prims"], 0.0005)
        pts = np.array([D[min(np.searchsorted(S_, s), len(D) - 1)] for s in sk])
        x, y, z, w = Rotation.from_matrix(R).as_quat()
        goal = GoalToolPose(tool_frames=[TOOL], position=dev(pts).view(1, H, 1, 1, 3),
                            quaternion=dev(np.tile([w, x, y, z], (H, 1))).view(1, H, 1, 1, 4))
        st = JointState.from_position(dev([c["q_start"]]), joint_names=mp.joint_names)
        rec = {"setting": name, "non_terminal_scale": scale, "fixed_dt": fix_dt, "cell": c["index"], "horizon": H}
        try:
            mp.update_tool_pose_criteria({TOOL: ToolPoseCriteria.track_position_and_orientation(non_terminal_scale=scale)})
            kw = {"finetune_attempts": 0}
            if fix_dt:
                n = ts.config.num_seeds
                kw["dt"] = torch.full((1, n), float(prof["duration"] / (H - 1)), device="cuda:0")
            torch.cuda.synchronize(); t0 = time.perf_counter()
            res = ts.solve_pose(goal, st, **kw)
            torch.cuda.synchronize(); rec["t_s"] = time.perf_counter() - t0
            rec["solver_success"] = bool(res.success.view(-1)[0].item())
            tr = res.interpolated_trajectory
            q = tr.position.view(-1, tr.position.shape[-1])[:, :7].cpu().numpy()
            n_last = int(res.interpolated_last_tstep.view(-1)[0].item()) if res.interpolated_last_tstep is not None else len(q) - 1
            q = q[: n_last + 1]
            v = validate(VM, c, q); m = metrics(q, prog, R)
            rec.update(gate_valid=bool(v["valid"]), max_vel_ratio=v["max_vel_ratio"], max_acc_ratio=v["max_acc_ratio"],
                       max_jerk_ratio=v["max_jerk_ratio"], colliding=v["colliding_samples"], **{kk: m[kk] for kk in
                       ("max_dev_outside_mm", "max_dev_blend_mm", "end_pos_err_mm", "final_progress", "cruise_dev_max", "duration_s")})
            rec["pass"] = bool(v["valid"] and m["max_dev_outside_mm"] <= 5)
        except Exception as e:
            rec.update(error=repr(e)[:400], trace=traceback.format_exc()[-1500:], **{"pass": False})
        finally:
            mp.update_tool_pose_criteria({TOOL: ToolPoseCriteria()})
        npass += rec["pass"]
        out.write(json.dumps(rec) + "\n"); out.flush()
        print(name, c["index"], {k: (round(v, 3) if isinstance(v, float) else v) for k, v in rec.items() if k not in ("trace",)}, flush=True)
    print("setting", name, "pass", npass, "of", len(cells), "->", "PASS" if npass >= 6 else "fail", flush=True)
    if npass >= 6:
        break
