"""Rule the cuRoboV2 between-waypoint collision finding in or out (orchestrator review, 2026-10-07).

For every flagged V2 success (VAMP's segment check failed), plus up to N unflagged controls per scene:
  0. Re-plan the query with the same settings and reset_seed(); record whether it reproduces the stored trajectory.
  (a) Re-interpolate V2's own optimized B-spline (js_solution) at interpolation_dt / 10 with V2's own
      interpolator. Self-check: every 10th dense sample must match the dt=0.025 interpolated trajectory.
  (b) Check every dense sample with V2's own RobotCollisionChecker (same config, same scene) at activation
      distance 0 and at the planner's 0.0025, and with VAMP's point check. Record the maximum penetration.
  Also: agreement of V2's checker vs VAMP on 4000 random configurations per robot (identical spheres/radii).
Writes results/v2_collision_check.json.
Run: PYTHONPATH=bench:.vamp_only ~/projects/clean_env/venv/bin/python bench/verify_v2_collisions.py
"""
import json, pickle, sys, time
from pathlib import Path

import numpy as np
import torch
import yaml
import vamp
from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
from curobo.types import JointState
from curobo.collision_checking import RobotCollisionChecker, RobotCollisionCheckerCfg
from curobo._src.types.robot import RobotCfg

from mbm_world import problem_to_world

ROOT = Path(__file__).resolve().parents[1]
N_CONTROLS = int(sys.argv[1]) if len(sys.argv) > 1 else 5
CASES = [  # tag, curobo robot, vamp module, v2 config, problems
    ("mbm", "panda", "panda", "configs/panda_vamp_spheres_v2.yml", ROOT / "third_party/vamp/resources/panda/problems.pkl"),
    ("gantry_pillars", "ur5e_rail", "ur5erail", "configs/ur5e_rail_shared_v2.yml", ROOT / "problems/gantry_pillars.pkl"),
    ("ur5e_pillars", "ur5e", "ur5e", "configs/ur5e_shared_v2.yml", ROOT / "problems/ur5e_pillars.pkl"),
]
SETTINGS = dict(  # identical to bench/curobo2_mbm.py --preset benchmark
    ik_optimizer_configs=["ik/particle_ik.yml", "ik/lbfgs_ik.yml"],
    trajopt_optimizer_configs=["trajopt/particle_trajopt.yml", "trajopt/lbfgs_bspline_trajopt.yml"],
    num_trajopt_seeds=4, optimizer_collision_activation_distance=0.0025, use_cuda_graph=True,
    collision_cache={"obb": 50, "mesh": 20})
dev = lambda x: torch.as_tensor(np.asarray(x, dtype=np.float32), device="cuda:0")


def checker(robot_dict, world, act):
    return RobotCollisionChecker(RobotCollisionCheckerCfg.load_from_config(
        robot_config=RobotCfg.create(robot_dict), scene_model=world, collision_activation_distance=act,
        self_collision_activation_distance=0.0))


def v2_check(chk, q):
    """Return (valid mask, scene distance, self distance) for configs q (n, dof). Positive distance = penetration."""
    x = dev(q).view(-1, 1, q.shape[1])
    d_scene, d_self = chk.get_scene_self_collision_distance_from_joints(x)
    # V2 returns per-sphere distances; a configuration collides if any sphere penetrates (max over spheres > 0)
    n = q.shape[0]
    d_scene = d_scene.reshape(n, -1).max(dim=1).values.cpu().numpy()
    d_self = d_self.reshape(n, -1).max(dim=1).values.cpu().numpy()
    return (d_scene <= 0) & (d_self <= 0), d_scene, d_self


out = {"settings": SETTINGS, "cases": {}}
for tag, robot, vm, cfg, pk in CASES:
    raw = json.load(open(ROOT / f"results/raw/curobo2_{tag}_{robot}.json"))
    probs = pickle.load(open(pk, "rb"))["problems"]
    vmod = getattr(vamp, vm)
    rdict = yaml.safe_load(open(ROOT / cfg))["robot_cfg"]
    succ = [r for r in raw["records"] if r.get("success")]
    flagged = [r for r in succ if not r.get("vamp_validated")]
    controls = [r for r in succ if r.get("vamp_validated")][:N_CONTROLS]
    first = probs[flagged[0]["scene"] if flagged else succ[0]["scene"]][(flagged or succ)[0]["index"]]
    mp = MotionPlanner(MotionPlannerCfg.create(robot=rdict, scene_model=problem_to_world(first), **SETTINGS))
    mp.warmup(enable_graph=True)
    ts = mp.trajopt_solver
    dt0 = ts.config.interpolation_dt
    case = {"flagged": len(flagged), "controls": len(controls), "interpolation_dt": dt0, "queries": []}

    # (b) identical-model check: V2's own checker vs VAMP on random configurations of this scene
    p0 = probs[succ[0]["scene"]][succ[0]["index"]]
    env0 = vamp.problem_dict_to_vamp(p0)
    chk0 = checker(rdict, problem_to_world(p0), 0.0)
    lo, hi = np.array(vmod.lower_bounds()), np.array(vmod.upper_bounds())
    qr = np.random.default_rng(0).uniform(lo, hi, size=(4000, len(lo))).astype(np.float32)
    v_ok = np.array([vmod.validate(x.tolist(), env0) for x in qr]); c_ok = v2_check(chk0, qr)[0]
    case["random_config_agreement_v2_vs_vamp"] = {"n": 4000, "agree": float((v_ok == c_ok).mean()),
                                                  "vamp_only_valid": int((v_ok & ~c_ok).sum()), "v2_only_valid": int((~v_ok & c_ok).sum())}
    print(tag, "agreement", case["random_config_agreement_v2_vs_vamp"], flush=True)

    def dense_interp(js, factor):
        """V2's own interpolator at interpolation_dt / factor. The best solution is tiled across the seed batch so
        the solver's goal-buffer layout is unchanged; row 0 is returned."""
        nb = ts.core._goal_buffer.idxs_current_js.shape[0]
        t = js.clone()
        for f in ("position", "velocity", "acceleration", "jerk", "knot"):
            v = getattr(t, f, None)
            if v is not None:
                setattr(t, f, v.reshape(1, v.shape[-2], v.shape[-1]).repeat(nb, 1, 1))
        for f in ("dt", "knot_dt"):
            v = getattr(t, f, None)
            if v is not None:
                setattr(t, f, v.reshape(1).repeat(nb))
        ts.config.interpolation_dt = dt0 / factor; ts.config.interpolation_buffer_size = 5000 * factor
        ts._interpolation_dt_buffer = None; ts._interpolated_traj_buffer = None
        st, last, _ = ts.get_interpolated_trajectory(t)
        ts.config.interpolation_dt = dt0; ts.config.interpolation_buffer_size = 5000
        ts._interpolation_dt_buffer = None; ts._interpolated_traj_buffer = None
        n_ = int(last.view(-1)[0].item()) + 1
        return st.position[0, :n_].cpu().numpy()

    chk_act0 = None
    for kind, r in [("flagged", x) for x in flagged] + [("control", x) for x in controls]:
        p = probs[r["scene"]][r["index"]]
        world = problem_to_world(p); env = vamp.problem_dict_to_vamp(p)
        q = {"kind": kind, "scene": r["scene"], "index": r["index"]}
        chk0p = checker(rdict, world, 0.0); chkm = checker(rdict, world, 0.0025)
        # Part A: the stored trajectory (as benchmarked). Configs on the straight chord between consecutive
        # stored waypoints, sampled at 1/200, checked by VAMP and by V2's own checker.
        stored = np.asarray(r["path"], np.float32)
        chord = np.concatenate([stored[j] + (stored[j + 1] - stored[j]) * np.linspace(0, 1, 201)[:, None]
                                for j in range(len(stored) - 1)]).astype(np.float32)
        v_bad = ~np.array([vmod.validate(x.tolist(), env) for x in chord])
        ok0, ds0, dss0 = v2_check(chk0p, chord)
        q["A_stored"] = {"chord_samples": int(len(chord)), "vamp_colliding": int(v_bad.sum()),
                         "v2_colliding_act0": int((~ok0).sum()), "both": int((v_bad & ~ok0).sum()),
                         "v2_max_penetration_m": float(max(ds0.max(), dss0.max())),
                         "v2_colliding_at_waypoints_act0": int((~v2_check(chk0p, stored)[0]).sum())}
        # Part B: fresh plan of the same query, V2's own spline re-sampled 10x denser
        mp.clear_scene_cache(); mp.update_world(world); mp.reset_seed()
        res = mp.plan_cspace(JointState.from_position(dev([p["start"]]), joint_names=mp.joint_names),
                             JointState.from_position(dev([p["goals"][0]]), joint_names=mp.joint_names),
                             max_attempts=20, enable_graph_attempt=1)
        if res is None or not bool(res.success.view(-1)[0].item()):
            q["B_fresh"] = {"replan_success": False}
        else:
            dof = len(p["start"])
            interp = res.get_interpolated_plan().position.view(-1, dof).cpu().numpy()
            base = dense_interp(res.js_solution, 1); dense = dense_interp(res.js_solution, 10)
            k = min(len(base), (len(dense) - 1) // 10 + 1)
            segs = sum(not vmod.validate_motion(interp[j].tolist(), interp[j + 1].tolist(), env) for j in range(len(interp) - 1))
            # how far the spline leaves the chord between dt0 samples (joint space, rad)
            dev_max = 0.0
            for j in range(k - 1):
                seg = dense[10 * j:10 * j + 11]; a_, b_ = seg[0], seg[-1]
                lin = a_ + (b_ - a_) * np.linspace(0, 1, len(seg))[:, None]
                dev_max = max(dev_max, float(np.abs(seg - lin).max()))
            okd0, dsd, dssd = v2_check(chk0p, dense); okdm = v2_check(chkm, dense)[0]
            vd = ~np.array([vmod.validate(x.tolist(), env) for x in dense.astype(np.float32)])
            q["B_fresh"] = {"replan_success": True, "same_as_stored": bool(interp.shape == stored.shape and np.abs(interp - stored).max() < 1e-3),
                            "selfcheck_interp_vs_base_max_diff": float(np.abs(interp[:len(base)] - base[:len(interp)]).max()),
                            "selfcheck_dense_vs_base_max_diff": float(np.abs(dense[::10][:k] - base[:k]).max()),
                            "n_interp": int(len(interp)), "n_dense": int(len(dense)),
                            "vamp_interp_segments_failing": int(segs), "vamp_dense_colliding": int(vd.sum()),
                            "v2_dense_colliding_act0": int((~okd0).sum()), "v2_dense_within_margin_act0.0025": int((~okdm).sum()),
                            "v2_dense_max_penetration_m": float(max(dsd.max(), dssd.max())),
                            "spline_vs_chord_max_dev_rad": dev_max}
        case["queries"].append(q)
        print(tag, kind, r["index"], "A", q["A_stored"], "B", q["B_fresh"], flush=True)
    out["cases"][tag] = case
    del mp; torch.cuda.empty_cache()

json.dump(out, open(ROOT / "results/v2_collision_check.json", "w"), indent=1)
print("wrote results/v2_collision_check.json")
