"""Problems from the cuRobo-for-Vention paper scenes (arXiv 2508.04146), in VAMP's MBM pickle format.

Scenes (geometry taken from ~/projects/curobo_berkeley, see docs/METHOD.md):
  gantry_pillars  UR5e on a 1 m-range linear rail (ur5e_x.yml), floor + pillars col_1/col_2
                  (mujoco_scripts/profiling_script.py + assets/object/collision_gantry.xml)
  gantry_free     same robot, no obstacles (src/vention_curobo/vention_curobo/pick_and_place.py)
  ur5e_pillars    fixed-base UR5e (cuRobo ur5e.yml), table + pillars (demos.yml `ur5e_with_suction`,
                  assets/object/collision.xml)
For each scene: the paper's exact queries ("exact") plus 100 randomized pick->place transfers ("random")
with goals jittered around the paper's pick/place poses. Joint goals come from cuRobo's collision-free IK
(fixed seed); both planners then solve the same joint-space start->goal problem.
"""
import math, pickle
from pathlib import Path

import numpy as np
import torch
from curobo.geom.types import WorldConfig
from curobo.types.base import TensorDeviceType
from curobo.types.math import Pose
from curobo.types.robot import RobotConfig
from curobo.util_file import load_yaml
from curobo.wrap.reacher.ik_solver import IKSolver, IKSolverConfig

from mbm_world import problem_to_world
from robot_cfg import load_robot_cfg

ROOT = Path(__file__).resolve().parents[1]
torch.manual_seed(0); rng = np.random.default_rng(0)
TA = TensorDeviceType()
DOWN_GANTRY = [0.0, 0.0, -1.0, 0.0]          # wxyz, profiling_script.py
DOWN_ROLL_PI = [0.0, 1.0, 0.0, 0.0]          # wxyz, euler(pi,0,0), pick_and_place.py


def box(name, center, dims):
    return {"name": name, "half_extents": [d / 2 for d in dims], "position": list(center),
            "orientation_euler_xyz": (0.0, 0.0, 0.0), "orientation_quat_xyzw": [0.0, 0.0, 0.0, 1.0]}


SCENES = {
    "gantry_pillars": dict(robot="ur5e_rail", boxes=[box("floor", (0, 0, -0.14), (2.2, 2.2, 0.2)),
                                                     box("col_1", (0, 0.5, 0.23), (0.14, 0.14, 0.7)),
                                                     box("col_2", (0, -0.5, 0.23), (0.14, 0.14, 0.7))],
                           pick=(-0.5, -0.7, 0.0), place=(0.5, 0.7, 0.15), quat=DOWN_GANTRY,  # paper z=-0.03 is in floor collision (docs/METHOD.md)
                           home=[0.0] * 7, jitter=(0.15, 0.15, 0.10), zmin=0.0),
    "gantry_free": dict(robot="ur5e_rail", boxes=[],
                        pick=(-0.5, -0.5, 0.5), place=(0.5, 0.5, -0.1), quat=DOWN_ROLL_PI,
                        home=[0.0] * 7, jitter=(0.15, 0.15, 0.10), zmin=-0.2),
    "ur5e_pillars": dict(robot="ur5e", boxes=[box("table", (0, 0, -0.2), (2.2, 2.2, 0.2)),
                                              box("col_1", (0, 0.5, 0.35), (0.14, 0.14, 0.7)),
                                              box("col_2", (0, -0.5, 0.35), (0.14, 0.14, 0.7))],
                         pick=(-0.45, -0.45, 0.10), place=(0.45, 0.45, 0.20), quat=DOWN_GANTRY,
                         home=[0.0, -2.2, 1.9, -1.383, -1.57, 0.0], jitter=(0.10, 0.10, 0.10), zmin=0.02),
}


def quat_mul(a, b):
    w1, x1, y1, z1 = a; w2, x2, y2, z2 = b
    return [w1*w2 - x1*x2 - y1*y2 - z1*z2, w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2, w1*z2 + x1*y2 - y1*x2 + z1*w2]


def make_ik(robot, boxes):
    cfg = RobotConfig.from_dict(load_robot_cfg(f"configs/{robot}_shared.yml"), TA)
    p = {"problem": "x", "box": boxes or [box("dummy", (50, 50, 50), (0.01, 0.01, 0.01))], "cylinder": [], "sphere": []}
    ik = IKSolver(IKSolverConfig.load_from_robot_config(cfg, problem_to_world(p), num_seeds=64,
                                                        self_collision_check=True, use_cuda_graph=False,
                                                        position_threshold=0.005, rotation_threshold=0.05,
                                                        tensor_args=TA))
    return ik


def solve(ik, pos, quat):
    r = ik.solve_single(Pose(TA.to_device([list(pos)]), TA.to_device([list(quat)])))
    return r.solution[0, 0].cpu().numpy().astype(float).tolist() if bool(r.success.item()) else None


def entry(scene, idx, boxes, start, goal, kind, note):
    return {"problem": scene, "index": idx, "kind": kind, "note": note, "start": start, "goals": [goal],
            "valid": True, "box": boxes, "cylinder": [], "sphere": []}


for scene, S in SCENES.items():
    ik = make_ik(S["robot"], S["boxes"])
    pick_q = solve(ik, S["pick"], S["quat"]); place_q = solve(ik, S["place"], S["quat"])
    probs = []
    if pick_q and place_q:
        probs += [entry(scene, 0, S["boxes"], S["home"], pick_q, "exact", "home -> pick"),
                  entry(scene, 1, S["boxes"], pick_q, place_q, "exact", "pick -> place"),
                  entry(scene, 2, S["boxes"], place_q, pick_q, "exact", "place -> pick")]
    else:
        print(scene, "exact IK failed", pick_q is not None, place_q is not None)
    tries = 0
    while sum(p["kind"] == "random" for p in probs) < 100 and tries < 1000:
        tries += 1
        yaw = rng.uniform(-math.pi / 2, math.pi / 2)
        qz = [math.cos(yaw / 2), 0, 0, math.sin(yaw / 2)]
        ends = []
        for c in (S["pick"], S["place"]):
            pos = np.array(c) + rng.uniform(-1, 1, 3) * np.array(S["jitter"])
            pos[2] = max(pos[2], S["zmin"])
            ends.append(solve(ik, pos, quat_mul(qz, S["quat"])))
        if all(ends):
            probs.append(entry(scene, len(probs), S["boxes"], ends[0], ends[1], "random", "pick -> place (jittered)"))
    print(scene, len(probs), "problems,", tries, "random draws")
    out = ROOT / f"problems/{scene}.pkl"; out.parent.mkdir(exist_ok=True)
    pickle.dump({"robot": S["robot"], "problems": {scene: probs}}, open(out, "wb"))
