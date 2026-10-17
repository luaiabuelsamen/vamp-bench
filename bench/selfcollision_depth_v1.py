"""Self-collision depth of cuRobo v1 UR5 trajectories flagged by VAMP's segment check but showing no obstacle
penetration (bench/penetration_depth.py). Uses cuRobo v1's own RobotWorld on the shared UR5 model
(configs/ur5_vamp_spheres.yml: VAMP's spheres, VAMP-equivalent self-collision pairs, 0 buffer), activation 0,
on the same straight chords (1/200) between consecutive returned waypoints. Positive distance = penetration.
Also reports the world (obstacle) distance as a cross-check against penetration_depth.json.
Run: PYTHONPATH=third_party/curobo_v1/src:bench ~/projects/curobo_berkeley/venv/bin/python bench/selfcollision_depth_v1.py
Writes results/selfcollision_depth_v1_ur5.json.
"""
import json, pickle
from pathlib import Path
import numpy as np
import torch
from curobo.geom.sdf.world import CollisionCheckerType
from curobo.types.base import TensorDeviceType
from curobo.util_file import load_yaml
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig
from mbm_world import problem_to_world
from robot_cfg import load_robot_cfg

ROOT = Path(__file__).resolve().parents[1]
TA = TensorDeviceType()
probs = pickle.load(open(ROOT / "third_party/vamp/resources/ur5/problems.pkl", "rb"))["problems"]
raw = json.load(open(ROOT / "results/raw/curobo_mbm_ur5.json"))["records"]
pdep = json.load(open(ROOT / "results/penetration_depth.json"))["curobo_mbm_ur5"]
obst = {tuple(k[:3]): d for k, d in zip(pdep["keys"], pdep["depth_mm"])}
cfg = load_robot_cfg("configs/ur5_vamp_spheres.yml")
flagged = [r for r in raw if r.get("success") and r.get("vamp_validated") is False]

rows = []
t = np.linspace(0, 1, 201)[:, None]
for r in flagged:
    p = probs[r["scene"]][r["index"]]
    rw = RobotWorld(RobotWorldConfig.load_from_config(cfg, problem_to_world(p), collision_activation_distance=0.0,
                                                      collision_checker_type=CollisionCheckerType.MESH))
    q = np.asarray(r["path"], np.float32)
    chord = np.concatenate([q[j] + (q[j + 1] - q[j]) * t for j in range(len(q) - 1)]).astype(np.float32)
    d_world, d_self = [], []
    for k in range(0, len(chord), 8192):
        dw, ds = rw.get_world_self_collision_distance_from_joints(TA.to_device(chord[k:k + 8192]))
        d_world.append(dw.view(-1).cpu().numpy()); d_self.append(ds.view(-1).cpu().numpy())
    d_world, d_self = np.concatenate(d_world), np.concatenate(d_self)
    wp_self = rw.get_world_self_collision_distance_from_joints(TA.to_device(q))[1].view(-1).cpu().numpy()
    rows.append({"scene": r["scene"], "index": r["index"], "obstacle_depth_mm_cpu": obst[(r["scene"], r["index"], r.get("trial", 0))],
                 "self_depth_mm": float(d_self.max()) * 1e3, "world_depth_mm_v1": float(d_world.max()) * 1e3,
                 "self_colliding_samples": int((d_self > 0).sum()), "self_colliding_waypoints": int((wp_self > 0).sum())})
    print(rows[-1], flush=True)
    del rw; torch.cuda.empty_cache()

json.dump({"source": "results/raw/curobo_mbm_ur5.json", "model": "configs/ur5_vamp_spheres.yml", "rows": rows},
          open(ROOT / "results/selfcollision_depth_v1_ur5.json", "w"), indent=1)
print("wrote results/selfcollision_depth_v1_ur5.json")
