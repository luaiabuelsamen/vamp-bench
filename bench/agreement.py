"""Verify the two planners share one collision model: classify random configurations with both checkers.

cuRobo: RobotWorld (world + self collision, zero activation distance) on the shared config.
VAMP:   <robot>.validate on the same environment.
usage: PYTHONPATH=third_party/curobo_v1/src:bench:.venv/lib/python3.10/site-packages <curobo python> bench/agreement.py
"""
import json, pickle
from pathlib import Path
import numpy as np, torch, vamp
from curobo.types.base import TensorDeviceType
from curobo.util_file import load_yaml
from curobo.wrap.model.robot_world import RobotWorld, RobotWorldConfig
from mbm_world import problem_to_world

ROOT = Path(__file__).resolve().parents[1]
TA = TensorDeviceType()
CASES = [("panda", "panda_vamp_spheres", ROOT / "third_party/vamp/resources/panda/problems.pkl", "bookshelf_small"),
         ("ur5", "ur5_vamp_spheres", ROOT / "third_party/vamp/resources/ur5/problems.pkl", "table_pick"),
         ("ur5erail", "ur5e_rail_shared", ROOT / "problems/gantry_pillars.pkl", "gantry_pillars"),
         ("ur5e", "ur5e_shared", ROOT / "problems/ur5e_pillars.pkl", "ur5e_pillars")]
out = {}
rng = np.random.default_rng(0)
for mod, cfgname, pk, scene in CASES:
    p = next(x for x in pickle.load(open(pk, "rb"))["problems"][scene] if x["valid"])
    vm = getattr(vamp, mod); env = vamp.problem_dict_to_vamp(p)
    cfg = load_yaml(str(ROOT / f"configs/{cfgname}.yml"))["robot_cfg"]
    rw = RobotWorld(RobotWorldConfig.load_from_config(cfg, problem_to_world(p), collision_activation_distance=0.0,
                                                      collision_checker_type=__import__("curobo.geom.sdf.world", fromlist=["x"]).CollisionCheckerType.MESH))
    lo, hi = np.array(vm.lower_bounds()) if hasattr(vm, "lower_bounds") else None, None
    lo = np.array(vm.lower_bounds()); hi = np.array(vm.upper_bounds())
    q = rng.uniform(lo, hi, size=(4000, len(lo))).astype(np.float32)
    v = np.array([vm.validate(x.tolist(), env) for x in q])
    d_world, d_self = rw.get_world_self_collision_distance_from_joints(TA.to_device(q))
    c = ((d_world.view(-1) <= 0) & (d_self.view(-1) <= 0)).cpu().numpy()
    agree = float((v == c).mean())
    out[mod] = {"n": len(q), "agree": agree, "vamp_valid": int(v.sum()), "curobo_valid": int(c.sum()),
                "vamp_only_valid": int((v & ~c).sum()), "curobo_only_valid": int((~v & c).sum())}
    print(mod, out[mod])
json.dump(out, open(ROOT / "results/collision_model_agreement.json", "w"), indent=1)
