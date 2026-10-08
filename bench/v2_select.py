"""Benchmark v2: select the MBM and MpiNets Panda subsets (seeds from the freeze hash) and attach geometry bins.

  MBM      third_party/vamp/resources/panda/problems.pkl (sha256 737640df...), valid problems,
           7 families x 43 (first 300 kept in family round-robin order), seed select_mbm
  MpiNets  third_party/robometrics/mpinets_set.yaml (sha256 9189186d...; robometrics, MIT), 12 families x 25,
           seed select_mpinets; goal = first goal_ik valid under the shared model; start and goal must be valid
Writes problems/v2_mbm_panda.pkl and problems/v2_mpinets_panda.pkl in VAMP's problem format.
Usage: python bench/v2_select.py --freeze <hash>
"""
import argparse, hashlib, math, pickle, sys
from penetration_depth import rot_xyzw
from pathlib import Path

import numpy as np
import vamp
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, bins, seed_from

SHA = {"mbm": "737640dfba1fc8494147a0d139e1808e6996cd2fcec197eef63d2f5f3192ebfe",
       "mpinets": "9189186d83e51600a2c768aa7657933aa052150501852771994757052b797bbf"}


def sha(path):
    return hashlib.sha256(open(path, "rb").read()).hexdigest()


def quat_wxyz_to_xyzw_euler(qw):
    w, x, y, z = qw
    r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)); p = math.asin(max(-1, min(1, 2 * (w * y - z * x))))
    yw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [x, y, z, w], (r, p, yw)


def _aabb(o):
    R = rot_xyzw(o["orientation_quat_xyzw"]); c = np.array(o["position"])
    h = np.array(o["half_extents"]) if "half_extents" in o else np.array([o["radius"], o["radius"], o["length"] / 2 + o["radius"]])
    e = np.abs(R) @ h
    return c - e, c + e


def touches_fixed_base(o):
    """Amendment A1: obstacles overlapping the fixed Panda base region (|x|,|y| < 0.15 m, z < 0.05 m), i.e. the
    robot stand and the table piece under the base. They collide with panda_link0 in every configuration under
    VAMP's spheres and carry no planning information; removed for all planners."""
    lo, hi = _aabb(o)
    return lo[0] < 0.15 and hi[0] > -0.15 and lo[1] < 0.15 and hi[1] > -0.15 and lo[2] < 0.05


def mpinets_to_vamp(name, i, raw):
    d = {"problem": name, "index": i, "sphere": [], "cylinder": [], "box": []}
    for k, c in raw["obstacles"].get("cylinder", {}).items():
        xyzw, eul = quat_wxyz_to_xyzw_euler(c["pose"][3:])
        d["cylinder"].append({"name": k, "length": c["height"], "radius": c["radius"], "position": c["pose"][:3],
                              "orientation_euler_xyz": eul, "orientation_quat_xyzw": xyzw})
    for k, c in raw["obstacles"].get("cuboid", {}).items():
        xyzw, eul = quat_wxyz_to_xyzw_euler(c["pose"][3:])
        d["box"].append({"name": k, "half_extents": [v / 2 for v in c["dims"]], "position": c["pose"][:3],
                         "orientation_euler_xyz": eul, "orientation_quat_xyzw": xyzw})
    d["removed_base_obstacles"] = [o["name"] for o in d["box"] + d["cylinder"] if touches_fixed_base(o)]
    d["box"] = [o for o in d["box"] if not touches_fixed_base(o)]
    d["cylinder"] = [o for o in d["cylinder"] if not touches_fixed_base(o)]
    env = vamp.problem_dict_to_vamp(d)
    d["start"] = list(map(float, raw["start"]))
    goal = next((g for g in raw["goal_ik"] if vamp.panda.validate(list(map(float, g)), env)), None)
    d["valid"] = goal is not None and vamp.panda.validate(d["start"], env)
    d["goals"] = [list(map(float, goal))] if goal is not None else []
    return d


def pick(families, per_family, n, rng):
    """Uniform within family; families interleaved round-robin; first n kept."""
    chosen = {f: list(rng.permutation(len(v))[:per_family]) for f, v in families.items()}
    out = []
    for k in range(per_family):
        for f in sorted(families):
            if k < len(chosen[f]):
                out.append(families[f][chosen[f][k]])
    return out[:n]


def main(freeze):
    mbm_path = ROOT / "third_party/vamp/resources/panda/problems.pkl"
    mp_path = ROOT / "third_party/robometrics/mpinets_set.yaml"
    assert sha(mbm_path) == SHA["mbm"] and sha(mp_path) == SHA["mpinets"], "input data changed since the freeze"
    mbm = pickle.load(open(mbm_path, "rb"))["problems"]
    fam = {f: [dict(p, problem=f) for p in ps if p["valid"]] for f, ps in mbm.items()}
    sel = pick(fam, 43, 300, np.random.default_rng(seed_from(freeze, "select_mbm")))
    for p in sel:
        p["bins"] = bins("panda", p)
    pickle.dump({"robot": "panda", "problems": {"v2_mbm": sel}}, open(ROOT / "problems/v2_mbm_panda.pkl", "wb"))
    print("mbm", len(sel))
    raw = yaml.safe_load(open(mp_path))
    fam = {f: [q for q in (mpinets_to_vamp(f, i, r) for i, r in enumerate(ps)) if q["valid"]] for f, ps in raw.items()}
    print({f: len(v) for f, v in fam.items()})
    sel = pick(fam, 25, 300, np.random.default_rng(seed_from(freeze, "select_mpinets")))
    for p in sel:
        p["bins"] = bins("panda", p)
    pickle.dump({"robot": "panda", "problems": {"v2_mpinets": sel}}, open(ROOT / "problems/v2_mpinets_panda.pkl", "wb"))
    print("mpinets", len(sel))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--freeze", required=True)
    main(ap.parse_args().freeze)
