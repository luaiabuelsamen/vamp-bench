"""Penetration depth of flagged cuRobo trajectories, for v1 and V2 alike, on the CPU.

For each cuRobo success that VAMP's segment check flags (bench/crossval.py), sample the straight chord between
consecutive waypoints at 1/200, place the robot's collision spheres with VAMP's FK (the shared model), and compute
the exact signed distance to every obstacle as VAMP models it: cuboids (oriented boxes), capsules (MBM cylinders),
spheres. depth = max(radius - distance) over spheres and samples (> 0 = penetration). Robot-obstacle only.
Validation: on V2's flagged trajectories this must reproduce V2's own checker (results/v2_collision_check.json).
Writes results/penetration_depth.json.
"""
import json, pickle
from pathlib import Path
import numpy as np
import vamp

ROOT = Path(__file__).resolve().parents[1]
SETS = [  # file stem, vamp module, problems
    ("mbm_panda", "panda", ROOT / "third_party/vamp/resources/panda/problems.pkl"),
    ("mbm_ur5", "ur5", ROOT / "third_party/vamp/resources/ur5/problems.pkl"),
    ("gantry_pillars_ur5e_rail", "ur5erail", ROOT / "problems/gantry_pillars.pkl"),
    ("gantry_free_ur5e_rail", "ur5erail", ROOT / "problems/gantry_free.pkl"),
    ("ur5e_pillars_ur5e", "ur5e", ROOT / "problems/ur5e_pillars.pkl"),
]


def rot_xyzw(q):
    x, y, z, w = q
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def obstacle_sdf(p):
    """Return f(points (n,3)) -> signed distance to the union of obstacles, matching vamp.problem_dict_to_vamp."""
    fns = []
    for o in p["sphere"]:
        c, r = np.array(o["position"]), o["radius"]
        fns.append(lambda P, c=c, r=r: np.linalg.norm(P - c, axis=1) - r)
    for o in p["cylinder"]:
        R, c = rot_xyzw(o["orientation_quat_xyzw"]), np.array(o["position"])
        if p["problem"] == "box":  # VAMP uses a cuboid with half extents (r, r, L/2) here
            h = np.array([o["radius"], o["radius"], o["length"] / 2])
            fns.append(lambda P, R=R, c=c, h=h: box_sdf((P - c) @ R, h))
        else:  # capsule, end points at +-L/2 on local z
            a, b = c + R @ [0, 0, -o["length"] / 2], c + R @ [0, 0, o["length"] / 2]
            fns.append(lambda P, a=a, b=b, r=o["radius"]: seg_dist(P, a, b) - r)
    for o in p["box"]:
        R, c, h = rot_xyzw(o["orientation_quat_xyzw"]), np.array(o["position"]), np.array(o["half_extents"])
        fns.append(lambda P, R=R, c=c, h=h: box_sdf((P - c) @ R, h))
    return lambda P: np.min(np.stack([f(P) for f in fns]), axis=0) if fns else np.full(len(P), np.inf)


def box_sdf(L, h):
    q = np.abs(L) - h
    return np.linalg.norm(np.maximum(q, 0), axis=1) + np.minimum(q.max(axis=1), 0)


def seg_dist(P, a, b):
    ab = b - a; t = np.clip(((P - a) @ ab) / (ab @ ab), 0, 1)
    return np.linalg.norm(P - (a + t[:, None] * ab), axis=1)


def depth(vmod, sdf, traj):
    t = np.linspace(0, 1, 201)[:, None]
    worst = -np.inf
    for j in range(len(traj) - 1):
        for q in traj[j] + (traj[j + 1] - traj[j]) * t:
            sp = vmod.fk(q.astype(np.float32).tolist())
            P = np.array([[s.x, s.y, s.z] for s in sp]); r = np.array([s.r for s in sp])
            worst = max(worst, float((r - sdf(P)).max()))
    return worst


out = {}
for stem, vm, pk in SETS:
    probs = pickle.load(open(pk, "rb"))["problems"]; vmod = getattr(vamp, vm)
    for planner in ("curobo", "curobo2"):
        f = ROOT / f"results/raw/{planner}_{stem}.json"
        if not f.exists():
            continue
        recs = [r for r in json.load(open(f))["records"] if r.get("success") and r.get("vamp_validated") is False]
        d = [depth(vmod, obstacle_sdf(probs[r["scene"]][r["index"]]), np.asarray(r["path"], np.float64)) for r in recs]
        out[f"{planner}_{stem}"] = {"flagged": len(recs), "depth_mm": [x * 1e3 for x in d],
                                    "keys": [[r["scene"], r["index"], r.get("trial", 0)] for r in recs]}
        if d:
            print(f"{planner}_{stem}: flagged {len(d)}  depth mm median {np.median(d)*1e3:.2f} max {max(d)*1e3:.2f}  "
                  f"self-collision-only (depth<=0) {sum(x <= 0 for x in d)}", flush=True)
json.dump(out, open(ROOT / "results/penetration_depth.json", "w"), indent=1)
print("wrote results/penetration_depth.json")
