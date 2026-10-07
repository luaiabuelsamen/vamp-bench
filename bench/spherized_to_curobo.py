"""Build a cuRobo v1 robot config from a VAMP spherized URDF + SRDF.

Both planners then check collisions against the identical sphere model:
same spheres, same radii, same self-collision pairs (SRDF disabled pairs are ignored by both),
and zero padding. This removes robot-geometry differences from the comparison.
"""
import sys
import xml.etree.ElementTree as ET
from collections import defaultdict

import yaml


def build(urdf, srdf, ee_link, retract, out):
    root = ET.parse(urdf).getroot()
    spheres = defaultdict(list)
    for link in root.findall("link"):
        for col in link.findall("collision"):
            g = col.find("geometry/sphere")
            if g is None:
                continue
            o = col.find("origin")
            xyz = [float(v) for v in o.get("xyz").split()] if o is not None and o.get("xyz") else [0, 0, 0]
            spheres[link.get("name")].append({"center": xyz, "radius": float(g.get("radius"))})

    joints = root.findall("joint")
    children = {j.find("child").get("link") for j in joints}
    base = next(l.get("name") for l in root.findall("link") if l.get("name") not in children)
    active = [j.get("name") for j in joints if j.get("type") in ("revolute", "prismatic", "continuous")]

    ignore = defaultdict(set)
    for d in ET.parse(srdf).getroot().findall("disable_collisions"):
        a, b = d.get("link1"), d.get("link2")
        if a in spheres and b in spheres:
            ignore[a].add(b)
    # Links on one rigid body (fixed joints) are never checked by VAMP/cricket (pinocchio merges them); match that.
    par = {}
    def find(x):
        while par.get(x, x) != x:
            x = par[x]
        return x
    for j in joints:
        if j.get("type") == "fixed":
            par[find(j.find("child").get("link"))] = find(j.find("parent").get("link"))
    names = sorted(spheres)
    for i, x in enumerate(names):
        for y in names[i + 1:]:
            if find(x) == find(y):
                ignore[x].add(y)
    # cricket applies an SRDF-disabled pair to the whole rigid bodies the two links belong to.
    body_pairs = {frozenset((find(a), find(b))) for a in list(ignore) for b in ignore[a]}
    for i, x in enumerate(names):
        for y in names[i + 1:]:
            if frozenset((find(x), find(y))) in body_pairs:
                ignore[x].add(y)
    links = sorted(spheres)
    n = len(active)
    cfg = {"robot_cfg": {"kinematics": {
        "urdf_path": urdf,
        "asset_root_path": str(__import__("pathlib").Path(urdf).parent),
        "base_link": base,
        "ee_link": ee_link,
        "collision_link_names": links,
        "collision_spheres": dict(spheres),
        "collision_sphere_buffer": 0.0,
        "self_collision_ignore": {k: sorted(v) for k, v in ignore.items()},
        "self_collision_buffer": {k: 0.0 for k in links},
        "use_global_cumul": True,
        "mesh_link_names": [],
        "lock_joints": None,
        "extra_links": None,
        "cspace": {
            "joint_names": active,
            "retract_config": retract,
            "null_space_weight": [1.0] * n,
            "cspace_distance_weight": [1.0] * n,
            "max_jerk": 500.0,
            "max_acceleration": 15.0,
            "position_limit_clip": 0.0,
        },
    }}}
    with open(out, "w") as f:
        yaml.safe_dump(cfg, f, sort_keys=False)
    print(f"{out}: base={base} dof={n} links={len(links)} spheres={sum(map(len, spheres.values()))}")


if __name__ == "__main__":
    urdf, srdf, ee, out = sys.argv[1:5]
    retract = [float(x) for x in sys.argv[5].split(",")]
    build(urdf, srdf, ee, retract, out)
