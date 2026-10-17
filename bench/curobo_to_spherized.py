"""From a cuRobo v1 robot config (URDF + collision spheres), emit one shared sphere model for both planners.

Outputs (in configs/):
  <name>_spherized.urdf  URDF whose only collision geometry is the cuRobo spheres (radii + collision_sphere_buffer)
  <name>.srdf            disable_collisions = cuRobo self_collision_ignore  (+ links on the same rigid body)
  <name>_cricket.json    cricket input -> VAMP robot header
  <name>_shared.yml      cuRobo config pointing at the same URDF, buffer 0, ignore = the same pair set
Both planners therefore check the identical spheres and identical self-collision pairs.
"""
import argparse, copy, json
import xml.etree.ElementTree as ET
from collections import defaultdict
from pathlib import Path

import yaml

ap = argparse.ArgumentParser()
ap.add_argument("--yml", required=True)
ap.add_argument("--urdf", required=True)
ap.add_argument("--name", required=True)
ap.add_argument("--struct", required=True)
ap.add_argument("--limit", action="append", default=[], help="joint=lo,hi override (applied to both planners)")
a = ap.parse_args()
ROOT = Path(__file__).resolve().parents[1]
out = Path("configs").resolve()

kin = yaml.safe_load(open(a.yml))["robot_cfg"]["kinematics"]
buf = float(kin.get("collision_sphere_buffer") or 0.0)
spheres = kin["collision_spheres"]
if isinstance(spheres, str):
    raise SystemExit("external sphere file not supported")
links = list(kin["collision_link_names"])

tree = ET.parse(a.urdf); root = tree.getroot()
for t in root.findall("transmission") + root.findall("gazebo"):
    root.remove(t)
for link in root.findall("link"):
    for tag in ("collision", "visual"):
        for e in link.findall(tag):
            link.remove(e)
    for s in spheres.get(link.get("name"), []) if link.get("name") in links else []:
        r = float(s["radius"]) + buf
        if r <= 0:
            continue
        c = ET.SubElement(link, "collision")
        ET.SubElement(c, "origin", xyz=" ".join(f"{v:.6f}" for v in s["center"]), rpy="0 0 0")
        ET.SubElement(ET.SubElement(c, "geometry"), "sphere", radius=f"{r:.6f}")
lim = {k: v for k, v in (x.split("=") for x in a.limit)}
for j in root.findall("joint"):
    if j.get("name") in lim:
        lo, hi = lim[j.get("name")].split(",")
        j.find("limit").set("lower", lo); j.find("limit").set("upper", hi)
urdf_out = out / f"{a.name}_spherized.urdf"
tree.write(urdf_out)

# Rigid groups: links joined by fixed joints share a body; pinocchio/cricket never checks pairs within a body.
parent = {}
def find(x):
    while parent.get(x, x) != x:
        x = parent[x]
    return x
for j in root.findall("joint"):
    if j.get("type") == "fixed":
        parent[find(j.find("child").get("link"))] = find(j.find("parent").get("link"))
coll = [l for l in links if any(float(s["radius"]) + buf > 0 for s in spheres.get(l, []))]
pairs = set()
for k, vs in (kin.get("self_collision_ignore") or {}).items():
    for v in vs:
        if k in coll and v in coll:
            pairs.add(tuple(sorted((k, v))))
for i, x in enumerate(coll):
    for y in coll[i + 1:]:
        if find(x) == find(y):
            pairs.add(tuple(sorted((x, y))))
srdf = ET.Element("robot", name=root.get("name"))
for x, y in sorted(pairs):
    ET.SubElement(srdf, "disable_collisions", link1=x, link2=y, reason="Never")
ET.ElementTree(srdf).write(out / f"{a.name}.srdf")

json.dump({"name": a.struct, "urdf": urdf_out.name, "srdf": f"{a.name}.srdf", "end_effector": kin["ee_link"],
           "resolution": 32, "template": "templates/fk_template.hh",
           "subtemplates": [{"name": "ccfk", "template": "templates/ccfk_template.hh"}],
           "output": f"{a.name}.hh"}, open(out / f"{a.name}_cricket.json", "w"), indent=2)

shared = copy.deepcopy(kin)
ign = defaultdict(list)
for x, y in sorted(pairs):
    ign[x].append(y)
shared.update(urdf_path=str(urdf_out.resolve().relative_to(ROOT)), asset_root_path=str(out.resolve().relative_to(ROOT)), collision_link_names=coll,
              collision_spheres={l: spheres[l] for l in coll}, collision_sphere_buffer=0.0,
              self_collision_ignore=dict(ign), self_collision_buffer={l: 0.0 for l in coll},
              extra_collision_spheres=None, mesh_link_names=[], use_usd_kinematics=False)
shared["collision_spheres"] = {l: [{"center": s["center"], "radius": float(s["radius"]) + buf}
                                   for s in spheres[l] if float(s["radius"]) + buf > 0] for l in coll}
shared["cspace"]["position_limit_clip"] = 0.0
yaml.safe_dump({"robot_cfg": {"kinematics": shared}}, open(out / f"{a.name}_shared.yml", "w"), sort_keys=False)
print(a.name, "links", len(coll), "spheres", sum(len(v) for v in shared["collision_spheres"].values()),
      "ignored pairs", len(pairs), "buffer folded", buf)
