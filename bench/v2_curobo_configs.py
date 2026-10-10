"""Benchmark v2: cuRobo v1 and V2 robot configs with the frozen shared limits (configs/v2_limits.yaml).

Geometry is unchanged from v1's shared models (VAMP spheres / paper spheres). Only limits change:
  - URDF copy with <limit velocity> per joint from v2_limits.yaml (cuRobo reads joint velocity from the URDF)
  - cspace.max_acceleration and cspace.max_jerk as per-joint lists, position_limit_clip 0
Writes configs/v2/{panda,ur5e_rail}_{v1,v2}.yml and configs/v2/{panda,ur5e_rail}.urdf
"""
import copy, sys, xml.etree.ElementTree as ET
from pathlib import Path
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path(__file__).parent))
L = yaml.safe_load(open(ROOT / "configs/v2_limits.yaml"))
OUT = ROOT / "configs/v2"; OUT.mkdir(exist_ok=True)
SRC = {"panda": ROOT / "configs/panda_vamp_spheres.yml", "ur5e_rail": ROOT / "configs/ur5e_rail_shared.yml"}

for name, src in SRC.items():
    lim = L[name]; cfg = yaml.safe_load(open(src)); k = cfg["robot_cfg"]["kinematics"]
    tree = ET.parse(k["urdf_path"]); vel = dict(zip(lim["joints"], lim["velocity"]))
    for j in tree.getroot().findall("joint"):
        if j.get("name") in vel:
            j.find("limit").set("velocity", repr(float(vel[j.get("name")])))
    urdf = OUT / f"{name}.urdf"; tree.write(urdf)
    k["urdf_path"] = str(urdf)
    order = k["cspace"]["joint_names"]; assert order == lim["joints"], (order, lim["joints"])
    k["cspace"]["max_acceleration"] = [float(x) for x in lim["acceleration"]]
    k["cspace"]["max_jerk"] = [float(x) for x in lim["jerk"]]
    k["cspace"]["position_limit_clip"] = 0.0
    yaml.safe_dump(cfg, open(OUT / f"{name}_v1.yml", "w"), sort_keys=False)
    c2 = copy.deepcopy(cfg); k2 = c2["robot_cfg"]["kinematics"]
    k2["format_version"] = 2.0; k2["tool_frames"] = [k2.pop("ee_link")]
    k2["cspace"]["default_joint_position"] = k2["cspace"].pop("retract_config")
    for drop in ("use_usd_kinematics", "use_global_cumul", "link_names", "usd_path", "usd_robot_root", "isaac_usd_path",
                 "usd_flip_joints", "usd_flip_joint_limits"):
        k2.pop(drop, None)
    yaml.safe_dump(c2, open(OUT / f"{name}_v2.yml", "w"), sort_keys=False)
    print(name, "velocity", lim["velocity"], "acc", lim["acceleration"], "jerk", lim["jerk"])
