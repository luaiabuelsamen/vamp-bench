"""Translate our cuRobo v1 robot configs to cuRoboV2's schema (format_version 2.0). Geometry is untouched:
same URDF, same spheres, same self-collision ignore pairs, zero buffer.
v1 -> v2 key changes: ee_link -> tool_frames, cspace.retract_config -> cspace.default_joint_position."""
import sys, yaml

src, dst = sys.argv[1:3]
cfg = yaml.safe_load(open(src))
k = cfg["robot_cfg"]["kinematics"]
k["format_version"] = 2.0
k["tool_frames"] = [k.pop("ee_link")]
k["cspace"]["default_joint_position"] = k["cspace"].pop("retract_config")
for drop in ("use_usd_kinematics", "usd_path", "usd_robot_root", "isaac_usd_path", "usd_flip_joints",
             "usd_flip_joint_limits", "use_global_cumul", "link_names"):
    k.pop(drop, None)
yaml.safe_dump(cfg, open(dst, "w"), sort_keys=False)
print(dst)
