"""Robot configs store urdf_path and asset_root_path relative to the repository root; cuRobo needs absolute paths."""
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]


def load_robot_cfg(path):
    """The "robot_cfg" dict of a configs/*.yml file, with its kinematics paths made absolute."""
    cfg = yaml.safe_load(open(ROOT / path))["robot_cfg"]
    k = cfg["kinematics"]
    for key in ("urdf_path", "asset_root_path"):
        if k.get(key) and not Path(k[key]).is_absolute():
            k[key] = str(ROOT / k[key])
    return cfg
