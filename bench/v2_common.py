"""Benchmark v2 shared definitions: limits, stratification bins (geometry only, computed before planning), seeds.

Bins (pre-registered, docs/V2_PREREGISTRATION.md):
  reach      max over {start, goal} of |TCP - shoulder| / reach        <0.5 | 0.5-0.8 | >0.8
  clearance  min over {start, goal} of robot-obstacle signed distance  <1 cm | 1-5 cm | >5 cm   (shared sphere model)
  passage    straight joint-space line start->goal collision-free?     free | blocked-wide | blocked-narrow
             (blocked-narrow: the generator placed a window narrower than 2x the largest arm-sphere diameter
             at the start or goal station; held-out set only. Every other blocked query is blocked-wide)
  limits     min over {start, goal} and joints of margin to a joint limit   <0.2 rad | >=0.2 rad
"""
import hashlib
from pathlib import Path

import numpy as np
import vamp
import yaml

from penetration_depth import obstacle_sdf

ROOT = Path(__file__).resolve().parents[1]
LIMITS = yaml.safe_load(open(ROOT / "configs/v2_limits.yaml"))
ROBOT = {  # vamp module -> (limits key, shoulder position fn, reach m)
    "panda": ("panda", lambda q: np.array([0.0, 0.0, 0.333]), 0.855),
    "ur5erail": ("ur5e_rail", lambda q: np.array([q[0], 0.0, 0.1625]), 0.85),
}


def seed_from(freeze_hash, label):
    return int(hashlib.sha256(f"{freeze_hash}:{label}".encode()).hexdigest()[:8], 16)


def limits(vm):
    L = LIMITS[ROBOT[vm][0]]
    return np.array(L["velocity"]), np.array(L["acceleration"]), np.array(L["jerk"])


def tcp(vm, q):
    return np.asarray(getattr(vamp, vm).eefk(np.asarray(q, np.float32).tolist()))


def clearance(vm, q, sdf):
    sp = getattr(vamp, vm).fk(np.asarray(q, np.float32).tolist())
    P = np.array([[s.x, s.y, s.z] for s in sp]); r = np.array([s.r for s in sp])
    return float((sdf(P) - r).min())


def bins(vm, p, window_ratio=None):
    vmod = getattr(vamp, vm); env = vamp.problem_dict_to_vamp(p); sdf = obstacle_sdf(p)
    s, g = np.asarray(p["start"]), np.asarray(p["goals"][0])
    _, shoulder, reach = ROBOT[vm]
    rn = max(np.linalg.norm(tcp(vm, q)[:3, 3] - shoulder(q)) / reach for q in (s, g))
    cl = min(clearance(vm, q, sdf) for q in (s, g))
    lo, hi = np.array(vmod.lower_bounds()), np.array(vmod.upper_bounds())
    lm = float(min(np.minimum(q - lo, hi - q).min() for q in (s, g)))
    free = bool(vmod.validate_motion(s.astype(np.float32).tolist(), g.astype(np.float32).tolist(), env))
    if free:
        passage = "free"
    elif window_ratio is not None and window_ratio < 2.0:
        passage = "blocked-narrow"
    else:
        passage = "blocked-wide"
    return {"reach": "<0.5" if rn < 0.5 else ("0.5-0.8" if rn <= 0.8 else ">0.8"),
            "clearance": "<1cm" if cl < 0.01 else ("1-5cm" if cl <= 0.05 else ">5cm"),
            "passage": passage, "limits": "<0.2rad" if lm < 0.2 else ">=0.2rad",
            "values": {"reach_norm": float(rn), "clearance_m": cl, "limit_margin_rad": lm, "line_free": free,
                       "window_ratio": window_ratio}}
