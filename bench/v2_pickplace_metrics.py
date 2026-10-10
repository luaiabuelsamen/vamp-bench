"""MoveL metrics shared by every pick-place runner (docs/V2_PICKPLACE_PREREGISTRATION.md), from the 1 kHz joint
trajectory with VAMP's FK (tool0, checked equal to the URDF's tool0): maximum TCP deviation from the nominal line
p0 -> p1, maximum orientation deviation from R, TCP speed over the cruise phase (middle 60% of the duration), and the
final TCP position error. FK on every 5th sample (5 ms)."""
import math

import numpy as np
import vamp

vmod = vamp.ur5erail
STRIDE = 5


def moveL_metrics(q, p0, p1, R, dt=1e-3):
    q = np.asarray(q, float)[::STRIDE]
    T = [np.asarray(vmod.eefk(x.astype(np.float32).tolist())) for x in q]
    P = np.array([t[:3, 3] for t in T]); p0 = np.asarray(p0, float); d = np.asarray(p1, float) - p0; d = d / np.linalg.norm(d)
    rel = P - p0; perp = rel - np.outer(rel @ d, d)
    ang = [math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(np.asarray(R).T @ t[:3, :3]) - 1) / 2)))) for t in T]
    speed = np.linalg.norm(np.diff(P, axis=0), axis=1) / (STRIDE * dt); n = len(speed)
    cruise = speed[int(0.2 * n): int(0.8 * n)]; vbar = float(np.mean(cruise)) if len(cruise) else 0.0
    return {"max_tcp_dev_mm": float(np.linalg.norm(perp, axis=1).max() * 1e3), "max_orient_dev_deg": float(max(ang)),
            "cruise_speed_mean_mps": vbar, "cruise_speed_max_mps": float(cruise.max()) if len(cruise) else 0.0,
            "cruise_speed_dev_pct": float(np.abs(cruise - vbar).max() / vbar * 100) if vbar > 0 else None,
            "final_pos_err_mm": float(np.linalg.norm(P[-1] - np.asarray(p1)) * 1e3)}
