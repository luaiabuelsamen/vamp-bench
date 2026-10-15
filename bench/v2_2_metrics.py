"""Benchmark v2.2 metrics, identical for every planner (docs/V2_2_MOTION_TYPES_PREREGISTRATION.md, Metrics).

Input: the 1 kHz joint trajectory q (n, 7) and the program. TCP from VAMP's FK (tool0) at every sample.
  - path deviation: distance from the TCP to the programmed path (the polyline through dense samples every 0.5 mm,
    whose chord error is < 0.01 mm on the smallest blend; global nearest sample, then the two adjacent segments); max
    over samples whose nearest path point is in a blend zone ("blend") and over the rest ("outside");
  - progress: arc length of the nearest path point found in a window that moves forward with the TCP's progress (so a
    path passing near itself is not confused), used for the speed, blend and ramp metrics;
  - orientation deviation from the constant R (max), end pose error (position mm, rotation deg);
  - speed: |dP/dt| by central differences over 5 ms; tangential acceleration: d(speed)/dt over 5 ms;
  - cruise: samples whose progress is in [s_r1, s_r2], the programmed constant-speed span in arc length;
    mean and max |v - v_prog| / v_prog, and the mean cruise speed / v_prog;
  - blends (W): per blend zone, min speed / v_prog over samples whose progress is in the zone;
  - ramps (T, also reported for W, C): per programmed ramp (progress in [0, s_r1] and [s_r2, L]), the median
    | |a_t| - a_prog | / a_prog over samples outside the programmed jerk corners, and the time spent in the ramp's
    arc-length interval / the programmed ramp duration.
"""
import math

import numpy as np
import vamp
from scipy.spatial import cKDTree

import v2_2_paths as G

vmod = vamp.ur5erail
DENSE = 0.0005
WIN_BACK, WIN_FWD = 0.01, 0.10  # m of arc length searched around the previous progress


def fk_all(q):
    T = [np.asarray(vmod.eefk(np.asarray(x, np.float32).tolist())) for x in q]
    return np.array([t[:3, 3] for t in T]), np.array([t[:3, :3] for t in T])


def project(P, D, S):
    """Nearest dense path point for each TCP sample, searched in a forward-moving window. Returns index array."""
    idx = np.empty(len(P), int); cur = 0
    for i, p in enumerate(P):
        lo = np.searchsorted(S, S[cur] - WIN_BACK); hi = np.searchsorted(S, S[cur] + WIN_FWD, side="right")
        k = lo + int(np.argmin(np.einsum("ij,ij->i", D[lo:hi] - p, D[lo:hi] - p)))
        idx[i] = k; cur = k
    return idx


def seg_refine(P, D, S, idx):
    """Distance from each P to the polyline D near sample idx (segments idx-1..idx+1) and the arc length there."""
    dev = np.linalg.norm(P - D[idx], axis=1); s = S[idx].copy()
    for off in (-1, 0):
        i0 = np.clip(idx + off, 0, len(D) - 2); A, B = D[i0], D[i0 + 1]; AB = B - A
        t = np.clip(np.einsum("ij,ij->i", P - A, AB) / np.maximum(np.einsum("ij,ij->i", AB, AB), 1e-18), 0, 1)
        d = np.linalg.norm(P - (A + t[:, None] * AB), axis=1); better = d < dev
        dev[better] = d[better]; s[better] = (S[i0] + t * (S[i0 + 1] - S[i0]))[better]
    return dev, s


def metrics(q, prog, R, dt=1e-3):
    q = np.asarray(q, float)
    prims = prog["prims"]; v, a = prog["v"], prog["a"]
    D, S, lab = G.sample(prims, DENSE)
    blend = np.array([prims[k][1] == "blend" for k in lab])
    P, Rs = fk_all(q)
    # deviation: exact distance to the polyline through the dense samples (global nearest sample by KD-tree, then the
    # two adjacent segments); the blend label follows that nearest point
    gi = cKDTree(D).query(P)[1]
    dev, near = seg_refine(P, D, S, gi)
    inb = blend[np.clip(np.searchsorted(S, near), 0, len(S) - 1)]
    # progress (for speed, blends and ramps): the same refinement on the forward-moving windowed nearest sample
    _, prog_s = seg_refine(P, D, S, project(P, D, S))
    R = np.asarray(R)
    ang = np.degrees(np.arccos(np.clip((np.einsum("ij,nij->n", R, Rs) - 1) / 2, -1, 1)))
    L = S[-1]; end = D[-1]
    k5 = 5
    sp = np.full(len(P), np.nan)
    if len(P) > 2 * k5:
        sp[k5:-k5] = np.linalg.norm(P[2 * k5:] - P[:-2 * k5], axis=1) / (2 * k5 * dt)
    at = np.full(len(P), np.nan)
    if len(P) > 4 * k5:
        at[2 * k5:-2 * k5] = (sp[3 * k5:-k5] - sp[k5:-3 * k5]) / (2 * k5 * dt)
    prof = G.profile(L, v, a)
    out = {"n_samples": int(len(q)), "duration_s": float((len(q) - 1) * dt),
           "max_dev_outside_mm": float(dev[~inb].max() * 1e3) if (~inb).any() else 0.0,
           "max_dev_blend_mm": float(dev[inb].max() * 1e3) if inb.any() else None,
           "max_orient_dev_deg": float(ang.max()),
           "end_pos_err_mm": float(np.linalg.norm(P[-1] - end) * 1e3),
           "end_rot_err_deg": float(ang[-1]),
           "final_progress": float(prog_s[-1] / L)}
    cr = (prog_s >= prof["s_r1"]) & (prog_s <= prof["s_r2"]) & ~np.isnan(sp)
    if cr.sum() >= 3:
        rel = np.abs(sp[cr] - v) / v
        out.update(cruise_dev_max=float(rel.max()), cruise_dev_mean=float(rel.mean()), cruise_speed_ratio=float(sp[cr].mean() / v),
                   cruise_samples=int(cr.sum()))
    else:
        out.update(cruise_dev_max=None, cruise_dev_mean=None, cruise_speed_ratio=None, cruise_samples=int(cr.sum()))
    bl = []
    for s0, s1, kind in G.zones(prims):
        if kind != "blend":
            continue
        m = (prog_s >= s0) & (prog_s <= s1) & ~np.isnan(sp)
        bl.append(float(sp[m].min() / v) if m.any() else None)
    out["blend_min_speed_ratio"] = bl
    # ramps: exclude the programmed jerk corners (a / j seconds at each end of each ramp)
    tc = a / G.TCP_J
    ramps = []
    for (s0, s1, t_prog) in ((0.0, prof["s_r1"], prof["t_r1"]), (prof["s_r2"], L, prof["t_ramp_down"])):
        m = (prog_s >= s0) & (prog_s <= s1)
        t_in = float(m.sum() * dt)
        mm = m & ~np.isnan(at)
        ii = np.flatnonzero(mm)
        if len(ii) > 0:
            tt = (ii - ii[0]) * dt; keep = (tt > tc) & (tt < tt[-1] - tc)
            err = float(np.median(np.abs(np.abs(at[ii[keep]]) - a)) / a) if keep.any() else None
        else:
            err = None
        ramps.append({"acc_err_median": err, "duration_ratio": t_in / t_prog if t_prog > 0 else None})
    out["ramps"] = ramps
    return out
