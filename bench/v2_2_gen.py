"""Benchmark v2.2: motion-type programs (docs/V2_2_MOTION_TYPES_PREREGISTRATION.md).

Cells: bench/v2_pickplace_gen.py's cell builder (bin, conveyor and fixture stations from gen_v2, home configuration),
seeded per cell with seed_from(H, "v22_cells_<k>") (dev cells: "v22_dev_<k>"); the first N valid k are kept.
Each cell gets one program of each type, all at the start station's tool-down orientation R (constant):
  W  waypoint path: start 0.10 m above station A -> 3-6 via points -> 0.10 m above station B, circular blends
     (bench/v2_2_paths.blended_polyline), v = 0.25 m/s, a = 1.0 m/s^2
  C  MoveC: an arc in the horizontal plane from the start, radius 0.10-0.30 m, sweep 90-270 deg, v 0.25, a 1.0
  T  straight line 0.30-0.60 m from the start toward the point above station B, v 0.25, a 0.25 (trapezoid)
Start configuration: IK at station A's sampled pose, the straight 0.10 m line up to the start point (as in the
pick-place cells), then every revolute joint moved to the 2pi branch nearest the cell's home (A6 rule). The rail
joint stays fixed along every path (path_ik), so each program is a fixed-base UR5e motion, as a UR controller runs it.
Validity: the path has a continuous collision-free IK solution from the start configuration (samples every 2 mm,
seeded by the previous sample, joint step <= 0.05 rad, VAMP validate_motion between samples, in bounds) and the
robot keeps >= 1 cm from every obstacle (shared sphere model, every 5th sample = 1 cm of path).
Writes problems/v2_2_programs.pkl (or v2_2_dev.pkl). Usage: python bench/v2_2_gen.py --freeze H [--dev] [--workers N]
"""
import argparse, math, multiprocessing as mp, pickle, sys
from pathlib import Path

import numpy as np
import vamp

sys.path.insert(0, str(Path(__file__).parent))
import gen_v2
import v2_2_paths as G
from penetration_depth import obstacle_sdf
from v2_common import ROOT, clearance, seed_from
from v2_pickplace_gen import LIFT, endpoint, check_endpoint, line_ik, make_cell
from v2_pickplace_gen import pose_err as pose_err_scipy


def pose_err(T, p, R):
    """v2_pickplace_gen.pose_err (position error and the log map of R T_R^T) in closed form: the same values without
    scipy's per-call SVD. Falls back to scipy within 1e-3 rad of a half turn."""
    M = R @ T[:3, :3].T
    c = np.clip((M[0, 0] + M[1, 1] + M[2, 2] - 1) / 2, -1.0, 1.0); th = math.acos(c)
    if th > math.pi - 1e-3:
        return pose_err_scipy(T, p, R)
    v = np.array([M[2, 1] - M[1, 2], M[0, 2] - M[2, 0], M[1, 0] - M[0, 1]])
    k = 0.5 if th < 1e-8 else th / (2 * math.sin(th))
    return np.concatenate([np.asarray(p) - T[:3, 3], k * v])

VM = "ur5erail"; vmod = getattr(vamp, VM)
STEP = 0.002          # path IK sample spacing (m)
MAX_JSTEP = 0.05      # rad between consecutive samples
MIN_CLEAR = 0.01      # m, robot-obstacle clearance along the path
INNER, OUTER = 0.35, 0.70  # m, amendment V1 (set on dev cells)
SPEC = {"W": (0.25, 1.0), "C": (0.25, 1.0), "T": (0.25, 0.25)}  # (v m/s, a m/s^2)


def path_ik(q0, P, R, env, max_joint_step=MAX_JSTEP):
    """IK along the sampled path P (n, 3) at fixed orientation R, rail fixed, each sample seeded by the previous
    solution (P[0] must be the TCP of q0). Returns (n, 7) configurations or None (IK failure, joint jump, or a
    colliding step under VAMP's motion check)."""
    qs = [np.asarray(q0, float)]
    R = np.asarray(R)
    for p in P[1:]:
        q = qs[-1].copy()
        for _ in range(60):
            T = np.asarray(vmod.eefk(q.astype(np.float32).tolist())); e = pose_err(T, p, R)
            if np.linalg.norm(e[:3]) < 2e-4 and np.linalg.norm(e[3:]) < 2e-3:
                break
            J = np.zeros((6, len(q)))
            for j in range(1, len(q)):
                dq = q.copy(); dq[j] += 1e-4
                J[:, j] = (pose_err(np.asarray(vmod.eefk(dq.astype(np.float32).tolist())), p, R) - e) / -1e-4
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-5 * np.eye(6), e), gen_v2.LO, gen_v2.HI)
        T = np.asarray(vmod.eefk(q.astype(np.float32).tolist())); e = pose_err(T, p, R)
        if np.linalg.norm(e[:3]) > 5e-4 or np.linalg.norm(e[3:]) > 5e-3 or np.abs(q - qs[-1]).max() > max_joint_step:
            return None
        if not vmod.validate_motion(qs[-1].astype(np.float32).tolist(), q.astype(np.float32).tolist(), env):
            return None
        qs.append(q)
    return np.array(qs)


def wrap_near(q, ref):
    """A6 rule: each revolute joint to the 2pi-equivalent value nearest ref, within the joint bounds."""
    q = np.asarray(q, float).copy()
    for j in range(1, len(q)):
        cands = [q[j] + 2 * math.pi * k for k in (-2, -1, 0, 1, 2)]
        cands = [c for c in cands if gen_v2.LO[j] - 1e-9 <= c <= gen_v2.HI[j] + 1e-9]
        q[j] = min(cands, key=lambda c: abs(c - ref[j]))
    return q


def station_tops(p, i=None):
    """Highest box top of station i (all stations if None)."""
    return max(b["position"][2] + b["half_extents"][2] for b in p["box"]
               if b["name"] != "floor" and (i is None or b["name"].startswith(f"s{i}_")))


def above(pos, p, i):
    """The point 0.10 m above station i's top, over the sampled target (or 0.10 m above the target if higher)."""
    return np.array([pos[0], pos[1], max(pos[2], station_tops(p, i)) + LIFT])


def geom_ok(prims, shoulder):
    """Amendment V1: every path point at least INNER = 0.35 m horizontally from the base axis (the rail carriage's vertical
    axis) and within OUTER = 0.70 m of the shoulder: a Cartesian move over the robot's own base, or at the edge of its reach,
    is not what a cell programmer writes."""
    P, _, _ = G.sample(prims, 0.02)
    return bool(np.all(np.linalg.norm(P[:, :2] - shoulder[:2], axis=1) >= INNER) and np.all(np.linalg.norm(P - shoulder, axis=1) <= OUTER))


def check_path(prims, q0, R, env, sdf):
    if not geom_ok(prims, np.array([q0[0], 0.0, 0.1625])):
        return None
    P, S, _ = G.sample(prims, STEP)
    if np.linalg.norm(P[0] - G.prim_point(prims[0], 0.0)) > 1e-9:
        return None
    qs = path_ik(q0, P, R, env)
    if qs is None:
        return None
    cl = min(clearance(VM, q, sdf) for q in qs[::5])
    return cl if cl >= MIN_CLEAR else None


def w_points(rng, PA, PB, ztop, shoulder):
    n = int(rng.integers(3, 7))
    d = PB[:2] - PA[:2]; dn = np.linalg.norm(d)
    ax = d / dn if dn > 1e-6 else np.array([1.0, 0.0]); perp = np.array([-ax[1], ax[0]])
    pts = [PA]
    for i in range(1, n + 1):
        t = i / (n + 1)
        for _ in range(20):  # within the pre-registered ranges, reject via points outside the V1 bounds
            xy = PA[:2] + t * d + perp * rng.uniform(-0.25, 0.25) + ax * rng.uniform(-0.05, 0.05)
            x = np.array([xy[0], xy[1], ztop + rng.uniform(0.10, 0.40)])
            if np.linalg.norm(x - shoulder) <= OUTER and np.linalg.norm(x[:2] - shoulder[:2]) >= INNER:
                break
        else:
            return None
        pts.append(x)
    pts.append(PB)
    legs = [np.linalg.norm(pts[i + 1] - pts[i]) for i in range(len(pts) - 1)]
    turns = [math.degrees(math.acos(np.clip(G._unit(pts[i] - pts[i - 1]) @ G._unit(pts[i + 1] - pts[i]), -1, 1)))
             for i in range(1, len(pts) - 1)]
    if min(legs) < 0.10 or min(turns) < 30 or max(turns) > 150:
        return None
    return pts


def build(args):
    freeze, label, k = args
    rng = np.random.default_rng(seed_from(freeze, f"{label}_{k}"))
    p, env, st, home = make_cell(rng, k)
    if home is None or len(st) < 2:
        return None
    sdf = obstacle_sdf(p); ztop = station_tops(p)
    a = int(rng.integers(len(st)))
    side = [np.sign(s[1](np.random.default_rng(0))[0][1]) for s in st]
    same = [i for i in range(len(st)) if i != a and side[i] == side[a]]  # amendment V1: B on A's side of the rail
    if not same:
        return None
    b = int(same[int(rng.integers(len(same)))])
    eA = endpoint(st[a], env, rng)
    if eA is None or not check_endpoint(eA):
        return None
    R = np.asarray(eA["R"]); PA = above(np.asarray(eA["pos"]), p, a)
    up = line_ik(np.asarray(eA["q_above"]), np.asarray(eA["pos"]) + [0, 0, LIFT], R, PA, env) \
        if PA[2] > eA["pos"][2] + LIFT + 1e-6 else [np.asarray(eA["q_above"])]
    if up is None:
        return None
    q0 = wrap_near(up[-1], np.asarray(home))
    if np.linalg.norm(np.asarray(vmod.eefk(q0.astype(np.float32).tolist()))[:3, 3] - PA) > 1e-3 or \
            not vmod.validate(q0.astype(np.float32).tolist(), env):
        return None
    sh = np.array([q0[0], 0.0, 0.1625])
    if np.linalg.norm(PA[:2] - sh[:2]) < INNER or np.linalg.norm(PA - sh) > OUTER:
        return None  # V1: the start itself is outside the bounds, so no program can satisfy them
    progs = {}
    for _ in range(30):  # W
        PB = above(np.asarray(st[b][1](rng)[0]), p, b)
        pts = w_points(rng, PA, PB, ztop, np.array([q0[0], 0.0, 0.1625]))
        if pts is None:
            continue
        prims, radii = G.blended_polyline(pts)
        cl = check_path(prims, q0, R, env, sdf)
        if cl is not None:
            progs["W"] = {"via_points": [x.tolist() for x in pts[1:-1]], "points": [x.tolist() for x in pts], "blend_r": radii,
                          "prims": prims, "min_clearance_m": cl}
            break
    for _ in range(30):  # C
        ang = rng.uniform(-math.pi, math.pi); u = np.array([math.cos(ang), math.sin(ang), 0.0])
        rho = rng.uniform(0.10, 0.30); sweep = math.radians(rng.uniform(90, 270))
        w = np.cross([0, 0, 1.0], u) * (1 if rng.random() < 0.5 else -1)
        prims = G.circle_arc(PA - rho * u, u, w, rho, sweep)
        cl = check_path(prims, q0, R, env, sdf)
        if cl is not None:
            progs["C"] = {"radius_m": rho, "sweep_deg": math.degrees(sweep), "prims": prims, "min_clearance_m": cl,
                          "via": G.prim_point(prims[0], rho * sweep / 2).tolist(), "end": G.prim_point(prims[0], rho * sweep).tolist()}
            break
    for _ in range(30):  # T
        PB = above(np.asarray(st[b][1](rng)[0]), p, b)
        d = G._unit(PB - PA) if np.linalg.norm(PB - PA) > 1e-6 else np.array([1.0, 0, 0])
        L = rng.uniform(0.30, 0.60); prims = G.line(PA, PA + L * d)
        cl = check_path(prims, q0, R, env, sdf)
        if cl is not None:
            progs["T"] = {"length_m": L, "end": (PA + L * d).tolist(), "prims": prims, "min_clearance_m": cl}
            break
    if set(progs) != {"W", "C", "T"}:
        return None
    for t, pr in progs.items():
        pr["v"], pr["a"] = SPEC[t]; pr["path_length_m"] = G.length(pr["prims"])
    return dict(p, k=k, home=list(home), R=R.tolist(), start_pos=PA.tolist(), q_start=q0.tolist(),
                stations=[str(s[0]) for s in st], A=int(a), B=int(b), programs=progs)


def main(freeze, n, label, out, workers, kmax):
    cells = []
    with mp.get_context("fork").Pool(workers) as pool:
        for c in pool.imap(build, [(freeze, label, k) for k in range(kmax)]):
            if c is not None and len(cells) < n:
                cells.append(dict(c, index=len(cells)))
                pr = c["programs"]
                print("cell", len(cells), "k", c["k"], "W vias", len(pr["W"]["via_points"]), "len %.2f" % pr["W"]["path_length_m"],
                      "C %.2fm %.0fdeg" % (pr["C"]["radius_m"], pr["C"]["sweep_deg"]), "T %.2f" % pr["T"]["length_m"], flush=True)
            if len(cells) == n:
                pool.terminate(); break
    assert len(cells) == n, len(cells)
    pickle.dump({"robot": VM, "label": label, "freeze": freeze, "spec": SPEC, "step_m": STEP, "cells": cells}, open(out, "wb"))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--freeze", required=True); ap.add_argument("--dev", action="store_true")
    ap.add_argument("--workers", type=int, default=2); ap.add_argument("--kmax", type=int, default=1500)
    a = ap.parse_args()
    if a.dev:
        main(a.freeze, 10, "v22_dev", ROOT / "problems/v2_2_dev.pkl", a.workers, a.kmax)
    else:
        main(a.freeze, 100, "v22_cells", ROOT / "problems/v2_2_programs.pkl", a.workers, a.kmax)
