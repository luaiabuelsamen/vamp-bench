"""Benchmark v2 step 4: pick-place cells (docs/V2_PICKPLACE_PREREGISTRATION.md).

30 cells from bench/gen_v2.py's station builder (bin, conveyor and fixture only: tool pointing down), seed
seed_from(H, "pickplace_cells"). Each cell: a home configuration and 4 picks (source station -> different
destination station). For every pick, the grasp and place TCP poses are sampled at the stations. Their IK
configurations (collision-free) and the pre-grasp / pre-place configurations, 0.10 m above along world +z on a
straight TCP line, are computed here and shared by every planner as the free-motion goals.
The MoveL segments themselves are NOT precomputed: each planner produces them at run time.
Writes problems/v2_pickplace.pkl. Usage: python bench/v2_pickplace_gen.py --freeze H [--workers N]
Amendment A5 (2026-10-11): gen_v2.pose_err's orientation term, 0.5 * sum_i x_i x r_i, vanishes at a 180 deg rotation as
well as at the target, so the first generation converged to flipped tool orientations (430/480 endpoints) and its
MoveL lines drifted up to 22 deg. Here the orientation error is the log map (rotation vector of R_target R^T), IK and
line IK use it, every endpoint is checked (orientation < 0.3 deg from the intended R, tool z pointing down, < 0.5 deg
drift along the line), and cells are seeded per cell, seed_from(H, "pickplace_cells_A5_<k>"), so they can be built in
parallel; the first 30 valid k in order are kept.
"""
import argparse, math, multiprocessing as mp, pickle, sys
from pathlib import Path

import numpy as np
import vamp
from scipy.spatial.transform import Rotation

sys.path.insert(0, str(Path(__file__).parent))
import gen_v2
from v2_common import ROOT, seed_from

VM = "ur5erail"; vmod = getattr(vamp, VM)
LIFT = 0.10


def pose_err(T, p, R):
    """Position error and log-map orientation error (world frame); zero only at the target pose."""
    return np.concatenate([np.asarray(p) - T[:3, 3], Rotation.from_matrix(np.asarray(R) @ T[:3, :3].T).as_rotvec()])


def ik(p, R, env, rng, restarts=40, iters=120):
    """gen_v2.ik with the log-map orientation error (rail fixed near the target x; collision-free and in bounds)."""
    for _ in range(restarts):
        q = rng.uniform(np.maximum(gen_v2.LO, -math.pi), np.minimum(gen_v2.HI, math.pi)); q[0] = np.clip(p[0] + rng.uniform(-0.15, 0.15), -0.9, 0.9)
        for _ in range(iters):
            T = np.asarray(vmod.eefk(q.astype(np.float32).tolist())); e = pose_err(T, p, R)
            if np.linalg.norm(e[:3]) < 5e-4 and np.linalg.norm(e[3:]) < 5e-3:
                break
            J = np.zeros((6, len(q)))
            for j in range(len(q)):
                dq = q.copy(); dq[j] += 1e-4
                J[:, j] = (pose_err(np.asarray(vmod.eefk(dq.astype(np.float32).tolist())), p, R) - e) / -1e-4
            J[:, 0] = 0.0
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e), gen_v2.LO, gen_v2.HI)
        T = np.asarray(vmod.eefk(q.astype(np.float32).tolist())); e = pose_err(T, p, R)
        if np.linalg.norm(e[:3]) < 1e-3 and np.linalg.norm(e[3:]) < 5e-3 and vmod.validate(q.astype(np.float32).tolist(), env):
            return q
    return None


def line_ik(q0, p0, R, p1, env, step=0.005, max_joint_step=0.05):
    """Straight TCP line p0 -> p1 at fixed orientation R, IK per 5 mm sample seeded by the previous solution,
    rail fixed. Returns the list of configurations (including both ends) or None (IK failure, joint jump,
    or a colliding step under VAMP's motion check)."""
    n = max(1, int(math.ceil(np.linalg.norm(np.asarray(p1) - p0) / step)))
    qs = [np.asarray(q0, float)]
    for i in range(1, n + 1):
        p = np.asarray(p0) + (np.asarray(p1) - p0) * i / n
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
    return qs


def make_cell(rng, k):
    slots = [(x, s) for x in (-0.5, 0.0, 0.5) for s in (-1, 1)]
    pick = rng.choice(len(slots), size=int(rng.integers(3, 5)), replace=False)
    kinds = rng.choice(["bin", "conveyor", "fixture"], size=len(pick))
    boxes = [gen_v2.box("floor", [-1.6, -1.2, -0.24], [1.6, 1.2, gen_v2.FLOOR_TOP])]
    st = []
    for i, (si, kind) in enumerate(zip(pick, kinds)):
        b, t, wr = gen_v2.station(kind, slots[si][0], slots[si][1], rng, i); boxes += b; st.append((kind, t))
    p = {"problem": "v2_pickplace", "box": boxes, "cylinder": [], "sphere": []}
    env = vamp.problem_dict_to_vamp(p)
    home = np.array([0.0, 0.0, -1.57, 1.2, -1.2, -1.57, 0.0])
    if not vmod.validate(home.astype(np.float32).tolist(), env):
        home = None
        for _ in range(200):
            q = rng.uniform(np.maximum(gen_v2.LO, -math.pi), np.minimum(gen_v2.HI, math.pi)); q[0] = 0.0
            if vmod.validate(q.astype(np.float32).tolist(), env):
                home = q; break
    return p, env, st, home


def endpoint(st, env, rng):
    """Grasp/place pose at a station + IK, with a valid straight 0.10 m line up to the approach pose."""
    for _ in range(20):
        pos, R = st[1](rng)
        q = ik(pos, R, env, rng)
        if q is None:
            continue
        line = line_ik(q, pos, R, pos + np.array([0, 0, LIFT]), env)
        if line is None:
            continue
        return {"pos": pos.tolist(), "R": R.tolist(), "q": q.tolist(), "q_above": line[-1].tolist()}
    return None


def check_endpoint(e):
    Tq = np.asarray(vmod.eefk(np.float32(e["q"]).tolist())); Ta = np.asarray(vmod.eefk(np.float32(e["q_above"]).tolist())); R = np.asarray(e["R"])
    rot = lambda A, B: math.degrees(Rotation.from_matrix(A.T @ B).magnitude())
    return (rot(Tq[:3, :3], R) < 0.3 and rot(Ta[:3, :3], R) < 0.3 and Tq[2, 2] < -0.99
            and np.linalg.norm(Tq[:3, 3] - e["pos"]) < 1e-3 and np.linalg.norm(Ta[:3, 3] - (np.asarray(e["pos"]) + [0, 0, LIFT])) < 1e-3)


def build(args):
    freeze, k = args
    rng = np.random.default_rng(seed_from(freeze, f"pickplace_cells_A5_{k}"))
    p, env, st, home = make_cell(rng, k)
    if home is None or len(st) < 2:
        return None
    picks = []
    for _ in range(12):
        if len(picks) == 4:
            break
        a, b = rng.choice(len(st), size=2, replace=False)
        g = endpoint(st[a], env, rng); pl = endpoint(st[b], env, rng) if g else None
        if g and pl and check_endpoint(g) and check_endpoint(pl):
            picks.append({"src": st[a][0], "dst": st[b][0], "grasp": g, "place": pl})
    return dict(p, k=k, home=home.tolist(), picks=picks) if len(picks) == 4 else None


def main(freeze, workers):
    cells = []
    with mp.get_context("fork").Pool(workers) as pool:
        for c in pool.imap(build, [(freeze, k) for k in range(80)]):
            if c is not None and len(cells) < 30:
                cells.append(dict(c, index=len(cells)))
                print("cell", len(cells), "k", c["k"], [(str(x["src"]), str(x["dst"])) for x in c["picks"]], flush=True)
            if len(cells) == 30:
                pool.terminate(); break
    assert len(cells) == 30, len(cells)
    pickle.dump({"robot": VM, "lift_m": LIFT, "amendment": "A5", "cells": cells}, open(ROOT / "problems/v2_pickplace.pkl", "wb"))


if __name__ == "__main__":
    ap = argparse.ArgumentParser(); ap.add_argument("--freeze", required=True); ap.add_argument("--workers", type=int, default=7)
    a = ap.parse_args(); main(a.freeze, a.workers)
