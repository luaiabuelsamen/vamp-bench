"""Benchmark v2 held-out problem generator: industrial cells for the UR5e on a 7th-axis rail (cuboid obstacles only).

Each cell has a floor and 3-4 stations in slots on both sides of the rail (x in {-0.5, 0, 0.5}, y = +-),
drawn from four families, plus 0-2 clutter pillars:
  bin       open box on a stand; TCP inside, tool pointing down
  conveyor  belt with side guards; TCP above the belt, tool pointing down
  shelf     2-3 levels, open toward the rail; TCP inside a level, tool pointing into the shelf
  fixture   table with a slot between two blocks; TCP in the slot, tool pointing down
Shelf and fixture stations get a guard with a window in front/above with probability 0.8. The window side is
window_ratio x D, where D is the largest arm-sphere diameter of the shared model and window_ratio ~ U[1.3, 3.5].
IK fixes the rail carriage near the target's x (|x| <= 0.9 m) and solves the arm (6 DOF).
Queries are ordered pairs of different stations in a cell (pick at one, place at the other); start and goal are
collision-free IK solutions (CPU damped least squares on VAMP's FK, random restarts) checked by VAMP's validator.

Usage: python bench/gen_v2.py --freeze <commit-hash> [--n 300] [--out problems/v2_heldout.pkl]
       python bench/gen_v2.py --dev   # geometry-only development seed; never used for results
The generator may only be adjusted from geometry statistics (bin occupancy), never from planner results.
"""
import argparse, json, math, pickle, sys
from pathlib import Path

import numpy as np
import vamp
import yaml

sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, bins, seed_from

VM = "ur5erail"
vmod = getattr(vamp, VM)
LO, HI = np.array(vmod.lower_bounds()), np.array(vmod.upper_bounds())
_sph = yaml.safe_load(open(ROOT / "configs/ur5e_rail_shared.yml"))["robot_cfg"]["kinematics"]["collision_spheres"]
D = 2 * max(s["radius"] for v in _sph.values() for s in v)  # largest arm-sphere diameter
FLOOR_TOP = -0.04


def box(name, lo, hi):
    lo, hi = np.asarray(lo, float), np.asarray(hi, float)
    return {"name": name, "half_extents": ((hi - lo) / 2).tolist(), "position": ((hi + lo) / 2).tolist(),
            "orientation_euler_xyz": (0.0, 0.0, 0.0), "orientation_quat_xyzw": [0.0, 0.0, 0.0, 1.0]}


def down(yaw):
    z = np.array([0, 0, -1.0]); x = np.array([math.cos(yaw), math.sin(yaw), 0.0]); return np.stack([x, np.cross(z, x), z], 1)


def horizontal(direction, roll):
    z = np.array([0, direction, 0.0]); x0 = np.array([0, 0, 1.0])
    x = math.cos(roll) * x0 + math.sin(roll) * np.cross(z, x0); return np.stack([x, np.cross(z, x), z], 1)


def station(kind, x, side, rng, k):
    """Return boxes, a target sampler rng -> (pos, R), and the window ratio (or None)."""
    B, wr = [], None
    if kind == "bin":
        L, W, h, z0 = rng.uniform(0.30, 0.45), rng.uniform(0.25, 0.40), rng.uniform(0.12, 0.25), rng.uniform(-0.04, 0.20)
        cy = side * rng.uniform(0.30, 0.65); t = 0.02
        B += [box(f"s{k}_stand", [x - L / 2, cy - W / 2, FLOOR_TOP], [x + L / 2, cy + W / 2, z0]),
              box(f"s{k}_wxm", [x - L / 2, cy - W / 2, z0], [x - L / 2 + t, cy + W / 2, z0 + h]),
              box(f"s{k}_wxp", [x + L / 2 - t, cy - W / 2, z0], [x + L / 2, cy + W / 2, z0 + h]),
              box(f"s{k}_wym", [x - L / 2, cy - W / 2, z0], [x + L / 2, cy - W / 2 + t, z0 + h]),
              box(f"s{k}_wyp", [x - L / 2, cy + W / 2 - t, z0], [x + L / 2, cy + W / 2, z0 + h])]
        tgt = lambda r: (np.array([x + r.uniform(-1, 1) * (L / 2 - 0.08), cy + r.uniform(-1, 1) * (W / 2 - 0.08),
                                   z0 + r.uniform(0.06, h + 0.20)]), down(r.uniform(-math.pi, math.pi)))
    elif kind == "conveyor":
        L, W, z0, g = rng.uniform(0.6, 0.75), rng.uniform(0.25, 0.35), rng.uniform(0.0, 0.15), rng.uniform(0.04, 0.10)
        cy = side * rng.uniform(0.30, 0.65)
        B += [box(f"s{k}_belt", [x - L / 2, cy - W / 2, FLOOR_TOP], [x + L / 2, cy + W / 2, z0]),
              box(f"s{k}_gm", [x - L / 2, cy - W / 2, z0], [x + L / 2, cy - W / 2 + 0.02, z0 + g]),
              box(f"s{k}_gp", [x - L / 2, cy + W / 2 - 0.02, z0], [x + L / 2, cy + W / 2, z0 + g])]
        tgt = lambda r: (np.array([x + r.uniform(-1, 1) * (L / 2 - 0.08), cy + r.uniform(-1, 1) * (W / 2 - 0.08),
                                   z0 + r.uniform(0.04, 0.30)]), down(r.uniform(-math.pi, math.pi)))
    elif kind == "shelf":
        Wd, dep, n, sp = rng.uniform(0.5, 0.7), 0.30, int(rng.integers(2, 4)), rng.uniform(0.25, 0.32)
        front = rng.uniform(0.35, 0.60); y0, y1 = sorted([side * front, side * (front + dep)]); t = 0.02
        levels = [0.05 + i * sp for i in range(n)]; top = levels[-1] + sp
        B += [box(f"s{k}_sxm", [x - Wd / 2, y0, FLOOR_TOP], [x - Wd / 2 + t, y1, top]),
              box(f"s{k}_sxp", [x + Wd / 2 - t, y0, FLOOR_TOP], [x + Wd / 2, y1, top]),
              box(f"s{k}_back", [x - Wd / 2, (y1 - t) if side > 0 else y0, FLOOR_TOP], [x + Wd / 2, y1 if side > 0 else y0 + t, top]),
              box(f"s{k}_top", [x - Wd / 2, y0, top], [x + Wd / 2, y1, top + t])]
        B += [box(f"s{k}_l{i}", [x - Wd / 2, y0, z - t], [x + Wd / 2, y1, z]) for i, z in enumerate(levels)]
        lev = levels[int(rng.integers(0, n))]
        zc = lev + sp / 2
        if rng.random() < 0.8:
            wr = rng.uniform(1.3, 3.5); a = wr * D / 2; yw = side * (front - 0.06)
            B += [box(f"s{k}_gL", [x - Wd / 2 - 0.1, yw - 0.01, FLOOR_TOP], [x - a, yw + 0.01, top]),
                  box(f"s{k}_gR", [x + a, yw - 0.01, FLOOR_TOP], [x + Wd / 2 + 0.1, yw + 0.01, top]),
                  box(f"s{k}_gB", [x - a, yw - 0.01, FLOOR_TOP], [x + a, yw + 0.01, zc - a]),
                  box(f"s{k}_gT", [x - a, yw - 0.01, zc + a], [x + a, yw + 0.01, top])]
        tgt = lambda r: (np.array([x + r.uniform(-1, 1) * min(Wd / 2 - 0.1, 0.1), side * (front + r.uniform(0.06, 0.16)),
                                   zc + r.uniform(-0.03, 0.03)]), horizontal(side, r.uniform(-math.pi, math.pi)))
    else:  # fixture
        z0, gap, bh = rng.uniform(0.0, 0.15), rng.uniform(0.08, 0.15), rng.uniform(0.08, 0.12)
        cy = side * rng.uniform(0.30, 0.6)
        B += [box(f"s{k}_table", [x - 0.2, cy - 0.17, FLOOR_TOP], [x + 0.2, cy + 0.17, z0]),
              box(f"s{k}_bl", [x - 0.18, cy - 0.12, z0], [x - gap / 2, cy + 0.12, z0 + bh]),
              box(f"s{k}_br", [x + gap / 2, cy - 0.12, z0], [x + 0.18, cy + 0.12, z0 + bh])]
        zw = z0 + bh + 0.30
        if rng.random() < 0.8:
            wr = rng.uniform(1.3, 3.5); a = wr * D / 2
            B += [box(f"s{k}_pL", [x - 0.4, cy - 0.4, zw], [x - a, cy + 0.4, zw + 0.02]),
                  box(f"s{k}_pR", [x + a, cy - 0.4, zw], [x + 0.4, cy + 0.4, zw + 0.02]),
                  box(f"s{k}_pF", [x - a, cy - 0.4, zw], [x + a, cy - a, zw + 0.02]),
                  box(f"s{k}_pB", [x - a, cy + a, zw], [x + a, cy + 0.4, zw + 0.02])]
        tgt = lambda r: (np.array([x + r.uniform(-1, 1) * max(gap / 2 - 0.04, 0.0), cy + r.uniform(-0.05, 0.05),
                                   z0 + r.uniform(0.04, bh + 0.20)]), down(r.choice([0.0, math.pi / 2, math.pi, -math.pi / 2])))
    return B, tgt, wr


def pose_err(T, p, R):
    ep = p - T[:3, 3]
    er = 0.5 * sum(np.cross(T[:3, i], R[:, i]) for i in range(3))
    return np.concatenate([ep, er])


def ik(p, R, env, rng, restarts=40, iters=120):
    """Damped least-squares IK on VAMP's FK (tool0), random restarts; collision-free and in bounds or None."""
    for _ in range(restarts):
        q = rng.uniform(np.maximum(LO, -math.pi), np.minimum(HI, math.pi)); q[0] = np.clip(p[0] + rng.uniform(-0.15, 0.15), -0.9, 0.9)
        for _ in range(iters):  # rail fixed at q[0] (gantry cells position the carriage, then reach with the arm)
            T = np.asarray(vmod.eefk(q.astype(np.float32).tolist())); e = pose_err(T, p, R)
            if np.linalg.norm(e[:3]) < 5e-4 and np.linalg.norm(e[3:]) < 5e-3:
                break
            J = np.zeros((6, len(q)))
            for j in range(len(q)):
                dq = q.copy(); dq[j] += 1e-4
                J[:, j] = (pose_err(np.asarray(vmod.eefk(dq.astype(np.float32).tolist())), p, R) - e) / -1e-4
            J[:, 0] = 0.0
            q = np.clip(q + J.T @ np.linalg.solve(J @ J.T + 1e-4 * np.eye(6), e), LO, HI)
        T = np.asarray(vmod.eefk(q.astype(np.float32).tolist())); e = pose_err(T, p, R)
        if np.linalg.norm(e[:3]) < 1e-3 and np.linalg.norm(e[3:]) < 1e-2 and vmod.validate(q.astype(np.float32).tolist(), env):
            return q
    return None


PAIRS = [("bin", "conveyor"), ("conveyor", "shelf"), ("shelf", "fixture"), ("fixture", "bin"),
         ("bin", "shelf"), ("conveyor", "fixture"), ("shelf", "bin"), ("fixture", "conveyor")]


def generate(seed, n, out):
    rng = np.random.default_rng(seed)
    probs, cells = [], 0
    while len(probs) < n:
        cells += 1
        slots = [(x, s) for x in (-0.5, 0.0, 0.5) for s in (-1, 1)]
        pick = rng.choice(len(slots), size=int(rng.integers(3, 5)), replace=False)
        kinds = rng.choice(["bin", "conveyor", "shelf", "fixture"], size=len(pick))
        boxes = [box("floor", [-1.6, -1.2, -0.24], [1.6, 1.2, FLOOR_TOP])]
        st = []
        for k, (si, kind) in enumerate(zip(pick, kinds)):
            b, t, wr = station(kind, slots[si][0], slots[si][1], rng, k); boxes += b; st.append((kind, t, wr))
        for c in range(int(rng.integers(0, 3))):
            px, py = rng.choice([-0.28, 0.28]), rng.choice([-1, 1]) * rng.uniform(0.40, 0.55); h = rng.uniform(0.6, 1.0)
            boxes.append(box(f"pillar{c}", [px - 0.06, py - 0.06, FLOOR_TOP], [px + 0.06, py + 0.06, h]))
        p0 = {"problem": "v2_heldout", "box": boxes, "cylinder": [], "sphere": []}
        env = vamp.problem_dict_to_vamp(p0)
        pairs = [(a, b) for a in range(len(st)) for b in range(len(st)) if a != b and (st[a][0], st[b][0]) in PAIRS]
        rng.shuffle(pairs)
        for a, b in pairs[:4]:
            qa = ik(*st[a][1](rng), env, rng); qb = ik(*st[b][1](rng), env, rng) if qa is not None else None
            if qa is None or qb is None:
                continue
            p = dict(p0, index=len(probs), start=qa.tolist(), goals=[qb.tolist()], valid=True, cell=cells,
                     stations=[st[a][0], st[b][0]])
            wr = [w for w in (st[a][2], st[b][2]) if w is not None]
            p["bins"] = bins(VM, p, window_ratio=min(wr) if wr else None)
            probs.append(p)
            if len(probs) >= n:
                break
        print(f"cell {cells}: {len(probs)}/{n}", flush=True)
    pickle.dump({"robot": VM, "seed": seed, "D_largest_sphere_diameter_m": D, "problems": {"v2_heldout": probs}}, open(out, "wb"))
    occ = {ax: {} for ax in ("reach", "clearance", "passage", "limits")}
    for p in probs:
        for ax in occ:
            occ[ax][p["bins"][ax]] = occ[ax].get(p["bins"][ax], 0) + 1
    print(json.dumps(occ, indent=1))
    return occ


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("--freeze", default=None); ap.add_argument("--dev", action="store_true")
    ap.add_argument("--n", type=int, default=300); ap.add_argument("--out", default=None)
    a = ap.parse_args()
    if a.dev:
        generate(12345, a.n, a.out or "/tmp/v2_dev.pkl")
    else:
        assert a.freeze, "--freeze <commit hash> required"
        generate(seed_from(a.freeze, "gen_v2"), a.n, a.out or ROOT / "problems/v2_heldout.pkl")
