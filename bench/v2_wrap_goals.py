"""Amendment A6: put every IK goal on the 2pi-equivalent branch nearest its start (identical geometry).

Revolute joints with +-2pi limits (ur5erail indices 1, 2, 4, 5, 6: shoulder pan, shoulder lift, wrists 1-3) are shifted
by multiples of 2pi, staying within +-2pi; the elbow (+-pi) and the rail are never shifted. FK and every collision sphere
are 2pi-periodic in a revolute joint, so a wrapped configuration has the same geometry: asserted for every one (FK equal
within float32 rounding, max difference reported; same VAMP validity).
  heldout:   problems/v2_heldout.pkl -> problems/v2_heldout_a6.pkl (set key v2_heldout_a6); goal nearest the start,
             bins recomputed with the same bins().
  pickplace: problems/v2_pickplace.pkl (A5) -> kept as problems/v2_pickplace_A5_unwrapped.pkl; A6 cells written to
             problems/v2_pickplace.pkl. Along each cell's pick order: pre-grasp nearest the pick's start (home, or the
             previous wrapped pre-place), grasp shifted by the same multiples; pre-place nearest the wrapped pre-grasp,
             place shifted by the same multiples.
Usage: python bench/v2_wrap_goals.py heldout|pickplace
"""
import copy, json, pickle, shutil, sys
from pathlib import Path

import numpy as np
import vamp

sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, bins

VM = "ur5erail"; vmod = getattr(vamp, VM)
LO, HI = np.array(vmod.lower_bounds()), np.array(vmod.upper_bounds())
TAU = 2 * np.pi
FKDIFF = []
WRAP = [j for j in range(len(LO)) if j > 0 and HI[j] - LO[j] > TAU + 1e-6]  # rail (0) is prismatic


def shift_for(start, q, partner=None):
    """Per-joint multiple of 2pi that brings q nearest start, keeping q (and partner, shifted alike) within limits."""
    k = np.zeros(len(q))
    for j in WRAP:
        ok = [m for m in range(-3, 4) if LO[j] - 1e-9 <= q[j] + m * TAU <= HI[j] + 1e-9
              and (partner is None or LO[j] - 1e-9 <= partner[j] + m * TAU <= HI[j] + 1e-9)]
        k[j] = min(ok, key=lambda m: abs(q[j] + m * TAU - start[j]))
    return k * TAU


def check(q0, q1, env):
    a = np.asarray(vmod.eefk(np.float32(q0).tolist())); b = np.asarray(vmod.eefk(np.float32(q1).tolist()))
    FKDIFF.append(float(np.abs(a - b).max())); assert FKDIFF[-1] < 1e-4, FKDIFF[-1]  # VAMP FK is float32
    assert vmod.validate(np.float32(q0).tolist(), env) == vmod.validate(np.float32(q1).tolist(), env)
    assert np.all(q1 >= LO - 1e-9) and np.all(q1 <= HI + 1e-9)


def maxmove(a, b):
    return float(np.abs(np.subtract(b, a))[1:].max())


def heldout():
    D = pickle.load(open(ROOT / "problems/v2_heldout.pkl", "rb")); P = D["problems"]["v2_heldout"]
    out, before, after, changed = [], [], [], 0
    for p in P:
        p2 = copy.deepcopy(p); env = vamp.problem_dict_to_vamp(p); s = np.asarray(p["start"], float)
        goals = []
        for g in p["goals"]:
            g = np.asarray(g, float); g2 = g + shift_for(s, g); check(g, g2, env)
            goals.append(g2.tolist()); before.append(maxmove(s, g)); after.append(maxmove(s, g2)); changed += bool(np.any(g2 != g))
        p2["goals"] = goals; p2["problem"] = p["problem"]
        p2["bins"] = bins(VM, p2, window_ratio=p["bins"]["values"].get("window_ratio"))
        out.append(p2)
    D2 = dict({k: v for k, v in D.items() if k != "problems"}, amendment="A6", source="problems/v2_heldout.pkl",
              problems={"v2_heldout_a6": out})
    pickle.dump(D2, open(ROOT / "problems/v2_heldout_a6.pkl", "wb"))
    moved = sum(a["bins"] != b["bins"] for a, b in zip(P, out))
    rep = {"queries": len(P), "goals_changed": changed, "max_joint_move_rad_median": [float(np.median(before)), float(np.median(after))],
           "share_over_4p5_rad": [float(np.mean(np.array(before) > 4.5)), float(np.mean(np.array(after) > 4.5))],
           "queries_whose_bins_changed": int(moved)}
    rep["fk_max_abs_diff"] = max(FKDIFF); print(json.dumps(rep)); return rep


def pickplace():
    src = ROOT / "problems/v2_pickplace.pkl"; keep = ROOT / "problems/v2_pickplace_A5_unwrapped.pkl"
    D = pickle.load(open(src, "rb"))
    assert D.get("amendment") == "A5", D.get("amendment")
    if not keep.exists():
        shutil.copy2(src, keep)
    before, after, changed = [], [], 0
    for c in D["cells"]:
        env = vamp.problem_dict_to_vamp(c); cur = np.asarray(c["home"], float)
        for pk in c["picks"]:
            for w in ("grasp", "place"):
                e = pk[w]; qa, q = np.asarray(e["q_above"], float), np.asarray(e["q"], float)
                d = shift_for(cur, qa, partner=q); qa2, q2 = qa + d, q + d
                check(qa, qa2, env); check(q, q2, env)
                before.append(maxmove(cur, qa)); after.append(maxmove(cur, qa2)); changed += bool(np.any(d != 0))
                e["q_above"], e["q"] = qa2.tolist(), q2.tolist(); cur = qa2
    D["amendment"] = "A6"; D["source"] = "problems/v2_pickplace_A5_unwrapped.pkl"
    pickle.dump(D, open(src, "wb"))
    rep = {"free_goals": len(before), "goals_changed": changed, "max_joint_move_rad_median": [float(np.median(before)), float(np.median(after))],
           "share_over_4p5_rad": [float(np.mean(np.array(before) > 4.5)), float(np.mean(np.array(after) > 4.5))],
           "max_joint_move_rad_max": [float(np.max(before)), float(np.max(after))]}
    rep["fk_max_abs_diff"] = max(FKDIFF); print(json.dumps(rep)); return rep


if __name__ == "__main__":
    which = sys.argv[1]
    rep = {"heldout": heldout, "pickplace": pickplace}[which]()
    json.dump(rep, open(ROOT / f"results/v2/a6_wrap_{which}.json", "w"), indent=1)
