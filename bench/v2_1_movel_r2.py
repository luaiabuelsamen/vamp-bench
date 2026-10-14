"""v2.1 amendment 7f (post-hoc): R2 (uniform slowdown) applied to every planner's MoveL, in the same columns as PyRoki.

R2: a segment failing the 1 kHz gate only on velocity/acceleration/jerk is slowed uniformly by
s = max(1, r_v, sqrt(r_a), cbrt(r_j)); it then passes if it has no colliding sample and no bound violation.
The v2 pick-place runs stored peak ratios and the collision/bound verdict per segment, not trajectories, so R2 is
computed from the records. The shortcut is checked first:
  (1) the cuRobo v1 trajectories dumped in one cell (results/v2/diag_jerk/dump_jerk{500,250}) are rescaled,
      resampled at 1 kHz (qdd interpolated linearly, qd and q consistently) and run through the full gate;
  (2) the s predicted from PyRoki's R1 peaks is compared with the s its R2 used.
If any rescaled dump fails the gate or exceeds 1.01, nothing else is computed (exit 2).
CPU only. Writes results/v2_1/movel_r2.json.
Usage: .venv/bin/python bench/v2_1_movel_r2.py
"""
import json, sys, time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import pickle
from v2_common import ROOT
from v2_validate import validate

VM = "ur5erail"
RAW = ROOT / "results/v2/raw"


def scale_of(rv, ra, rj):
    return max(1.0, rv, np.sqrt(ra), np.cbrt(rj))


def rescale(q, qd, qdd, s, dt=1e-3):
    """q(t/s) sampled at 1 kHz: q and qd by cubic Hermite, qdd linearly (so the gate's finite-difference jerk is the
    original slope / s^3)."""
    n = len(q); T = (n - 1) * dt; m = int(round(T * s / dt)) + 1
    u = np.minimum(np.arange(m) * dt / s, T) / dt  # position in original sample index
    i = np.minimum(np.floor(u).astype(int), n - 2); f = (u - i)[:, None]
    h00, h10, h01, h11 = 2 * f**3 - 3 * f**2 + 1, f**3 - 2 * f**2 + f, -2 * f**3 + 3 * f**2, f**3 - f**2
    qs = h00 * q[i] + h10 * dt * qd[i] + h01 * q[i + 1] + h11 * dt * qd[i + 1]
    qds = ((1 - f) * qd[i] + f * qd[i + 1]) / s
    qdds = ((1 - f) * qdd[i] + f * qdd[i + 1]) / s**2
    return qs, qds, qdds


def check_dumps():
    cells = {c["index"]: c for c in pickle.load(open(ROOT / "problems/v2_pickplace.pkl", "rb"))["cells"]}
    c = cells[0]; rows = []
    for d in sorted((ROOT / "results/v2/diag_jerk").glob("dump_jerk*")):
        for f in sorted(d.glob("*.npz")):
            z = np.load(f); q, qd, qdd = z["q"], z["qd"], z["qdd"]
            v1 = validate(VM, c, q, qd, qdd)
            s = scale_of(v1["max_vel_ratio"], v1["max_acc_ratio"], v1["max_jerk_ratio"])
            t0 = time.perf_counter(); qs, qds, qdds = rescale(q, qd, qdd, s); t_scale = time.perf_counter() - t0
            v2 = validate(VM, c, qs, qds, qdds)
            rows.append({"file": f"{d.name}/{f.name}", "kind": f.stem.split("_")[1], "r1_valid": v1["valid"],
                         "r1": [v1["max_vel_ratio"], v1["max_acc_ratio"], v1["max_jerk_ratio"]], "s": s, "t_scale_s": t_scale,
                         "r2_valid": v2["valid"], "r2": [v2["max_vel_ratio"], v2["max_acc_ratio"], v2["max_jerk_ratio"]],
                         "r2_colliding": v2["colliding_samples"], "r2_bound": v2["bound_violation_rad"]})
    ok = all(r["r2_valid"] and max(r["r2"]) <= 1.01 for r in rows)
    return ok, rows


def check_pyroki():
    mv = [json.loads(l) for l in open(ROOT / "results/v2_1/raw/pyroki_movel_gpu.jsonl")]
    d = [abs(scale_of(r["R1"]["max_vel_ratio"], r["R1"]["max_acc_ratio"], r["R1"]["max_jerk_ratio"]) - r["R2"]["scale"]) / r["R2"]["scale"]
         for r in mv if r["R2"]]
    return {"n": len(d), "max_rel_diff_s": float(max(d)), "median_rel_diff_s": float(np.median(d))}


def seg_r2(s0, t_scale):
    """R1/R2 verdicts for one stored v2 segment. Needs the stored gate inputs; a segment the planner did not return
    fails both."""
    if "valid" not in s0:
        return dict(attempted=True, returned=False, r1_gate=False, r1_ok=False, r2_gate=False, r2_ok=False, s=None, dt_exec=0.0)
    r1_gate = bool(s0["valid"])
    rv, ra, rj = s0.get("max_vel_ratio", 0.0), s0.get("max_acc_ratio", 0.0), s0.get("max_jerk_ratio", 0.0)
    # path verdict (collision, bounds): explicit when stored (ok_nojerk), else from 'valid' when every peak is within 1.01
    coll = s0.get("colliding_samples")
    if coll is None and not r1_gate:  # no stored peaks/collision count (VAMP free segments): no rescue
        return dict(attempted=True, returned=True, r1_gate=False, r1_ok=False, r2_gate=False, r2_ok=False, s=None, dt_exec=0.0,
                    t_exec=s0.get("t_exec_s"))
    if "ok_nojerk" in s0:
        path_ok = coll == 0 and (s0["ok_nojerk"] or max(rv, ra) > 1.01)
        bound_unknown = not s0["ok_nojerk"] and max(rv, ra) > 1.01
    else:
        path_ok = r1_gate or (coll == 0 and max(rv, ra, rj) > 1.01)
        bound_unknown = not r1_gate and coll == 0
    s = 1.0 if r1_gate else scale_of(rv, ra, rj)
    r2_gate = r1_gate or path_ok
    t_ready = s0["t_ready_s"] + (0.0 if r1_gate else t_scale)
    return dict(attempted=True, returned=True, r1_gate=r1_gate, r1_ok=bool(s0["ok"]), r2_gate=bool(r2_gate),
                r2_ok=bool(r2_gate and t_ready <= 30.0), s=s, dt_exec=(s - 1) * s0.get("t_exec_s", 0.0),
                t_exec=s0.get("t_exec_s"), bound_unknown=bool(bound_unknown), t_ready_r2=t_ready)


def stats(x):
    x = np.asarray([v for v in x if v is not None], float)
    return {"n": int(len(x)), "median": float(np.median(x)) if len(x) else None, "p95": float(np.percentile(x, 95)) if len(x) else None,
            "max": float(x.max()) if len(x) else None}


def planner_v2(name, path, t_scale):
    R = [json.loads(l) for l in open(path)]
    segs = [seg_r2(s, t_scale) for r in R for s in r["segments"] if s["kind"] == "moveL"]
    out = {"source": str(path.relative_to(ROOT)), "moveL_attempted": len(segs),
           "moveL_returned": sum(x["returned"] for x in segs),
           "R1_gate": sum(x["r1_gate"] for x in segs), "R1_success": sum(x["r1_ok"] for x in segs),
           "R2_gate": sum(x["r2_gate"] for x in segs), "R2_success": sum(x["r2_ok"] for x in segs),
           "R2_bound_unverified": sum(x.get("bound_unknown", False) for x in segs),
           "slowdown_s_rescued": stats([x["s"] for x in segs if x["returned"] and not x["r1_gate"]]),
           "added_exec_s_per_moveL_rescued": stats([x["dt_exec"] for x in segs if x["returned"] and not x["r1_gate"] and x["r2_gate"]]),
           "added_exec_s_per_moveL_all": stats([x["dt_exec"] for x in segs if x["r2_gate"]]),
           "t_exec_moveL_R1_passing": stats([x["t_exec"] for x in segs if x["r1_gate"]]),
           "t_exec_moveL_R2": stats([x["t_exec"] * x["s"] for x in segs if x["r2_gate"]])}
    # per pick: R2 applied to every segment (free too). Not attempted segments fail.
    picks = []
    for r in R:
        sv = [seg_r2(s, t_scale) if s["kind"] != "not_attempted" else None for s in r["segments"]]
        ok1 = bool(r["success"]); ok2 = all(x is not None and x["r2_ok"] for x in sv)
        c2 = sum(x["t_ready_r2"] + x["t_exec"] * x["s"] for x in sv) if ok2 else None
        picks.append({"ok1": ok1, "ok2": ok2, "c1": r["t_cycle_s"], "c2": c2, "key": (r["cell"], r["pick"], r["seed_k"])})
    out["picks"] = {"n": len(picks), "R1_completed": sum(p["ok1"] for p in picks), "R2_completed": sum(p["ok2"] for p in picks),
                    "cycle_R1_s": stats([p["c1"] for p in picks if p["ok1"]]), "cycle_R2_s": stats([p["c2"] for p in picks if p["ok2"]])}
    if name == "curobo1_as_configured":
        # jerk-continue run: R1 pick success is "every segment passes as returned"; the stored success already says so
        out["picks"]["note"] = "jerk-continue run: segments after a jerk-only failure were planned from its end configuration"
    return out, {p["key"]: p for p in picks}


def pyroki():
    mv = [json.loads(l) for l in open(ROOT / "results/v2_1/raw/pyroki_movel_gpu.jsonl")]
    r2 = [r for r in mv if r["R2"]]
    return {"source": "results/v2_1/raw/pyroki_movel_gpu.jsonl", "moveL_attempted": len(mv), "moveL_returned": len(mv),
            "R1_gate": sum(r["R1"]["valid"] for r in mv), "R1_success": sum(r["R1"]["success"] for r in mv),
            "R2_gate": sum(r["R2"]["valid"] for r in r2), "R2_success": sum(r["R2"]["success"] for r in r2),
            "R2_not_computed": len(mv) - len(r2),
            "R2_gate_and_start_ok": sum(r["R2"]["valid"] and r["start_ok"] for r in r2),
            "end_ok": sum(r["end_ok"] for r in mv), "start_ok": sum(r["start_ok"] for r in mv),
            "slowdown_s_rescued": stats([r["R2"]["scale"] for r in r2]),
            "added_exec_s_per_moveL_rescued": stats([r["R2"]["t_exec_s"] - r["R1"]["t_exec_s"] for r in r2 if r["R2"]["valid"]]),
            "t_exec_moveL_R1": stats([r["R1"]["t_exec_s"] for r in mv]),
            "t_exec_moveL_R2": stats([r["R2"]["t_exec_s"] for r in r2 if r["R2"]["valid"]]),
            "t_solve_s": stats([r["t_solve_s"] for r in mv]),
            "note": "480 MoveL segments of the 120 A6 pick slots, one run each, from the stored start configurations"}


def main():
    ok, rows = check_dumps()
    rep = {"amendment": "7f (post-hoc)", "shortcut_check": {"dumps": rows, "passed": ok, "pyroki_s": check_pyroki()}}
    print(json.dumps(rep["shortcut_check"]["pyroki_s"]), "dumps ok:", ok)
    for r in rows:
        print(r["file"], r["r1_valid"], "s=%.3f" % r["s"], "R2", r["r2_valid"], ["%.3f" % x for x in r["r2"]], r["r2_colliding"])
    out = ROOT / "results/v2_1/movel_r2.json"
    if not ok:
        json.dump(rep, open(out, "w"), indent=1); print("shortcut check FAILED: not computing R2 from records"); sys.exit(2)
    t_scale = float(max(r["t_scale_s"] for r in rows))
    rep["t_scale_s_used"] = t_scale
    rep["planners"] = {"pyroki": pyroki()}
    keys = {}
    for name, f in [("vamp", "vamp_pickplace.A6.jsonl"), ("curobo1_as_configured", "curobo1_pickplace.A6.jerkcontinue.jsonl"),
                    ("curobo2", "curobo2_pickplace.A6.jsonl")]:
        rep["planners"][name], keys[name] = planner_v2(name, RAW / f, t_scale)
    # paired cycle time under R2: picks every v2 planner completes under R2
    both = [k for k in keys["vamp"] if all(keys[n].get(k, {}).get("ok2") for n in keys)]
    rep["paired_R2"] = {"n": len(both), **{n: stats([keys[n][k]["c2"] for k in both]) for n in keys}}
    json.dump(rep, open(out, "w"), indent=1)
    for n, p in rep["planners"].items():
        print(n, {k: p[k] for k in ("moveL_attempted", "R1_gate", "R1_success", "R2_gate", "R2_success")},
              "s", p["slowdown_s_rescued"], "picks", p.get("picks", {}).get("R1_completed"), p.get("picks", {}).get("R2_completed"))
    print("wrote", out)


if __name__ == "__main__":
    main()
