"""Benchmark v2.2, Cartesian pipeline (ours) + VAMP collision check (docs/V2_2_MOTION_TYPES_PREREGISTRATION.md).

Per program, timed as T_ready: the programmed path sampled every 2 mm, IK per sample seeded by the previous one with
the rail fixed (bench/v2_2_gen.path_ik: joint-jump check and VAMP validate_motion between samples), a cubic spline
q(s) in arc length, and the programmed time law (1-D Ruckig on s, the program's v and a, TCP jerk 10 m/s^3). If the
1 kHz check finds a velocity, acceleration or jerk ratio > 1.01, v is halved (up to 3 times; recorded).
The program is deterministic: one geometric run; T_ready is the median of --repeats timed repeats.
Every result goes through the shared 1 kHz gate (bench/v2_validate.py) and the shared metrics (bench/v2_2_metrics.py),
always against the programmed v (so a halved speed shows as a speed deviation).
Writes results/v2_2/raw/cartesian.jsonl. Usage: python bench/v2_2_cartesian.py [--problems PKL] [--repeats 10]
"""
import argparse, json, pickle, statistics, sys, time
from pathlib import Path

import numpy as np
import vamp
from scipy.interpolate import CubicSpline

sys.path.insert(0, str(Path(__file__).parent))
import v2_2_paths as G
from v2_2_gen import STEP, path_ik
from v2_2_metrics import metrics
from v2_common import ROOT
from v2_validate import validate

VM = "ur5erail"


def run(cell, env, prog):
    R = np.asarray(cell["R"])
    P, S, _ = G.sample(prog["prims"], STEP)
    qs = path_ik(np.asarray(cell["q_start"]), P, R, env)
    if qs is None:
        return {"ok": False, "reason": "path IK failed"}, None
    sp = CubicSpline(S, qs, axis=0)
    v, retries = prog["v"], 0
    while True:
        pr = G.profile(S[-1], v, prog["a"])
        if pr is None:
            return {"ok": False, "reason": "ruckig"}, None
        s, sd, sdd = pr["s"], pr["sd"], pr["sdd"]
        dq = sp(s, 1)
        q, qd, qdd = sp(s), dq * sd[:, None], sp(s, 2) * (sd ** 2)[:, None] + dq * sdd[:, None]
        val = validate(VM, cell, q, qd, qdd)
        lim_ok = val["max_vel_ratio"] <= 1.01 and val["max_acc_ratio"] <= 1.01 and val["max_jerk_ratio"] <= 1.01
        if lim_ok or retries == 3:
            break
        v /= 2; retries += 1
    return {"ok": bool(val["valid"]), "tcp_speed_cmd": v, "speed_retries": retries, "t_exec_s": pr["duration"],
            "gate": {k: val[k] for k in ("valid", "colliding_samples", "bound_violation_rad", "max_vel_ratio", "max_acc_ratio", "max_jerk_ratio")}}, q


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--problems", default=str(ROOT / "problems/v2_2_programs.pkl"))
    ap.add_argument("--repeats", type=int, default=10); ap.add_argument("--out", default=str(ROOT / "results/v2_2/raw/cartesian.jsonl"))
    ap.add_argument("--limit", type=int, default=0); a = ap.parse_args()
    D = pickle.load(open(a.problems, "rb")); out = Path(a.out); out.parent.mkdir(parents=True, exist_ok=True)
    done = {(r["cell"], r["type"]) for r in map(json.loads, open(out))} if out.exists() else set()
    for c in D["cells"][: a.limit or None]:
        env = vamp.problem_dict_to_vamp(c)
        for t in ("W", "C", "T"):
            if (c["index"], t) in done:
                continue
            prog = c["programs"][t]
            times = []
            for _ in range(max(1, a.repeats)):
                t0 = time.perf_counter(); rec, q = run(c, env, prog); times.append(time.perf_counter() - t0)
            rec.update(cell=c["index"], type=t, planner="cartesian", t_ready_s=statistics.median(times), t_ready_all_s=times)
            if q is not None:
                rec["metrics"] = metrics(q, prog, c["R"])
                m = rec["metrics"]
                rec["success"] = bool(rec["ok"] and rec["t_ready_s"] <= 30 and m["end_pos_err_mm"] <= 1 and m["end_rot_err_deg"] <= 1
                                      and m["max_dev_outside_mm"] <= 5 and m["max_orient_dev_deg"] <= 1)
                rec["t_cycle_s"] = rec["t_ready_s"] + rec["t_exec_s"]
            else:
                rec["success"] = False
            with open(out, "a") as f:
                f.write(json.dumps(rec) + "\n")
            print(c["index"], t, rec["success"], rec.get("speed_retries"), "%.3fs" % rec["t_ready_s"],
                  {k: (round(v, 3) if isinstance(v, float) else v) for k, v in rec.get("metrics", {}).items() if k in
                   ("max_dev_outside_mm", "max_dev_blend_mm", "cruise_dev_max", "blend_min_speed_ratio")}, flush=True)


if __name__ == "__main__":
    main()
