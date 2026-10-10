"""Benchmark v2 step 4, VAMP: pick-place sequences with MoveL segments (docs/V2_PICKPLACE_PREREGISTRATION.md).

Per pick (6 segments): free (VAMP rrtc + simplify + P1 Ruckig) to the pre-grasp; MoveL down; MoveL up; free to
pre-place; MoveL down; MoveL up.
MoveL (VAMP has no native linear motion; post-processing, timed): straight TCP line IK at run time (5 mm samples,
seeded, rail fixed, joint-jump and VAMP motion checks; bench/v2_pickplace_gen.py line_ik), a cubic spline q(s) in
TCP arc length s, and 1-D Ruckig on s with v = 0.25 m/s, a = 1.0 m/s^2, j = 10 m/s^3. If the 1 kHz check finds a
joint-limit violation, v is halved (up to 3 retries).
Every segment is validated at 1 kHz (bench/v2_validate.py). The per-segment planning budget is T = 30 s (A2).
Timed: plan / line IK / parameterization, measured in this process. Writes results/v2/raw/vamp_pickplace.jsonl.
Usage: python bench/v2_pickplace_vamp.py --freeze H [--seeds 5]
"""
import argparse, json, pickle, sys, time
from pathlib import Path

import numpy as np
import vamp
from ruckig import InputParameter, Result, Ruckig, Trajectory
from scipy.interpolate import CubicSpline

sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, limits, seed_from
from v2_pickplace_gen import LIFT, line_ik
from v2_pickplace_metrics import moveL_metrics
from v2_timing import ruckig_p1
from v2_validate import validate

VM = "ur5erail"; vmod = getattr(vamp, VM)
VMAX, AMAX, JMAX = limits(VM)
T_BUDGET = 30.0
TCP_V, TCP_A, TCP_J = 0.25, 1.0, 10.0


def tcp_time_scale(qs, v):
    """q(t) at 1 kHz along a joint-space line sampled at equal TCP arc-length steps; 1-D Ruckig on s."""
    qs = np.asarray(qs, float)
    P = np.array([np.asarray(vmod.eefk(q.astype(np.float32).tolist()))[:3, 3] for q in qs])
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    sp = CubicSpline(s, qs, axis=0)
    inp = InputParameter(1); inp.current_position = [0.0]; inp.target_position = [float(s[-1])]
    inp.current_velocity = inp.current_acceleration = inp.target_velocity = inp.target_acceleration = [0.0]
    inp.max_velocity, inp.max_acceleration, inp.max_jerk = [v], [TCP_A], [TCP_J]
    tr = Trajectory(1)
    if Ruckig(1).calculate(inp, tr) not in (Result.Working, Result.Finished):
        return None
    ts = np.arange(0, tr.duration + 1e-9, 1e-3)
    st = np.array([tr.at_time(t) for t in ts])  # (n, 3, 1): s, sdot, sddot
    sv, sd, sdd = st[:, 0, 0], st[:, 1, 0], st[:, 2, 0]
    q = sp(sv); dq = sp(sv, 1); ddq = sp(sv, 2)
    return q, dq * sd[:, None], ddq * (sd ** 2)[:, None] + dq * sdd[:, None], float(tr.duration)


def free_segment(p, env, q0, q1, sampler, planner, ps, ss, seed):
    sampler.reset(); sampler.skip(seed % 100000)
    t0 = time.perf_counter(); res = planner(list(q0), [list(q1)], env, ps, sampler)
    if not res.solved:
        return {"kind": "free", "ok": False, "reason": "not solved", "t_ready_s": time.perf_counter() - t0}
    simple = vmod.simplify(res.path, env, ss, sampler); path = np.array(simple.path.numpy(), float)
    T1 = ruckig_p1(path, VMAX, AMAX, JMAX); t1 = time.perf_counter()
    if T1 is None:
        return {"kind": "free", "ok": False, "reason": "ruckig", "t_ready_s": t1 - t0}
    q, qd, qdd = T1.sample(np.arange(0, T1.duration + 1e-9, 1e-3)); v = validate(VM, p, q, qd, qdd)
    ok = v["valid"] and (t1 - t0) <= T_BUDGET
    return {"kind": "free", "ok": bool(ok), "t_ready_s": t1 - t0, "t_exec_s": T1.duration, "valid": v["valid"]}


def moveL_segment(p, env, q_from, p_from, p_to, R, reuse=None):
    t0 = time.perf_counter()
    qs = reuse if reuse is not None else line_ik(np.asarray(q_from), np.asarray(p_from), np.asarray(R), np.asarray(p_to), env)
    if qs is None:
        return {"kind": "moveL", "ok": False, "reason": "line IK failed", "t_ready_s": time.perf_counter() - t0}, None
    v, retries = TCP_V, 0
    while True:
        out = tcp_time_scale(qs, v)
        if out is None:
            return {"kind": "moveL", "ok": False, "reason": "ruckig", "t_ready_s": time.perf_counter() - t0}, qs
        q, qd, qdd, dur = out
        val = validate(VM, p, q, qd, qdd)
        lim_ok = val["max_vel_ratio"] <= 1.01 and val["max_acc_ratio"] <= 1.01 and val["max_jerk_ratio"] <= 1.01
        if lim_ok or retries == 3:
            break
        v /= 2; retries += 1
    t1 = time.perf_counter()
    rec = {"kind": "moveL", "ok": bool(val["valid"]), "t_ready_s": t1 - t0, "t_exec_s": dur, "tcp_speed_cmd": v,
           "speed_retries": retries, "valid": val["valid"], "colliding_samples": val["colliding_samples"]}
    rec.update(moveL_metrics(q, p_from, p_to, R))
    return rec, qs


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--freeze", required=True); ap.add_argument("--seeds", type=int, default=5)
    ap.add_argument("--limit", type=int, default=0); ap.add_argument("--out", default=None)
    ap.add_argument("--problems", default=str(ROOT / "problems/v2_pickplace.pkl")); a = ap.parse_args()
    D = pickle.load(open(a.problems, "rb")); cells = D["cells"][: a.limit or None]
    _, planner, ps, ss = vamp.configure_robot_and_planner_with_kwargs(VM, "rrtc"); sampler = vmod.halton()
    out = Path(a.out) if a.out else ROOT / "results/v2/raw/vamp_pickplace.jsonl"; out.parent.mkdir(parents=True, exist_ok=True)
    done = {(r["cell"], r["pick"], r["seed_k"]) for r in map(json.loads, open(out))} if out.exists() else set()
    for k in range(a.seeds):
        seed = seed_from(a.freeze, f"pickplace_seed{k}")
        for c in cells:
            env = vamp.problem_dict_to_vamp(c); cur = np.asarray(c["home"])
            for i, pk in enumerate(c["picks"]):
                g, pl = pk["grasp"], pk["place"]
                if (c["index"], i, k) in done:
                    cur = np.asarray(pl["q_above"]); continue
                above_g = np.asarray(g["pos"]) + [0, 0, LIFT]; above_p = np.asarray(pl["pos"]) + [0, 0, LIFT]
                # A failed segment ends the pick (remaining segments "not attempted"); every pick starts from the
                # previous pick's pre-place configuration, so picks are independent of earlier failures.
                steps = [lambda: free_segment(c, env, cur, g["q_above"], sampler, planner, ps, ss, seed),
                         lambda: moveL_segment(c, env, g["q_above"], above_g, g["pos"], g["R"]),
                         lambda: moveL_segment(c, env, None, g["pos"], above_g, g["R"], reuse=lines[-1][::-1]),
                         lambda: free_segment(c, env, g["q_above"], pl["q_above"], sampler, planner, ps, ss, seed),
                         lambda: moveL_segment(c, env, pl["q_above"], above_p, pl["pos"], pl["R"]),
                         lambda: moveL_segment(c, env, None, pl["pos"], above_p, pl["R"], reuse=lines[-1][::-1])]
                segs, lines = [], []
                for step in steps:
                    r = step()
                    seg, line = r if isinstance(r, tuple) else (r, None)
                    segs.append(seg)
                    if line is not None:
                        lines.append(line)
                    if not seg["ok"]:
                        break
                segs += [{"kind": "not_attempted", "ok": False}] * (6 - len(segs))
                ok = all(s["ok"] for s in segs)
                rec = {"cell": c["index"], "pick": i, "seed_k": k, "seed": seed, "planner": "vamp", "success": bool(ok), "segments": segs,
                       "t_ready_s": float(sum(s.get("t_ready_s", 0) for s in segs)), "t_exec_s": float(sum(s.get("t_exec_s", 0) for s in segs))}
                rec["t_cycle_s"] = rec["t_ready_s"] + rec["t_exec_s"] if ok else None
                with open(out, "a") as f:
                    f.write(json.dumps(rec) + "\n")
                cur = np.asarray(pl["q_above"])
            print(f"seed {k} cell {c['index']} done", flush=True)


if __name__ == "__main__":
    main()
