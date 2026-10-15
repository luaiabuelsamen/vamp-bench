"""Benchmark v2.2, PyRoki (own category, as in v2.1; docs/V2_2_MOTION_TYPES_PREREGISTRATION.md).

Per program (W, C, T), one trajopt with N knots at fixed dt = programmed duration / (N - 1):
  init       PyRoki's own IK (01_basic_ik costs) on each knot target in order, each seeded by the previous knot's
             solution (start: the given start configuration); one jitted lax.scan, counted in T_ready
  endpoints  the v2.1 7g recipe (pyroki_snippets/_trajopt.py): start configuration held on knots 0-1, end configuration
             (the last IK of the init) held on knots N-2..N-1, rest cost 0.01
  path       pose cost (50 / 10, as in v2.1) on knots 2..N-3 toward the programmed path at its programmed time law
  common     v2.1 terms: smoothness, 5-point acceleration, self-collision, joint, velocity, acceleration and jerk
             limit constraints, world collision constraint at the v2.1 margin (PYROKI_MARGIN, 5 mm)
N per program type: the smallest N with knot spacing <= 40 ms for the longest program of that type in the set, so
each type compiles once (compile timed on the first dev program of the type and reported separately).
The knots go through the v2.1 path: clamped cubic spline at 1 kHz -> the shared gate (R1) and, if R1 fails only on
velocity, acceleration or jerk, R2 (uniform slow-down); shared metrics (bench/v2_2_metrics.py) for both.
Deterministic: one run per program. Writes results/v2_2/raw/pyroki_<backend>.jsonl (resumable).
Usage: PYROKI_MARGIN=0.005 bench/pyroki_env.sh python bench/v2_2_pyroki.py [--limit N] [--deadline unix] [--types W C T]
"""
import argparse, json, math, os, pickle, sys, time
from pathlib import Path

import jax, jax.numpy as jnp, jax_dataclasses as jdc, jaxlie, jaxls
import numpy as np
import pyroki as pk
from jaxls import Cost

sys.path.insert(0, str(Path(__file__).parent))
_argv = sys.argv; sys.argv = sys.argv[:1]
import v2_1_pyroki as V
sys.argv = _argv
import v2_2_paths as G
from v2_2_metrics import metrics
from v2_common import ROOT

KNOT_DT_MAX = 0.040


def _ik_step(robot, link, wxyz, pos, q_init):
    v = robot.joint_var_cls(0)
    costs = [pk.costs.pose_cost_analytic_jac(robot, v, jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(wxyz), pos),
                                             link, pos_weight=50.0, ori_weight=10.0),
             pk.costs.limit_constraint(robot, v)]
    sol = jaxls.LeastSquaresProblem(costs=costs, variables=[v]).analyze().solve(
        verbose=False, linear_solver="dense_cholesky", trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0),
        initial_vals=jaxls.VarValues.make((v.with_value(q_init),)))
    return sol[v]


@jdc.jit
def ik_chain(robot, link, wxyz, positions, q0):
    def step(q, pos):
        qn = _ik_step(robot, link, wxyz, pos, q)
        return qn, qn
    _, qs = jax.lax.scan(step, q0, positions)
    return qs


@jdc.jit
def trajopt_path(robot, coll, wc, q0, q1, targets_wxyz_xyz, init, dt):
    T = init.shape[0]; tv = robot.joint_var_cls(jnp.arange(T))

    @Cost.factory(kind="constraint_eq_zero", name="start")
    def start(vals, var):
        return (vals[var] - q0).flatten()

    @Cost.factory(kind="constraint_eq_zero", name="end")
    def end(vals, var):
        return (vals[var] - q1).flatten()

    rb = jax.tree.map(lambda x: x[None], robot)
    costs = V._common_costs(robot, coll, wc, tv, T, dt) + [
        start(robot.joint_var_cls(jnp.arange(0, 2))), end(robot.joint_var_cls(jnp.arange(T - 2, T))),
        pk.costs.rest_cost(tv, tv.default_factory()[None], jnp.array([0.01])[None]),
        pk.costs.pose_cost(rb, robot.joint_var_cls(jnp.arange(2, T - 2)), jaxlie.SE3(targets_wxyz_xyz[2:T - 2]),
                           jnp.array(V.LINK), jnp.array([50.0] * 3)[None], jnp.array([10.0] * 3)[None])]
    sol = jaxls.LeastSquaresProblem(costs=costs, variables=[tv]).analyze().solve(
        verbose=False, initial_vals=jaxls.VarValues.make((tv.with_value(init),)))
    return sol[tv]


def knot_targets(prog, n):
    prof = G.profile(G.length(prog["prims"]), prog["v"], prog["a"])
    tk = np.linspace(0, prof["duration"], n); sk = np.interp(tk, prof["t"], prof["s"])
    D, S, _ = G.sample(prog["prims"], 0.0005)
    return np.array([D[min(int(np.searchsorted(S, s)), len(D) - 1)] for s in sk]), prof["duration"] / (n - 1)


def n_knots(cells, t):
    dmax = max(G.profile(G.length(c["programs"][t]["prims"]), c["programs"][t]["v"], c["programs"][t]["a"])["duration"] for c in cells)
    return int(math.ceil(dmax / KNOT_DT_MAX)) + 1


def solve(c, prog, n):
    R = np.asarray(c["R"]); wxyz = np.asarray(jaxlie.SO3.from_matrix(jnp.asarray(R)).wxyz)
    pos, dt = knot_targets(prog, n); q0 = np.asarray(c["q_start"])
    wc = V.padded_world(c, 25)
    t0 = time.perf_counter()
    init = jax.block_until_ready(ik_chain(V.ROBOT, jnp.array(V.LINK), jnp.asarray(wxyz), jnp.asarray(pos[1:]), jnp.asarray(q0)))
    init = jnp.concatenate([jnp.asarray(q0)[None], init]); t_ik = time.perf_counter() - t0
    targets = jnp.asarray(np.concatenate([np.tile(wxyz, (n, 1)), pos], 1))
    knots = np.asarray(jax.block_until_ready(trajopt_path(V.ROBOT, V.COLL, wc, jnp.asarray(q0), init[-1], targets, init, dt)))
    return knots, dt, t_ik, time.perf_counter() - t0


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--limit", type=int, default=0); ap.add_argument("--deadline", type=float, default=0)
    ap.add_argument("--types", nargs="+", default=["W", "C", "T"]); ap.add_argument("--problems", default=str(ROOT / "problems/v2_2_programs.pkl"))
    ap.add_argument("--out", default=None); a = ap.parse_args()
    cells = pickle.load(open(a.problems, "rb"))["cells"]; dev = pickle.load(open(ROOT / "problems/v2_2_dev.pkl", "rb"))["cells"]
    N = {t: n_knots(cells, t) for t in a.types}
    backend = jax.default_backend()
    out_p = Path(a.out or ROOT / f"results/v2_2/raw/pyroki_{backend}.jsonl"); out_p.parent.mkdir(parents=True, exist_ok=True)
    done = {(r["cell"], r["type"]) for r in map(json.loads, open(out_p))} if out_p.exists() else set()
    compile_s = {}
    for t in a.types:  # compile + warm on a dev program of the same type and knot count (not in the measured set)
        t0 = time.perf_counter(); solve(dev[0], dev[0]["programs"][t], N[t]); c0 = time.perf_counter() - t0
        t0 = time.perf_counter(); solve(dev[1], dev[1]["programs"][t], N[t]); w = time.perf_counter() - t0
        compile_s[t] = {"first_call_s": c0, "warm_s": w, "n_knots": N[t]}
    print("compile", compile_s, flush=True)
    meta = {"backend": backend, "device": str(jax.devices()[0]), "margin_m": V.MARGIN, "knot_dt_max_s": KNOT_DT_MAX, "compile": compile_s}
    json.dump(meta, open(out_p.with_suffix(".meta.json"), "w"), indent=1)
    out = open(out_p, "a")
    for c in cells[: a.limit or None]:
        for t in a.types:
            if (c["index"], t) in done:
                continue
            if a.deadline and time.time() > a.deadline:
                return
            prog = c["programs"][t]
            knots, dt, t_ik, t_ready = solve(c, prog, N[t])
            r1, r2, tm, q = V.gate(c, knots, dt)
            m1 = metrics(q, prog, c["R"])
            start_ok = bool(np.all(np.abs(knots[0] - np.asarray(c["q_start"])) <= V.END_TOL))
            path_ok = lambda m: m["end_pos_err_mm"] <= 1 and m["end_rot_err_deg"] <= 1 and m["max_dev_outside_mm"] <= 5 and m["max_orient_dev_deg"] <= 1
            rec = {"cell": c["index"], "type": t, "planner": "pyroki", "n_knots": N[t], "knot_dt_s": dt, "t_ik_init_s": t_ik,
                   "t_ready_s": t_ready, **tm, "start_ok": start_ok, "load1": os.getloadavg()[0],
                   "R1": {"gate": {k: r1[k] for k in ("valid", "colliding_samples", "bound_violation_rad", "max_vel_ratio", "max_acc_ratio", "max_jerk_ratio")},
                          "t_exec_s": r1["t_exec_s"], "metrics": m1}}
            rec["R1"]["success"] = bool(r1["valid"] and start_ok and path_ok(m1) and t_ready <= 30)
            if r2 is not None:
                q2 = V.knots_to_1khz(knots, dt, r2["scale"])[0]; m2 = metrics(q2, prog, c["R"])
                rec["R2"] = {"gate": {k: r2[k] for k in ("valid", "colliding_samples", "max_vel_ratio", "max_acc_ratio", "max_jerk_ratio")},
                             "scale": r2["scale"], "t_exec_s": r2["t_exec_s"], "metrics": m2,
                             "success": bool(r2["valid"] and start_ok and path_ok(m2) and t_ready + r2["t_scale_s"] <= 30)}
            else:
                rec["R2"] = None
            out.write(json.dumps(rec) + "\n"); out.flush()
            print(c["index"], t, rec["R1"]["success"], None if r2 is None else rec["R2"]["success"], "%.2fs" % t_ready,
                  "dev %.2fmm jerk %.2f" % (m1["max_dev_outside_mm"], r1["max_jerk_ratio"]), flush=True)


if __name__ == "__main__":
    main()
