"""v2.1 MoveL harness check (diagnostic, not a result): is PyRoki's MoveL shortfall our setup or PyRoki?

On a stratified subsample of the 480 MoveL segments (every 20th: 24 segments), compare:
  ours:       the v2.1 run as executed (start held on 2 knots, end only via the per-knot pose cost 50/10, end at rest,
              no rest cost, jaxls default termination: 100 iterations)
  recipe:     PyRoki's own trajopt recipe (examples/pyroki_snippets/_trajopt.py): both endpoints held in joint space on
              2 knots each (end = the stored IK configuration of the segment's end pose), rest cost 0.01; plus the same
              per-knot pose cost toward the line on the interior knots, and the same acc/jerk/velocity/limit/collision terms
  recipe_it:  recipe with max_iterations 500 (jaxls default 100)
  ours_it:    ours with max_iterations 500
Records start/end joint error, end pose error, gate R1/R2, TCP deviation, iterations and the termination flags.
Usage: bench/pyroki_env.sh python bench/v2_1_movel_diag.py   (PYROKI_MARGIN=0.005)
Writes results/v2_1/movel_diag.jsonl.
"""
import json, pickle, sys, time
from pathlib import Path

import jax, jax.numpy as jnp, jax_dataclasses as jdc, jaxlie, jaxls
import numpy as np
import pyroki as pk
from jaxls import Cost

sys.path.insert(0, str(Path(__file__).parent))
sys.argv = sys.argv[:1]
import v2_1_pyroki as V

N = V.N_KNOTS


def build(robot, coll, wc, q0, q1, targets, dt, recipe):
    T = N; tv = robot.joint_var_cls(jnp.arange(T)); a = jnp.arange

    @Cost.factory(kind="constraint_eq_zero", name="start")
    def start(vals, var):
        return (vals[var] - q0).flatten()

    @Cost.factory(kind="constraint_eq_zero", name="end")
    def end(vals, var):
        return (vals[var] - q1).flatten()

    @Cost.factory(kind="constraint_eq_zero", name="end_at_rest")
    def end_rest(vals, va, vb):
        return (vals[va] - vals[vb]).flatten()

    rb = jax.tree.map(lambda x: x[None], robot)
    costs = V._common_costs(robot, coll, wc, tv, T, dt) + [start(robot.joint_var_cls(a(0, 2)))]
    if recipe:
        costs += [end(robot.joint_var_cls(a(T - 2, T))),
                  pk.costs.rest_cost(tv, tv.default_factory()[None], jnp.array([0.01])[None]),
                  pk.costs.pose_cost(rb, robot.joint_var_cls(a(2, T - 2)), jaxlie.SE3(targets[2:T - 2]), jnp.array(V.LINK),
                                     jnp.array([50.0] * 3)[None], jnp.array([10.0] * 3)[None])]
    else:
        costs += [end_rest(robot.joint_var_cls(T - 1), robot.joint_var_cls(T - 2)),
                  pk.costs.pose_cost(rb, tv, jaxlie.SE3(targets), jnp.array(V.LINK), jnp.array([50.0] * 3)[None], jnp.array([10.0] * 3)[None])]
    return tv, costs


@jdc.jit
def solve(robot, coll, wc, q0, q1, targets, init, dt, recipe: jdc.Static[bool], iters: jdc.Static[int]):
    tv, costs = build(robot, coll, wc, q0, q1, targets, dt, recipe)
    sol, summ = jaxls.LeastSquaresProblem(costs=costs, variables=[tv]).analyze().solve(
        verbose=False, initial_vals=jaxls.VarValues.make((tv.with_value(init),)),
        termination=jaxls.TerminationConfig(max_iterations=iters), return_summary=True)
    return sol[tv], summ.iterations, summ.termination_criteria


def main():
    cells = pickle.load(open(V.ROOT / "problems/v2_pickplace.pkl", "rb"))["cells"]
    s_k, dt = V.movel_profile()
    segs = []
    for ci, c in enumerate(cells):
        for pi, pick in enumerate(c["picks"]):
            for side in ("grasp", "place"):
                e = pick[side]; pos = np.asarray(e["pos"]); up = pos + np.array([0, 0, V.LIFT])
                segs.append((ci, pi, side, "down", e["q_above"], up, pos, e["q"], e["R"]))
                segs.append((ci, pi, side, "up", e["q"], pos, up, e["q_above"], e["R"]))
    out = open(V.ROOT / "results/v2_1/movel_diag.jsonl", "w")
    for si in range(0, len(segs), 20):
        ci, pi, side, d, qf, p0, p1, q_end, R = segs[si]
        c = cells[ci]; wc = V.padded_world(c, 25)
        wxyz = np.asarray(jaxlie.SO3.from_matrix(jnp.asarray(R)).wxyz)
        targets = jnp.asarray(np.array([np.concatenate([wxyz, p0 + s * (p1 - p0) / V.LIFT]) for s in s_k]))
        init = jnp.asarray(np.linspace(qf, q_end, N))
        rec = {"seg": si, "cell": ci, "dir": d}
        for name, recipe, iters in (("ours", False, 100), ("recipe", True, 100), ("ours_it", False, 500), ("recipe_it", True, 500)):
            f = lambda: solve(V.ROBOT, V.COLL, wc, jnp.asarray(qf), jnp.asarray(q_end), targets, init, dt, recipe, iters)
            if si == 0:
                jax.block_until_ready(f())  # compile (segment 0 only), then time warm
            t0 = time.perf_counter(); knots, it, term = jax.block_until_ready(f()); t = time.perf_counter() - t0
            knots = np.asarray(knots)
            r1, r2, tm, q = V.gate(c, knots, dt)
            Tend = V.tcp(V.VM, knots[-1]); dp = float(np.linalg.norm(Tend[:3, 3] - p1) * 1e3)
            dr = float(np.degrees(np.arccos(np.clip((np.trace(np.asarray(R).T @ Tend[:3, :3]) - 1) / 2, -1, 1))))
            m = V.moveL_metrics(q, p0, p1, R)
            rec[name] = {"t_s": t, "iterations": int(it), "termination": np.asarray(term).tolist(),
                         "start_err_rad": float(np.abs(knots[0] - qf).max()), "end_joint_err_rad": float(np.abs(knots[-1] - q_end).max()),
                         "end_pos_err_mm": dp, "end_rot_err_deg": dr, "R1_valid": r1["valid"], "R1_jerk": r1["max_jerk_ratio"],
                         "R1_coll": r1["colliding_samples"], "R2_valid": None if r2 is None else r2["valid"],
                         "R2_scale": None if r2 is None else r2["scale"], "max_tcp_dev_mm": m["max_tcp_dev_mm"],
                         "max_orient_dev_deg": m["max_orient_dev_deg"]}
        out.write(json.dumps(rec) + "\n"); out.flush()
        print(si, *[(n, rec[n]["iterations"], f'{rec[n]["start_err_rad"]:.1e}', f'{rec[n]["end_pos_err_mm"]:.2f}mm',
                     rec[n]["R1_valid"], rec[n]["R2_valid"], f'{rec[n]["max_tcp_dev_mm"]:.2f}', f'{rec[n]["t_s"]:.1f}s')
                    for n in ("ours", "recipe", "ours_it", "recipe_it")], flush=True)


if __name__ == "__main__":
    main()
