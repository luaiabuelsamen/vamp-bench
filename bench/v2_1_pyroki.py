"""v2.1: PyRoki as a trajectory optimiser (docs/BENCHMARK_V2.md section 7, amendments 7a-7c).

  ik       TCP (tool0) poses of the 300 held-out A6 goals; (a) PyRoki's default initial configuration,
           (b) 64 random initial configurations solved batched, lowest pose error returned
  movel    the 480 MoveL segments of the 120 A6 pick slots (problems/v2_pickplace.pkl), isolated
  trajopt  held-out A6 free-space trajopt; --seeds s1 (straight line) or s2 (cuRobo v1's per-attempt seeds)
Run: bench/pyroki_env.sh python bench/v2_1_pyroki.py <mode> [--limit N] [--deadline unix_t]   (GPU)
     JAX_PLATFORMS=cpu bench/pyroki_env.sh python bench/v2_1_pyroki.py <mode> ...                (CPU)
Writes results/v2_1/raw/pyroki_<mode>[_<seeds>]_<backend>.jsonl (resumable).
Settings: PyRoki's examples (01_basic_ik, 07_trajopt) with the changes in section 7; validity is decided by the v2
1 kHz gate (bench/v2_validate.py), in this process, with the same VAMP build.
"""
import argparse, json, os, pickle, platform, sys, time
from pathlib import Path

import jax, jax.numpy as jnp, jax_dataclasses as jdc, jaxlie, jaxls
import numpy as np
import pyroki as pk
from jaxls import Cost
from pyroki._residuals._residuals import limit_acceleration_residual, limit_jerk_residual
from scipy.interpolate import CubicSpline
from ruckig import InputParameter, Result, Ruckig, Trajectory

sys.path.insert(0, str(Path(__file__).parent))
import vamp
from pyroki_smoke import load_robot
from v2_common import ROOT, limits, seed_from, tcp
from v2_pickplace_metrics import moveL_metrics
from v2_timing import ruckig_p1
from v2_validate import validate

VM = "ur5erail"; VMAX, AMAX, JMAX = limits(VM)
T_BUDGET = 30.0; N_KNOTS = 50; EE = "tool0"
TCP_V, TCP_A, TCP_J, LIFT = 0.25, 1.0, 10.0, 0.10
END_TOL = np.array([1e-3] * 7)  # rad (rail: m)
MOVEL_ENDS = os.environ.get("PYROKI_MOVEL_ENDS", "pose")  # 7g
MARGIN = float(os.environ.get("PYROKI_MARGIN", "0.0"))  # world-collision margin (m); chosen on the gantry dev set (7d)
acc_limit_constraint = Cost.factory(kind="constraint_leq_zero")(limit_acceleration_residual)
jerk_limit_constraint = Cost.factory(kind="constraint_leq_zero")(limit_jerk_residual)
URDF, ROBOT, COLL = load_robot()
LINK = ROBOT.links.names.index(EE)


def padded_world(p, n):
    """Obstacles as PyRoki Boxes, padded to n with 1 cm boxes 100 m away (inert) so shapes are fixed per set."""
    B = p["box"]; assert len(B) <= n and not p["cylinder"] and not p["sphere"]
    ext = [2 * np.asarray(b["half_extents"]) for b in B] + [np.full(3, 0.01)] * (n - len(B))
    pos = [np.asarray(b["position"]) for b in B] + [np.array([100.0, 100.0, 100.0 + i]) for i in range(n - len(B))]
    wxyz = [np.asarray(b["orientation_quat_xyzw"])[[3, 0, 1, 2]] for b in B] + [np.array([1.0, 0, 0, 0])] * (n - len(B))
    return pk.collision.Box.from_extent(np.array(ext), np.array(pos), np.array(wxyz))


# ---------------------------------------------------------------- IK (01_basic_ik)
def _ik_problem(robot, link, wxyz, pos, q_init):
    v = robot.joint_var_cls(0)
    costs = [pk.costs.pose_cost_analytic_jac(robot, v, jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(wxyz), pos),
                                             link, pos_weight=50.0, ori_weight=10.0),
             pk.costs.limit_constraint(robot, v)]
    init = None if q_init is None else jaxls.VarValues.make((v.with_value(q_init),))
    sol = jaxls.LeastSquaresProblem(costs=costs, variables=[v]).analyze().solve(
        verbose=False, linear_solver="dense_cholesky", trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0), initial_vals=init)
    return sol[v]


def _ik_coll_problem(robot, coll, wc, link, wxyz, pos, q_init):
    """04_ik_with_coll recipe (pose 5/1, rest 0.01, self-collision 0.02 m soft, world collision hard), margin 0 (7c/7d)."""
    v = robot.joint_var_cls(0)
    costs = [pk.costs.pose_cost(robot, v, jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(wxyz), pos), link, 5.0, 1.0),
             pk.costs.rest_cost(v, jnp.array(v.default_factory()), 0.01),
             pk.costs.self_collision_cost(robot, coll, v, 0.02, 5.0),
             pk.costs.limit_constraint(robot, v),
             pk.costs.world_collision_constraint(robot, coll, v, wc, MARGIN)]
    sol = jaxls.LeastSquaresProblem(costs=costs, variables=[v]).analyze().solve(
        verbose=False, linear_solver="dense_cholesky", initial_vals=jaxls.VarValues.make((v.with_value(q_init),)))
    return sol[v]


@jdc.jit
def ik_coll_batch(robot, coll, wc, link, wxyz, pos, q_inits):
    return jax.vmap(lambda q: _ik_coll_problem(robot, coll, wc, link, wxyz, pos, q))(q_inits)


@jdc.jit
def ik_default(robot, link, wxyz, pos):
    return _ik_problem(robot, link, wxyz, pos, None)


@jdc.jit
def ik_batch(robot, link, wxyz, pos, q_inits):
    return jax.vmap(lambda q: _ik_problem(robot, link, wxyz, pos, q))(q_inits)


def pose_err(q, T):
    Tq = tcp(VM, q)
    dp = float(np.linalg.norm(Tq[:3, 3] - T[:3, 3]) * 1e3)
    c = (np.trace(T[:3, :3].T @ Tq[:3, :3]) - 1) / 2
    return dp, float(np.degrees(np.arccos(np.clip(c, -1, 1))))


# ---------------------------------------------------------------- trajopt (07_trajopt + section 7 changes)
def _common_costs(robot, coll, wc, tv, T, dt):
    rb, cb = jax.tree.map(lambda x: x[None], robot), jax.tree.map(lambda x: x[None], coll)
    V = robot.joint_var_cls; a = jnp.arange
    costs = [pk.costs.smoothness_cost(V(a(1, T)), V(a(0, T - 1)), jnp.array([1.0])[None]),
             pk.costs.five_point_acceleration_cost(V(a(2, T - 2)), V(a(4, T)), V(a(3, T - 1)), V(a(1, T - 3)), V(a(0, T - 4)),
                                                   dt, jnp.array([0.1])[None]),
             pk.costs.self_collision_cost(rb, cb, tv, 0.02, 5.0),
             pk.costs.limit_constraint(rb, tv),
             pk.costs.limit_velocity_constraint(rb, V(a(1, T)), V(a(0, T - 1)), dt),
             acc_limit_constraint(V(a(2, T - 2)), V(a(4, T)), V(a(3, T - 1)), V(a(1, T - 3)), V(a(0, T - 4)), dt, jnp.asarray(AMAX)[None]),
             jerk_limit_constraint(V(a(6, T)), V(a(5, T - 1)), V(a(4, T - 2)), V(a(2, T - 4)), V(a(1, T - 5)), V(a(0, T - 6)),
                                   dt, jnp.asarray(JMAX)[None])]

    def wres(vals, robot, coll, w, pa, pb):
        caps = coll.get_swept_capsules(robot, vals[pa], vals[pb])
        return pk.collision.collide(caps.reshape((-1, 1)), w.reshape((1, -1))).flatten() - MARGIN  # >= 0 (7c/7d)

    costs.append(Cost(wres, (rb, cb, jax.tree.map(lambda x: x[None], wc), V(a(0, T - 1)), V(a(1, T))),
                      kind="constraint_geq_zero", name="world"))
    return costs


@jdc.jit
def trajopt_js(robot, coll, wc, q0, q1, init, dt):
    T = N_KNOTS; tv = robot.joint_var_cls(jnp.arange(T))

    @Cost.factory(kind="constraint_eq_zero", name="start")
    def start(vals, var):
        return (vals[var] - q0).flatten()

    @Cost.factory(kind="constraint_eq_zero", name="goal")
    def goal(vals, var):
        return (vals[var] - q1).flatten()

    # endpoints held on two knots each, as in 07_trajopt (zero velocity at both ends; the stencils skip the end knots)
    costs = _common_costs(robot, coll, wc, tv, T, dt) + [start(robot.joint_var_cls(jnp.arange(0, 2))),
                                                          goal(robot.joint_var_cls(jnp.arange(T - 2, T)))]
    sol = jaxls.LeastSquaresProblem(costs=costs, variables=[tv]).analyze().solve(
        verbose=False, initial_vals=jaxls.VarValues.make((tv.with_value(init),)))
    return sol[tv]


@jdc.jit
def trajopt_movel(robot, coll, wc, q0, q1, targets_wxyz_xyz, init, dt):
    """MoveL. MOVEL_ENDS 'pose' (pre-registered run): start held on 2 knots, end reached through the per-knot pose cost,
    end at rest. 'recipe' (7g): PyRoki's own trajopt recipe for endpoints (pyroki_snippets/_trajopt.py): start and end
    configurations held on 2 knots each (end = the segment's stored IK configuration), rest cost 0.01, and the per-knot
    pose cost on the interior knots."""
    T = N_KNOTS; tv = robot.joint_var_cls(jnp.arange(T))

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
    costs = _common_costs(robot, coll, wc, tv, T, dt) + [start(robot.joint_var_cls(jnp.arange(0, 2)))]
    if MOVEL_ENDS == "recipe":
        costs += [end(robot.joint_var_cls(jnp.arange(T - 2, T))),
                  pk.costs.rest_cost(tv, tv.default_factory()[None], jnp.array([0.01])[None]),
                  pk.costs.pose_cost(rb, robot.joint_var_cls(jnp.arange(2, T - 2)), jaxlie.SE3(targets_wxyz_xyz[2:T - 2]),
                                     jnp.array(LINK), jnp.array([50.0] * 3)[None], jnp.array([10.0] * 3)[None])]
    else:
        costs += [end_rest(robot.joint_var_cls(T - 1), robot.joint_var_cls(T - 2)),
                  pk.costs.pose_cost(rb, tv, jaxlie.SE3(targets_wxyz_xyz), jnp.array(LINK), jnp.array([50.0] * 3)[None],
                                     jnp.array([10.0] * 3)[None])]
    sol = jaxls.LeastSquaresProblem(costs=costs, variables=[tv]).analyze().solve(
        verbose=False, initial_vals=jaxls.VarValues.make((tv.with_value(init),)))
    return sol[tv]


def knots_to_1khz(knots, dt, s=1.0):
    """C2 cubic spline through the knots (zero end velocities), time stretched by s, sampled at 1 kHz."""
    t = np.arange(len(knots)) * dt * s
    sp = CubicSpline(t, knots, axis=0, bc_type="clamped")
    ts = np.arange(0, t[-1] + 1e-9, 1e-3)
    return sp(ts), sp(ts, 1), sp(ts, 2), float(t[-1])


def gate(p, knots, dt):
    """R1 (as returned) and, if R1 fails only on vel/acc/jerk, R2 (uniform time scaling). Returns (r1, r2, timings)."""
    t0 = time.perf_counter(); q, qd, qdd, dur = knots_to_1khz(knots, dt); t_spline = time.perf_counter() - t0
    t0 = time.perf_counter(); v1 = validate(VM, p, q, qd, qdd); t_val = time.perf_counter() - t0
    r1 = {**v1, "t_exec_s": dur}
    r2 = None
    limits_only = (not v1["valid"]) and v1["colliding_samples"] == 0 and v1["bound_violation_rad"] <= 1e-5
    if limits_only:
        t0 = time.perf_counter()
        s = max(1.0, v1["max_vel_ratio"], np.sqrt(v1["max_acc_ratio"]), np.cbrt(v1["max_jerk_ratio"]))
        q2, qd2, qdd2, dur2 = knots_to_1khz(knots, dt, s); t_scale = time.perf_counter() - t0
        t0 = time.perf_counter(); v2 = validate(VM, p, q2, qd2, qdd2); t_val += time.perf_counter() - t0
        r2 = {**v2, "t_exec_s": dur2, "scale": float(s), "t_scale_s": t_scale}
    return r1, r2, {"t_spline_s": t_spline, "t_validate_s": t_val}, q


def timed(f):
    t0 = time.perf_counter(); r = jax.block_until_ready(f()); return np.asarray(r), time.perf_counter() - t0


def line_seconds(q0, q1):
    return ruckig_p1(np.array([q0, q1], float), VMAX, AMAX, JMAX).duration


# ---------------------------------------------------------------- modes
def run_ik(a, out, done):
    probs = pickle.load(open(ROOT / "problems/v2_heldout_a6.pkl", "rb"))["problems"]["v2_heldout_a6"]
    rng = np.random.default_rng(seed_from(a.freeze, "pyroki_ik"))
    lo, hi = np.asarray(ROBOT.joints.lower_limits), np.asarray(ROBOT.joints.upper_limits)
    inits = jnp.asarray(rng.uniform(lo, hi, size=(64, 7)))
    pp = pickle.load(open(ROOT / "problems/v2_pickplace.pkl", "rb"))["cells"]
    compile_s = {}
    wc0 = padded_world(pp[0], 40)
    for name, f in (("default", lambda T: ik_default(ROBOT, jnp.array(LINK), *T)), ("rand64", lambda T: ik_batch(ROBOT, jnp.array(LINK), *T, inits)),
                    ("coll_rand64", lambda T: ik_coll_batch(ROBOT, COLL, wc0, jnp.array(LINK), *T, inits))):
        Tw = [tcp(VM, pp[0]["picks"][k]["grasp"]["q"]) for k in range(3)]  # warm-up targets outside the measured set
        args = [(jnp.asarray(jaxlie.SO3.from_matrix(jnp.asarray(T[:3, :3])).wxyz), jnp.asarray(T[:3, 3])) for T in Tw]
        _, c = timed(lambda: f(args[0])); _, w1 = timed(lambda: f(args[1])); _, w2 = timed(lambda: f(args[2]))
        compile_s[name] = {"first_call_s": c, "compile_s": c - min(w1, w2)}
    for qi in [i for i in range(len(probs))[: a.limit or None] if i not in done]:
        if a.deadline and time.time() > a.deadline:
            break
        p = probs[qi]; env = vamp.problem_dict_to_vamp(p); T = tcp(VM, p["goals"][0])
        arg = (jnp.asarray(jaxlie.SO3.from_matrix(jnp.asarray(T[:3, :3])).wxyz), jnp.asarray(T[:3, 3]))
        rec = {"q": qi, "bins": {x: p["bins"][x] for x in ("reach", "clearance", "passage", "limits")}, "load1": os.getloadavg()[0]}
        wc = padded_world(p, 40)
        for name in ("default", "rand64", "coll_rand64"):
            if name == "default":
                qs, t = timed(lambda: ik_default(ROBOT, jnp.array(LINK), *arg)); cand = qs[None]
            elif name == "rand64":
                qs, t = timed(lambda: ik_batch(ROBOT, jnp.array(LINK), *arg, inits)); cand = qs
            else:
                qs, t = timed(lambda: ik_coll_batch(ROBOT, COLL, wc, jnp.array(LINK), *arg, inits)); cand = qs
            errs = [pose_err(c, T) for c in cand]
            k = int(np.argmin([e[0] + e[1] for e in errs]))  # lowest pose error (mm + deg)
            within = [e[0] <= 1.0 and e[1] <= 1.0 for e in errs]
            if name == "coll_rand64" and any(within):  # 7d: among starts within tolerance, the lowest PyRoki cost proxy = pose error
                k = min((i for i in range(len(cand)) if within[i]), key=lambda i: errs[i][0] + errs[i][1])
            qb = cand[k]; dp, dr = errs[k]
            bounds = bool(np.all(qb >= np.array(vamp.ur5erail.lower_bounds()) - 1e-5) and np.all(qb <= np.array(vamp.ur5erail.upper_bounds()) + 1e-5))
            free = bool(vamp.ur5erail.validate(np.clip(qb, vamp.ur5erail.lower_bounds(), vamp.ur5erail.upper_bounds()).astype(np.float32).tolist(), env))
            within = [e[0] <= 1.0 and e[1] <= 1.0 for e in errs]
            rec[name] = {"t_s": t, "pos_err_mm": dp, "rot_err_deg": dr, "in_bounds": bounds, "collision_free": free,
                         "success": bool(dp <= 1.0 and dr <= 1.0 and bounds and free), "n_inits_within_tol": int(sum(within)),
                         "q": qb.tolist(), "dist_to_a6_goal_rad": float(np.abs(qb - np.asarray(p["goals"][0])).max())}
        out.write(json.dumps(rec) + "\n"); out.flush()
        print(qi, *[(n, rec[n]["success"], round(rec[n]["t_s"] * 1e3, 1)) for n in ("default", "rand64", "coll_rand64")], flush=True)
    return {"compile": compile_s, "n_rand_inits": 64}


def movel_profile():
    inp = InputParameter(1); inp.current_position = [0.0]; inp.target_position = [LIFT]
    inp.current_velocity = inp.current_acceleration = inp.target_velocity = inp.target_acceleration = [0.0]
    inp.max_velocity, inp.max_acceleration, inp.max_jerk = [TCP_V], [TCP_A], [TCP_J]
    tr = Trajectory(1); assert Ruckig(1).calculate(inp, tr) in (Result.Working, Result.Finished)
    ts = np.linspace(0, tr.duration, N_KNOTS)
    return np.array([tr.at_time(t)[0][0] for t in ts]), tr.duration / (N_KNOTS - 1)


def run_movel(a, out, done):
    cells = pickle.load(open(ROOT / "problems/v2_pickplace.pkl", "rb"))["cells"]
    s_k, dt = movel_profile()
    segs = []
    for ci, c in enumerate(cells):
        for pi, pick in enumerate(c["picks"]):
            for side in ("grasp", "place"):
                e = pick[side]; pos = np.asarray(e["pos"]); up = pos + np.array([0, 0, LIFT])
                segs.append((ci, pi, side, "down", e["q_above"], up, pos, e["q"], e["R"]))
                segs.append((ci, pi, side, "up", e["q"], pos, up, e["q_above"], e["R"]))
    first = True; info = {}
    for si, (ci, pi, side, d, qf, p0, p1, q_end, R) in enumerate(segs[: a.limit or None]):
        if si in done:
            continue
        if a.deadline and time.time() > a.deadline:
            break
        c = cells[ci]; wc = padded_world(c, 25)
        wxyz = np.asarray(jaxlie.SO3.from_matrix(jnp.asarray(R)).wxyz)
        targets = np.array([np.concatenate([wxyz, p0 + s * (p1 - p0) / LIFT]) for s in s_k])
        init = np.linspace(qf, q_end, N_KNOTS)
        f = lambda: trajopt_movel(ROBOT, COLL, wc, jnp.asarray(qf), jnp.asarray(q_end), jnp.asarray(targets), jnp.asarray(init), dt)
        if first:  # compile, on segment 0 (reported separately), then time warm
            _, c0 = timed(f); first = False; info["first_call_s"] = c0
        knots, t_solve = timed(f)
        r1, r2, tm, q = gate(c, knots, dt)
        start_ok = bool(np.all(np.abs(knots[0] - np.asarray(qf)) <= END_TOL))
        Tend = tcp(VM, knots[-1]); dp = float(np.linalg.norm(Tend[:3, 3] - p1) * 1e3)
        dr = float(np.degrees(np.arccos(np.clip((np.trace(np.asarray(R).T @ Tend[:3, :3]) - 1) / 2, -1, 1))))
        end_ok = dp <= 1.0 and dr <= 1.0
        rec = {"seg": si, "cell": ci, "pick": pi, "side": side, "dir": d, "t_solve_s": t_solve, **tm, "load1": os.getloadavg()[0],
               "start_ok": start_ok, "end_pos_err_mm": dp, "end_rot_err_deg": dr, "end_ok": end_ok,
               "R1": {**r1, "success": bool(r1["valid"] and start_ok and end_ok and t_solve <= T_BUDGET)},
               "R2": None if r2 is None else {**r2, "success": bool(r2["valid"] and start_ok and end_ok and t_solve + r2["t_scale_s"] <= T_BUDGET)}}
        rec["R1"].update(moveL_metrics(q, p0, p1, R))
        if r2 is not None:
            q2 = knots_to_1khz(knots, dt, r2["scale"])[0]; rec["R2"].update(moveL_metrics(q2, p0, p1, R))
        out.write(json.dumps(rec) + "\n"); out.flush()
        print(si, rec["R1"]["success"], None if r2 is None else rec["R2"]["success"], f"{t_solve:.2f}s",
              f'jerk {r1["max_jerk_ratio"]:.2f} dev {rec["R1"]["max_tcp_dev_mm"]:.2f}mm', flush=True)
    return {**info, "knot_dt_s": dt, "n_knots": N_KNOTS}


def s2_seeds(qi, q0, q1):
    d = np.load(ROOT / f"results/v2_1/raw/curobo1_seeds_heldout_a6/q{qi:03d}.npz")
    out = []
    for s in d["seeds"][:, 0]:
        s = np.asarray(s, float); s[0], s[-1] = q0, q1
        r = np.stack([np.interp(np.linspace(0, 1, N_KNOTS), np.linspace(0, 1, len(s)), s[:, j]) for j in range(7)], axis=1)
        if not out or np.abs(r - out[-1]).max() > 1e-6:
            out.append(r)
    return out


def run_trajopt(a, out, done):
    probs = (pickle.load(open(ROOT / "problems/v2_heldout_a6.pkl", "rb"))["problems"]["v2_heldout_a6"] if a.set == "heldout_a6"
             else pickle.load(open(ROOT / "problems/gantry_pillars.pkl", "rb"))["problems"]["gantry_pillars"])  # dev set (7d)
    info = {"set": a.set, "margin_m": MARGIN}
    for qi in [i for i in range(0, len(probs), a.stride)[: a.limit or None] if i not in done]:
        if a.deadline and time.time() > a.deadline:
            break
        p = probs[qi]; q0, q1 = np.asarray(p["start"], float), np.asarray(p["goals"][0], float); wc = padded_world(p, 40)
        T_line = line_seconds(q0, q1)
        seeds = [np.linspace(q0, q1, N_KNOTS)] if a.seeds == "s1" else s2_seeds(qi, q0, q1)
        base = [1.5 * max(T_line, float(np.max(np.abs(np.diff(s, axis=0)).sum(0) / VMAX))) for s in seeds]
        if not info:  # compile once (shapes fixed per set), reported separately
            f0 = lambda: trajopt_js(ROBOT, COLL, wc, jnp.asarray(q0), jnp.asarray(q1), jnp.asarray(seeds[0]), base[0] / (N_KNOTS - 1))
            _, c0 = timed(f0); info["first_call_s"] = c0
        attempts, t_ready, success, rnd = [], 0.0, None, 0
        while success is None and t_ready <= T_BUDGET:
            for si, (s, D0) in enumerate(zip(seeds, base)):
                if t_ready > T_BUDGET:
                    break
                D = D0 * 1.5 ** rnd; dt = D / (N_KNOTS - 1)
                knots, t_solve = timed(lambda: trajopt_js(ROBOT, COLL, wc, jnp.asarray(q0), jnp.asarray(q1), jnp.asarray(s), dt))
                r1, r2, tm, _ = gate(p, knots, dt)
                ends = bool(np.all(np.abs(knots[0] - q0) <= END_TOL) and np.all(np.abs(knots[-1] - q1) <= END_TOL))
                t_ready += t_solve + tm["t_spline_s"]
                att = {"round": rnd, "seed_i": si, "D_s": D, "t_solve_s": t_solve, **tm, "ends_ok": ends,
                       "start_err": float(np.abs(knots[0] - q0).max()), "goal_err": float(np.abs(knots[-1] - q1).max()),
                       "R1": {k: r1[k] for k in ("valid", "colliding_samples", "bound_violation_rad", "max_vel_ratio", "max_acc_ratio", "max_jerk_ratio", "t_exec_s")},
                       "R2": None if r2 is None else {k: r2[k] for k in ("valid", "colliding_samples", "max_vel_ratio", "max_acc_ratio", "max_jerk_ratio", "t_exec_s", "scale", "t_scale_s")}}
                attempts.append(att)
                if ends and r1["valid"] and t_ready <= T_BUDGET:
                    success = {"variant": "R1", "t_ready_s": t_ready, "t_exec_s": r1["t_exec_s"], **{k: r1.get(k) for k in ("peak_jerk_norm", "rms_jerk_norm", "min_clearance_m", "joint_length_rad")}}
                    break
            rnd += 1
            if rnd > 12:
                break
        # R2 (post-processed) success: first attempt whose R2 passes, time including scaling, within budget
        t2, s2 = 0.0, None
        for att in attempts:
            t2 += att["t_solve_s"] + att["t_spline_s"]
            if att["ends_ok"] and (att["R1"]["valid"] or (att["R2"] and att["R2"]["valid"])):
                tt = t2 + (att["R2"]["t_scale_s"] if (att["R2"] and not att["R1"]["valid"]) else 0.0)
                if tt <= T_BUDGET:
                    s2 = {"t_ready_s": tt, "t_exec_s": att["R1"]["t_exec_s"] if att["R1"]["valid"] else att["R2"]["t_exec_s"]}
                break
        rec = {"q": qi, "seeds": a.seeds, "n_seeds": len(seeds), "bins": {x: p["bins"][x] for x in ("reach", "clearance", "passage", "limits")} if "bins" in p else None,
               "T_line_s": T_line, "load1": os.getloadavg()[0], "success_R1": success is not None, "R1": success,
               "success_R2": s2 is not None, "R2": s2, "t_ready_total_s": t_ready, "attempts": attempts}
        out.write(json.dumps(rec) + "\n"); out.flush()
        print(qi, a.seeds, len(seeds), "R1" if success else "-", "R2" if s2 else "-", f"{t_ready:.1f}s", len(attempts), flush=True)
    return info


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("mode", choices=["ik", "movel", "trajopt"]); ap.add_argument("--seeds", choices=["s1", "s2"], default="s1")
    ap.add_argument("--freeze", default="6aea6eecf76a173d7ebdf548104aaec854b6b6c7")
    ap.add_argument("--limit", type=int, default=0); ap.add_argument("--deadline", type=float, default=0)
    ap.add_argument("--tag", default=""); ap.add_argument("--set", choices=["heldout_a6", "gantry"], default="heldout_a6")
    ap.add_argument("--stride", type=int, default=1)
    a = ap.parse_args()
    be = jax.default_backend()
    name = f"pyroki_{a.mode}{'_' + a.seeds if a.mode == 'trajopt' else ''}{'_gantry' if a.set == 'gantry' else ''}_{be}{a.tag}"
    OUT = ROOT / "results/v2_1/raw"; OUT.mkdir(parents=True, exist_ok=True)
    f = OUT / f"{name}.jsonl"
    key = {"ik": "q", "movel": "seg", "trajopt": "q"}[a.mode]
    done = {json.loads(l)[key] for l in open(f)} if f.exists() else set()
    with open(f, "a") as out:
        info = {"ik": run_ik, "movel": run_movel, "trajopt": run_trajopt}[a.mode](a, out, done)
    meta = {"mode": a.mode, "backend": be, "devices": [str(d) for d in jax.devices()], "jax": jax.__version__, "host": platform.node(),
            "xla_flags": os.environ.get("XLA_FLAGS"), "pyroki": "388e43e", "jaxls": "50a58be", "T_budget_s": T_BUDGET, "n_knots": N_KNOTS, "margin_m": MARGIN, "movel_ends": MOVEL_ENDS, **info}
    json.dump(meta, open(OUT / f"{name}.meta.json", "w"), indent=1)
    print(json.dumps(meta), flush=True)


if __name__ == "__main__":
    main()
