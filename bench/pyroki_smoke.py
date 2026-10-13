"""v2.1 PyRoki smoke test (CPU): FK agreement, IK and free-space trajopt on the UR5e rail, held-out A6 query 0.

Run in the PyRoki venv with a clean PYTHONPATH:
  env -u PYTHONPATH JAX_PLATFORMS=cpu .venv-pyroki/bin/python bench/pyroki_smoke.py
Same URDF and sphere model as every other planner (configs/ur5e_rail_spherized.urdf, configs/ur5e_rail_shared_v2.yml).
GPU (PyPI jax-cuda12-plugin 0.6.2; see bench/pyroki_env.sh for the cuDNN and XLA flags it needs on this board):
  bench/pyroki_env.sh python bench/pyroki_smoke.py
Writes results/v2_1/smoke_pyroki.json (CPU) or smoke_pyroki_gpu.json and the trajopt knots to results/v2_1/smoke_traj.npz.
"""
import json, os, pickle, platform, time
from pathlib import Path

import jax, jax.numpy as jnp, jax_dataclasses as jdc, jaxlie, jaxls
import numpy as np
import pyroki as pk
import yaml, yourdfpy

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results/v2_1"; OUT.mkdir(parents=True, exist_ok=True)
EE = "tool0"


def load_robot():
    urdf = yourdfpy.URDF.load(str(ROOT / "configs/ur5e_rail_spherized.urdf"), load_meshes=False,
                              build_collision_scene_graph=False, load_collision_meshes=False)
    cfg = yaml.safe_load(open(ROOT / "configs/ur5e_rail_shared_v2.yml"))["robot_cfg"]["kinematics"]
    sph = {l: {"centers": [s["center"] for s in v], "radii": [s["radius"] for s in v]}
           for l, v in cfg["collision_spheres"].items()}
    ign = tuple((a, b) for a, bs in cfg["self_collision_ignore"].items() for b in bs)
    robot = pk.Robot.from_urdf(urdf)
    coll = pk.collision.RobotCollision.from_sphere_decomposition(sph, urdf, user_ignore_pairs=ign)
    return urdf, robot, coll


def world(p):
    boxes = [b for b in p["box"]]
    xyzw = np.array([b["orientation_quat_xyzw"] for b in boxes])
    return pk.collision.Box.from_extent(np.array([2 * np.array(b["half_extents"]) for b in boxes]),
                                        np.array([b["position"] for b in boxes]), xyzw[:, [3, 0, 1, 2]])


@jdc.jit
def ik(robot, link, wxyz, pos):
    v = robot.joint_var_cls(0)
    costs = [pk.costs.pose_cost_analytic_jac(robot, v, jaxlie.SE3.from_rotation_and_translation(jaxlie.SO3(wxyz), pos),
                                             link, pos_weight=50.0, ori_weight=10.0),
             pk.costs.limit_constraint(robot, v)]
    sol = jaxls.LeastSquaresProblem(costs=costs, variables=[v]).analyze().solve(
        verbose=False, linear_solver="dense_cholesky", trust_region=jaxls.TrustRegionConfig(lambda_initial=1.0))
    return sol[v]


@jdc.jit
def trajopt(robot, coll, wc, q0, q1, timesteps: jdc.Static[int], dt: float):
    """PyRoki's 07_trajopt recipe, joint-space start/goal: smoothness + acceleration costs, limits, velocity limits,
    swept-sphere world collision (margin 0.05 m in the example; 0 in this smoke test). The v2.1 runs use the settings in docs/BENCHMARK_V2.md section 7."""
    tv = robot.joint_var_cls(jnp.arange(timesteps))
    rb, cb = jax.tree.map(lambda x: x[None], robot), jax.tree.map(lambda x: x[None], coll)
    costs = [pk.costs.smoothness_cost(robot.joint_var_cls(jnp.arange(1, timesteps)),
                                      robot.joint_var_cls(jnp.arange(0, timesteps - 1)), jnp.array([1.0])[None]),
             pk.costs.five_point_acceleration_cost(
                 robot.joint_var_cls(jnp.arange(2, timesteps - 2)), robot.joint_var_cls(jnp.arange(4, timesteps)),
                 robot.joint_var_cls(jnp.arange(3, timesteps - 1)), robot.joint_var_cls(jnp.arange(1, timesteps - 3)),
                 robot.joint_var_cls(jnp.arange(0, timesteps - 4)), dt, jnp.array([0.1])[None]),
             pk.costs.self_collision_cost(rb, cb, tv, 0.0, 5.0),
             pk.costs.limit_constraint(rb, tv),
             pk.costs.limit_velocity_constraint(rb, robot.joint_var_cls(jnp.arange(1, timesteps)),
                                                robot.joint_var_cls(jnp.arange(0, timesteps - 1)), dt)]

    @jaxls.Cost.factory(kind="constraint_eq_zero", name="start")
    def start(vals, var):
        return (vals[var] - q0).flatten()

    @jaxls.Cost.factory(kind="constraint_eq_zero", name="goal")
    def goal(vals, var):
        return (vals[var] - q1).flatten()

    def wres(vals, robot, coll, w, a, b):
        caps = coll.get_swept_capsules(robot, vals[a], vals[b])
        return pk.collision.collide(caps.reshape((-1, 1)), w.reshape((1, -1))).flatten()

    costs += [start(robot.joint_var_cls(jnp.arange(0, 1))), goal(robot.joint_var_cls(jnp.arange(timesteps - 1, timesteps))),
              jaxls.Cost(wres, (rb, cb, jax.tree.map(lambda x: x[None], wc), robot.joint_var_cls(jnp.arange(0, timesteps - 1)),
                                robot.joint_var_cls(jnp.arange(1, timesteps))), kind="constraint_geq_zero", name="world")]
    sol = jaxls.LeastSquaresProblem(costs=costs, variables=[tv]).analyze().solve(
        verbose=False, initial_vals=jaxls.VarValues.make((tv.with_value(jnp.linspace(q0, q1, timesteps)),)))
    return sol[tv]


def timed(f, n=5):
    t = time.perf_counter(); r = jax.block_until_ready(f()); first = time.perf_counter() - t
    w = []
    for _ in range(n):
        t = time.perf_counter(); r = jax.block_until_ready(f()); w.append(time.perf_counter() - t)
    return r, first, float(np.median(w))


def main():
    urdf, robot, coll = load_robot()
    names = list(robot.joints.actuated_names)
    p = pickle.load(open(ROOT / "problems/v2_heldout_a6.pkl", "rb"))["problems"]["v2_heldout_a6"][0]
    q0, q1 = np.array(p["start"]), np.array(p["goals"][0])
    link = robot.links.names.index(EE)
    rep = {"jax": jax.__version__, "backend": jax.default_backend(), "devices": [str(d) for d in jax.devices()],
           "host": platform.node(), "joints": names, "query": "heldout_a6[0]"}

    # FK agreement with yourdfpy (independent URDF parser)
    urdf.update_cfg(dict(zip(names, q1)))
    T_ref = urdf.get_transform(EE, urdf.base_link)
    T_pk = jaxlie.SE3(robot.forward_kinematics(jnp.array(q1))[link]).as_matrix()
    rep["fk_max_abs_diff"] = float(np.abs(np.array(T_pk) - T_ref).max())

    # IK to the goal TCP pose, from PyRoki's default initialization
    tgt = jaxlie.SE3.from_matrix(jnp.array(T_ref))
    qi, t_first, t_warm = timed(lambda: ik(robot, jnp.array(link), tgt.rotation().wxyz, tgt.translation()))
    Ti = jaxlie.SE3(robot.forward_kinematics(qi)[link])
    err = (tgt.inverse() @ Ti).log()
    rep["ik"] = {"first_call_s": t_first, "warm_median_s": t_warm, "pos_err_mm": float(jnp.linalg.norm(err[:3]) * 1e3),
                 "rot_err_deg": float(jnp.rad2deg(jnp.linalg.norm(err[3:]))), "q": np.asarray(qi).tolist()}

    # Free-space trajopt start -> goal from the straight-line seed
    wc = world(p); T, dt = 64, 0.05
    tr, t_first, t_warm = timed(lambda: trajopt(robot, coll, wc, jnp.array(q0), jnp.array(q1), T, dt), n=3)
    tr = np.asarray(tr)
    caps_d = pk.collision.collide(coll.at_config(robot, jnp.array(tr)).reshape((T, -1, 1)), wc.reshape((1, 1, -1)))
    rep["trajopt"] = {"timesteps": T, "dt": dt, "first_call_s": t_first, "warm_median_s": t_warm,
                      "start_err": float(np.abs(tr[0] - q0).max()), "goal_err": float(np.abs(tr[-1] - q1).max()),
                      "min_knot_world_dist_m": float(caps_d.min())}
    np.savez(OUT / f"smoke_traj_{jax.default_backend()}.npz", q=tr, dt=dt)
    name = "smoke_pyroki.json" if jax.default_backend() == "cpu" else f"smoke_pyroki_{jax.default_backend()}.json"
    json.dump(rep, open(OUT / name, "w"), indent=1)
    print(json.dumps({k: v for k, v in rep.items() if k != "joints"}, indent=1, default=str)[:3000])


if __name__ == "__main__":
    main()
