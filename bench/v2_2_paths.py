"""Benchmark v2.2: programmed TCP paths and time laws (docs/V2_2_MOTION_TYPES_PREREGISTRATION.md).

A path is a list of primitives in arc length:
  ("line", p0, p1)                                   straight segment
  ("arc", c, u, w, rho, phi0, sweep)                 circle centre c, radius rho, in the plane spanned by unit u, w:
                                                     p(phi) = c + rho (cos phi u + sin phi w), phi from phi0 over sweep
Each primitive carries a label: "line", "blend" (the fillet at a via point of a W program) or "arc" (MoveC).
Orientation is constant along every v2.2 path, so a path is position only.
The time law is 1-D Ruckig on arc length with the program's (v, a, j); it starts and ends at rest.
"""
import math

import numpy as np
from ruckig import InputParameter, Result, Ruckig, Trajectory

TCP_J = 10.0  # m/s^3, the programmed TCP jerk for every program


def _unit(x):
    return x / np.linalg.norm(x)


def blended_polyline(points, r_max=0.05, r_frac=0.4):
    """W program: straight legs through the via points, with a circular fillet at each via point that starts and ends
    at distance r from it along the adjacent legs, r = min(r_max, r_frac * shorter adjacent leg). The fillet is tangent
    to both legs; its radius is r * tan(alpha / 2), alpha the interior angle at the via point."""
    P = [np.asarray(p, float) for p in points]
    legs = [np.linalg.norm(P[i + 1] - P[i]) for i in range(len(P) - 1)]
    prims, cur, radii = [], P[0], []
    for i in range(1, len(P) - 1):
        u1, u2 = _unit(P[i] - P[i - 1]), _unit(P[i + 1] - P[i])
        r = min(r_max, r_frac * min(legs[i - 1], legs[i])); radii.append(r)
        t1, t2 = P[i] - r * u1, P[i] + r * u2
        turn = math.acos(np.clip(u1 @ u2, -1, 1)); alpha = math.pi - turn
        rho = r * math.tan(alpha / 2)
        c = P[i] + _unit(u2 - u1) * (r / math.cos(alpha / 2))
        u = _unit(t1 - c); w = _unit(u1 - (u1 @ u) * u)  # w: direction of travel at t1, orthogonal to u
        prims.append(("line", "line", cur, t1))
        prims.append(("arc", "blend", c, u, w, rho, 0.0, turn))
        cur = t2
    prims.append(("line", "line", cur, P[-1]))
    return prims, radii


def circle_arc(c, u, w, rho, sweep):
    """C program: one arc from c + rho u, turning toward w, over the given sweep (rad)."""
    return [("arc", "arc", np.asarray(c, float), _unit(np.asarray(u, float)), _unit(np.asarray(w, float)), float(rho), 0.0, float(sweep))]


def line(p0, p1):
    return [("line", "line", np.asarray(p0, float), np.asarray(p1, float))]


def prim_length(pr):
    return float(np.linalg.norm(pr[3] - pr[2])) if pr[0] == "line" else pr[5] * pr[7]


def prim_point(pr, d):
    """Point at arc length d along primitive pr."""
    if pr[0] == "line":
        L = prim_length(pr)
        return pr[2] + (pr[3] - pr[2]) * (d / L if L > 0 else 0.0)
    _, _, c, u, w, rho, phi0, _ = pr
    phi = phi0 + d / rho
    return c + rho * (math.cos(phi) * u + math.sin(phi) * w)


def length(prims):
    return float(sum(prim_length(p) for p in prims))


def sample(prims, step):
    """Dense samples of the path: positions (n, 3), arc length s (n,), label index per sample, and the label list.
    Primitive boundaries are always included."""
    P, S, lab = [], [], []
    s0 = 0.0
    for k, pr in enumerate(prims):
        L = prim_length(pr); n = max(1, int(math.ceil(L / step)))
        ds = np.linspace(0, L, n + 1)
        if k > 0:
            ds = ds[1:]
        P += [prim_point(pr, d) for d in ds]; S += list(s0 + ds); lab += [k] * len(ds)
        s0 += L
    return np.array(P), np.array(S), np.array(lab)


def zones(prims):
    """Arc-length intervals [(s_start, s_end, label)] per primitive."""
    out, s0 = [], 0.0
    for pr in prims:
        L = prim_length(pr); out.append((s0, s0 + L, pr[1])); s0 += L
    return out


def time_law(L, v, a, j=TCP_J):
    """Programmed 1-D profile on arc length (Ruckig, rest to rest). Returns the Ruckig trajectory or None."""
    inp = InputParameter(1); inp.current_position = [0.0]; inp.target_position = [float(L)]
    inp.current_velocity = inp.current_acceleration = inp.target_velocity = inp.target_acceleration = [0.0]
    inp.max_velocity, inp.max_acceleration, inp.max_jerk = [v], [a], [j]
    tr = Trajectory(1)
    return tr if Ruckig(1).calculate(inp, tr) in (Result.Working, Result.Finished) else None


def profile(L, v, a, dt=1e-3, j=TCP_J):
    """s, sdot, sddot at dt, the duration, and the programmed phases in arc length: the ramp-up end s_r1 and the
    ramp-down start s_r2 (where the programmed speed first reaches / last leaves v; s_r1 = s_r2 if v is never reached)."""
    tr = time_law(L, v, a, j)
    if tr is None:
        return None
    ts = np.arange(0, tr.duration + 1e-12, dt)
    st = np.array([tr.at_time(t) for t in ts])
    s, sd, sdd = st[:, 0, 0], st[:, 1, 0], st[:, 2, 0]
    at_v = np.flatnonzero(sd >= v * (1 - 1e-6))
    s_r1, s_r2 = (float(s[at_v[0]]), float(s[at_v[-1]])) if len(at_v) else (float(L / 2), float(L / 2))
    t_r1, t_r2 = (float(ts[at_v[0]]), float(ts[at_v[-1]])) if len(at_v) else (tr.duration / 2, tr.duration / 2)
    return {"t": ts, "s": s, "sd": sd, "sdd": sdd, "duration": float(tr.duration), "s_r1": s_r1, "s_r2": s_r2,
            "t_r1": t_r1, "t_r2": t_r2, "t_ramp_down": float(tr.duration - t_r2)}
