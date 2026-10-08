"""Benchmark v2: time-parameterization of geometric joint-space paths under shared joint limits.

P1 (primary)   Ruckig, phase-synchronized, segment by segment through the waypoints, rest-to-rest at each waypoint.
               Phase synchronization keeps every segment on the straight joint-space line between waypoints, so the
               timed trajectory is exactly the polyline the planner validated. Limits: velocity, acceleration, jerk.
P2 (secondary) TOPP-RA (time-optimal under velocity + acceleration limits) on a cubic spline through the waypoints.
               The spline leaves the polyline, so P2 trajectories must pass the 1 kHz validation to count;
               jerk is not limited by TOPP-RA and is reported.
Both return a callable sampler q(t), qd(t), qdd(t) and a duration. Timing of the parameterization itself is measured
by the caller.
"""
import numpy as np
from ruckig import InputParameter, Ruckig, Synchronization, Trajectory, Result
import toppra as ta
import toppra.algorithm as algo
import toppra.constraint as constraint

ta.setup_logging("WARNING")


class Timed:
    def __init__(self, duration, sample):
        self.duration, self._sample = duration, sample

    def sample(self, t):
        """t: (n,) array -> q, qd, qdd each (n, dof)."""
        return self._sample(np.asarray(t, float))


def ruckig_p1(path, vmax, amax, jmax):
    q = np.asarray(path, float)
    keep = [0] + [k for k in range(1, len(q)) if np.abs(q[k] - q[k - 1]).max() > 1e-9]
    q = q[keep]
    dof = q.shape[1]
    segs, t0s, t = [], [], 0.0
    otg = Ruckig(dof)
    for k in range(len(q) - 1):
        inp = InputParameter(dof)
        inp.current_position, inp.target_position = q[k].tolist(), q[k + 1].tolist()
        inp.current_velocity = inp.current_acceleration = [0.0] * dof
        inp.target_velocity = inp.target_acceleration = [0.0] * dof
        inp.max_velocity, inp.max_acceleration, inp.max_jerk = list(vmax), list(amax), list(jmax)
        inp.synchronization = Synchronization.Phase
        tr = Trajectory(dof)
        res = otg.calculate(inp, tr)
        if res not in (Result.Working, Result.Finished):
            raise RuntimeError(f"ruckig failed on segment {k}: {res}")
        segs.append(tr); t0s.append(t); t += tr.duration
    t0s = np.array(t0s)

    def sample(ts):
        out = np.zeros((3, len(ts), dof))
        for i, tt in enumerate(np.clip(ts, 0, t)):
            k = max(0, min(len(segs) - 1, int(np.searchsorted(t0s, tt, side="right") - 1)))
            p, v, a = segs[k].at_time(min(tt - t0s[k], segs[k].duration))
            out[0, i], out[1, i], out[2, i] = p, v, a
        return out[0], out[1], out[2]
    if len(segs) == 0:
        return Timed(0.0, lambda ts: (np.repeat(q[:1], len(ts), 0), np.zeros((len(ts), dof)), np.zeros((len(ts), dof))))
    return Timed(t, sample)


def toppra_p2(path, vmax, amax):
    q = np.asarray(path, float)
    keep = [0] + [k for k in range(1, len(q)) if np.abs(q[k] - q[k - 1]).max() > 1e-9]
    q = q[keep]
    if len(q) < 2:
        return None
    s = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(q, axis=0), axis=1))]); s /= s[-1]
    pth = ta.SplineInterpolator(s, q)
    vlim = np.stack([-np.asarray(vmax), np.asarray(vmax)], 1); alim = np.stack([-np.asarray(amax), np.asarray(amax)], 1)
    inst = algo.TOPPRA([constraint.JointVelocityConstraint(vlim), constraint.JointAccelerationConstraint(alim)], pth,
                       parametrizer="ParametrizeConstAccel")
    jt = inst.compute_trajectory()
    if jt is None:
        return None
    return Timed(float(jt.duration), lambda ts: (jt(np.clip(ts, 0, jt.duration)), jt(np.clip(ts, 0, jt.duration), 1),
                                                 jt(np.clip(ts, 0, jt.duration), 2)))
