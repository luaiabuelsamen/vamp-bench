"""Benchmark v2: 1 kHz validation and motion-quality metrics, identical for every planner.

Input: joint positions sampled at exactly 1 kHz (q: (n, dof)), optionally the sampler's velocities/accelerations.
Validity (all must hold for a run to count as solved):
  - collision-free at every sample under the shared model (VAMP validate, joint limits with 1e-5 rad tolerance)
  - |qd| <= 1.01 v_max, |qdd| <= 1.01 a_max, |jerk| <= 1.01 j_max at every sample (jerk not gated for P2,
    whose TOPP-RA constant-acceleration output has acceleration steps by construction; reported instead)
    (derivatives: the sampler's own qd/qdd when given, else finite differences; jerk always by finite
    differences of qdd at 1 kHz)
Quality (reported for valid runs):
  joint L2 length, TCP length (100 Hz), RMS and peak jerk (normalized by j_max), integrated squared jerk,
  min clearance and time within 1 cm (100 Hz, shared sphere model), min joint-limit margin,
  max TCP tilt from vertical (angle between tool z and world -z; meaningful for the held-out pick-place set).
"""
import numpy as np
import vamp

from penetration_depth import obstacle_sdf
from v2_common import clearance, limits, tcp

DT = 1e-3


def validate(vm, p, q, qd=None, qdd=None, check_jerk=True):
    vmod = getattr(vamp, vm); env = vamp.problem_dict_to_vamp(p)
    lo, hi = np.array(vmod.lower_bounds()), np.array(vmod.upper_bounds())
    vmax, amax, jmax = limits(vm)
    q = np.asarray(q, float)
    if qd is None:
        qd = np.gradient(q, DT, axis=0)
    if qdd is None:
        qdd = np.gradient(qd, DT, axis=0)
    jerk = np.gradient(np.asarray(qdd, float), DT, axis=0)
    bound_viol = float(np.max(np.maximum(lo - q, 0) + np.maximum(q - hi, 0))) if len(q) else 0.0
    qc = np.clip(q, lo, hi).astype(np.float32)
    coll = [k for k in range(len(qc)) if not vmod.validate(qc[k].tolist(), env)]
    rv = float(np.max(np.abs(qd) / vmax)) if len(q) else 0.0
    ra = float(np.max(np.abs(qdd) / amax)) if len(q) else 0.0
    rj = float(np.max(np.abs(jerk) / jmax)) if len(q) > 2 else 0.0
    ok = (not coll) and bound_viol <= 1e-5 and rv <= 1.01 and ra <= 1.01 and (rj <= 1.01 or not check_jerk)
    out = {"valid": bool(ok), "jerk_checked": bool(check_jerk), "n_samples": int(len(q)), "colliding_samples": len(coll), "bound_violation_rad": bound_viol,
           "max_vel_ratio": rv, "max_acc_ratio": ra, "max_jerk_ratio": rj}
    if not ok:
        return out
    sdf = obstacle_sdf(p)
    q100 = q[::10]
    cl = np.array([clearance(vm, x, sdf) for x in q100])
    T = [tcp(vm, x) for x in q100]
    P = np.array([t[:3, 3] for t in T]); Z = np.array([t[:3, 2] for t in T])
    tilt = np.degrees(np.arccos(np.clip(-Z[:, 2], -1, 1)))
    jn = jerk / jmax
    out.update(joint_length_rad=float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum()),
               tcp_length_m=float(np.linalg.norm(np.diff(P, axis=0), axis=1).sum()),
               rms_jerk_norm=float(np.sqrt(np.mean(jn ** 2))), peak_jerk_norm=float(np.abs(jn).max()),
               integrated_sq_jerk=float(np.sum(jerk ** 2) * DT),
               min_clearance_m=float(cl.min()), time_within_1cm_s=float((cl < 0.01).sum() * 0.01),
               min_limit_margin_rad=float(np.min(np.minimum(q - lo, hi - q))), max_tcp_tilt_deg=float(tilt.max()))
    return out
