"""Pre-freeze cost measurement of the v2 time-parameterization (P1 Ruckig, P2 TOPP-RA) on 200 v1 VAMP Panda MBM paths.
Not held-out data; measures compute cost only. Writes results/v2/timing_cost.json."""
import sys, json, time, numpy as np
sys.path.insert(0, 'bench')
from v2_timing import ruckig_p1, toppra_p2
V = json.load(open('results/raw/vamp_mbm_panda.json'))['records']
paths = [r['path'] for r in V if r.get('success')][:200]
vmax = [2.175]*4 + [2.61]*3; amax = [15, 7.5, 10, 12.5, 15, 20, 20]; jmax = [7500, 3750, 5000, 6250, 7500, 10000, 10000]
t1, t2, d1, d2, fail2 = [], [], [], [], 0
for p in paths:
    a = time.perf_counter(); T1 = ruckig_p1(p, vmax, amax, jmax); b = time.perf_counter(); T2 = toppra_p2(p, vmax, amax); c = time.perf_counter()
    t1.append((b - a) * 1e3); t2.append((c - b) * 1e3); d1.append(T1.duration)
    if T2 is None: fail2 += 1
    else: d2.append(T2.duration)
# 1 kHz sampling cost
a = time.perf_counter(); q, qd, qdd = T1.sample(np.arange(0, T1.duration, 1e-3)); b = time.perf_counter()
pct = lambda x: [round(float(np.percentile(x, k)), 3) for k in (50, 95)]
print('waypoints median', np.median([len(p) for p in paths]))
print('P1 ruckig ms [p50,p95]', pct(t1), 'duration s', pct(d1))
print('P2 toppra ms [p50,p95]', pct(t2), 'duration s', pct(d2), 'failures', fail2)
print('P1 1kHz sampling of one traj:', round((b - a) * 1e3, 1), 'ms for', len(q), 'samples')
import platform
json.dump({'host': platform.node(), 'paths': len(paths), 'p1_ruckig_ms_p50_p95': pct(t1), 'p2_toppra_ms_p50_p95': pct(t2), 'p1_duration_s_p50_p95': pct(d1), 'p2_duration_s_p50_p95': pct(d2), 'p2_failures': fail2, 'p1_1khz_sampling_ms_per_traj': round((b - a) * 1e3, 1), 'samples': len(q)}, open('results/v2/timing_cost.json', 'w'), indent=1)
