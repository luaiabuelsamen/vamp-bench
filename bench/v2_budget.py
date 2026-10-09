"""Amendment A2: choose the uniform per-query wall-clock budget T so the cuRoboV2 stratified subsample
(300 queries x 3 seeds = 900 runs) fits ~5 GPU-hours under pessimistic assumptions, from measured v1-slot data.
Writes results/v2/budget_choice.json."""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
def load(n):
    d = json.load(open(ROOT / f"results/raw/{n}.json"))["records"]; v = [r for r in d if r["valid"]]
    ok = [(r["world_update_us"] + r["plan_wall_us"]) / 1e6 for r in v if r["success"]]
    return {"fail_rate": 1 - len(ok) / len(v), "ok_mean_s": float(np.mean(ok)), "ok_p95_s": float(np.percentile(ok, 95)), "ok_max_s": float(max(ok))}
panda = load("curobo2_mbm_panda"); rail = load("curobo2_gantry_pillars_ur5e_rail")
runs_panda, runs_held, warm = 600, 300, 9 * 60.0          # 200 Panda + 100 held-out queries, x3 seeds; 9 planner builds
held_fail_pess = 0.70                                     # held-out: VAMP fails ~43%; assume cuRobo up to 70%
panda_fail_pess = min(1.0, 2 * panda["fail_rate"])         # twice the observed V2 Panda failure rate
def gpu_s(T, pf, hf):
    return (runs_panda * ((1 - pf) * panda["ok_mean_s"] + pf * T) + runs_held * ((1 - hf) * rail["ok_mean_s"] + hf * T) + warm)
grid = {T: {"expected_h": gpu_s(T, panda["fail_rate"], 0.43) / 3600, "pessimistic_h": gpu_s(T, panda_fail_pess, held_fail_pess) / 3600,
            "all_timeout_h": (900 * T + warm) / 3600} for T in (15, 20, 30, 45, 60)}
T = 30
out = {"T_s": T, "inputs": {"v2_panda_mbm": panda, "v2_rail_pillars": rail, "pessimistic_fail": {"panda": panda_fail_pess, "heldout": held_fail_pess}},
       "gpu_hours_by_T": grid,
       "rationale": "largest round T whose pessimistic estimate is <= ~5 GPU-h (T=30 -> %.1f h; T=45 -> %.1f h); "
                    "T exceeds the max observed V2 success time (%.1f s), so the budget cuts failures, not successes"
                    % (grid[30]["pessimistic_h"], grid[45]["pessimistic_h"], panda["ok_max_s"])}
(ROOT / "results/v2").mkdir(exist_ok=True); json.dump(out, open(ROOT / "results/v2/budget_choice.json", "w"), indent=1)
print(json.dumps(out, indent=1))
