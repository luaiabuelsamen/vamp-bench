"""Check every successful cuRobo trajectory with VAMP's collision checker (same spheres, same obstacles).

A trajectory passes if every consecutive pair of waypoints is a valid motion under vamp.<robot>.validate_motion
(VAMP checks the straight joint-space segment at its resolution). This verifies cuRobo's solutions are
collision-free under the shared model, so success rates are comparable.
usage: .venv/bin/python bench/crossval.py results/raw/curobo_X.json VAMP_MODULE [problems.pkl]
"""
import json, pickle, sys
from pathlib import Path
import numpy as np
import vamp

ROOT = Path(__file__).resolve().parents[1]
res = json.load(open(sys.argv[1])); vmod = getattr(vamp, sys.argv[2])
pk = sys.argv[3] if len(sys.argv) > 3 else ROOT / f"third_party/vamp/resources/{res['robot']}/problems.pkl"
probs = pickle.load(open(pk, "rb"))["problems"]
n = bad = 0
for r in res["records"]:
    if not r.get("success"):
        continue
    env = vamp.problem_dict_to_vamp(probs[r["scene"]][r["index"]])
    q = np.asarray(r["path"], dtype=np.float32)
    ok = all(vmod.validate_motion(q[k].tolist(), q[k + 1].tolist(), env) for k in range(len(q) - 1))
    r["vamp_validated"] = bool(ok); n += 1; bad += (not ok)
json.dump(res, open(sys.argv[1], "w"))
print(sys.argv[1], f"cuRobo successes checked by VAMP: {n}, in collision under shared model: {bad}")
