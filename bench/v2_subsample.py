"""Benchmark v2: cuRoboV2 stratified subsample (pre-registered): 100 queries per set, round-robin over occupied
(reach, clearance, passage, limits) bin combinations, queries within a combination in an order shuffled with
seed_from(H, "select_v2"). Geometry bins only. Writes results/v2/v2_subsample.json {set: [query indices]}."""
import argparse, json, pickle, sys
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, seed_from

ap = argparse.ArgumentParser(); ap.add_argument("--freeze", required=True); a = ap.parse_args()
rng = np.random.default_rng(seed_from(a.freeze, "select_v2"))
out = {}
for s, pk, key in (("heldout", "problems/v2_heldout.pkl", "v2_heldout"), ("mbm", "problems/v2_mbm_panda.pkl", "v2_mbm"),
                   ("mpinets", "problems/v2_mpinets_panda.pkl", "v2_mpinets")):
    ps = pickle.load(open(ROOT / pk, "rb"))["problems"][key]
    combos = {}
    for i, p in enumerate(ps):
        combos.setdefault(tuple(p["bins"][x] for x in ("reach", "clearance", "passage", "limits")), []).append(i)
    order = {c: list(rng.permutation(v)) for c, v in sorted(combos.items())}
    sel = []
    while len(sel) < 100 and any(order.values()):
        for c in sorted(order):
            if order[c] and len(sel) < 100:
                sel.append(int(order[c].pop(0)))
    out[s] = sorted(sel)
    print(s, len(sel), "from", len(combos), "bin combinations")
(ROOT / "results/v2").mkdir(parents=True, exist_ok=True)
json.dump(out, open(ROOT / "results/v2/v2_subsample.json", "w"), indent=1)
