"""Benchmark v2.1 (PyRoki, provisional) summary from the stored runs; no new runs.

Inputs (results/v2_1/raw/, GPU runs of 2026-10-13/14, docs/BENCHMARK_V2.md section 7):
  pyroki_movel_gpu.jsonl          MoveL as pre-registered (end reached through the per-knot pose cost only)
  pyroki_movel_gpu_recipe.jsonl   MoveL with PyRoki's own endpoint recipe (amendment 7g, post-hoc)
  pyroki_ik_gpu.jsonl             IK (a) default init, (b) 64 random inits, (c) collision-aware, 64 random inits
  curobo1_ik_heldout_a6.jsonl     cuRobo v1 IKSolver, same poses, default and high-precision configs
  pyroki_trajopt_s{1,2}_gpu.jsonl free-space trajopt from (S1) the straight line, (S2) cuRobo v1's seeds
  curobo1_seeds_heldout_a6.jsonl  cuRobo v1 at the seed-dump random seed (S2 source)
Writes results/v2_1/summary.json.
"""
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / "results/v2_1/raw"
load = lambda f: [json.loads(l) for l in open(RAW / f)]
med = lambda x: float(np.median(x)) if len(x) else None


def movel(f):
    R = load(f)
    r1 = [bool(r["R1"]["success"]) for r in R if r.get("R1")]
    r2 = [bool(r["R2"] and r["R2"]["success"] and r["start_ok"] and r["end_ok"]) for r in R]
    start_off = [r for r in R if not r["start_ok"]]
    only_start = [r for r in start_off if r["end_ok"] and r["R2"] and r["R2"]["valid"]]
    return {"segments": len(R), "R1_success": sum(r1), "R2_success": sum(r2), "end_ok": sum(r["end_ok"] for r in R),
            "start_off": len(start_off), "start_off_only_reason": len(only_start),
            "end_pos_err_mm_median": med([r["end_pos_err_mm"] for r in R]),
            "R2_gate_fail": sum(1 for r in R if not (r["R2"] and r["R2"]["valid"])),
            "t_solve_s_median": med([r["t_solve_s"] for r in R])}


def ik(f, variants):
    R = load(f)
    return {"poses": len(R), **{v: {"success": sum(bool(r[v]["success"]) for r in R),
                                    "t_s_median": med([r[v]["t_s"] for r in R])} for v in variants}}


def trajopt(f):
    R = load(f)
    ok1 = [r for r in R if r["success_R1"]]; ok2 = [r for r in R if r["success_R2"]]
    return {"queries": len(R), "R1_success": len(ok1), "R2_success": len(ok2),
            "t_ready_s_median_R2_success": med([r["t_ready_total_s"] for r in ok2])}


def main():
    pre, rec = movel("pyroki_movel_gpu.jsonl"), movel("pyroki_movel_gpu_recipe.jsonl")
    seeds = load("curobo1_seeds_heldout_a6.jsonl")
    diag = [json.loads(l) for l in open(ROOT / "results/v2_1/ik_margin_diag_m0.0.jsonl")]
    fails = [d for d in diag if not d["run_success"]]; ctrl = [d for d in diag if d["run_success"]]
    out = {"status": "provisional (v2.1 write-up pending)",
           "movel_preregistered": pre, "movel_recipe_7g": rec,
           "ik_pyroki": ik("pyroki_ik_gpu.jsonl", ("default", "rand64", "coll_rand64")),
           "ik_curobo1": ik("curobo1_ik_heldout_a6.jsonl", ("default", "high_precision")),
           "ik_margin_sensitivity_m0": {"failed_at_5mm": len(fails), "succeed_at_0": sum(d["success"] for d in fails),
                                        "controls": len(ctrl), "controls_collide_at_0": sum(not d["collision_free"] for d in ctrl)},
           "trajopt_s1": trajopt("pyroki_trajopt_s1_gpu.jsonl"), "trajopt_s2": trajopt("pyroki_trajopt_s2_gpu.jsonl"),
           "curobo1_at_seed_dump": {"queries": len(seeds), "success": sum(bool(r["plan_success"]) for r in seeds)},
           "margin_choice": json.load(open(ROOT / "results/v2_1/dev_margin.json"))}
    json.dump(out, open(ROOT / "results/v2_1/summary.json", "w"), indent=1)
    print(json.dumps({k: v for k, v in out.items() if k != "margin_choice"}, indent=1))


if __name__ == "__main__":
    main()
