"""Benchmark v2, amendment A4 pass (a): parallel classification of VAMP runs.

Every (query, seed) run executes in its own forked worker process, killed at KILL_S = 2T = 60 s wall (a kill is a
failure). VAMP's own 1M-iteration default still applies inside the run. Records: success, simplified path, and the
contended plan time (informative only; pass 2a re-times every success on a quiet CPU and that time is what counts).
Checkpointed to results/v2/raw/vamp_pass1_<set>.jsonl; re-running resumes and skips finished runs.
Usage: python bench/v2_vamp_pass1.py --freeze H --workers N [--sets heldout mbm mpinets]
"""
import argparse, json, multiprocessing as mp, os, pickle, signal, sys, time
from pathlib import Path

import numpy as np
import vamp

sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, seed_from

SETS = {"heldout": ("ur5erail", "problems/v2_heldout.pkl", "v2_heldout"),
        "mbm": ("panda", "problems/v2_mbm_panda.pkl", "v2_mbm"),
        "mpinets": ("panda", "problems/v2_mpinets_panda.pkl", "v2_mpinets")}
KILL_S = 60.0


def run_one(vm, p, seed, conn):
    vmod, planner, ps, ss = vamp.configure_robot_and_planner_with_kwargs(vm, "rrtc")
    sampler = vmod.halton(); sampler.reset(); sampler.skip(seed % 100000)
    t0 = time.perf_counter_ns(); env = vamp.problem_dict_to_vamp(p)
    res = planner(p["start"], p["goals"], env, ps, sampler)
    out = {"plan_success": bool(res.solved)}
    if res.solved:
        simple = vmod.simplify(res.path, env, ss, sampler)
        out["path"] = np.array(simple.path.numpy(), float).round(6).tolist()
    out["contended_t_plan_ms"] = (time.perf_counter_ns() - t0) / 1e6
    conn.send(out); conn.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--freeze", required=True); ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--sets", nargs="+", default=["heldout", "mbm", "mpinets"]); ap.add_argument("--seeds", type=int, default=10); ap.add_argument("--out-dir", default=None); ap.add_argument("--limit", type=int, default=0)
    a = ap.parse_args()
    ctx = mp.get_context("fork")
    seeds = [seed_from(a.freeze, f"seed{k}") for k in range(a.seeds)]
    for s in a.sets:
        vm, pk, key = SETS[s]
        probs = pickle.load(open(ROOT / pk, "rb"))["problems"][key]
        ck = Path(a.out_dir or ROOT / "results/v2/raw") / f"vamp_pass1_{s}.jsonl"; ck.parent.mkdir(parents=True, exist_ok=True)
        done = {(r["q"], r["seed_k"]) for r in map(json.loads, open(ck))} if ck.exists() else set()
        todo = [(q, k) for q in range(a.limit or len(probs)) for k in range(a.seeds) if (q, k) not in done]
        print(f"{s}: {len(done)} done, {len(todo)} to run, workers {a.workers}", flush=True)
        running = {}
        n_ok = n_kill = 0
        while todo or running:
            while todo and len(running) < a.workers:
                q, k = todo.pop(0)
                rd, wr = ctx.Pipe(duplex=False)
                pr = ctx.Process(target=run_one, args=(vm, probs[q], seeds[k], wr)); pr.start(); wr.close()
                running[pr.pid] = (pr, rd, q, k, time.time())
            time.sleep(0.005)
            for pid, (pr, rd, q, k, t0) in list(running.items()):
                rec = None
                if rd.poll():
                    rec = rd.recv(); pr.join()
                elif time.time() - t0 > KILL_S:
                    os.kill(pid, signal.SIGKILL); pr.join(); rec = {"plan_success": False, "killed_at_s": KILL_S}; n_kill += 1
                elif not pr.is_alive():
                    pr.join(); rec = {"plan_success": False, "worker_exit_code": pr.exitcode}
                if rec is None:
                    continue
                rec.update(q=q, index=probs[q]["index"], family=probs[q]["problem"], seed_k=k, seed=seeds[k], set=s,
                           bins={x: probs[q]["bins"][x] for x in ("reach", "clearance", "passage", "limits")})
                n_ok += rec["plan_success"]
                with open(ck, "a") as f:
                    f.write(json.dumps(rec) + "\n")
                del running[pid]
                n_done = len(done) + 1; done.add((q, k))
                if n_done % 100 == 0:
                    print(f"{s}: {n_done} runs, session solved {n_ok}, killed {n_kill}", flush=True)
        print(f"{s}: finished; this session solved {n_ok}, killed {n_kill}", flush=True)


if __name__ == "__main__":
    main()
