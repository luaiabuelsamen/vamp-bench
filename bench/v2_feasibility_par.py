"""Amendment A3 reference runs, parallel and checkpointed (Orin only; Luai declined the Mac 2026-10-09).

Queries: every query of a set that no timed run has solved yet (VAMP pass (a) checkpoint + cuRobo v1 raw results;
cuRoboV2 successes certify feasibility on their own when they arrive, so they need no reference run).
Reference (untimed, deterministic): VAMP rrtc with max_iterations = max_samples = 6e6, then VAMP PRM with the same
caps, seed0. Caps are iteration counts, not wall time, so CPU contention changes duration but never the verdict.
Checkpoint: results/v2/raw/feasibility_<set>.jsonl (resume skips finished queries).
Memory gate (the board is shared; the flagship must never be OOM-killed): at most --workers (2) references run while
MemAvailable (/proc/meminfo) has not stayed above --mem-gate-gb (4 GB) for --gate-hold-min (10) minutes of running
time (the hold restarts after the runner has been stopped, e.g. SIGSTOP during a GPU slot); up to
--max-workers (6) once it has. Extra workers (beyond --workers) are killed and requeued, newest first, when MemAvailable
falls under --shed-gb (1.5 GB). Workers beyond --workers are never started at or below the
gate; no worker at all is started while MemAvailable < floor + 0.5 GB; every worker sets oom_score_adj = 1000 so the kernel picks it before anything else. If MemAvailable falls
under --mem-floor-gb (0.6 GB), the largest worker is killed and its query requeued (relaunched only above the gate;
the reference is deterministic, so a rerun gives the same verdict).
Standing rule while the flagship runs a SmolVLA evaluation (a python train_local.py process with --skip-train and
"smolvla" in its arguments): at most 1 worker, and no new reference starts unless MemAvailable > --eval-gate-gb (2.5).
Running references are not killed by this rule (only by the floor); the cap applies as they finish.
Usage: python bench/v2_feasibility_par.py --freeze H [--workers 2 --max-workers 6 --mem-gate-gb 4] [--sets heldout mbm mpinets]
"""
import argparse, json, multiprocessing as mp, os, pickle, re, signal, sys, time
from pathlib import Path

import vamp

sys.path.insert(0, str(Path(__file__).parent))
from v2_common import ROOT, seed_from

SETS = {"heldout": ("ur5erail", "problems/v2_heldout.pkl", "v2_heldout"),
        "mbm": ("panda", "problems/v2_mbm_panda.pkl", "v2_mbm"),
        "mpinets": ("panda", "problems/v2_mpinets_panda.pkl", "v2_mpinets")}
CAP = 6_000_000


def reference(args):
    s, q, vm, p, seed = args
    env = vamp.problem_dict_to_vamp(p); rec = {"set": s, "q": q}
    for name in ("rrtc", "prm"):
        vmod, planner, ps, ss = vamp.configure_robot_and_planner_with_kwargs(vm, name)
        ps.max_iterations = CAP; ps.max_samples = CAP
        sampler = vmod.halton(); sampler.reset(); sampler.skip(seed % 100000)
        t = time.perf_counter(); r = planner(p["start"], p["goals"], env, ps, sampler)
        rec[name] = {"solved": bool(r.solved), "s": time.perf_counter() - t}
        if r.solved:
            break
    rec["path_exists"] = any(rec.get(n, {}).get("solved") for n in ("rrtc", "prm"))
    return rec


def mem_available_gb():
    for line in open("/proc/meminfo"):
        if line.startswith("MemAvailable:"):
            return int(line.split()[1]) / 2**20
    return 0.0


def rss_gb(pid):
    try:
        return int(open(f"/proc/{pid}/statm").read().split()[1]) * os.sysconf("SC_PAGE_SIZE") / 2**30
    except OSError:
        return 0.0


EVAL_RE = re.compile(r"^\S*python[0-9.]* \S*train_local\.py .*--skip-train")


def smolvla_eval_running():
    for d in os.listdir("/proc"):
        if d.isdigit():
            try:
                args = open(f"/proc/{d}/cmdline", "rb").read().replace(b"\0", b" ").decode(errors="replace")
            except OSError:
                continue
            if EVAL_RE.search(args) and "smolvla" in args.lower():
                return True
    return False


def child(job, conn):
    open("/proc/self/oom_score_adj", "w").write("1000")
    conn.send(reference(job)); conn.close()


def run_gated(jobs, ck, a):
    """Run jobs with a memory-dependent worker count; append each result to the checkpoint as it finishes."""
    ctx = mp.get_context("fork"); queue = list(jobs); deferred = []; running = {}  # pid -> (proc, conn, job)
    last_log = 0; above_since = None; last_ev = None; last_tick = time.time()
    while queue or deferred or running:
        avail = mem_available_gb()
        if time.time() - last_tick > 60:  # the runner was stopped (SIGSTOP during a GPU slot): the hold starts over
            above_since = None
        last_tick = time.time()
        above_since = (above_since or time.time()) if avail > a.mem_gate_gb else None
        if avail < a.shed_gb and len(running) > a.workers:
            pid = max(running); proc, conn, job = running.pop(pid)
            print(f"{time.strftime('%H:%M:%S')} MemAvailable {avail:.2f} GB < {a.shed_gb} GB: shed extra worker for q {job[1]}, requeued", flush=True)
            proc.kill(); proc.join(); queue.insert(0, job); continue
        if avail < a.mem_floor_gb and running:
            pid = max(running, key=rss_gb); proc, conn, job = running.pop(pid)
            print(f"{time.strftime('%H:%M:%S')} MemAvailable {avail:.2f} GB < floor: killed worker for q {job[1]} "
                  f"(rss {rss_gb(pid):.2f} GB), requeued above the gate", flush=True)
            proc.kill(); proc.join(); deferred.append(job); continue
        for pid in list(running):
            proc, conn, job = running[pid]
            if conn.poll():
                rec = conn.recv(); proc.join(); running.pop(pid)
                with open(ck, "a") as f:
                    f.write(json.dumps(rec) + "\n")
                print(rec, flush=True)
            elif not proc.is_alive():
                proc.join(); running.pop(pid); deferred.append(job)
                print(f"{time.strftime('%H:%M:%S')} worker for q {job[1]} died (exit {proc.exitcode}); requeued above the gate", flush=True)
        above = above_since is not None and time.time() - above_since >= 60 * a.gate_hold_min
        ev = smolvla_eval_running()
        cap = 1 if ev else (a.max_workers if above else a.workers)
        if ev != last_ev:
            print(f"{time.strftime('%H:%M:%S')} flagship SmolVLA eval {'running: cap 1 worker, new start only above ' + str(a.eval_gate_gb) + ' GB' if ev else 'not running'}", flush=True)
            last_ev = ev
        while len(running) < cap and (queue or (above and deferred)):
            m = mem_available_gb()
            if ev and m <= a.eval_gate_gb:
                break
            if m < a.mem_floor_gb + 0.5 or (len(running) >= a.workers and (m <= a.mem_gate_gb or not above)):
                break
            job = deferred.pop(0) if (above and deferred) else queue.pop(0)
            r, w = ctx.Pipe(duplex=False); proc = ctx.Process(target=child, args=(job, w)); proc.start(); w.close()
            running[proc.pid] = (proc, r, job)
            time.sleep(5 if len(running) > a.workers else 0)  # let a new worker's memory show before adding another
        if not running and deferred and not queue:
            if time.time() - last_log > 600:
                print(f"{time.strftime('%H:%M:%S')} {len(deferred)} requeued queries wait for MemAvailable > {a.mem_gate_gb} GB "
                      f"(now {avail:.2f})", flush=True); last_log = time.time()
        time.sleep(10)


def solved_by_timed_runs(s):
    ok = set()
    f = ROOT / f"results/v2/raw/vamp_pass1_{s}.jsonl"
    if f.exists():
        ok |= {r["q"] for r in map(json.loads, open(f)) if r["plan_success"]}
    for v in ("curobo1", "curobo2"):
        f = ROOT / f"results/v2/raw/{v}_{s}.json"
        if f.exists():
            ok |= {r["q"] for r in json.load(open(f))["records"] if r.get("solved_any_time")}
    return ok


def main():
    ap = argparse.ArgumentParser(); ap.add_argument("--freeze", required=True); ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--max-workers", type=int, default=6); ap.add_argument("--mem-gate-gb", type=float, default=4.0)
    ap.add_argument("--mem-floor-gb", type=float, default=0.6); ap.add_argument("--shed-gb", type=float, default=1.5)
    ap.add_argument("--gate-hold-min", type=float, default=10.0); ap.add_argument("--eval-gate-gb", type=float, default=2.5)
    ap.add_argument("--sets", nargs="+", default=["heldout", "mbm", "mpinets"]); a = ap.parse_args()
    seed = seed_from(a.freeze, "seed0")
    for s in a.sets:
        vm, pk, key = SETS[s]; probs = pickle.load(open(ROOT / pk, "rb"))["problems"][key]
        pass1 = ROOT / f"results/v2/raw/vamp_pass1_{s}.jsonl"
        n_pass1 = sum(1 for _ in open(pass1)) if pass1.exists() else 0
        assert n_pass1 == 10 * len(probs), f"{s}: pass (a) incomplete ({n_pass1} runs); run reference runs after it"
        ck = ROOT / f"results/v2/raw/feasibility_{s}.jsonl"
        done = {r["q"] for r in map(json.loads, open(ck))} if ck.exists() else set()
        ok = solved_by_timed_runs(s)
        todo = [q for q in range(len(probs)) if q not in ok and q not in done]
        print(f"{s}: {len(ok)} queries already solved by a timed run, {len(done)} references done, {len(todo)} to run", flush=True)
        run_gated([(s, q, vm, probs[q], seed) for q in todo], ck, a)


if __name__ == "__main__":
    main()
