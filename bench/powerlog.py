"""Run a command while sampling tegrastats; write a JSON summary next to the results.

usage: python bench/powerlog.py OUT.json -- cmd args...
Records board input power (VDD_IN), the CPU+GPU+CV rail, GPU load (GR3D_FREQ), max CPU/GPU temperature,
and the other heavy processes on the machine at start, so contention is visible in the results.
"""
import json, re, subprocess, sys, time, statistics as st

out = sys.argv[1]; cmd = sys.argv[sys.argv.index("--") + 1:]
others = [l.replace(__import__("os").path.expanduser("~") + "/", "~/")  # no home paths in committed results
         for l in subprocess.run(["ps", "-eo", "pcpu,args", "--sort=-pcpu"], capture_output=True, text=True).stdout.splitlines()[1:8]]
tg = subprocess.Popen(["tegrastats", "--interval", "250"], stdout=subprocess.PIPE, text=True)
t0 = time.time(); rc = None
samples = []
proc = subprocess.Popen(cmd)
import threading
def reader():
    for line in tg.stdout:
        samples.append((time.time() - t0, line))
th = threading.Thread(target=reader, daemon=True); th.start()
rc = proc.wait(); tg.terminate(); th.join(timeout=2)
def series(pat):
    return [float(m.group(1)) for _, l in samples if (m := re.search(pat, l))]
vin, vcg, gr = series(r"VDD_IN (\d+)mW"), series(r"VDD_CPU_GPU_CV (\d+)mW"), series(r"GR3D_FREQ (\d+)%")
cpu_t, gpu_t = series(r"cpu@([\d.]+)C"), series(r"gpu@([\d.]+)C")
summ = lambda x: {"mean": st.mean(x), "median": st.median(x), "max": max(x)} if x else None
json.dump({"cmd": cmd, "returncode": rc, "wall_s": time.time() - t0, "n_samples": len(samples),
           "VDD_IN_mW": summ(vin), "VDD_CPU_GPU_CV_mW": summ(vcg), "GR3D_pct": summ(gr),
           "cpu_temp_C": summ(cpu_t), "gpu_temp_C": summ(gpu_t), "top_processes_at_start": others},
          open(out, "w"), indent=1)
print("powerlog", out, "VDD_IN mean mW", summ(vin) and round(summ(vin)["mean"]), "GR3D mean", summ(gr) and round(summ(gr)["mean"]))
sys.exit(rc)
