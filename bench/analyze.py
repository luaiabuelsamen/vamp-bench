"""Aggregate raw runs into results/summary.json and results/figure.png.

Primary timing metric (apples to apples): wall-clock per query on the same Jetson, measured from Python,
  VAMP:   env construction + RRT-Connect + simplification          (wall_us)
  cuRobo: update_world + plan_single_js incl. CUDA sync            (world_update_us + plan_wall_us)
Planner-internal times are reported too (VAMP total_us; cuRobo solve_time_us).
Distributions are over successful queries; success rate is reported separately.
"""
import json, glob
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
GROUPS = [  # (label, tag, vamp robot, curobo robot)
    ("Panda · MBM (VAMP std.)", "mbm", "panda", "panda"),
    ("UR5 · MBM (VAMP std.)", "mbm", "ur5", "ur5"),
    ("UR5e + rail · pillars (paper)", "gantry_pillars", "ur5erail", "ur5e_rail"),
    ("UR5e + rail · free (paper)", "gantry_free", "ur5erail", "ur5e_rail"),
    ("UR5e · pillars (paper)", "ur5e_pillars", "ur5e", "ur5e"),
]


def stats(x):
    x = np.asarray(x, float)
    if not len(x):
        return None
    q = np.percentile(x, [5, 25, 50, 75, 95])
    return {"n": int(len(x)), "mean": float(x.mean()), "p5": q[0], "p25": q[1], "median": q[2], "p75": q[3], "p95": q[4], "max": float(x.max())}


def load(planner, tag, robot):
    f = ROOT / f"results/raw/{planner}_{tag}_{robot}.json"
    return json.load(open(f)) if f.exists() else None


summary = {"groups": [], "notes": __doc__}
for label, tag, vr, cr in GROUPS:
    V, C = load("vamp", tag, vr), load("curobo", tag, cr)
    if not (V and C):
        print("missing", label); continue
    g = {"label": label, "tag": tag}
    vv = [r for r in V["records"] if r["valid"]]
    cv = [r for r in C["records"] if r["valid"]]
    vs = [r for r in vv if r["success"]]
    cs = [r for r in cv if r["success"]]
    g["vamp"] = {"queries": len(vv), "success": len(vs), "success_rate": len(vs) / len(vv),
                 "wall_ms": stats([r["wall_us"] / 1e3 for r in vs]),
                 "internal_ms": stats([r["total_us"] / 1e3 for r in vs]),
                 "path_length_rad": stats([r["path_length_l2"] for r in vs])}
    cval = [r for r in cs if r.get("vamp_validated", True)]
    g["curobo"] = {"queries": len(cv), "success": len(cs), "success_rate": len(cs) / len(cv),
                   "success_vamp_validated": sum(bool(r.get("vamp_validated")) for r in cs),
                   "wall_ms": stats([(r["world_update_us"] + r["plan_wall_us"]) / 1e3 for r in cs]),
                   "plan_only_wall_ms": stats([r["plan_wall_us"] / 1e3 for r in cs]),
                   "internal_ms": stats([r["solve_time_us"] / 1e3 for r in cs]),
                   "path_length_rad": stats([r["path_length_l2"] for r in cs]),
                   "failure_status": {s: sum(r["status"] == s for r in cv if not r["success"]) for s in {r["status"] for r in cv if not r["success"]}}}
    # paired ratio on queries both solved (first trial each)
    vm = {(r["scene"], r["index"]): r for r in vs if r.get("trial", 0) == 0}
    cm = {(r["scene"], r["index"]): r for r in cs if r.get("trial", 0) == 0}
    both = sorted(set(vm) & set(cm))
    ratio = [(cm[k]["world_update_us"] + cm[k]["plan_wall_us"]) / vm[k]["wall_us"] for k in both]
    g["paired_wall_ratio_curobo_over_vamp"] = stats(ratio)
    g["median_wall_ratio"] = g["curobo"]["wall_ms"]["median"] / g["vamp"]["wall_ms"]["median"]
    for k in ("vamp", "curobo"):
        P = V if k == "vamp" else C
        g[k]["power"] = P.get("power")
    summary["groups"].append(g)
    print(f"{label:34s} VAMP {len(vs)}/{len(vv)} med {g['vamp']['wall_ms']['median']:.2f} ms | "
          f"cuRobo {len(cs)}/{len(cv)} med {g['curobo']['wall_ms']['median']:.0f} ms | ratio {g['median_wall_ratio']:.0f}x")

for planner in ("vamp", "curobo"):
    for f in glob.glob(str(ROOT / f"results/raw/{planner}_*.power.json")):
        summary.setdefault("power_logs", {})[Path(f).name] = json.load(open(f))
json.dump(summary, open(ROOT / "results/summary.json", "w"), indent=1, default=float)

# ---- figure: one panel, log-scale wall time per query, paired rows per problem set ----
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
BLUE, ORANGE, INK, INK2, GRID, SURF = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e6e5e0", "#fcfcfb"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.edgecolor": INK2, "axes.labelcolor": INK2,
                     "xtick.color": INK2, "ytick.color": INK})
G = summary["groups"]
fig, ax = plt.subplots(figsize=(8, 0.9 + 0.75 * len(G)), facecolor=SURF)
ax.set_facecolor(SURF)
rng = np.random.default_rng(0)
yt, yl = [], []
for i, g in enumerate(G[::-1]):
    y0 = i * 1.0
    V, C = load("vamp", g["tag"], [x for x in GROUPS if x[0] == g["label"]][0][2]), load("curobo", g["tag"], [x for x in GROUPS if x[0] == g["label"]][0][3])
    for dy, col, vals, s in ((0.18, BLUE, [r["wall_us"] / 1e3 for r in V["records"] if r.get("success")], g["vamp"]),
                             (-0.18, ORANGE, [(r["world_update_us"] + r["plan_wall_us"]) / 1e3 for r in C["records"] if r.get("success")], g["curobo"])):
        y = y0 + dy
        ax.scatter(vals, y + rng.uniform(-0.09, 0.09, len(vals)), s=4, color=col, alpha=0.35, linewidths=0)
        st = s["wall_ms"]
        ax.plot([st["p25"], st["p75"]], [y, y], color=col, lw=4, solid_capstyle="round")
        ax.plot([st["median"]], [y], marker="|", ms=12, mew=2, color=INK)
        ax.text(st["p95"] * 1.25, y, f"{st['median']:.3g} ms  ·  {s['success']}/{s['queries']} solved",
                va="center", fontsize=8, color=INK2)
    yt.append(y0); yl.append(g["label"])
ax.set_xscale("log"); ax.set_yticks(yt); ax.set_yticklabels(yl)
ax.set_xlabel("wall-clock time per query (ms, log scale) — successful queries; bar = IQR, tick = median")
ax.grid(axis="x", color=GRID, lw=0.8); ax.set_axisbelow(True)
for s in ("top", "right", "left"):
    ax.spines[s].set_visible(False)
ax.tick_params(axis="y", length=0)
lo = min(g["vamp"]["wall_ms"]["p5"] for g in G) / 2; hi = max(g["curobo"]["wall_ms"]["max"] for g in G) * 8
ax.set_xlim(lo, hi)
from matplotlib.lines import Line2D
ax.legend(handles=[Line2D([], [], color=BLUE, lw=4, label="VAMP (1 CPU core, NEON)"),
                   Line2D([], [], color=ORANGE, lw=4, label="cuRobo v1 (Orin NX GPU)")],
          loc="upper center", bbox_to_anchor=(0.45, 1.0 + 0.6 / len(G)), ncol=2, frameon=False)
ax.set_title("VAMP vs cuRobo on one Jetson Orin NX 16GB, identical sphere models and obstacles", loc="left",
             fontsize=10, color=INK, pad=28)
fig.tight_layout()
fig.savefig(ROOT / "results/figure.png", dpi=180, facecolor=SURF)
print("wrote results/summary.json results/figure.png")
