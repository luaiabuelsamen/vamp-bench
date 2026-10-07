"""Aggregate raw runs into committed results: results/summary.json, results/records/*.json, results/figure.png.

Primary timing metric (apples to apples): wall-clock per query on the same Jetson, measured from Python.
  VAMP:   env construction + RRT-Connect + simplification          (wall_us)
  cuRobo: update_world + plan_single_js incl. CUDA sync            (world_update_us + plan_wall_us)
Planner-internal times are reported too (VAMP total_us; cuRobo solve_time_us).
Time distributions are over successful queries; success rate is reported separately.
cuRobo cells are null ("pending") until its clean GPU run exists in results/raw.
"""
import json
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RAW = ROOT / "results/raw"
GROUPS = [  # (label, tag, vamp module, curobo robot)
    ("Panda · MBM", "mbm", "panda", "panda"),
    ("UR5 · MBM", "mbm", "ur5", "ur5"),
    ("UR5e+rail · pillars (paper)", "gantry_pillars", "ur5erail", "ur5e_rail"),
    ("UR5e+rail · no obstacles (paper)", "gantry_free", "ur5erail", "ur5e_rail"),
    ("UR5e · pillars (paper)", "ur5e_pillars", "ur5e", "ur5e"),
]


def stats(x):
    x = np.asarray(x, float)
    if not len(x):
        return None
    q = np.percentile(x, [5, 25, 50, 75, 95])
    return {"n": int(len(x)), "mean": float(x.mean()), "p5": q[0], "p25": q[1], "median": q[2],
            "p75": q[3], "p95": q[4], "max": float(x.max())}


def load(name):
    f = RAW / f"{name}.json"
    if not f.exists():
        return None, None
    p = RAW / f"{name}.power.json"
    return json.load(open(f)), (json.load(open(p)) if p.exists() else None)


def slim(name, D):
    """Committed per-query records (paths dropped to keep git small)."""
    out = ROOT / f"results/records/{name}.json"; out.parent.mkdir(parents=True, exist_ok=True)
    meta = {k: v for k, v in D.items() if k != "records"}
    recs = [{k: v for k, v in r.items() if k != "path"} for r in D["records"]]
    json.dump({**meta, "records": recs}, open(out, "w"))


def vamp_times(r):
    return r["wall_us"] / 1e3


def curobo_times(r):
    return (r["world_update_us"] + r["plan_wall_us"]) / 1e3


summary = {"notes": __doc__, "groups": []}
for label, tag, vm, cr in GROUPS:
    vname, cname = f"vamp_{tag}_{vm}", f"curobo_{tag}_{cr}"
    V, Vp = load(vname); C, Cp = load(cname)
    g = {"label": label, "tag": tag, "vamp": None, "curobo": None}
    if V:
        slim(vname, V)
        vv = [r for r in V["records"] if r["valid"]]; vs = [r for r in vv if r["success"]]
        g["vamp"] = {"source": f"results/records/{vname}.json", "queries": len(vv), "success": len(vs),
                     "success_rate": len(vs) / len(vv), "wall_ms": stats([vamp_times(r) for r in vs]),
                     "internal_ms": stats([r["total_us"] / 1e3 for r in vs]),
                     "path_length_rad": stats([r["path_length_l2"] for r in vs]), "power": Vp}
    if C:
        slim(cname, C)
        cv = [r for r in C["records"] if r["valid"]]; cs = [r for r in cv if r["success"]]
        g["curobo"] = {"source": f"results/records/{cname}.json", "queries": len(cv), "success": len(cs),
                       "success_rate": len(cs) / len(cv),
                       "success_vamp_validated": sum(bool(r.get("vamp_validated")) for r in cs),
                       "wall_ms": stats([curobo_times(r) for r in cs]),
                       "plan_only_wall_ms": stats([r["plan_wall_us"] / 1e3 for r in cs]),
                       "internal_ms": stats([r["solve_time_us"] / 1e3 for r in cs]),
                       "path_length_rad": stats([r["path_length_l2"] for r in cs]),
                       "failure_status": {s: sum(r["status"] == s for r in cv if not r["success"])
                                          for s in {r["status"] for r in cv if not r["success"]}}, "power": Cp}
    if V and C:
        vmap = {(r["scene"], r["index"]): r for r in V["records"] if r.get("success") and r.get("trial", 0) == 0}
        cmap = {(r["scene"], r["index"]): r for r in C["records"] if r.get("success") and r.get("trial", 0) == 0}
        both = sorted(set(vmap) & set(cmap))
        g["paired_wall_ratio_curobo_over_vamp"] = stats([curobo_times(cmap[k]) / vamp_times(vmap[k]) for k in both])
        g["median_wall_ratio"] = g["curobo"]["wall_ms"]["median"] / g["vamp"]["wall_ms"]["median"]
    summary["groups"].append(g)
    fmt = lambda s: f"{s['success']}/{s['queries']} med {s['wall_ms']['median']:.3g} ms" if s else "pending"
    print(f"{label:34s} VAMP {fmt(g['vamp'])} | cuRobo {fmt(g['curobo'])}")

agree = ROOT / "results/collision_model_agreement.json"
if agree.exists():
    summary["collision_model_agreement"] = json.load(open(agree))
json.dump(summary, open(ROOT / "results/summary.json", "w"), indent=1, default=float)

# ---- figure: one panel, log-scale wall time per query, paired rows per problem set ----
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
BLUE, ORANGE, INK, INK2, GRID, SURF = "#2a78d6", "#eb6834", "#0b0b0b", "#52514e", "#e6e5e0", "#fcfcfb"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.edgecolor": INK2,
                     "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK})
G = summary["groups"]
fig, ax = plt.subplots(figsize=(9, 1.2 + 0.8 * len(G)), facecolor=SURF)
ax.set_facecolor(SURF)
rng = np.random.default_rng(0)
yt, yl, xs = [], [], []
for i, (g, (label, tag, vm, cr)) in enumerate(list(zip(G, GROUPS))[::-1]):
    y0 = float(i)
    for dy, col, name, s, tf in ((0.18, BLUE, f"vamp_{tag}_{vm}", g["vamp"], vamp_times),
                                 (-0.18, ORANGE, f"curobo_{tag}_{cr}", g["curobo"], curobo_times)):
        y = y0 + dy
        if not s:
            ax.text(1.0, y, "cuRobo: pending clean GPU run", va="center", fontsize=8, color=INK2, style="italic")
            continue
        D, _ = load(name)
        vals = [tf(r) for r in D["records"] if r.get("success")]
        xs += vals
        ax.scatter(vals, y + rng.uniform(-0.09, 0.09, len(vals)), s=3, color=col, alpha=0.3, linewidths=0)
        st = s["wall_ms"]
        ax.plot([st["p25"], st["p75"]], [y, y], color=col, lw=4, solid_capstyle="round")
        ax.plot([st["median"]], [y], marker="|", ms=12, mew=2, color=INK)
        ax.text(max(vals) * 1.5, y, f"median {st['median']:.3g} ms · {s['success']}/{s['queries']} solved",
                va="center", fontsize=8, color=INK2)
    yt.append(y0); yl.append(label)
ax.set_xscale("log"); ax.set_yticks(yt); ax.set_yticklabels(yl)
ax.set_ylim(-0.6, len(G) - 0.4)
from matplotlib.ticker import FuncFormatter
ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:g}"))
ax.set_xlim(min(xs) / 2, max(10000, max(xs) * 30))
ax.set_xlabel("wall-clock time per query, ms (log scale). Dots = successful queries, bar = IQR, tick = median")
ax.grid(axis="x", color=GRID, lw=0.8); ax.set_axisbelow(True)
for sp in ("top", "right", "left"):
    ax.spines[sp].set_visible(False)
ax.tick_params(axis="y", length=0)
ax.legend(handles=[Line2D([], [], color=BLUE, lw=4, label="VAMP, 1 CPU core (Cortex-A78AE, NEON)"),
                   Line2D([], [], color=ORANGE, lw=4, label="cuRobo v1, Orin NX GPU")],
          loc="lower left", bbox_to_anchor=(0, 1.0), ncol=2, frameon=False)
ax.set_title("Same Jetson Orin NX 16GB · identical sphere models, obstacles and joint-space queries",
             loc="left", fontsize=10, color=INK, pad=26)
fig.tight_layout()
fig.savefig(ROOT / "results/figure.png", dpi=180, facecolor=SURF)
print("wrote results/summary.json results/records/ results/figure.png")
