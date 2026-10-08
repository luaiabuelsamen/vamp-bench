"""Aggregate raw runs into committed results: results/summary.json, results/records/*.json, results/figure.png.

Primary timing metric (apples to apples): wall-clock per query on the same Jetson, measured from Python.
  VAMP:      env construction + RRT-Connect + simplification            (wall_us)
  cuRobo v1: update_world + plan_single_js incl. CUDA sync              (world_update_us + plan_wall_us)
  cuRoboV2:  clear cache + update_world + reset_seed + plan_cspace, sync (world_update_us + plan_wall_us)
Planner-internal times are reported too (VAMP total_us; cuRobo solve_time_us).
Time distributions are over successful queries; success rate is reported separately.
Success = planner-reported. The VAMP segment check of cuRobo trajectories is reported separately and is provisional.
A planner's cells are null ("pending") until its raw file exists in results/raw.
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
# key, file prefix, which robot name the file uses, display name, colour (validated categorical slots 1-3)
PLANNERS = [("vamp", "vamp", 2, "VAMP · 1 CPU core", "#2a78d6"),
            ("curobo_v1", "curobo", 3, "cuRobo v1 (paper's version) · GPU", "#eb6834"),
            ("curobo_v2", "curobo2", 3, "cuRoboV2 (current) · GPU", "#1baf7a")]
SHORT = {"vamp": "VAMP", "curobo_v1": "cuRobo v1", "curobo_v2": "cuRoboV2"}


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
    json.dump({**meta, "records": [{k: v for k, v in r.items() if k != "path"} for r in D["records"]]}, open(out, "w"))


def wall_ms(key, r):
    return r["wall_us"] / 1e3 if key == "vamp" else (r["world_update_us"] + r["plan_wall_us"]) / 1e3


def internal_ms(key, r):
    return r["total_us"] / 1e3 if key == "vamp" else (r["solve_time_us"] or np.nan) / 1e3


def fname(prefix, rix, g):
    return f"{prefix}_{g[1]}_{g[rix]}"


summary = {"notes": __doc__, "groups": []}
for g in GROUPS:
    out = {"label": g[0], "tag": g[1]}
    first, ran, allrec = {}, {}, {}
    for key, prefix, rix, _, _ in PLANNERS:
        name = fname(prefix, rix, g)
        D, P = load(name)
        out[key] = None
        if not D:
            continue
        slim(name, D)
        val = [r for r in D["records"] if r["valid"]]
        raw_ok = [r for r in val if r["success"]]
        # Primary: planner-reported success. The VAMP segment check (bench/crossval.py) is reported alongside and is
        # PROVISIONAL until docs/V2_COLLISION_CHECK.md rules it in or out as a measurement artifact.
        ok = raw_ok
        s = {"source": f"results/records/{name}.json", "queries": len(val), "success": len(ok),
             "success_rate": len(ok) / len(val), "wall_ms": stats([wall_ms(key, r) for r in ok]),
             "internal_ms": stats([internal_ms(key, r) for r in ok]),
             "path_length_rad": stats([r["path_length_l2"] for r in ok]), "power": P,
             "settings": D.get("settings"), "plan_config": D.get("plan_config"), "planner": D.get("planner")}
        scenes = sorted({r["scene"] for r in val})
        s["per_scene"] = {sc: {"success": sum(r["success"] for r in val if r["scene"] == sc),
                               "queries": sum(r["scene"] == sc for r in val)} for sc in scenes}
        fails = [r for r in val if not r["success"]]
        s["failures"] = {"n": len(fails), "queries": sorted({(r["scene"], r["index"]) for r in fails}),
                         "time_to_failure_ms": stats([wall_ms(key, r) if key != "vamp" else r["plan_us"] / 1e3 for r in fails])}
        if key != "vamp":
            s["success_passing_vamp_segment_check_PROVISIONAL"] = sum(bool(r.get("vamp_validated")) for r in raw_ok)
            s["success_flagged_by_vamp_segment_check_PROVISIONAL"] = sum(not r.get("vamp_validated") for r in raw_ok)
            s["failure_status"] = {st: sum(r["status"] == st for r in val if not r["success"])
                                   for st in {r["status"] for r in val if not r["success"]}}
        out[key] = s
        first[key] = {(r["scene"], r["index"]): r for r in ok if r.get("trial", 0) == 0}
        ran[key] = {(r["scene"], r["index"]) for r in val}
        allrec[key] = D["records"]
    for key in ("curobo_v1", "curobo_v2"):
        if out["vamp"] and out[key]:
            both = sorted(set(first["vamp"]) & set(first[key]))
            # VAMP restricted to exactly the queries this cuRobo version ran (v2 runs a subset)
            sub = [r for r in allrec["vamp"] if r["valid"] and (r["scene"], r["index"]) in ran[key]]
            vs = [r for r in sub if r["success"]]
            vamp_sub = {"queries": len(sub), "success": len(vs), "wall_ms": stats([wall_ms("vamp", r) for r in vs])}
            out[f"{key}_vs_vamp_same_queries"] = {
                "vamp_on_same_queries": vamp_sub,
                "median_wall_ratio": out[key]["wall_ms"]["median"] / vamp_sub["wall_ms"]["median"],
                "paired_wall_ratio": stats([wall_ms(key, first[key][k]) / wall_ms("vamp", first["vamp"][k]) for k in both])}
    if out["curobo_v1"] and out["curobo_v2"]:  # v1 restricted to exactly the queries v2 ran
        sub = [r for r in allrec["curobo_v1"] if r["valid"] and (r["scene"], r["index"]) in ran["curobo_v2"]]
        ok1 = [r for r in sub if r["success"]]
        out["curobo_v1_on_v2_queries"] = {"queries": len(sub), "success": len(ok1),
                                          "wall_ms": stats([wall_ms("curobo_v1", r) for r in ok1])}
    summary["groups"].append(out)
    cell = lambda s: f"{s['success']}/{s['queries']} med {s['wall_ms']['median']:.3g} ms" if s else "pending"
    print(f"{g[0]:34s} " + " | ".join(f"{SHORT[k]} {cell(out[k])}" for k, *_ in PLANNERS))

# superseded runs (kept for traceability; see results/raw/superseded/README.md)
summary["superseded"] = {}
for f in sorted((RAW / "superseded").glob("curobo*_mbm_*.json")):
    if f.name.endswith(".power.json"):
        continue
    D = json.load(open(f)); val = [r for r in D["records"] if r["valid"]]
    summary["superseded"][f.stem] = {"reason": "UR5 URDF velocity cap 0.5 rad/s (fixed: configs/ur5_spherized_vel_pi.urdf)",
                                     "queries": len(val), "success": sum(r["success"] for r in val)}

vc = ROOT / "results/v2_collision_check.json"
if vc.exists():
    d = json.load(open(vc)); fl = [q for c in d["cases"].values() for q in c["queries"] if q["kind"] == "flagged"]
    ct = [q for c in d["cases"].values() for q in c["queries"] if q["kind"] == "control"]
    pen = [q["A_stored"]["v2_max_penetration_m"] * 1e3 for q in fl]
    fresh = [q["B_fresh"] for q in fl if q["B_fresh"].get("replan_success")]
    summary["v2_collision_check"] = {
        "flagged": len(fl), "controls": len(ct),
        "flagged_confirmed_by_v2_checker": sum(q["A_stored"]["v2_colliding_act0"] > 0 for q in fl),
        "flagged_same_samples_as_vamp": sum(q["A_stored"]["v2_colliding_act0"] == q["A_stored"]["vamp_colliding"] == q["A_stored"]["both"] for q in fl),
        "controls_clean_both": sum(q["A_stored"]["v2_colliding_act0"] == 0 and q["A_stored"]["vamp_colliding"] == 0 for q in ct),
        "waypoint_collisions_v2_act0": sum(q["A_stored"]["v2_colliding_at_waypoints_act0"] for q in fl + ct),
        "penetration_mm": stats(pen), "over_1mm": sum(x > 1 for x in pen), "over_2p5mm": sum(x > 2.5 for x in pen),
        "random_config_agreement": {t: c["random_config_agreement_v2_vs_vamp"] for t, c in d["cases"].items()},
        "fresh_replans": len(fresh), "fresh_flagged_again": sum(b["vamp_interp_segments_failing"] > 0 for b in fresh),
        "fresh_reproduced_stored": sum(b["same_as_stored"] for b in fresh),
        "interpolation_dt_s": sorted({c["interpolation_dt"] for c in d["cases"].values()}),
        "dense_resampling_selfcheck_passed": all(b["selfcheck_dense_vs_base_max_diff"] < 1e-3 for b in fresh)}

agree = ROOT / "results/collision_model_agreement.json"
if agree.exists():
    summary["collision_model_agreement"] = json.load(open(agree))
json.dump(summary, open(ROOT / "results/summary.json", "w"), indent=1, default=float)

# ---- figure: one panel, log-scale wall time per query, three rows per problem set ----
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.ticker import FuncFormatter
INK, INK2, GRID, SURF = "#0b0b0b", "#52514e", "#e6e5e0", "#fcfcfb"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 9, "axes.edgecolor": INK2,
                     "axes.labelcolor": INK2, "xtick.color": INK2, "ytick.color": INK})
fig, ax = plt.subplots(figsize=(9.5, 1.3 + 1.05 * len(GROUPS)), facecolor=SURF)
ax.set_facecolor(SURF)
rng = np.random.default_rng(0)
yt, xs = [], []
offs = [0.27, 0.0, -0.27]
for i, (g, s) in enumerate(list(zip(GROUPS, summary["groups"]))[::-1]):
    for (key, prefix, rix, _, col), dy in zip(PLANNERS, offs):
        y = i + dy
        if not s[key]:
            ax.text(1.0, y, f"{SHORT[key]}: pending rerun", va="center", fontsize=7.5, color=INK2, style="italic")
            continue
        D, _ = load(fname(prefix, rix, g))
        vals = [wall_ms(key, r) for r in D["records"] if r.get("success")]
        xs += vals
        ax.scatter(vals, y + rng.uniform(-0.07, 0.07, len(vals)), s=3, color=col, alpha=0.3, linewidths=0)
        st = s[key]["wall_ms"]
        ax.plot([st["p25"], st["p75"]], [y, y], color=col, lw=4, solid_capstyle="round")
        ax.plot([st["median"]], [y], marker="|", ms=10, mew=2, color=INK)
        med = f"{st['median'] / 1e3:.3g} s" if st["median"] >= 1000 else f"{st['median']:.3g} ms"
        cav = " (max_attempts 20)" if key == "curobo_v2" else ""
        ax.text(max(vals) * 1.5, y, f"{SHORT[key]} {med} · {s[key]['success']}/{s[key]['queries']}{cav}",
                va="center", fontsize=7.5, color=INK2)
    yt.append(i)
ax.set_xscale("log"); ax.set_yticks(yt); ax.set_yticklabels([g[0] for g in GROUPS][::-1])
ax.set_ylim(-0.6, len(GROUPS) - 0.4)
ax.xaxis.set_major_formatter(FuncFormatter(lambda v, _: f"{v:,.0f}" if v >= 1 else f"{v:g}"))
ax.set_xlim(min(xs) / 2, max(10000, max(xs) * 120))
ax.set_xticks([t for t in (0.1, 1, 10, 100, 1000, 10000, 100000) if min(xs) / 2 <= t <= max(xs) * 120])
ax.set_xlabel("wall-clock time per query, ms (log scale)\ndots = solved queries (solved = planner-reported success) · bar = IQR · tick = median")
ax.grid(axis="x", color=GRID, lw=0.8); ax.set_axisbelow(True)
for sp in ("top", "right", "left"):
    ax.spines[sp].set_visible(False)
ax.tick_params(axis="y", length=0)
ax.legend(handles=[Line2D([], [], color=c, lw=4, label=n) for _, _, _, n, c in PLANNERS],
          loc="lower left", bbox_to_anchor=(-0.02, 1.0), ncol=3, frameon=False, fontsize=7.5, handlelength=1.5, columnspacing=1.0)
ax.set_title("Same Jetson Orin NX 16GB · identical sphere models, obstacles and joint-space queries",
             loc="left", fontsize=10, color=INK, pad=26)
fig.tight_layout()
fig.savefig(ROOT / "results/figure.png", dpi=180, facecolor=SURF)
print("wrote results/summary.json results/records/ results/figure.png")
