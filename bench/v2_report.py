"""Benchmark v2 report: hypothesis checks H1-H5 (docs/V2_PREREGISTRATION.md), result tables and per-bin figures,
from results/v2/summary.json (bench/v2_analyze.py).

Sets: heldout_a6 (PRIMARY held-out, amendment A6), mbm, mpinets; heldout (original, unwrapped goals) as the appendix.
Planners compared on all queries: VAMP (P1) and cuRobo v1 (10 seeds); cuRoboV2 only on its 100-query subsample
(3 seeds), against the other two on the same queries and seeds.
Writes results/v2/hypotheses.json, results/v2/tables.md and results/v2/per_bin_<set>.png.
"""
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
S = json.load(open(ROOT / "results/v2/summary.json"))
MAIN = ("heldout_a6", "mbm", "mpinets"); ALL = MAIN + ("heldout",)
AXES = ("reach", "clearance", "passage", "limits")
NAME = {"heldout_a6": "Held-out (A6, primary)", "heldout": "Held-out, unwrapped goals (appendix)", "mbm": "MotionBenchMaker", "mpinets": "MπNets"}
ROBOT = {"heldout_a6": "UR5e + rail", "heldout": "UR5e + rail", "mbm": "Panda", "mpinets": "Panda"}
COL = {"vamp": "#2a78b5", "curobo1": "#b8442a", "curobo2": "#4a9a5b"}
LAB = {"vamp": "VAMP", "curobo1": "cuRobo v1", "curobo2": "cuRoboV2"}
ORDER = {"reach": ["<0.5", "0.5-0.8", ">0.8"], "clearance": ["<1cm", "1-5cm", ">5cm"],
         "passage": ["free", "blocked-wide", "blocked-narrow"], "limits": ["<0.2rad", ">=0.2rad"]}


def ordered(ax, present):
    return [b for b in ORDER[ax] if b in present] + sorted(set(present) - set(ORDER[ax]))


def P(s, pl, view="all_queries"):
    return S["sets"][s]["planners"].get(pl, {}).get(view)


def h1(s):
    """VAMP's median T_ready below cuRobo v1's in every bin (bins with no solved v1 query count as VAMP lower)."""
    v, c = P(s, "vamp"), P(s, "curobo1"); rows = []
    for ax in AXES:
        for b in ordered(ax, v["per_bin"][ax]):
            x = v["per_bin"][ax][b]
            y = c["per_bin"][ax].get(b, {})
            a_, b_ = x["t_ready_med_ms"], y.get("t_ready_med_ms")
            rows.append({"axis": ax, "bin": b, "queries": x["queries"], "underpowered": x["underpowered"], "vamp_ms": a_, "curobo1_ms": b_,
                         "vamp_lower": bool(a_ is not None and (b_ is None or a_ < b_)),
                         "cis_disjoint": bool(x["t_ready_ci95"] and y.get("t_ready_ci95") and x["t_ready_ci95"][1] < y["t_ready_ci95"][0])})
    return {"holds_every_bin": all(r["vamp_lower"] for r in rows), "bins": rows,
            "overall_ms": {"vamp": P(s, "vamp")["t_ready_ms_median_runs"], "curobo1": P(s, "curobo1")["t_ready_ms_median_runs"]}}


def h2(s):
    """Relative T_cycle difference smaller than the relative T_ready difference (medians over solved runs / query medians)."""
    v, c = P(s, "vamp"), P(s, "curobo1")
    rr = c["t_ready_ms_median_runs"] / v["t_ready_ms_median_runs"]
    rc = c["t_cycle_med_of_query_meds"] / v["t_cycle_med_of_query_meds"]
    adv_ready_s = (c["t_ready_ms_median_runs"] - v["t_ready_ms_median_runs"]) / 1e3
    adv_exec_s = v["t_exec_s_median_runs"] - c["t_exec_s_median_runs"]
    return {"t_ready_ratio_curobo1_over_vamp": rr, "t_cycle_ratio_curobo1_over_vamp": rc,
            "holds": bool(abs(np.log(rc)) < abs(np.log(rr))),
            "vamp_t_ready_advantage_s": adv_ready_s, "curobo1_t_exec_advantage_s": adv_exec_s,
            "curobo1_wins_cycle_by_exec": bool(adv_exec_s > adv_ready_s),
            "note": "T_cycle ratio uses the median over queries of per-query median T_cycle, failures at T = 30 s"}


def h3(s):
    v, c = P(s, "vamp"), P(s, "curobo1"); qv, qc = v["quality_median"], c["quality_median"]
    return {"rms_jerk_norm": {"vamp_p1": qv["rms_jerk_norm"], "curobo1": qc["rms_jerk_norm"]},
            "peak_jerk_norm": {"vamp_p1": qv["peak_jerk_norm"], "curobo1": qc["peak_jerk_norm"]},
            "t_exec_s": {"vamp_p1": v["t_exec_s_median_runs"], "curobo1": c["t_exec_s_median_runs"], "vamp_p2": v.get("p2_t_exec_s_median"),
                         "vamp_p1_same_runs_as_p2": v.get("p1_t_exec_s_median_same_runs")},
            "p2_validity_rate_of_planned": v.get("p2_validity_rate_of_planned"),
            "min_clearance_m": {"vamp_p1": qv["min_clearance_m"], "curobo1": qc["min_clearance_m"]},
            "curobo1_lower_jerk": bool(qc["rms_jerk_norm"] < qv["rms_jerk_norm"]),
            "curobo1_shorter_exec": bool(c["t_exec_s_median_runs"] < v["t_exec_s_median_runs"]),
            "p2_closes_exec_gap": bool(v.get("p2_t_exec_s_median") is not None and v["p2_t_exec_s_median"] <= c["t_exec_s_median_runs"])}


def h4(s):
    out = {}
    for pl in ("vamp", "curobo1"):
        x = P(s, pl); out[pl] = {"queries_failing_some_seed": x["queries_failing_some_seed"], "queries_failing_all_seeds": x["queries_failing_all_seeds"],
                                 "success_rate_runs": x["success_rate_runs"], "t_cycle_worst_s": x["t_cycle_worst_over_queries"], "queries": x["queries"]}
    sub = {pl: P(s, pl, "v2_subsample_seeds012") for pl in ("vamp", "curobo1")}; sub["curobo2"] = P(s, "curobo2")
    out["v2_subsample"] = {pl: {"queries_failing_some_seed": x["queries_failing_some_seed"], "success_rate_runs": x["success_rate_runs"],
                                "t_ready_ms_median_runs": x["t_ready_ms_median_runs"], "t_cycle_med_of_query_meds": x["t_cycle_med_of_query_meds"]}
                           for pl, x in sub.items() if x}
    out["more_robust"] = min(("vamp", "curobo1"), key=lambda p: out[p]["queries_failing_some_seed"]) \
        if out["vamp"]["queries_failing_some_seed"] != out["curobo1"]["queries_failing_some_seed"] else "tie"
    return out


def figure(s, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, axs = plt.subplots(3, 4, figsize=(11, 7.2), dpi=150, sharey="row")
    rows = (("success_mean", "success rate", None), ("t_ready_med_ms", "median T_ready (ms)", "t_ready_ci95"),
            ("t_cycle_med", "median T_cycle (s)", "t_cycle_ci95"))
    for j, ax_name in enumerate(AXES):
        bins = ordered(ax_name, P(s, "vamp")["per_bin"][ax_name])
        for i, (k, lab, ci) in enumerate(rows):
            a = axs[i, j]
            for o, pl in ((-0.12, "vamp"), (0.12, "curobo1")):
                pb = P(s, pl)["per_bin"][ax_name]
                for x, b in enumerate(bins):
                    d = pb.get(b)
                    if not d or d[k] is None:
                        continue
                    filled = not d["underpowered"]
                    if ci and d.get(ci):
                        a.plot([x + o, x + o], d[ci], color=COL[pl], lw=1.5)
                    a.plot(x + o, d[k], "o", ms=6, color=COL[pl], mfc=COL[pl] if filled else "white", mew=1.5,
                           label=None)
            a.set_xticks(range(len(bins))); a.set_xticklabels([f"{b.replace('blocked-', 'blocked ')}\n(n={P(s, 'vamp')['per_bin'][ax_name][b]['queries']})".replace("blocked ", "blocked\n") for b in bins], fontsize=7)
            a.set_xlim(-0.6, len(bins) - 0.4)
            if k == "t_ready_med_ms":
                a.set_yscale("log")
            if k == "success_mean":
                a.set_ylim(0, 1.05)
            if k == "t_cycle_med":
                a.set_ylim(bottom=0)
            if j == 0:
                a.set_ylabel(lab, fontsize=8.5)
            if i == 0:
                a.set_title(ax_name, fontsize=9)
            for sp in ("top", "right"):
                a.spines[sp].set_visible(False)
            a.tick_params(labelsize=7.5); a.grid(axis="y", color="#ddd", lw=0.6)
    from matplotlib.lines import Line2D
    handles = [Line2D([], [], marker="o", ls="", ms=6, color=COL[pl], label=LAB[pl]) for pl in ("vamp", "curobo1")]
    fig.legend(handles=handles, loc="upper right", bbox_to_anchor=(0.99, 0.955), ncol=2, fontsize=8, frameon=False)
    fig.suptitle(f"{NAME[s]} ({ROBOT[s]}): per-bin medians of per-query medians, 95% bootstrap CI over queries", fontsize=10, x=0.01, ha="left")
    fig.text(0.01, 0.005, "Hollow marker: bin with < 30 queries (underpowered). T_ready over solved runs; T_cycle counts failures at T = 30 s. "
             "VAMP timed on CPU, cuRobo on GPU, same Jetson Orin NX 16 GB.", fontsize=7, color="#555")
    fig.tight_layout(rect=(0, 0.02, 1, 0.93)); fig.savefig(path); plt.close(fig)


def fmt(x, d=2):
    return "—" if x is None else (f"{x:.{d}f}" if isinstance(x, float) else str(x))


def tables(H):
    L = ["# Benchmark v2 tables (generated by bench/v2_report.py from results/v2/summary.json)", ""]
    L += ["## Overall, all queries", "", "| Set | Robot | Planner | Runs | Success | Queries failing some seed | Median T_ready (ms) | Median T_exec (s) | Median T_cycle (s) |",
          "|---|---|---|---|---|---|---|---|---|"]
    for s in ALL:
        for pl in ("vamp", "curobo1"):
            x = P(s, pl)
            L.append(f"| {NAME[s]} | {ROBOT[s]} | {LAB[pl]} | {x['runs']} | {x['success_rate_runs']:.1%} | {x['queries_failing_some_seed']}/{x['queries']} | "
                     f"{fmt(x['t_ready_ms_median_runs'], 1)} | {fmt(x['t_exec_s_median_runs'])} | {fmt(x['t_cycle_med_of_query_meds'])} |")
    L += ["", "## cuRoboV2 subsample (100 queries x 3 seeds per set, all three planners on the same runs)", "",
          "| Set | Planner | Success | Queries failing some seed | Median T_ready (ms) | Median T_cycle (s) |", "|---|---|---|---|---|---|"]
    for s in ALL:
        for pl, x in H[s]["H4"]["v2_subsample"].items():
            L.append(f"| {NAME[s]} | {LAB[pl]} | {x['success_rate_runs']:.1%} | {x['queries_failing_some_seed']}/100 | {fmt(x['t_ready_ms_median_runs'], 1)} | {fmt(x['t_cycle_med_of_query_meds'])} |")
    L += ["", "## Feasibility (A3)", "", "| Set | Certified feasible | Unresolved | Pending reference runs |", "|---|---|---|---|"]
    for s in ALL:
        f = S["sets"][s]["feasibility"]; L.append(f"| {NAME[s]} | {f['certified']} | {f['unresolved']} | {f['pending_reference']} |")
    L += ["", "## Hypotheses", "", "| Set | H1 VAMP T_ready lower in every bin | H2 cycle gap smaller than ready gap | H3 v1 lower jerk / shorter T_exec | H4 more robust |",
          "|---|---|---|---|---|"]
    for s in ALL:
        h = H[s]
        L.append(f"| {NAME[s]} | {'yes' if h['H1']['holds_every_bin'] else 'no'} | {'yes' if h['H2']['holds'] else 'no'} "
                 f"(T_ready x{h['H2']['t_ready_ratio_curobo1_over_vamp']:.0f}, T_cycle x{h['H2']['t_cycle_ratio_curobo1_over_vamp']:.2f}) | "
                 f"{'yes' if h['H3']['curobo1_lower_jerk'] else 'no'} / {'yes' if h['H3']['curobo1_shorter_exec'] else 'no'} | {LAB.get(h['H4']['more_robust'], 'tie')} |")
    L += ["", f"H5 (sign the same on held-out A6, MBM and MπNets): {json.dumps(H['H5'])}", ""]
    pa = S.get("paired_A6_vs_original")
    if pa:
        L += ["## Appendix: original held-out goals vs A6, paired query by query", "",
              "| Planner | Queries | Success original | Success A6 | Difference (95% CI) | Sensitive to long moves |", "|---|---|---|---|---|---|"]
        for pl, x in pa["per_planner"].items():
            L.append(f"| {pl} | {x['queries']} | {x['success_original']:.1%} | {x['success_a6']:.1%} | {x['diff_mean']:+.1%} "
                     f"({x['diff_ci95'][0]:+.1%}, {x['diff_ci95'][1]:+.1%}) | {'yes' if x['sensitive_to_long_moves'] else 'no'} |")
        L += ["", "| Gap | Success gap original | Success gap A6 | Explained by move length | T_cycle gap original (s) | T_cycle gap A6 (s) | Explained |", "|---|---|---|---|---|---|---|"]
        for g, x in pa["gaps"].items():
            L.append(f"| {g} | {x['success_gap_original']:+.1%} | {x['success_gap_a6']:+.1%} | {'yes' if x['success_gap_explained_by_move_length'] else 'no'} | "
                     f"{x['t_cycle_gap_s_original']:+.2f} | {x['t_cycle_gap_s_a6']:+.2f} | {'yes' if x['t_cycle_gap_explained_by_move_length'] else 'no'} |")
    L += ["", "## Per bin (median T_ready, ms; VAMP vs cuRobo v1)", ""]
    for s in ALL:
        L += [f"### {NAME[s]}", "", "| Axis | Bin | Queries | VAMP | cuRobo v1 | VAMP lower | CIs disjoint |", "|---|---|---|---|---|---|---|"]
        for r in H[s]["H1"]["bins"]:
            L.append(f"| {r['axis']} | {r['bin']} | {r['queries']}{' (underpowered)' if r['underpowered'] else ''} | {fmt(r['vamp_ms'], 1)} | {fmt(r['curobo1_ms'], 1)} | "
                     f"{'yes' if r['vamp_lower'] else 'no'} | {'yes' if r['cis_disjoint'] else 'no'} |")
        L.append("")
    return "\n".join(L)


def main():
    H = {s: {"H1": h1(s), "H2": h2(s), "H3": h3(s), "H4": h4(s)} for s in ALL}
    H["H5"] = {k: len({H[s][hk][field] for s in MAIN}) == 1 for k, hk, field in
               (("H1_same_sign", "H1", "holds_every_bin"), ("H2_same_sign", "H2", "holds"), ("H3_jerk_same_sign", "H3", "curobo1_lower_jerk"),
                ("H3_exec_same_sign", "H3", "curobo1_shorter_exec"), ("H4_same_sign", "H4", "more_robust"))}
    json.dump(H, open(ROOT / "results/v2/hypotheses.json", "w"), indent=1)
    open(ROOT / "results/v2/tables.md", "w").write(tables(H))
    for s in ALL:
        figure(s, ROOT / f"results/v2/per_bin_{s}.png")
    for s in ALL:
        print(s, "H1", H[s]["H1"]["holds_every_bin"], "H2", H[s]["H2"]["holds"], "H3", H[s]["H3"]["curobo1_lower_jerk"], H[s]["H3"]["curobo1_shorter_exec"], "H4", H[s]["H4"]["more_robust"])
    print("H5", H["H5"])


if __name__ == "__main__":
    main()
