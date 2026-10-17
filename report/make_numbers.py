"""Every number in the technical report, generated from the committed result files.

Writes report/generated/numbers.tex (one macro per quoted number) and report/generated/tab_*.tex (table bodies).
The hand-written report/vamp_bench_report.tex contains no digits in its body; bench/check_consistency.py checks
that, regenerates these files and compares, and checks every number in the built PDF against them.

Sources: results/summary.json (v1), results/collision_model_agreement.json, results/v2/summary.json,
results/v2/hypotheses.json, results/v2/pickplace_summary.json, results/v2_1/movel_r2.json,
results/v2_1/summary.json, results/v2_2/summary.json, configs/v2_limits.yaml, configs/v2_planners.yaml.
PROTOCOL holds the few settings that exist only in the pre-registration documents (cited per entry).
Usage: python report/make_numbers.py [--check]   (--check: compare with the files on disk, write nothing)
"""
import json, re, subprocess, sys
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
GEN = ROOT / "report/generated"
J = lambda p: json.load(open(ROOT / p))
V1 = J("results/summary.json"); V2 = J("results/v2/summary.json"); HYP = J("results/v2/hypotheses.json")
PP = J("results/v2/pickplace_summary.json"); R2MOVEL = J("results/v2_1/movel_r2.json"); V21 = J("results/v2_1/summary.json")
V22 = J("results/v2_2/summary.json"); LIM = yaml.safe_load(open(ROOT / "configs/v2_limits.yaml"))
PLN = yaml.safe_load(open(ROOT / "configs/v2_planners.yaml"))
PROG = __import__("pickle").load(open(ROOT / "problems/v2_2_programs.pkl", "rb"))

# Settings stated only in the pre-registrations (not in a result or config file).
PROTOCOL = {
    "board_mem_gb": 16,            # hardware: Jetson Orin NX 16 GB (README, every pre-registration)
    "path_limit_mm": 5,            # v2.2 success: path deviation outside blends (V2_2 pre-registration, Metrics)
    "end_mm": 1, "end_deg": 1,     # v2.2 and v2.1 end-pose tolerance; v2.2 orientation limit (same section)
    "blend_r_cm": 5,               # v2.2 W blend radius cap (Programs, W)
    "tcp_jerk": 10,                # v2.2 TCP jerk of the programmed time law (All program types)
    "inner_m": 0.35, "outer_m": 0.70,  # v2.2 amendment V1 workspace bounds
    "axis_hold_mm": 1,             # v2.2 amendment V2a displacement rule
    "pp_cells": 30, "pp_picks_per_cell": 4, "pp_seeds": 5, "pp_segments": 6,  # v2 pick-place design
    "v2_sub_queries": 100, "v2_sub_seeds": 3,  # cuRoboV2 subsample in v2
    "v1_attempts": 20, "v2_nvidia_attempts": 100,  # max_attempts (README settings)
    "pyroki_margin_mm": 5, "pyroki_knots": 50, "ik_starts": 64,  # v2.1 design (BENCHMARK_V2 section 7, 7c/7d)
    "knot_dt_ms": 40,              # v2.2 PyRoki knot spacing rule
    "start_tol_rad": "1e-3",       # v2.1 endpoint tolerance
    "q_bins_min": 30,              # underpowered-bin threshold
}

macros, tables = {}, {}


def M(name, text):
    assert re.fullmatch(r"[A-Za-z]+", name), name
    assert name not in macros, name
    macros[name] = str(text)


def d(x, n):
    return f"{x:.{n}f}"


def tm(ms, unit=True):
    """Time as README prints it: < 10 ms two decimals, < 1 s whole ms, else seconds to 3 significant figures."""
    if ms < 10:
        s = f"{ms:.2f}"; u = "ms"
    elif ms < 1000:
        s = f"{ms:.0f}"; u = "ms"
    else:
        x = ms / 1000; s = f"{x:.2f}" if x < 10 else f"{x:.1f}"; u = "s"
    return f"{s}\\,{u}" if unit else s


def pct(x, n=1):
    return f"{100 * x:.{n}f}\\%"


def frac(a, b):
    return f"{a}/{b}"


def ratio(x):
    return f"{x:.0f}$\\times$"


def num(x):
    return f"{x:,}".replace(",", "{,}")


# ---------------------------------------------------------------- common
M("boardMem", PROTOCOL["board_mem_gb"])
M("gateHz", f"{PLN['validation']['rate_hz'] / 1000:.0f}")
tol_pct = float(re.search(r"tolerance (\d+)%", PLN["validation"]["limits"]).group(1))
M("gateTol", f"{1 + tol_pct / 100:.2f}")
M("jerkLimit", f"{LIM['ur5e_rail']['jerk'][1]:.0f}")
M("accLimit", f"{LIM['ur5e_rail']['acceleration'][1]:.0f}")
M("budget", f"{V2['budget_T_s']:.0f}")
M("vOneAttempts", PROTOCOL["v1_attempts"]); M("nvidiaAttempts", PROTOCOL["v2_nvidia_attempts"])
vers = lambda s: re.search(r"([0-9a-f]{7})", s).group(1)
M("vampCommit", vers(PLN["vamp"]["version"])); M("curoboOneCommit", vers(PLN["curobo_v1"]["version"]))
M("curoboTwoCommit", vers(PLN["curobo_v2"]["version"]))
M("curoboOneVersion", re.search(r"\((v[^)]*)\)", PLN["curobo_v1"]["version"]).group(1))
M("curoboTwoVersion", re.search(r"\((v[^)]*)\)", PLN["curobo_v2"]["version"]).group(1))
M("freezeHash", PROG["freeze"][:12])
M("reportDate", "2026-10-15")  # date of the report's content; change it when the report is revised (was: last commit date)

# ---------------------------------------------------------------- v1
agree = V1["collision_model_agreement"]
M("agreeN", num(sum(v["n"] for v in agree.values())))
M("agreeShare", pct(min(v["agree"] for v in agree.values()), 0))
SHORT = {"mbm": None, "gantry_pillars": "Rail, pillars", "gantry_free": "Rail, no obstacles", "ur5e_pillars": "UR5e, pillars"}
G = {(SHORT.get(g["tag"]) or g["label"].replace(" · ", " ")): g for g in V1["groups"]}
names = list(G)
rows = []
for lab, g in G.items():
    cells = []
    for k in ("vamp", "curobo_v1", "curobo_v2"):
        p = g[k]; w = p["wall_ms"]
        cells.append(f"{frac(p['success'], p['queries'])} & {tm(w['median'])} ({tm(w['p95'], unit=False) if tm(w['p95']).endswith('ms') == tm(w['median']).endswith('ms') else tm(w['p95'])})")
    rows.append(f"{lab} & " + " & ".join(cells) + r" \\")
tables["tab_v1_sets"] = "\n".join(rows)
rows = []
for k, lab in (("curobo_v1_vs_vamp_same_queries", "cuRobo v1"), ("curobo_v2_vs_vamp_same_queries", "cuRoboV2")):
    rows.append(lab + " & " + " & ".join(ratio(g[k]["median_wall_ratio"]) for g in G.values()) + r" \\")
tables["tab_v1_ratios"] = "\n".join(rows)
tables["tab_v1_ratio_head"] = " & ".join(G)
r1 = [g["curobo_v1_vs_vamp_same_queries"]["median_wall_ratio"] for g in G.values()]
r2 = [g["curobo_v2_vs_vamp_same_queries"]["median_wall_ratio"] for g in G.values()]
M("vOneRatioMin", ratio(min(r1))); M("vOneRatioMax", ratio(max(r1)))
M("vTwoRatioMin", ratio(min(r2))); M("vTwoRatioMax", ratio(max(r2)))
pan = G[names[0]]
M("pandaVampMs", tm(pan["vamp"]["wall_ms"]["median"])); M("pandaVOneMs", tm(pan["curobo_v1"]["wall_ms"]["median"]))
M("pandaVTwoMs", tm(pan["curobo_v2"]["wall_ms"]["median"]))
M("pandaVTwoSolved", frac(pan["curobo_v2"]["success"], pan["curobo_v2"]["queries"]))
M("pandaVOneOnVTwo", frac(pan["curobo_v1_on_v2_queries"]["success"], pan["curobo_v1_on_v2_queries"]["queries"]))
M("pandaVOneOnVTwoMs", tm(pan["curobo_v1_on_v2_queries"]["wall_ms"]["median"]))
fl = {lab: (g["curobo_v1"]["success_flagged_by_vamp_segment_check_PROVISIONAL"], g["curobo_v1"]["success"],
                    g["curobo_v2"]["success_flagged_by_vamp_segment_check_PROVISIONAL"], g["curobo_v2"]["success"]) for lab, g in G.items()}
tables["tab_v1_flagged"] = "\n".join(f"{lab} & {a} of {b} & {c} of {e} " + r"\\" for lab, (a, b, c, e) in fl.items())
pen = [v["depth_mm_of_penetrating"] for v in V1["penetration_depth"].values() if v.get("depth_mm_of_penetrating", {}).get("n")]
M("penMedMin", d(min(p["median"] for p in pen), 2)); M("penMedMax", d(max(p["median"] for p in pen), 2))
M("penMax", d(max(p["max"] for p in pen), 2))
ur5 = G[names[1]]
M("urFiveVOneSolved", frac(ur5["curobo_v1"]["success"], ur5["curobo_v1"]["queries"]))

# ---------------------------------------------------------------- v2
S2 = V2["sets"]
for key, nm in (("heldout_a6", "A"), ("mbm", "Mbm"), ("mpinets", "Mpi"), ("heldout", "Orig")):
    f = S2[key]["feasibility"]
    M(f"feas{nm}", f["certified"]); M(f"feasUnres{nm}", f["unresolved"]); M(f"feasQ{nm}", f["certified"] + f["unresolved"])
A = S2["heldout_a6"]["planners"]
rows = []
for lab, fn in (("Success (runs)", lambda p: pct(p["success_rate_runs"])),
                ("Queries failing on some seed / every seed", lambda p: f"{p['queries_failing_some_seed']} / {p['queries_failing_all_seeds']}"),
                ("Median T\\textsubscript{ready}, solved runs", lambda p: tm(p["t_ready_ms_median_runs"])),
                ("Median T\\textsubscript{exec}, solved runs", lambda p: f"{d(p['t_exec_s_median_runs'], 2)}\\,s"),
                ("Median T\\textsubscript{cycle} (failures at T)", lambda p: f"{d(p['t_cycle_med_of_query_meds'], 2)}\\,s")):
    rows.append(lab + " & " + " & ".join(fn(A[p]["all_queries"]) for p in ("vamp", "curobo1", "curobo2")) + r" \\")
tables["tab_v2_heldout"] = "\n".join(rows)
a6 = HYP["heldout_a6"]
M("vTwoQueries", A["vamp"]["all_queries"]["queries"])
M("vTwoSeeds", A["vamp"]["all_queries"]["runs"] // A["vamp"]["all_queries"]["queries"])
M("vTwoSubQ", A["curobo2"]["all_queries"]["queries"]); M("vTwoSubSeeds", A["curobo2"]["all_queries"]["runs"] // A["curobo2"]["all_queries"]["queries"])
M("aVampSucc", pct(A["vamp"]["all_queries"]["success_rate_runs"])); M("aVOneSucc", pct(A["curobo1"]["all_queries"]["success_rate_runs"]))
M("aVTwoSucc", pct(A["curobo2"]["all_queries"]["success_rate_runs"]))
M("aSubVamp", pct(a6["H4"]["v2_subsample"]["vamp"]["success_rate_runs"])); M("aSubVOne", pct(a6["H4"]["v2_subsample"]["curobo1"]["success_rate_runs"]))
M("aVampCycle", f"{d(A['vamp']['all_queries']['t_cycle_med_of_query_meds'], 2)}\\,s")
M("aVOneCycle", f"{d(A['curobo1']['all_queries']['t_cycle_med_of_query_meds'], 2)}\\,s")
M("aBothVamp", f"{d(a6['H2']['both_solved_runs']['vamp_t_cycle_s_median'], 2)}\\,s")
M("aBothVOne", f"{d(a6['H2']['both_solved_runs']['curobo1_t_cycle_s_median'], 2)}\\,s")
M("aExecAdv", f"{d(a6['H2']['curobo1_t_exec_advantage_s'], 2)}\\,s"); M("aReadyAdv", f"{d(a6['H2']['vamp_t_ready_advantage_s'], 2)}\\,s")
rr = {k: HYP[k]["H2"]["t_ready_ratio_curobo1_over_vamp"] for k in ("heldout_a6", "mbm", "mpinets")}
M("readyRatioA", ratio(rr["heldout_a6"])); M("readyRatioMbm", ratio(rr["mbm"])); M("readyRatioMpi", ratio(rr["mpinets"]))
M("readyRatioMin", ratio(min(rr.values()))); M("readyRatioMax", ratio(max(rr.values())))
yn = lambda b: "yes" if b else "no"
NM = {"vamp": "VAMP", "curobo1": "cuRobo v1"}
rows = []
H = [HYP[k] for k in ("heldout_a6", "mbm", "mpinets")]
rows.append("H1: VAMP's T\\textsubscript{ready} lower in every bin & " + " & ".join(yn(h["H1"]["holds_every_bin"]) for h in H) + r" \\")
rows.append("T\\textsubscript{ready} ratio, cuRobo v1 / VAMP & " + " & ".join(ratio(h["H2"]["t_ready_ratio_curobo1_over_vamp"]) for h in H) + r" \\")
rows.append("H2: cycle-time gap smaller (T\\textsubscript{cycle} ratio) & " + " & ".join(f"{yn(h['H2']['holds'])} ({d(h['H2']['t_cycle_ratio_curobo1_over_vamp'], 2)})" for h in H) + r" \\")
rows.append("T\\textsubscript{cycle}, runs both solve, VAMP / v1 & " + " & ".join(f"{d(h['H2']['both_solved_runs']['vamp_t_cycle_s_median'], 2)} / {d(h['H2']['both_solved_runs']['curobo1_t_cycle_s_median'], 2)}\\,s" for h in H) + r" \\")
rows.append("H3: v1 lower jerk / shorter T\\textsubscript{exec} & " + " & ".join(f"{yn(h['H3']['curobo1_lower_jerk'])} / {yn(h['H3']['curobo1_shorter_exec'])}" for h in H) + r" \\")
rows.append("VAMP + TOPP-RA: T\\textsubscript{exec}, share valid & " + " & ".join(f"{d(h['H3']['t_exec_s']['vamp_p2'], 2)}\\,s, {pct(h['H3']['p2_validity_rate_of_planned'], 0)}" for h in H) + r" \\")
rows.append("H4: queries failing on some seed, VAMP / v1 & " + " & ".join(f"{h['H4']['vamp']['queries_failing_some_seed']} / {h['H4']['curobo1']['queries_failing_some_seed']}" for h in H) + r" \\")
rows.append("H4: more robust & " + " & ".join(NM[h["H4"]["more_robust"]] for h in H) + r" \\")
tables["tab_v2_hyp"] = "\n".join(rows)
p2v = [h["H3"]["p2_validity_rate_of_planned"] for h in H]
M("toppraValidMin", pct(min(p2v), 0)); M("toppraValidMax", pct(max(p2v), 0))
h5 = lambda k: k.replace("_same_sign", "").replace("H3_jerk", "H3 (jerk)").replace("H3_exec", "H3 (execution time)")
M("hFiveHolds", ", ".join(h5(k) for k, v in HYP["H5"].items() if v))
M("hFiveFails", ", ".join(h5(k) for k, v in HYP["H5"].items() if not v))
sub = {k: HYP[k]["H4"]["v2_subsample"] for k in ("heldout_a6", "mbm", "mpinets")}
vr = [s["curobo2"]["t_ready_ms_median_runs"] / s["curobo1"]["t_ready_ms_median_runs"] for s in sub.values()]
M("vTwoOverVOneMin", ratio(min(vr))); M("vTwoOverVOneMax", ratio(max(vr)))

# pick-place
R = PP["runs"]
M("ppCells", PP["cells"]); M("ppPicksPerRun", PP["picks_per_run"]); M("ppPicks", R["vamp"]["picks"])
M("ppSeeds", R["vamp"]["picks"] // R["vamp"]["P3_pick_pairs"]); M("ppSegments", PROTOCOL["pp_segments"])
M("ppPerCell", R["vamp"]["P3_pick_pairs"] // PP["cells"])
M("ppJerkLow", re.search(r"max_jerk (\d+)", R["curobo1_J1b"]["role"]).group(1))
cols = (("vamp", "VAMP"), ("curobo1", "v1 as configured"), ("curobo1_J1b", "v1, max\\_jerk 200"), ("curobo2", "cuRoboV2"))
opt = lambda x, n=2, u="\\,s": "--" if x is None else f"{d(x, n)}{u}"
rows = [
    "Picks completed & " + " & ".join(frac(R[k]["success"], R[k]["picks"]) for k, _ in cols) + r" \\",
    "Pick slots failing on some seed & " + " & ".join(frac(R[k]["P3_picks_failing_some_seed"], R[k]["P3_pick_pairs"]) for k, _ in cols) + r" \\",
    "Median cycle per completed pick & " + " & ".join(opt(R[k]["P2_t_cycle_s_median"]) for k, _ in cols) + r" \\",
    "\\quad planning & " + " & ".join(opt(R[k]["P2_t_ready_s_median"]) for k, _ in cols) + r" \\",
    "\\quad execution & " + " & ".join(opt(R[k]["P2_t_exec_s_median"]) for k, _ in cols) + r" \\",
    "MoveL TCP deviation, median / p95 (mm) & " + " & ".join(f"{d(R[k]['P1_tcp_dev_mm_median'], 2)} / {d(R[k]['P1_tcp_dev_mm_p95'], 2)}" for k, _ in cols) + r" \\",
    "MoveL orientation deviation, p95 & " + " & ".join(f"{d(R[k]['P1_orient_dev_deg_p95'], 2)}$^\\circ$" for k, _ in cols) + r" \\",
]
tables["tab_pickplace"] = "\n".join(rows)
tables["tab_pickplace_head"] = " & ".join(lab for _, lab in cols)
M("ppVampDone", frac(R["vamp"]["success"], R["vamp"]["picks"])); M("ppVTwoDone", frac(R["curobo2"]["success"], R["curobo2"]["picks"]))
M("ppVOneDone", frac(R["curobo1"]["success"], R["curobo1"]["picks"])); M("ppJDone", frac(R["curobo1_J1b"]["success"], R["curobo1_J1b"]["picks"]))
M("ppVampCycle", opt(R["vamp"]["P2_t_cycle_s_median"])); M("ppVTwoCycle", opt(R["curobo2"]["P2_t_cycle_s_median"]))
M("ppJCycle", opt(R["curobo1_J1b"]["P2_t_cycle_s_median"]))
pv = PP["P2_paired"]["vamp_vs_curobo2"]; pj = PP["P2_paired"]["vamp_vs_curobo1_J1b"]
M("ppPairTwoN", pv["picks"]); M("ppPairTwoVamp", opt(pv["vamp_t_cycle_s_median"])); M("ppPairTwo", opt(pv["curobo2_t_cycle_s_median"]))
M("ppPairTwoRatio", d(pv["ratio_t_cycle_median"], 2))
M("ppPairJN", pj["picks"]); M("ppPairJVamp", opt(pj["vamp_t_cycle_s_median"])); M("ppPairJ", opt(pj["curobo1_J1b_t_cycle_s_median"]))
M("ppPairJRatio", d(pj["ratio_t_cycle_median"], 2))
m1 = R2MOVEL["planners"]["curobo1_as_configured"]
M("ppRTwoDone", frac(m1["picks"]["R2_completed"], m1["picks"]["n"])); M("ppRTwoCycle", opt(m1["picks"]["cycle_R2_s"]["median"]))
M("ppRTwoSlow", d(m1["slowdown_s_rescued"]["median"], 2))
M("ppMoveLPassOne", frac(m1["R1_gate"], m1["moveL_attempted"]))

# ---------------------------------------------------------------- v2.2
ps = V22["program_set"]; el = V22["expressibility_lines"]; P22 = V22["planners"]
M("ttCells", ps["cells"]); M("ttCandidates", ps["candidates"]); M("ttAccept", pct(ps["acceptance"], 0))
M("ttViasMin", min(int(k) for k in ps["w_vias"])); M("ttViasMax", max(int(k) for k in ps["w_vias"]))
M("ttViasThree", ps["w_vias"]["3"]); M("ttViasFour", ps["w_vias"]["4"]); M("ttViasFive", ps["w_vias"]["5"]); M("ttViasSix", ps["w_vias"]["6"])
M("ttSweepMed", f"{ps['c_sweep_deg']['median']:.0f}"); M("ttSweepMin", f"{ps['c_sweep_deg']['min']:.0f}"); M("ttSweepMax", f"{ps['c_sweep_deg']['max']:.0f}")
M("ttRadiusMed", d(ps["c_radius_m_median"], 2))
M("ttTLenMin", d(ps["path_length_m"]["T"]["min"], 2)); M("ttTLenMax", d(ps["path_length_m"]["T"]["max"], 2)); M("ttTLenMed", d(ps["path_length_m"]["T"]["median"], 2))
M("ttV", d(ps["v_mps"]["W"], 2)); M("ttA", d(ps["a_mps2"]["W"], 1)); M("ttTA", d(ps["a_mps2"]["T"], 2))
M("ttClearW", ps["clearance_below_5cm"]["W"]); M("ttClearC", ps["clearance_below_5cm"]["C"]); M("ttClearT", ps["clearance_below_5cm"]["T"])
M("ttPathLimit", PROTOCOL["path_limit_mm"]); M("ttEndMm", PROTOCOL["end_mm"]); M("ttEndDeg", PROTOCOL["end_deg"])
M("ttBlendR", PROTOCOL["blend_r_cm"]); M("ttTcpJerk", PROTOCOL["tcp_jerk"]); M("ttInner", d(PROTOCOL["inner_m"], 2)); M("ttOuter", d(PROTOCOL["outer_m"], 2))
M("ttAxisHold", PROTOCOL["axis_hold_mm"]); M("ttBinMin", PROTOCOL["q_bins_min"]); M("ttKnotDt", PROTOCOL["knot_dt_ms"])
M("ttLinesW", el["W"]["lines"]); M("ttLinesWPlane", el["W"]["plane"]); M("ttLinesWOrient", el["W"]["orient"]); M("ttLinesWNative", el["W"]["native"])
M("ttLinesT", el["T"]["lines"]); M("ttLinesTPlane", el["T"]["plane"]); M("ttLinesTOrient", el["T"]["orient"]); M("ttLinesTNative", el["T"]["native"])
M("ttLinesAll", el["all_lines"]); M("ttLinesAllNative", el["all_native"])
PL = (("cartesian", "Cartesian pipeline"), ("curobo1", "cuRobo v1"), ("curobo1_jerk200", "v1, max\\_jerk 200"), ("curobo2", "cuRoboV2"), ("pyroki", "PyRoki"))
tables["tab_tt_head"] = " & ".join(lab for _, lab in PL)
M("ttSeedsVOne", P22["curobo1"]["T"]["runs"] // P22["curobo1"]["T"]["programs"]); M("ttSeedsVTwo", P22["curobo2"]["T"]["runs"] // P22["curobo2"]["T"]["programs"])


def cell(p, t, fn, na="--"):
    s = P22[p][t]
    return fn(s) if s.get("runs") else na


def runs_frac(key):
    return lambda s: frac(s[key], s["runs"])


def mm2(q):
    return lambda s: "--" if s[q]["dev_outside_mm_median"] is None else f"{d(s[q]['dev_outside_mm_median'], 2)} / {d(s[q]['dev_outside_mm_p95'], 2)}"


for t in "WCT":
    rows = [
        "Success, as returned (R1) & " + " & ".join(cell(p, t, runs_frac("successes_runs"), "not expressible") for p, _ in PL) + r" \\",
        "Success after slow-down (R2) & " + " & ".join(cell(p, t, runs_frac("r2_successes_runs")) for p, _ in PL) + r" \\",
        "Programs failing on every seed & " + " & ".join(cell(p, t, lambda s: str(s["programs_failing_every_seed"])) for p, _ in PL) + r" \\",
        "Max path deviation, all runs, median / p95 (mm) & " + " & ".join(cell(p, t, mm2("Q1")) for p, _ in PL) + r" \\",
        "Cruise speed / programmed, median & " + " & ".join(cell(p, t, lambda s: d(s["Q2"]["cruise_speed_ratio_median"], 2)) for p, _ in PL) + r" \\",
    ]
    if t == "W":
        rows.append("Via points: median min speed / programmed & " + " & ".join(cell(p, t, lambda s: d(s["Q3"]["min_speed_ratio_median"], 2)) for p, _ in PL) + r" \\")
        rows.append("Via points stopped / held & " + " & ".join(cell(p, t, lambda s: f"{pct(s['Q3']['share_stopped'], 0)} / {pct(s['Q3']['share_held'], 0)}") for p, _ in PL) + r" \\")
    if t == "T":
        rows.append("Ramp acceleration error, median & " + " & ".join(cell(p, t, lambda s: pct(s["Q4"]["ramp_acc_err_median"], 0)) for p, _ in PL) + r" \\")
        rows.append("Ramp duration / programmed, median & " + " & ".join(cell(p, t, lambda s: d(s["Q4"]["ramp_duration_ratio_median"], 2)) for p, _ in PL) + r" \\")
        rows.append("Median T\\textsubscript{cycle}, successful runs & " + " & ".join(cell(p, t, lambda s: "--" if s["Q5"]["t_cycle_s"] is None else f"{d(s['Q5']['t_cycle_s']['median'], 2)}\\,s") for p, _ in PL) + r" \\")
    rows.append("Median planning time, all runs & " + " & ".join(cell(p, t, lambda s: f"{d(s['t_ready_s_all']['median'], 2)}\\,s") for p, _ in PL) + r" \\")
    tables[f"tab_tt_{t}"] = "\n".join(rows)


def g(p, t, *path):
    x = P22[p][t]
    for k in path:
        x = x[k]
    return x


cart = lambda t, *k: g("cartesian", t, *k)
M("ttCartDevPNinetyFive", d(max(cart(t, "Q1", "dev_outside_mm_p95") for t in "WCT"), 2))
M("ttCartHalvedW", sum(v for k, v in cart("W", "speed_halvings").items() if k != "0"))
M("ttCartHalvedC", sum(v for k, v in cart("C", "speed_halvings").items() if k != "0"))
M("ttCartReadyMin", d(min(cart(t, "t_ready_s_all", "median") for t in "WCT"), 2)); M("ttCartReadyMax", d(max(cart(t, "t_ready_s_all", "median") for t in "WCT"), 2))
M("ttCartCycleW", d(cart("W", "Q5", "t_cycle_s", "median"), 2))
cs = [g(p, "T", "Q2", "cruise_speed_ratio_median") for p in ("curobo1", "curobo1_jerk200", "curobo2")]
M("ttCuroboSpeedMin", d(min(cs), 1)); M("ttCuroboSpeedMax", d(max(cs), 1))
ws = [g(p, "W", "Q2", "cruise_speed_ratio_median") for p in ("curobo1", "curobo1_jerk200", "curobo2")]
M("ttCuroboWSpeedMin", d(min(ws), 1)); M("ttCuroboWSpeedMax", d(max(ws), 1))
dv = [g(p, "T", "Q1", "dev_outside_mm_median") for p in ("curobo1", "curobo1_jerk200", "curobo2")]
M("ttCuroboDevMin", d(min(dv), 1)); M("ttCuroboDevMax", d(max(dv), 1))
dvs = [g(p, "T", "Q1_success", "dev_outside_mm_median") for p in ("curobo1", "curobo1_jerk200", "curobo2")]
M("ttCuroboDevSuccMin", d(min(dvs), 1)); M("ttCuroboDevSuccMax", d(max(dvs), 1))
rd = [g(p, "T", "Q4", "ramp_duration_ratio_median") for p in ("curobo1", "curobo1_jerk200", "curobo2")]
M("ttCuroboRampMin", d(min(rd), 2)); M("ttCuroboRampMax", d(max(rd), 2))
for p, nm in (("curobo1_jerk200", "J"), ("curobo2", "Two")):
    b = g(p, "T", "bins", "path_length")
    M(f"ttLen{nm}Short", pct(b["short"]["success_rate_runs"], 0)); M(f"ttLen{nm}Long", pct(b["long"]["success_rate_runs"], 0))
    pp = g(p, "T", "Q5_paired_vs_cartesian")
    M(f"ttPair{nm}N", pp["programs"]); M(f"ttPair{nm}", d(pp["t_cycle_s_median"], 2)); M(f"ttPair{nm}Cart", d(pp["cartesian_t_cycle_s_median"], 2))
M("ttVOneTJerkFail", g("curobo1", "T", "fail_reasons", "jerk")); M("ttVOneTFail", g("curobo1", "T", "fail_reasons", "failed_runs"))
M("ttVOneTDynOnly", g("curobo1", "T", "fail_reasons", "dynamics_only"))
M("ttVOneWJerkFail", g("curobo1", "W", "fail_reasons", "jerk"))
py = lambda t, *k: g("pyroki", t, *k)
M("ttPyReadyW", d(py("W", "t_ready_s_all", "median"), 1)); M("ttPyOverW", py("W", "t_ready_s_all", "over_30s"))
M("ttPyJerkW", py("W", "fail_reasons", "jerk")); M("ttPyFailW", py("W", "fail_reasons", "failed_runs"))
M("ttPyJerkC", py("C", "fail_reasons", "jerk")); M("ttPyFailC", py("C", "fail_reasons", "failed_runs"))
M("ttPyScaleW", d(py("W", "R2", "scale_median"), 2)); M("ttPyScaleC", d(py("C", "R2", "scale_median"), 2))
M("ttPyBlendW", d(py("W", "Q3", "min_speed_ratio_median"), 2))
M("ttPyDevT", d(py("T", "Q1", "dev_outside_mm_median"), 2)); M("ttPyRampT", pct(py("T", "Q4", "ramp_acc_err_median"), 1))
M("ttPySuccT", frac(py("T", "successes_runs"), py("T", "runs"))); M("ttPyRTwoW", frac(py("W", "r2_successes_runs"), py("W", "runs")))
M("ttPyRTwoC", frac(py("C", "r2_successes_runs"), py("C", "runs")))
M("ttPyReadyT", d(py("T", "Q5", "t_ready_s", "median"), 2)); M("ttPyExecT", d(py("T", "Q5", "t_exec_s", "median"), 2))
M("ttCartSuccAll", frac(sum(cart(t, "successes_runs") for t in "WCT"), sum(cart(t, "runs") for t in "WCT")))
for p, nm in (("curobo1", "VOne"), ("curobo1_jerk200", "J"), ("curobo2", "VTwo")):
    M(f"ttT{nm}", frac(g(p, "T", "successes_runs"), g(p, "T", "runs"))); M(f"ttW{nm}", frac(g(p, "W", "successes_runs"), g(p, "W", "runs")))
M("ttWOneProgram", P22["curobo2"]["W"]["programs"] - P22["curobo2"]["W"]["programs_failing_every_seed"])
M("ttStopShare", pct(min(g(p, "W", "Q3", "share_stopped") for p in ("curobo1", "curobo1_jerk200", "curobo2")), 0))

# ---------------------------------------------------------------- v2.1 (provisional)
mp, mr = V21["movel_preregistered"], V21["movel_recipe_7g"]
M("moSegs", mr["segments"]); M("moRecipe", frac(mr["R2_success"], mr["segments"])); M("moPre", frac(mp["R2_success"], mp["segments"]))
M("moROne", frac(mr["R1_success"], mr["segments"])); M("moEndRecipe", frac(mr["end_ok"], mr["segments"])); M("moEndPre", frac(mp["end_ok"], mp["segments"]))
M("moEndMedRecipe", d(mr["end_pos_err_mm_median"], 2)); M("moEndMedPre", d(mp["end_pos_err_mm_median"], 1))
M("moStartOff", mr["start_off"]); M("moStartOnly", mr["start_off_only_reason"]); M("moGateFail", mr["R2_gate_fail"])
M("moSolve", d(mr["t_solve_s_median"], 2)); M("moStartTol", PROTOCOL["start_tol_rad"].replace("1e-3", "$10^{-3}$"))
M("moPicks", R["vamp"]["P3_pick_pairs"])
ikp, ikc = V21["ik_pyroki"], V21["ik_curobo1"]
rows = [f"PyRoki, default initial configuration & {frac(ikp['default']['success'], ikp['poses'])} & {tm(ikp['default']['t_s_median'] * 1000)} \\\\",
        f"PyRoki, {PROTOCOL['ik_starts']} random starts & {frac(ikp['rand64']['success'], ikp['poses'])} & {tm(ikp['rand64']['t_s_median'] * 1000)} \\\\",
        f"PyRoki, collision-aware, {PROTOCOL['ik_starts']} random starts & {frac(ikp['coll_rand64']['success'], ikp['poses'])} & {tm(ikp['coll_rand64']['t_s_median'] * 1000)} \\\\",
        f"cuRobo v1 IKSolver, default & {frac(ikc['default']['success'], ikc['poses'])} & {tm(ikc['default']['t_s_median'] * 1000)} \\\\",
        f"cuRobo v1 IKSolver, high precision & {frac(ikc['high_precision']['success'], ikc['poses'])} & {tm(ikc['high_precision']['t_s_median'] * 1000)} \\\\"]
tables["tab_ik"] = "\n".join(rows)
ms_ = V21["ik_margin_sensitivity_m0"]
M("ikFailFive", ms_["failed_at_5mm"]); M("ikZeroOk", ms_["succeed_at_0"]); M("ikCtrl", ms_["controls"]); M("ikCtrlColl", ms_["controls_collide_at_0"])
M("pyMargin", PROTOCOL["pyroki_margin_mm"]); M("pyKnots", PROTOCOL["pyroki_knots"]); M("ikStarts", PROTOCOL["ik_starts"])
s1, s2 = V21["trajopt_s1"], V21["trajopt_s2"]
rows = [f"S1: straight joint-space line & {frac(s1['R1_success'], s1['queries'])} & {frac(s1['R2_success'], s1['queries'])} & {d(s1['t_ready_s_median_R2_success'], 1)}\\,s \\\\",
        f"S2: cuRobo v1's seeds, in v1's order & {frac(s2['R1_success'], s2['queries'])} & {frac(s2['R2_success'], s2['queries'])} & {d(s2['t_ready_s_median_R2_success'], 1)}\\,s \\\\"]
tables["tab_trajopt"] = "\n".join(rows)
cs_ = V21["curobo1_at_seed_dump"]; M("vOneSeedDump", frac(cs_["success"], cs_["queries"]))
mc = V21["margin_choice"]
M("marginRule", mc["rule"]); M("marginChosen", f"{1000 * mc['chosen_m']:.0f}")
M("marginQueries", next(iter(mc["results"].values()))["queries"])
M("marginList", ", ".join(f"{1000 * float(k):.0f}\\,mm: {v['R1']}" for k, v in mc["results"].items()))


def render():
    out = {"numbers.tex": "% generated by report/make_numbers.py from the committed result files; do not edit\n" +
           "".join(f"\\newcommand{{\\{k}}}{{{v}}}\n" for k, v in macros.items())}
    for k, v in tables.items():
        # table bodies end with \bottomrule here: after \input, a rule in the main file is a misplaced \noalign
        out[f"{k}.tex"] = "% generated by report/make_numbers.py; do not edit\n" + v + ("" if k.endswith("_head") else "\n\\bottomrule") + "\n"
    return out


if __name__ == "__main__":
    files = render()
    if "--check" in sys.argv:
        bad = [f for f, txt in files.items() if not (GEN / f).exists() or (GEN / f).read_text() != txt]
        bad += [p.name for p in GEN.glob("*.tex") if p.name not in files]
        print("report/generated up to date" if not bad else "report/generated out of date: " + ", ".join(bad))
        sys.exit(1 if bad else 0)
    GEN.mkdir(parents=True, exist_ok=True)
    for p in GEN.glob("*.tex"):
        p.unlink()
    for f, txt in files.items():
        (GEN / f).write_text(txt)
    print(f"wrote {len(macros)} macros and {len(tables)} tables to {GEN.relative_to(ROOT)}")
