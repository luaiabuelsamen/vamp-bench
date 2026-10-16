"""Benchmark v2.2 figure: path deviation and cruise speed per planner and program type, with success counts.

Reads results/v2_2/raw/*.jsonl (through bench/v2_2_analyze.normalise) and results/v2_2/summary.json.
Writes results/v2_2/motion_types.png.
"""
import json, sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

sys.path.insert(0, str(Path(__file__).parent))
import v2_2_analyze as A

ROOT = A.ROOT
ROWS = [("cartesian", "Cartesian pipeline (ours)"), ("curobo1", "cuRobo v1 (as configured)"),
        ("curobo1_jerk200", "cuRobo v1, max_jerk 200"), ("curobo2", "cuRoboV2"), ("pyroki", "PyRoki")]
COL = {"cartesian": "#2a78b5", "curobo1": "#b8442a", "curobo1_jerk200": "#e0956f", "curobo2": "#4a9a5b", "pyroki": "#7a5195"}
NAMES = {"W": "W  waypoints + blends", "C": "C  circular arc (MoveC)", "T": "T  line, trapezoidal profile"}


def main():
    S = json.load(open(ROOT / "results/v2_2/summary.json"))["planners"]
    data = {p: [A.normalise(p, json.loads(l)) for l in open(A.RAW / f)] for p, f in A.RUNS.items() if (A.RAW / f).exists()}
    fig, axes = plt.subplots(1, 2, figsize=(11, 6.4), dpi=150, sharey=True, gridspec_kw={"width_ratios": [1.15, 1]})
    rng = np.random.default_rng(0)
    ylab, y = [], 0
    ypos = {}
    for t in "WCT":
        for p, lab in ROWS:
            ypos[(t, p)] = y; ylab.append(lab); y -= 1
        y -= 0.8
    for (t, p), yy in ypos.items():
        rs = [r for r in data.get(p, []) if r["type"] == t and r["metrics"] is not None]
        s = S.get(p, {}).get(t, {})
        for ax, key, clip in ((axes[0], "max_dev_outside_mm", (0.01, 2000)), (axes[1], "cruise_speed_ratio", (0.1, 10))):
            v = np.clip([r["metrics"][key] for r in rs if r["metrics"][key] is not None], *clip)
            if len(v) == 0:
                ax.text(clip[0] * 1.3, yy, "not expressible: not run" if not s.get("runs") else "no trajectory", va="center",
                        fontsize=7.5, color="#777", style="italic")
                continue
            ax.scatter(v, yy + rng.uniform(-0.3, 0.3, len(v)), s=4, alpha=0.3, color=COL[p], linewidths=0, rasterized=True)
            ax.boxplot(v, positions=[yy], vert=False, widths=0.62, showfliers=False, medianprops={"color": "#222"},
                       boxprops={"color": "#444"}, whiskerprops={"color": "#444"}, capprops={"color": "#444"})
        if s.get("runs"):
            axes[1].text(14, yy, f"R1 {s['success_rate_runs'] * 100:.0f}%   R2 {s['r2_success_rate_runs'] * 100:.0f}%",
                         va="center", fontsize=7.5, color="#333")
    for t in "WCT":
        top = ypos[(t, "cartesian")]
        axes[0].text(0.011, top + 0.75, NAMES[t], fontsize=9, weight="bold", va="center")
    axes[0].axvline(5, color="#222", lw=1.1, ls="--"); axes[0].text(5.4, 0.7, "5 mm success limit", fontsize=7.5)
    axes[1].axvline(1, color="#222", lw=1.1, ls="--"); axes[1].text(1.06, 0.7, "programmed speed", fontsize=7.5)
    axes[0].set_xscale("log"); axes[1].set_xscale("log")
    axes[0].set_xlim(0.01, 2000); axes[1].set_xlim(0.1, 10)
    axes[0].set_xlabel("max TCP distance from the programmed path,\noutside blend zones (mm, log scale)")
    axes[1].set_xlabel("mean TCP cruise speed as returned (R1)\n÷ programmed 0.25 m/s (log scale)")
    axes[0].set_yticks(list(ypos.values())); axes[0].set_yticklabels(ylab, fontsize=8)
    for ax in axes:
        for sp in ("top", "right"):
            ax.spines[sp].set_visible(False)
    fig.suptitle("UR5e motion programs (100 per type): path fidelity and TCP speed, every run with a trajectory", fontsize=10.5)
    fig.text(0.01, 0.01, "Success (right): R1 as returned, R2 after a uniform slow-down; both need the 1 kHz gate, end pose ≤ 1 mm / 1°, "
             "path ≤ 5 mm outside blends, orientation ≤ 1°, planning ≤ 30 s.\ncuRobo runs W as one line per leg (stops at each via "
             "point). Values outside the axes are drawn at the edge. R2 divides a run's speed by its slow-down factor.", fontsize=7, color="#555")
    fig.tight_layout(rect=(0, 0.05, 0.93, 0.97), w_pad=3); fig.savefig(ROOT / "results/v2_2/motion_types.png"); plt.close(fig)


if __name__ == "__main__":
    main()
