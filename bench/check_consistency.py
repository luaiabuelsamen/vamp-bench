"""Every count and time in README.md and docs/EMAIL_NOTES.md must exist in results/summary.json (STANDARD #7).

Checks:
  - every "a/b" success count in a sentence or table row matches a (success, queries) pair, a flagged count
    (flagged/success), or the collision-model agreement total in summary.json;
  - every time value "X ms" / "X s" matches some statistic in summary.json at the precision written.
The figure is drawn from summary.json by bench/analyze.py, so it is consistent by construction.
Exit code 1 on any mismatch.
"""
import json, re, sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
S = json.load(open(ROOT / "results/summary.json"))
pairs, times = set(), []
for g in S["groups"]:
    for k in ("vamp", "curobo_v1", "curobo_v2"):
        v = g.get(k)
        if not v:
            continue
        pairs.add((v["success"], v["queries"]))
        f = v.get("success_flagged_by_vamp_segment_check_PROVISIONAL")
        if f is not None:
            pairs.add((f, v["success"])); pairs.add((v["success"] - f, v["queries"]))
        for st in ("wall_ms", "internal_ms", "plan_only_wall_ms"):
            if v.get(st):
                times += [x for kk, x in v[st].items() if kk != "n"]
    for k in ("curobo_v1_on_v2_queries",):
        if g.get(k):
            pairs.add((g[k]["success"], g[k]["queries"])); times += [x for kk, x in g[k]["wall_ms"].items() if kk != "n"]
    for k in ("curobo_v1_vs_vamp_same_queries", "curobo_v2_vs_vamp_same_queries"):
        if g.get(k):
            vs = g[k]["vamp_on_same_queries"]; pairs.add((vs["success"], vs["queries"]))
            times += [x for kk, x in vs["wall_ms"].items() if kk != "n"]
for v in S.get("superseded", {}).values():
    pairs.add((v["success"], v["queries"]))
for g in S["groups"]:
    for k in ("vamp", "curobo_v1", "curobo_v2"):
        if g.get(k) and g[k].get("failures", {}).get("time_to_failure_ms"):
            times += [x for kk, x in g[k]["failures"]["time_to_failure_ms"].items() if kk != "n"]
vc = S.get("v2_collision_check")
if vc:
    for a, b in ((vc["flagged_confirmed_by_v2_checker"], vc["flagged"]), (vc["flagged_same_samples_as_vamp"], vc["flagged"]),
                 (vc["controls_clean_both"], vc["controls"]), (vc["fresh_flagged_again"], vc["fresh_replans"]),
                 (vc["over_1mm"], vc["flagged"]), (vc["over_2p5mm"], vc["flagged"])):
        pairs.add((a, b))
    n2 = sum(v["n"] for v in vc["random_config_agreement"].values()); pairs.add((n2, n2))
    times += [dt * 1000 for dt in vc.get("interpolation_dt_s", [])]  # settings quoted in docs (ms)
agree = S.get("collision_model_agreement", {})
n = sum(v["n"] for v in agree.values()); pairs.add((n, n))
# successes checked by the segment check with zero flagged (e.g. v1 0 of 638)
pairs |= {(0, a) for a, _ in list(pairs)}


def matches_time(val, unit, digits):
    ms = val * (1000 if unit == "s" else 1)
    tol = 0.5 * 10 ** (-digits) * (1000 if unit == "s" else 1)
    return any(abs(ms - t) <= tol + 1e-9 for t in times)


bad = []
for doc in ("README.md", "docs/EMAIL_NOTES.md"):
    text = (ROOT / doc).read_text()
    for ln, line in enumerate(text.splitlines(), 1):
        if line.strip().startswith(("```", "#", "cp ", "git ", "CMAKE", "(cd")) or "github.com" in line:
            continue
        for a, b in re.findall(r"(?<![\d.,])(\d[\d,]*)\s*/\s*(\d[\d,]*)(?![\d.])", line):
            a, b = int(a.replace(",", "")), int(b.replace(",", ""))
            if (a, b) not in pairs:
                bad.append(f"{doc}:{ln}: count {a}/{b} not in summary.json")
        for m in re.finditer(r"(?<![\w.])(\d+(?:\.(\d+))?)\s*(ms|s)\b", line):
            val, dec, unit = float(m.group(1)), m.group(2), m.group(3)
            if "rad/s" in line[m.start():m.end() + 4] or "°/s" in line:
                continue
            if not matches_time(val, unit, len(dec) if dec else 0):
                bad.append(f"{doc}:{ln}: time {m.group(0)} not in summary.json")
print("\n".join(bad) if bad else "consistent: every count and time in README.md and docs/EMAIL_NOTES.md is in results/summary.json")
sys.exit(1 if bad else 0)
