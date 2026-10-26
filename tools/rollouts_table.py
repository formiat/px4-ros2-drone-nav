#!/usr/bin/env python3
"""rollouts_table.py SERIES_FILE: the table of roadmap item 28 step 1 from log/tools/board/rollouts_series.txt (the
lines "<profile> <count> <i> <run> ..." the series chain writes): per flight the mean speed, B by the truth, the
no-route hold ratio, the production tick p50/p95, the GPU utilisation p50, the time under 1 m/s and the limiters,
the true clearance's least and seconds under 0.10/0.20 m (tools/body_truth_clearance.py), the contacts; per group
the minimum and mean speed, the least clearance, the most seconds under 0.10 m, the hold ratio's mean, and the
verdict against the group's own 8192 (the rule: no contact, B <= 2 m, mean speed >= 8192's mean - 5 percent, least
clearance >= 8192's - 0.05 m, seconds under 0.10 m <= 8192's). Host python, repository root."""
import re, subprocess, sys
import numpy as np

rows = {}
for line in open(sys.argv[1]):
    m = re.match(r"(cameras|lidar) (\d+) (\d) (r\d+)", line)
    if m:
        rows.setdefault((m.group(1), int(m.group(2))), []).append(m.group(4))


def number(text, pattern):
    m = re.search(pattern, text)
    return float(m.group(1)) if m else float("nan")


def flight(run):
    val = open(f"log/tools/run_{run}.log", errors="ignore").read()
    lim = subprocess.run(["python3", "tools/speed_limiters.py", run], capture_output=True, text=True).stdout
    clr = subprocess.run(["python3", "tools/body_truth_clearance.py", run], capture_output=True, text=True).stdout
    limiters = re.search(r"limiters (.*?); clearance", lim)
    c = re.search(r"least ([\d.]+) m .*under 0.10 m ([\d.]+) s, under 0.20 m ([\d.]+) s", clr)
    return dict(run=run,
                speed=number(val, r"mean flight speed (?:is |exceeds [\d.]+ m/s \()([\d.]+) m/s"),
                b=number(val, r"the true position is ([\d.]+) m from the goal"),
                hold=number(val, r"no-route hold ratio is ([\d.]+)%"),
                tick50=number(val, r"production tick wall time is ([\d.]+) ms at p50"),
                tick95=number(val, r"production tick wall time is [\d.]+ ms at p50 and ([\d.]+) ms at p95"),
                gpu=number(val, r"GPU utilisation is (\d+)% at p50"),
                contact=0 if "OK: no crash was reported" in val else 1,
                fails=[l[6:60] for l in val.splitlines() if l.startswith("FAIL")],
                under1=number(lim, r"under 1 m/s (\d+)%"),
                limiters=limiters.group(1) if limiters else "?",
                least=float(c.group(1)) if c else float("nan"),
                s10=float(c.group(2)) if c else float("nan"),
                s20=float(c.group(3)) if c else float("nan"))


groups = {}
print("| profile | rollouts | run | speed | B | hold % | tick p50/p95 ms | GPU % | under 1 m/s | limiters | least clearance | s<0.10 / s<0.20 | contact | FAIL |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
for (profile, count), runs in sorted(rows.items(), key=lambda kv: (kv[0][0], -kv[0][1])):
    fl = [flight(r) for r in runs]
    groups[(profile, count)] = fl
    for f in fl:
        print(f"| {profile} | {count} | {f['run']} | {f['speed']:.3f} | {f['b']:.2f} | {f['hold']:.2f} | "
              f"{f['tick50']:.1f}/{f['tick95']:.1f} | {f['gpu']:.0f} | {f['under1']:.0f} % | {f['limiters']} | "
              f"{f['least']:.2f} | {f['s10']:.1f} / {f['s20']:.1f} | {f['contact']} | {'; '.join(f['fails']) or '-'} |")
print()
print("| profile | rollouts | speed min / mean | B max | hold % mean | tick p50 / p95 mean | GPU % mean | least clearance | s<0.10 max | contacts | verdict against 8192 |")
print("|---|---|---|---|---|---|---|---|---|---|---|")
for (profile, count), fl in sorted(groups.items(), key=lambda kv: (kv[0][0], -kv[0][1])):
    base = groups.get((profile, 8192), fl)
    sp = [f["speed"] for f in fl]
    bsp = np.mean([f["speed"] for f in base])
    least = min(f["least"] for f in fl)
    bleast = min(f["least"] for f in base)
    s10 = max(f["s10"] for f in fl)
    bs10 = max(f["s10"] for f in base)
    ok = (all(f["contact"] == 0 for f in fl) and max(f["b"] for f in fl) <= 2.0 and np.mean(sp) >= 0.95 * bsp
          and least >= bleast - 0.05 and s10 <= bs10)
    print(f"| {profile} | {count} | {min(sp):.3f} / {np.mean(sp):.3f} | {max(f['b'] for f in fl):.2f} | "
          f"{np.mean([f['hold'] for f in fl]):.2f} | {np.mean([f['tick50'] for f in fl]):.1f} / "
          f"{np.mean([f['tick95'] for f in fl]):.1f} | {np.mean([f['gpu'] for f in fl]):.0f} | {least:.2f} | {s10:.1f} | "
          f"{sum(f['contact'] for f in fl)} | {'holds' if ok else 'FAILS'} (speed >= {0.95*bsp:.3f}, least >= {bleast-0.05:.2f}, s<0.10 <= {bs10:.1f}) |")
