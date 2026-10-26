#!/usr/bin/env python3
"""rollouts_midflight.py SERIES_FILE: the true clearance of the modelled body in the middle of each flight of the
rollouts series (the pad's first 10 s and the landing's last 12 s left out, as tools/lio_flight_table.py does): the
least clearance and the seconds under 0.10 and 0.20 m, per flight and per group. Host python, repository root."""
import csv, re, sys
sys.path.insert(0, "tools")
import numpy as np
import body_truth_clearance as btc
rows = {}
for line in open(sys.argv[1]):
    m = re.match(r"(cameras|lidar) (\d+) (\d) (r\d+)", line)
    if m: rows.setdefault((m.group(1), int(m.group(2))), []).append(m.group(4))
print("| profile | rollouts | run | least mid-flight clearance m | s < 0.10 | s < 0.20 |")
print("|---|---|---|---|---|---|")
groups = {}
for (profile, count), runs in sorted(rows.items(), key=lambda kv: (kv[0][0], -kv[0][1])):
    out = []
    for run in runs:
        g = np.array([[float(v) for v in r[:4]] for r in csv.reader(open(f"log/runs/{run}/gz_pose.csv")) if len(r) >= 4])
        t = g[:, 0] - g[0, 0]; mid = (t > 10.0) & (t < t[-1] - 12.0)
        c = np.array([btc.clearance(x, y, z + btc.CENTRE) for _, x, y, z in g[mid, :4]])
        dt = np.median(np.diff(g[:, 0]))
        least = float(c.min()); s10 = float((c < 0.10).sum() * dt); s20 = float((c < 0.20).sum() * dt)
        out.append((least, s10, s20)); print(f"| {profile} | {count} | {run} | {least:.2f} | {s10:.1f} | {s20:.1f} |")
    groups[(profile, count)] = out
print()
print("| profile | rollouts | least clearance (group) | s < 0.10 max | s < 0.20 max | against 8192: least >= base - 0.05, s<0.10 <= base |")
print("|---|---|---|---|---|---|")
for (profile, count), out in sorted(groups.items(), key=lambda kv: (kv[0][0], -kv[0][1])):
    base = groups.get((profile, 8192), out)
    least = min(o[0] for o in out); bl = min(o[0] for o in base); s10 = max(o[1] for o in out); bs = max(o[1] for o in base)
    print(f"| {profile} | {count} | {least:.2f} | {s10:.1f} | {max(o[2] for o in out):.1f} | {'holds' if least >= bl - 0.05 and s10 <= bs else 'FAILS'} (base least {bl:.2f}, s<0.10 {bs:.1f}) |")
