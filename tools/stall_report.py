#!/usr/bin/env python3
"""stall_report.py RUN: a B-and-back flight's legs and its stall, if any (part 9, P10).

B reached (nearest approach), the return's progress toward A, the longest stretch the vehicle stood
within 0.5 m after B, and for a stall the raw snapshot nearest to its start measured with closure.py
from the stall to the flown trail's points before it (every 5 m back) and to A."""
import csv
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

B = np.array([63.009, 23.857, 12.593])
A = np.array([0.749, 21.247, 9.6])
run = sys.argv[1]
d = Path("log/runs") / run
a = np.array([[float(x) for x in r] for r in csv.reader(open(d / "gz_pose.csv")) if len(r) > 7])
t, p = a[:, 0], a[:, 1:4]
ib = int(np.argmin(np.linalg.norm(p - B, axis=1)))
print(f"B nearest {np.linalg.norm(p[ib] - B):.2f} m at {t[ib]:.1f} s; end {t[-1]:.1f} s at {np.round(p[-1], 2)}, "
      f"{np.linalg.norm(p[-1] - A):.1f} m from A")
best = (0.0, None, None)
j = ib
for i in range(ib, len(t)):
    while np.linalg.norm(p[i] - p[j]) > 0.5:
        j += 1
    if t[i] - t[j] > best[0]:
        best = (t[i] - t[j], j, i)
span, j, i = best
if span < 30.0:
    print(f"no stall after B (longest stand {span:.1f} s)")
    sys.exit(0)
print(f"stall: {span:.1f} s from {t[j]:.1f} s at {np.round(p[j], 2)}")
snaps = sorted(d.glob("raw_snapshot_3d_revision_*.json"), key=lambda f: int(f.stem.rsplit("_", 1)[1]))
stamps = []
for f in snaps:
    h = json.load(open(f))["header"]["stamp"]
    stamps.append(h["sec"] + 1e-9 * h["nanosec"])
if not snaps:
    sys.exit("no snapshot")
k = int(np.argmin(np.abs(np.array(stamps) - (t[j] + 10.0))))
snap = snaps[k]
print(f"snapshot {snap.name} at {stamps[k]:.1f} s")
start = ",".join(f"{v:.2f}" for v in p[j])
targets = []
walked = 0.0
for m in range(j - 1, ib - 1, -1):
    walked += np.linalg.norm(p[m + 1] - p[m])
    if walked >= 5.0 * (len(targets) + 1):
        targets.append(p[m])
for target in targets[:12]:
    out = subprocess.run([sys.executable, "tools/closure.py", str(snap), start,
                          ",".join(f"{v:.2f}" for v in target)], capture_output=True, text=True)
    lines = [line for line in out.stdout.splitlines() if "widest" in line or "bottleneck" in line]
    print(f"trail point {np.round(target, 1)}: {' | '.join(lines) or out.stdout.strip() or out.stderr.strip()}")
