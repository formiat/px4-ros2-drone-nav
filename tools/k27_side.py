#!/usr/bin/env python3
"""k27_side.py RUN...: at the slow ticks (actual speed under 1 m/s) the clearance laws limit, which of K27's shifts
brings the truth nearest the body: the true pose (gz_pose), the body as tools/body_truth_clearance.py models it,
measured where it is, 0.25 and 0.5 m lower and 0.5 m higher; the share of those ticks where a shifted body is
nearer than 0.5 m to the truth while the unshifted one is not, by side."""
import csv, json, sys
import numpy as np
from body_truth_clearance import CENTRE, clearance
runs = sys.argv[1:]
for run in runs:
    rows = [json.loads(l) for l in open(f'log/runs/{run}/mppi/mppi_track.jsonl')]
    rows = [r for r in rows if r.get('planning_state') == 'planned' and r['actual_speed_mps'] < 1.0
            and r['limiter'] in ('clearance', 'route_clearance')][::4]
    g = np.array([[float(v) for v in r[:4]] for r in csv.reader(open(f'log/runs/{run}/gz_pose.csv'))])
    counts = {'none': 0, 'below 0.5 only': 0, 'below 0.25 too': 0, 'above 0.5': 0, 'both': 0, 'unshifted': 0}
    for r in rows:
        i = min(np.searchsorted(g[:, 0], r['stamp_ns'] * 1e-9), len(g) - 1)
        x, y, z = g[i, 1], g[i, 2], g[i, 3] + CENTRE
        c0 = clearance(x, y, z); cd5 = clearance(x, y, z - 0.5); cd25 = clearance(x, y, z - 0.25); cu5 = clearance(x, y, z + 0.5)
        if c0 < 0.5: counts['unshifted'] += 1; continue
        down, up = cd5 < 0.5, cu5 < 0.5
        if down and up: counts['both'] += 1
        elif down: counts['below 0.25 too' if cd25 < 0.5 else 'below 0.5 only'] += 1
        elif up: counts['above 0.5'] += 1
        else: counts['none'] += 1
    n = max(1, len(rows))
    print(f"{run}: {len(rows)} slow clearance-limited ticks (every 4th): " + ", ".join(f"{k} {100*v/n:.0f}%" for k, v in counts.items()))
