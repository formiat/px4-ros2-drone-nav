#!/usr/bin/env python3
"""slow_places.py LABEL RUN... [-- LABEL RUN...]: where a group of flights spends its time under 1 m/s: the seconds a
flight per 10 m cell of the location (x, y of the planner's pose), the cell's main limiter, the top cells of the group
and each flight's total. Host python, repository root (the root goal of 2026-10-24, part 1)."""
import json, sys
from collections import defaultdict
import numpy as np
groups, cur = [], None
for a in sys.argv[1:]:
    if a == "--": cur = None; continue
    if cur is None: cur = (a, []); groups.append(cur)
    else: cur[1].append(a)
for label, runs in groups:
    cells = defaultdict(lambda: defaultdict(float)); totals = {}
    for run in runs:
        rows = [json.loads(l) for l in open(f"log/runs/{run}/mppi/mppi_track.jsonl")]
        rows = [r for r in rows if r.get("planning_state") == "planned"]
        t = np.array([r["stamp_ns"] * 1e-9 for r in rows]); dt = np.minimum(np.diff(t, append=t[-1]), 0.2)
        slow = 0.0
        for r, d in zip(rows, dt):
            if r["actual_speed_mps"] >= 1.0: continue
            p = r["p"]; x, y = (p[0], p[1]) if isinstance(p, (list, tuple)) else (p["x"], p["y"])
            cells[(int(np.floor(x / 10)) * 10, int(np.floor(y / 10)) * 10)][r["limiter"]] += d / len(runs); slow += d
        totals[run] = (slow, t[-1] - t[0])
    print(f"== {label}: " + ", ".join(f"{r} {s:.0f}/{T:.0f} s" for r, (s, T) in totals.items()))
    top = sorted(cells.items(), key=lambda kv: -sum(kv[1].values()))[:6]
    for (x, y), lim in top:
        main = max(lim.items(), key=lambda kv: kv[1])
        print(f"   cell x {x:4d}..{x+10:4d} y {y:4d}..{y+10:4d}: {sum(lim.values()):5.1f} s a flight, {main[0]} {100*main[1]/sum(lim.values()):.0f}%")
