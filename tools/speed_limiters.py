#!/usr/bin/env python3
"""speed_limiters.py RUN...: what holds a flight under 1 m/s and what the clearance laws allow, over the mission
(from the first planned tick to the goal's acknowledgement or the last tick): the time under 1 m/s, the limiter's
share, and the 5th percentile of the clearance and route-clearance limits where they bind."""
import json, sys
import numpy as np
for run in sys.argv[1:]:
    rows = [json.loads(l) for l in open(f'log/runs/{run}/mppi/mppi_track.jsonl')]
    rows = [r for r in rows if r.get('planning_state') == 'planned']
    if not rows:
        print(run, 'no planned ticks'); continue
    t = np.array([r['stamp_ns'] * 1e-9 for r in rows]); dt = np.diff(t, append=t[-1]); dt = np.minimum(dt, 0.2)
    spd = np.array([r['actual_speed_mps'] for r in rows])
    lim = [r['limiter'] for r in rows]
    total = dt.sum(); slow = dt[spd < 1.0].sum()
    share = {}
    for name in set(lim):
        share[name] = dt[np.array([l == name for l in lim])].sum() / total
    clr = np.array([r['limits']['clearance'] for r in rows]); rcl = np.array([r['limits']['route_clearance'] for r in rows])
    bind_c = clr[(clr >= 0) & np.array([l == 'clearance' for l in lim])]
    bind_r = rcl[(rcl >= 0) & np.array([l == 'route_clearance' for l in lim])]
    top = sorted(share.items(), key=lambda kv: -kv[1])[:4]
    print(f"{run}: under 1 m/s {100*slow/total:.0f}% of {total:.0f} s; limiters " + ", ".join(f"{k} {100*v:.0f}%" for k, v in top)
          + f"; clearance law p5 {np.percentile(bind_c,5) if len(bind_c) else float('nan'):.2f} (n {len(bind_c)}),"
          f" route clearance p5 {np.percentile(bind_r,5) if len(bind_r) else float('nan'):.2f} (n {len(bind_r)})")
