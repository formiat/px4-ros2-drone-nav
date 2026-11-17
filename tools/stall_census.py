#!/usr/bin/env python3
"""stall_census.py RUN...: where a flight stands with a route (speed under 0.25 m/s), by the binding limiter.

For each run: the seconds at rest by limiter, and of those bound by the executed motion's clearance, the seconds the
route's own clearance law admitted the contact's departure speed (1 m/s) or more: the vehicle off its route's line.
"""
import collections, json, math, sys
for run in sys.argv[1:]:
    rows = [json.loads(l) for l in open(f'log/runs/{run}/mppi/mppi_track.jsonl')]
    by = collections.Counter(); off = 0.0; spots = collections.Counter()
    for a, b in zip(rows, rows[1:]):
        dt = (b['stamp_ns'] - a['stamp_ns']) / 1e9
        if not 0.0 < dt < 1.0 or a['planning_state'] != 'planned': continue
        if math.hypot(math.hypot(a['v'][0], a['v'][1]), a['v'][2]) >= 0.25: continue
        if a['limits']['goal'] < 0.5 or a['limits']['route_endpoint'] < 0.5: continue
        by[a['limiter']] += dt
        if a['limiter'] == 'clearance' and a['limits']['route_clearance'] >= 1.0:
            off += dt; spots[(round(a['p'][0]), round(a['p'][1]), round(a['p'][2]))] += dt
    total = sum(by.values())
    print(run, f'at rest with a route {total:5.1f} s:', ' '.join(f'{k} {v:.1f}' for k, v in by.most_common(5)),
          f'| executed-motion clearance with the route admitting 1 m/s: {off:.1f} s at',
          ' '.join(f'{k}:{v:.0f}' for k, v in spots.most_common(4)))
