#!/usr/bin/env python3
"""reference_lag.py RUN...: how late the vehicle's speed follows a falling reference on the approaches to its goals:
over the ticks where the route endpoint's or the goal's law binds and the reference falls, the time shift that best
lays the flown speed onto the reference (least mean absolute difference, 0 to 1.5 s), and the mean excess of the
flown speed over the reference there."""
import bisect, json, math, sys

for run in sys.argv[1:]:
    rows = [json.loads(l) for l in open(f'log/runs/{run}/mppi/mppi_track.jsonl')]
    t = [r['stamp_ns'] / 1e9 for r in rows]
    ref = [r['reference_speed_mps'] for r in rows]
    spd = [math.hypot(math.hypot(r['v'][0], r['v'][1]), r['v'][2]) for r in rows]
    idx = [k for k in range(1, len(rows))
           if rows[k]['limiter'] in ('route_endpoint', 'goal') and ref[k] < ref[k - 1] - 1e-6
           and spd[k] > 1.5 and rows[k]['execution_reason'] == 'none']
    if not idx:
        print(run, 'no samples'); continue
    def ref_at(x):
        k = bisect.bisect_right(t, x) - 1
        return ref[max(0, min(k, len(ref) - 1))]
    best = None
    for shift in [s / 20.0 for s in range(0, 31)]:
        err = sum(abs(spd[k] - ref_at(t[k] - shift)) for k in idx) / len(idx)
        if best is None or err < best[1]:
            best = (shift, err)
    excess = sum(spd[k] - ref[k] for k in idx) / len(idx)
    print(f"{run}: samples {len(idx)}, best shift {best[0]:.2f} s (mean |diff| {best[1]:.2f} m/s), mean excess of the flown speed over the reference {excess:+.2f} m/s")
