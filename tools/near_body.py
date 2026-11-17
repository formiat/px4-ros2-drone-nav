#!/usr/bin/env python3
"""near_body.py RUN [WITHIN_M]: what the speed policy left the vehicle while it was within WITHIN_M (12) of the walking
body, by the planner's track: the seconds each limiter bound, the time and the way there, and a sample every two
seconds; then the bodies the memory read at once, at most, and the seconds it read more than one."""
import bisect, csv, json, math, re, sys
from collections import Counter
run = sys.argv[1]; within = float(sys.argv[2]) if len(sys.argv) > 2 else 12.0
track = [json.loads(l) for l in open(f'log/runs/{run}/mppi/mppi_track.jsonl')]
body = [(float(r['sim_s']), float(r['x']), float(r['y'])) for r in csv.DictReader(open(f'log/runs/{run}/moving_bodies.csv'))]
bt = [b[0] for b in body]
lim = Counter(); near = []; prev = None; way = 0.0; last = None
for r in track:
    t = r['stamp_ns'] / 1e9
    b = body[min(len(body) - 1, bisect.bisect_left(bt, t))]
    d = math.hypot(r['p'][0] - b[1], r['p'][1] - b[2])
    if d < within:
        if prev is not None and t - prev < 1.0:
            lim[r['limiter']] += t - prev
            way += math.dist(r['p'], last)
        near.append((t, d, r['p'][2], r['actual_speed_mps'], r['reference_speed_mps'], r['limiter']))
        prev = t; last = r['p']
print(run, 'within', within, 'm of the body:', round(sum(lim.values()), 1), 's,', round(way, 1), 'm;', {k: round(v, 1) for k, v in lim.most_common(6)})
t0 = None
for n in near:
    if t0 is None or n[0] - t0 >= 2.0:
        t0 = n[0]; print(f'  t={n[0]:6.1f} d={n[1]:4.1f} z={n[2]:.1f} v={n[3]:.2f} ref={n[4]:.2f} {n[5]}')
counts = [int(m.group(1)) for m in re.finditer(r'FOLLOWED_BODIES followed=(\d+)', open(f'log/runs/{run}/ros_drone_nav.log', errors='replace').read())]
print('  bodies read at once, most:', max(counts or [0]), '; scans with more than one:', sum(c > 1 for c in counts), 'of', len(counts))
