#!/usr/bin/env python3
"""stale_cells.py RUN: where the stale trace cells tools/body_encounters.py counts stand in each snapshot: along the
body's walk from the body, their heights and ages, and what the memory read at that scan."""
import sys, json, math, bisect
import body_encounters as be
import numpy as np
from pathlib import Path
run = Path('log/runs') / sys.argv[1]
body = be.read_bodies(run)[0]; truth = be.read_truth(run)
volume = be.swept_volume(body); stamps = [r[0] for r in body]
places = be.read_places(run); ps = [p[0] for p in places]; ts = [r[0] for r in truth]
for path in sorted(run.glob('raw_snapshot_3d_revision_*.json')):
    d = json.loads(path.read_text()); st = d['header']['stamp']; t = st['sec'] + 1e-9 * st['nanosec']
    est = be.read_estimate(run); es = [r[0] for r in est]
    offset = np.array(be.at(est, es, t)[1:4]) - np.array(be.at(truth, ts, t)[1:4]) if est else np.zeros(3)
    shifted = tuple(volume[a] + offset[a % 3] + (-0.25 if a < 3 else 0.25) for a in range(6))
    try: _, res, origin, _, occ = be.load_memory(path, shifted)
    except ValueError: continue
    if not occ.size: continue
    place = be.at(body, stamps, t); walk = be.walk_at(body, stamps, t)
    cells = np.argwhere(occ); laid = origin + (cells + 0.5) * res; centres = laid - offset
    inside = np.all((centres >= volume[:3]) & (centres <= volume[3:]), axis=1)
    apart = inside & (np.hypot(centres[:, 0] - place[1], centres[:, 1] - place[2]) > be.APART_M)
    apart &= ~be.in_the_sweep(centres, place, walk, res)
    k = bisect.bisect_right(ps, t) - 1
    read = places[k][1] if k >= 0 and t - ps[k] <= be.STAMP_SLACK_S else []
    v = be.at(truth, ts, t)
    ev = be.at(est, es, t) if est else v
    if read: apart &= ~be.in_a_kept_place(laid, read, (ev[1], ev[2]), res)
    out = [(round(float(c[0]), 2), round(float(c[1]), 2), round(float(c[2]), 2), round(be.trace_age_s(body, stamps, (c[0], c[1]), t), 1)) for c in centres[apart]]
    stale = [o for o in out if o[3] > be.LAYER_LIFE_S + be.STAMP_SLACK_S]
    if stale:
        xs = sorted(set((o[0], o[1]) for o in stale))
        print(f't={t:.1f} vehicle ({v[1]:.1f},{v[2]:.1f},{v[3]:.1f}) body ({place[1]:.1f},{place[2]:.1f}) walk ({walk[0]:+.1f},{walk[1]:+.1f}) stale {len(stale)} in {len(xs)} columns: z {min(o[2] for o in stale)}..{max(o[2] for o in stale)}, age {min(o[3] for o in stale)}..{max(o[3] for o in stale)} s; columns {xs[:6]}; read {[(r[0], r[1], r[2], r[5], r[6]) for r in read][:3]}')
