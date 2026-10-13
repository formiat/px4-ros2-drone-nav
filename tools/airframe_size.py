#!/usr/bin/env python3
"""airframe_size.py RUN...: what the routes flown ask of the vehicle's size, and what a smaller body would give.
(1) Widths: every 0.5 s of the mission, the horizontal distance from the body's axis to the nearest truly occupied
voxel at the body's height (the 0.25 m truth grid, from the body's centre to its top, 0.35 m above, so that a floor
under the body is no wall; up to 3 m): the half-width of the passage where the vehicle flew. (2) Speed, a what-if
on the same flights: the clearance laws admit clearance / 0.075 s (the tube law), so a body smaller by D gives every
lateral sample D more clearance and the two clearance limits D / 0.075 m/s more; each planned tick's reference is recomputed against its other
limits and cruise, and the path is re-timed at the actual speed scaled by new / old reference. An optimistic
bound for the same routes: no dynamics, no new routes through narrower places."""
import csv, json, sys
import numpy as np
from body_truth_clearance import B, CENTRE, RES, UP, occupied
runs = sys.argv[1:]
REACH = 3.0
def axis_to_wall(x, y, z):
    best = REACH
    i0 = int(np.floor((x - REACH - B.origin_x_m) / RES)); i1 = int(np.floor((x + REACH - B.origin_x_m) / RES))
    j0 = int(np.floor((y - REACH - B.origin_y_m) / RES)); j1 = int(np.floor((y + REACH - B.origin_y_m) / RES))
    k0 = int(np.floor((z + 0.05 - B.origin_z_m) / RES)); k1 = int(np.floor((z + UP - B.origin_z_m) / RES))
    for i in range(i0, i1 + 1):
        dx = max(B.origin_x_m + i * RES - x, 0.0, x - (B.origin_x_m + (i + 1) * RES))
        if dx >= best: continue
        for j in range(j0, j1 + 1):
            dy = max(B.origin_y_m + j * RES - y, 0.0, y - (B.origin_y_m + (j + 1) * RES))
            d = np.hypot(dx, dy)
            if d >= best: continue
            if any(occupied(i, j, k) for k in range(k0, k1 + 1)): best = d
    return best
TAU, CRUISE = 0.075, 6.5
CLEARANCE = ('clearance', 'route_clearance')
for run in runs:
    rows = [json.loads(l) for l in open(f'log/runs/{run}/mppi/mppi_track.jsonl')]
    rows = [r for r in rows if r.get('planning_state') == 'planned']
    t0, t1 = rows[0]['stamp_ns'] * 1e-9, rows[-1]['stamp_ns'] * 1e-9
    g = np.array([[float(v) for v in r[:4]] for r in csv.reader(open(f'log/runs/{run}/gz_pose.csv'))])
    g = g[(g[:, 0] >= t0 + 3.0) & (g[:, 0] <= t1 - 3.0)]
    samples, last = [], -1.0
    for s in g:
        if s[0] - last >= 0.5:
            samples.append(axis_to_wall(s[1], s[2], s[3] + CENTRE)); last = s[0]
    w = np.array(samples)
    width = (f"axis to wall min {w.min():.2f} m, p5 {np.percentile(w, 5):.2f}, p50 {np.percentile(w, 50):.2f}; "
             f"share under 0.82 m {100*np.mean(w < 0.82):.0f}%, under 1.0 {100*np.mean(w < 1.0):.0f}%, "
             f"under 1.5 {100*np.mean(w < 1.5):.0f}%, under 2.0 {100*np.mean(w < 2.0):.0f}%")
    t = np.array([r['stamp_ns'] * 1e-9 for r in rows]); dt = np.minimum(np.diff(t, append=t[-1]), 0.2)
    v = np.array([r['actual_speed_mps'] for r in rows]); ds = v * dt
    base_time = dt.sum(); path = ds.sum()
    out = []
    for d in (0.10, 0.20, 0.30):
        new_time = 0.0
        for r, dti, dsi, vi in zip(rows, dt, ds, v):
            lim = {k: x for k, x in r['limits'].items() if x >= 0}
            old = min(list(lim.values()) + [CRUISE])
            new = min([x + d / TAU if k in CLEARANCE else x for k, x in lim.items()] + [CRUISE])
            if vi < 0.05 or old <= 0.05:
                new_time += dti
            else:
                new_time += dsi / (vi * new / old)
        out.append(f"-{d:.2f} m: {path/new_time:.2f} m/s")
    print(f"{run}: {width}; mean {path/base_time:.2f} m/s over the planned ticks; a body smaller by " + ", ".join(out))
