#!/usr/bin/env python3
"""in_field.py RUN...: for each flight, how long before its closest approach the walking body stood inside the
navigating sensors' guaranteed field by the truth (the pair: 5.2 m, 60 degrees either side of the heading, 52.4 up
and down; the lidar: 14 m all round), counted over the five seconds before the approach less the last second (a
body first in the field later than that leaves the stack no time to answer). Walls are not read: a body behind a
corner counts as in the field."""
import bisect, csv, math, sys
from pathlib import Path
import body_encounters as be
for name in sys.argv[1:]:
    run = Path('log/runs') / name
    cameras = any('[stereo_depth_node]' in line for line in (run / 'ros_drone_nav.log').open(errors='replace'))
    truth = []
    for row in csv.reader((run / 'gz_pose.csv').open()):
        try: truth.append(tuple(float(v) for v in row[:8]))
        except ValueError: pass
    body = be.read_body(run / 'moving_bodies.csv') if hasattr(be, 'read_body') else None
    if body is None:
        body = [(float(r['sim_s']), float(r['x']), float(r['y']), float(r['z_base'])) for r in csv.DictReader((run / 'moving_bodies.csv').open())]
    stamps = [b[0] for b in body]
    met = be.meeting([t[:4] for t in truth], body)
    t0 = met['t_s']
    seen = 0.0; first = None; prev = None
    for t, x, y, z, qw, qx, qy, qz in truth:
        if not (t0 - 5.0 <= t <= t0 - 1.0):
            continue
        b = body[min(len(body) - 1, bisect.bisect_left(stamps, t))]
        dx, dy, dz = b[1] - x, b[2] - y, (b[3] + 0.85) - z
        rng = math.sqrt(dx * dx + dy * dy + dz * dz)
        yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        bearing = (math.atan2(dy, dx) - yaw + math.pi) % (2.0 * math.pi) - math.pi
        elevation = math.atan2(dz, math.hypot(dx, dy))
        inside = rng <= 14.0 if not cameras else (rng <= 5.2 and abs(bearing) <= math.radians(60.0) and abs(elevation) <= math.radians(52.4))
        if inside and prev is not None:
            seen += t - prev
            if first is None: first = t - t0
        prev = t
    print(f"{name} {'cameras' if cameras else 'lidar'} closest {met['distance_m']:.2f} m margin {met['margin_m']:+.2f} | in the field {seen:.1f} s of 4.0, first at {first if first is None else round(first, 1)} s")
