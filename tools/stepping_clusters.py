#!/usr/bin/env python3
"""stepping_clusters.py RUN...: the clusters a flight's memory saw moving without reading them as bodies (the "step"
items of its FOLLOWED_BODIES lines, one line a second): where each stood from the vehicle (range, bearing off the
nose, height over the vehicle), its speed against the vehicle's own, its age, and how far the true body was, if the
flight had one."""
import bisect, csv, json, math, os, re, sys
for run in sys.argv[1:]:
    pose = []
    for line in open(f'log/runs/{run}/gz_pose.csv'):
        f = line.split(',')
        try: pose.append((float(f[0]), float(f[-1])))
        except ValueError: pass
    pw = [p[1] for p in pose]
    track = [json.loads(l) for l in open(f'log/runs/{run}/mppi/mppi_track.jsonl')]
    tt = [r['stamp_ns'] / 1e9 for r in track]
    body = []
    path = f'log/runs/{run}/moving_bodies.csv'
    if os.path.exists(path):
        body = [(float(r['sim_s']), float(r['x']), float(r['y'])) for r in csv.DictReader(open(path))]
    bt = [b[0] for b in body]
    rows = []
    for line in open(f'log/runs/{run}/ros_drone_nav.log', errors='replace'):
        m = re.search(r'\[(\d+\.\d+)\].*FOLLOWED_BODIES followed=(\d+) stepping=(\d+) nearest=(.*)', line)
        if not m: continue
        t = pose[min(len(pose) - 1, bisect.bisect_left(pw, float(m.group(1))))][0]
        r = track[min(len(track) - 1, bisect.bisect_left(tt, t))]
        p, v, yaw = r['p'], r['v'], r['yaw']
        for n in re.findall(r'\[(-?[\d.]+) (-?[\d.]+) (-?[\d.]+) r([\d.]+) h([\d.]+) v(-?[\d.]+),(-?[\d.]+) n(\d+) a([\d.]+) d([\d.]+) step', m.group(4)):
            x, y, z, rad, h, vx, vy, vox, age, d = map(float, n)
            bearing = math.degrees((math.atan2(y - p[1], x - p[0]) - yaw + math.pi) % (2 * math.pi) - math.pi)
            vs = math.hypot(v[0], v[1]); cs = math.hypot(vx, vy)
            cos = (vx * v[0] + vy * v[1]) / (vs * cs) if vs > 0.05 and cs > 0 else float('nan')
            true_m = float('nan')
            if body:
                b = body[min(len(body) - 1, bisect.bisect_left(bt, t))]; true_m = math.hypot(x - b[1], y - b[2])
            rows.append((t, d, bearing, z - p[2], h, int(vox), age, cs, vs, cos, true_m, x, y, z))
    print(f'{run}: {len(rows)} stepping clusters in the logged seconds')
    for r in rows:
        print(f'  t={r[0]:6.1f} range {r[1]:4.1f} bearing {r[2]:+5.0f} dz {r[3]:+4.1f} h {r[4]:.2f} n {r[5]:3d} age {r[6]:.1f} speed {r[7]:.2f} vehicle {r[8]:.2f} cos {r[9]:+.2f} true body {r[10]:5.1f} m at ({r[11]:.1f},{r[12]:.1f},{r[13]:.1f})')
