#!/usr/bin/env python3
"""time_budget.py RUN...: where an open-sky flight's time goes, from the true pose (gz_pose.csv): per leg, the time
and distance, the time spent under 3, 3-8, 8-12 and over 12 m/s horizontally, climbing/descending, and the time
within 30 m of a goal."""
import csv, math, re, sys

for run in sys.argv[1:]:
    rows = []
    for r in csv.reader(open(f'log/runs/{run}/gz_pose.csv')):
        try:
            rows.append([float(v) for v in r[:4]])
        except (ValueError, IndexError):
            pass
    log = open(f'log/runs/{run}/ros_drone_nav.log', errors='replace').read()
    goals = [tuple(map(float, g)) for g in re.findall(r'goal=\(([-\d.]+),([-\d.]+),([-\d.]+)\)', log)]
    uniq = []
    for g in goals:
        if not uniq or math.dist(g, uniq[-1]) > 1.0:
            uniq.append(g)
    ready = re.search(r'\[(\d+\.\d+)\] \[mission_monitor_node\]: MISSION_READINESS ready=true', log)
    result = re.search(r'\[(\d+\.\d+)\] \[mission_monitor_node\]: MISSION_RESULT success=true', log)
    # pose time is sim time; the ros stamps are sim time too
    t0 = float(ready.group(1)) if ready else rows[0][0]
    t1 = float(result.group(1)) if result else rows[-1][0]
    off = 0.0
    if rows[0][0] < 1e6 < t0:  # pose relative, log absolute: align by end
        off = t1 - rows[-1][0] if not result else None
    bands = {'<3': 0.0, '3-8': 0.0, '8-12': 0.0, '>12': 0.0}
    vert = 0.0; near = 0.0; total = 0.0; dist = 0.0
    for a, b in zip(rows, rows[1:]):
        dt = b[0] - a[0]
        if dt <= 0 or dt > 1.0:
            continue
        h = math.hypot(b[1] - a[1], b[2] - a[2]) / dt
        v = abs(b[3] - a[3]) / dt
        if math.hypot(b[1] - rows[0][1], b[2] - rows[0][2]) < 1.0 and b[3] < rows[0][3] + 0.5:
            continue  # on the ground before takeoff
        total += dt; dist += h * dt
        k = '<3' if h < 3 else '3-8' if h < 8 else '8-12' if h < 12 else '>12'
        bands[k] += dt
        if v > 1.0 and h < 3:
            vert += dt
        if any(math.dist((b[1], b[2]), g[:2]) < 30.0 for g in uniq[:3]):
            near += dt
    print(f"{run}: airborne {total:6.1f} s, {dist:6.0f} m, mean {dist/max(total,1e-9):5.2f} m/s | " +
          ' '.join(f"{k}:{v:5.1f}s" for k, v in bands.items()) + f" | vertical-only {vert:5.1f}s | within 30 m of a goal {near:5.1f}s")
