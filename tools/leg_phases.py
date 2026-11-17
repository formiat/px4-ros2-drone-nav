#!/usr/bin/env python3
"""leg_phases.py RUN...: each leg's simulation time by phase from the controller's track (mppi_track.jsonl): the wait
before it moves (speed under 1 m/s), the climb (to within 2 m of the leg's top), the cruise (its time, distance and
mean speed), the descent to 3 m from the goal, and the dwell from there to the next leg's first motion."""
import json, math, re, sys

for run in sys.argv[1:]:
    rows = [json.loads(l) for l in open(f'log/runs/{run}/mppi/mppi_track.jsonl')]
    log = open(f'log/runs/{run}/ros_drone_nav.log', errors='replace').read()
    goals = []
    for g in re.findall(r'goal=\(([-\d.]+), ?([-\d.]+), ?([-\d.]+)\)', log):
        g = tuple(map(float, g))
        if all(math.dist(g, k) > 1.0 for k in goals):
            goals.append(g)
    t = [r['stamp_ns'] / 1e9 for r in rows]
    p = [r['p'] for r in rows]
    spd = [math.hypot(math.hypot(r['v'][0], r['v'][1]), r['v'][2]) for r in rows]
    i = 0
    total = [0.0] * 5
    print(f"== {run}")
    for gi, g in enumerate(goals[:3]):
        j_arr = next((k for k in range(i, len(rows)) if math.dist(p[k], g) < 3.0), None)
        if j_arr is None:
            break
        top = max(p[k][2] for k in range(i, j_arr + 1))
        j_move = next((k for k in range(i, j_arr) if spd[k] > 1.0), i)
        j_top = next((k for k in range(j_move, j_arr) if p[k][2] > top - 2.0), j_move)
        j_desc = max((k for k in range(j_top, j_arr) if p[k][2] > top - 2.0), default=j_top)
        # the dwell ends when the vehicle next leaves 3 m at over 1 m/s, or the track ends
        j_leave = next((k for k in range(j_arr, len(rows)) if math.dist(p[k], g) > 3.0 and spd[k] > 1.0), len(rows) - 1)
        cruise_m = sum(math.dist(p[k], p[k + 1]) for k in range(j_top, j_desc))
        ph = [t[j_move] - t[i], t[j_top] - t[j_move], t[j_desc] - t[j_top], t[j_arr] - t[j_desc], t[j_leave] - t[j_arr]]
        if gi == 2:
            ph[4] = 0.0
        total = [a + b for a, b in zip(total, ph)]
        print(f"  leg {gi+1}: wait {ph[0]:5.1f} climb {ph[1]:5.1f} cruise {ph[2]:5.1f} s ({cruise_m:5.0f} m, {cruise_m/max(ph[2],1e-9):5.2f} m/s, top {top:4.1f}) descent {ph[3]:5.1f} dwell {ph[4]:5.1f}")
        i = j_leave
    print(f"  sum: wait {total[0]:5.1f} climb {total[1]:5.1f} cruise {total[2]:5.1f} descent {total[3]:5.1f} dwell {total[4]:5.1f} = {sum(total):6.1f} s")
