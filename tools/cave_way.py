#!/usr/bin/env python3
"""cave_way.py ROOT RUN...: the way of a point-to-point flight of the cave world (cave_circuit_practice_01) by the
truth (ROOT/log/runs/RUN/gz_pose.csv; ROOT is the repository, or another copy of it flown for a comparison): the
time and the length flown, the nearest approach to the goal (52.25, 270.75), whether the vehicle left the
junction's chamber (38..42, 22..27) west and came back to the start (x under 0, y under 60, after 200 s), the
largest height in the chamber, the first time north of y 150, and the mission check's goal line and speed. Host,
repository root (the goal of 2026-11-17: the cave on the camera set against its base, the register's row)."""
import csv, math, re, sys
root = sys.argv[1]; goal = (52.25, 270.75)
for run in sys.argv[2:]:
    try:
        rows = [(float(r[0]), float(r[1]), float(r[2]), float(r[3])) for r in csv.reader(open(f'{root}/log/runs/{run}/gz_pose.csv')) if len(r) >= 9]
    except FileNotFoundError:
        print(run, 'no truth'); continue
    t0 = rows[0][0]; fl = [r for r in rows if r[0] > t0 + 10]
    path = sum(math.dist(a[1:], b[1:]) for a, b in zip(fl[::10], fl[10::10]))
    best = min((math.dist(r[1:3], goal), r[0] - t0) for r in fl[::10])
    back = next((round(r[0] - t0) for r in fl if r[0] - t0 > 200 and r[1] < 0 and r[2] < 60), None)
    chamber = max((r[3] for r in fl if 34 < r[1] < 46 and 18 < r[2] < 30), default=0)
    north = next((round(r[0] - t0) for r in fl if r[2] >= 150), None)
    check = ''
    try:
        text = open(f'{root}/log/tools/run_{run}.log').read()
        m = re.search(r'true position is ([0-9.]+) m from the goal', text); f = re.search(r'\(([0-9.]+) m from the goal, radius', text)
        s = re.search(r'([0-9.]+) m/s: ', text)
        check = f"goal {m.group(1) if m else (f.group(1) + ' (outside)' if f else 'none')} m, {s.group(1) if s else '-'} m/s"
    except FileNotFoundError:
        check = 'no check'
    print(f"{run}: {fl[-1][0] - fl[0][0]:.0f} s, {path:.0f} m | nearest {best[0]:.1f} m at {best[1]:.0f} s | back at the start {back} | chamber height {chamber:.1f} m | north of y 150 at {north} | {check}")
