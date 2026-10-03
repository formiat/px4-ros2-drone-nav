"""room_time.py RUN...: seconds of simulation time each flight spends in the room of the shaft (x 46..56, y -9..-4)
from its true pose, the entry and exit heights, and the flight's mean speed from the validator log."""
import csv, re, sys
for run in sys.argv[1:]:
    rows = [(float(r[0]), float(r[1]), float(r[2]), float(r[3])) for r in csv.reader(open(f'log/runs/{run}/gz_pose.csv'))]
    inside = [r for r in rows if 46 <= r[1] <= 56 and -9 <= r[2] <= -4]
    t = 0.0
    for a, b in zip(rows, rows[1:]):
        if 46 <= a[1] <= 56 and -9 <= a[2] <= -4:
            t += b[0] - a[0]
    try:
        spd = re.search(r'mean flight speed is ([\d.]+)', open(f'log/tools/run_{run}.log').read())
    except FileNotFoundError:
        spd = None
    z = f'z {inside[0][3]:.1f}..{max(r[3] for r in inside):.1f}' if inside else ''
    print(f'{run}: {t:5.1f} s in the room {z}  speed {spd.group(1) if spd else "?"}')
