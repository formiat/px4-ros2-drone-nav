"""true_path.py RUN...: how far and how long a flight flew by the truth, a line a run: the length of the true path
(`log/runs/RUN/gz_pose.csv` sampled every half second, steps under 2 cm left out as a hover's jitter), the seconds in
motion, and the simulation time of the record. The mission check's mean speed is the estimate's path over the
mission's time; a vehicle that wanders flies a longer path at the same speed, and only the path tells (roadmap item
45: the stereo set under the one ceiling). Evaluation only."""
import csv
import sys

import numpy as np

for run in sys.argv[1:]:
    rows = np.array([[float(value) for value in row[:4]] for row in csv.reader(open(f'log/runs/{run}/gz_pose.csv'))])
    times = np.arange(rows[0, 0], rows[-1, 0], 0.5)
    points = np.stack([np.interp(times, rows[:, 0], rows[:, axis]) for axis in (1, 2, 3)], axis=1)
    steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    moving = steps > 0.02
    print(f'{run}: true path {steps[moving].sum():.0f} m, in motion {0.5 * moving.sum():.0f} s, '
          f'record {rows[-1, 0] - rows[0, 0]:.0f} s')
