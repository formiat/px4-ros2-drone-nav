#!/usr/bin/env python3
"""ideal_closure.py RUN... [--radius=M] [--heading=DEG]: what an ideal loop closure would be worth to a flight's
estimate (roadmap item 41: whether the lidar set needs a closure).

From `log/runs/RUN/mppi/mppi_track.jsonl` (the estimate the planner flew by) and `log/runs/RUN/gz_pose.csv` (the
truth), sampled every second after the first ten: the estimate's horizontal error as a vector, and the returns, read
from the truth alone: the vehicle is at a return while a pose it held thirty seconds and thirty metres of path before
lies within the radius (5 m) of where it is, and, with --heading, faced within that many degrees (a lidar that sees
all round needs none; a forward camera does: 45).

An ideal closure at a return measures the vehicle's pose from the earlier one exactly: the estimate's error there
becomes the error the earlier pose was laid with, and what the estimate drifts by afterwards adds to that. The tool
applies such a closure at every sample of every return, to the earliest pose in reach, and prints beside the flight's
own figures (median, 95th percentile, the error at the end) the same with the ideal closures, the error at the
returns before and after, and how much of the path lies in returns. The closure is a shift: the heading's error is
left as flown. A run under another directory is named as estimate_error.py names it (../tools/replay/RUN/stack).
Host.

  python3 tools/ideal_closure.py r4236 r4237
"""
from __future__ import annotations

import bisect
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

SETTLE_S = 10.0
RETURN_AGE_S = 30.0
RETURN_PATH_M = 30.0


def closures_of(run: str, radius_m: float, heading_rad: float) -> str:
    base = Path("log/runs") / run
    truth = [[float(v) for v in row[:8]] for row in csv.reader(open(base / "gz_pose.csv")) if row and row[0][:1].isdigit()]
    stamps = [row[0] for row in truth]
    samples = []
    next_stamp = None
    for line in open(base / "mppi" / "mppi_track.jsonl"):
        record = json.loads(line)
        stamp = record["stamp_ns"] / 1.0e9
        i = bisect.bisect_left(stamps, stamp)
        if i >= len(truth) or stamp - stamps[0] < SETTLE_S or (next_stamp is not None and stamp < next_stamp):
            continue
        next_stamp = stamp + 1.0
        _, x, y, _, w, qx, qy, qz = truth[i]
        heading = math.atan2(2.0 * (w * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
        samples.append((stamp, np.array([x, y]), heading, np.array(record["p"][:2]) - np.array([x, y])))
    if len(samples) < 2:
        return f"{run}: no overlapping samples"
    path = [0.0]
    for before, after in zip(samples, samples[1:]):
        path.append(path[-1] + float(np.linalg.norm(after[1] - before[1])))
    flown = np.array([s[3] for s in samples])
    closed = flown.copy()
    # What the closures so far took off the estimate's error.
    taken = np.zeros(2)
    at_return_before, at_return_after, in_return = [], [], 0
    for index, (stamp, position, heading, error) in enumerate(samples):
        closed[index] = error - taken
        for old in range(index):
            if stamp - samples[old][0] < RETURN_AGE_S or path[index] - path[old] < RETURN_PATH_M:
                break
            if np.linalg.norm(samples[old][1] - position) <= radius_m and (
                    heading_rad <= 0.0 or abs(math.remainder(samples[old][2] - heading, math.tau)) <= heading_rad):
                at_return_before.append(float(np.linalg.norm(closed[index])))
                # The pose seen again as it was laid: its own error is what the closure leaves.
                taken += closed[index] - closed[old]
                closed[index] = closed[old]
                at_return_after.append(float(np.linalg.norm(closed[index])))
                in_return += 1
                break
    flown_m = np.linalg.norm(flown, axis=1)
    closed_m = np.linalg.norm(closed, axis=1)

    def figures(values: np.ndarray) -> str:
        return (f"p50 {np.percentile(values, 50):.2f} p95 {np.percentile(values, 95):.2f} max {values.max():.2f} "
                f"end {np.median(values[-5:]):.2f} m")

    returns = (f"at the returns {np.median(at_return_before):.2f} -> {np.median(at_return_after):.2f} m (median), "
               f"{max(at_return_before):.2f} -> {max(at_return_after):.2f} m (worst)") if at_return_before else "no return"
    return (f"{run}: path {path[-1]:.0f} m, in returns {100.0 * in_return / len(samples):.0f} % of the samples | "
            f"as flown {figures(flown_m)} | with ideal closures {figures(closed_m)} | {returns}")


def main() -> int:
    runs = [a for a in sys.argv[1:] if not a.startswith("--")]
    options = dict(a[2:].split("=", 1) for a in sys.argv[1:] if a.startswith("--") and "=" in a)
    if not runs:
        print(__doc__.split("\n")[0], file=sys.stderr)
        return 2
    for run in runs:
        try:
            print(closures_of(run, float(options.get("radius", 5.0)), math.radians(float(options.get("heading", 0.0)))))
        except OSError as error:
            print(f"{run}: {error}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
