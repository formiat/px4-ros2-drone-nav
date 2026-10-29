#!/usr/bin/env python3
"""estimate_error.py RUN...: the error of the estimate the planner flies by against the truth over a flight (roadmap
item 35, K52; the register's row on the heading drift): for each run, from `log/runs/RUN/mppi/mppi_track.jsonl` (the
estimate's position at every planner tick) against `log/runs/RUN/gz_pose.csv` (the true pose on the simulation clock),
after the first ten seconds, the vertical rest offset of 0.24 m (K22) removed: the horizontal error's median, 95th
percentile and maximum, the vertical error's median and 95th percentile of its magnitude with its extremes, and a
similarity fit of the estimate's path to the truth's about the start (the turn of the estimate's frame in degrees,
its scale, the rms of what remains), which tells a heading drift from a scale error or noise. Works for both sensor
sets, since both estimates reach the planner the same way.
"""
from __future__ import annotations

import bisect
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

REST_OFFSET_M = 0.24
SETTLE_S = 10.0


def error_of(run: str) -> str:
    base = Path("log/runs") / run
    truth = [[float(v) for v in row] for row in csv.reader(open(base / "gz_pose.csv")) if row and row[0][0].isdigit()]
    stamps = [row[0] for row in truth]
    estimate = [json.loads(line) for line in open(base / "mppi" / "mppi_track.jsonl")]
    pairs = []
    for record in estimate:
        t = record["stamp_ns"] / 1.0e9
        i = bisect.bisect_left(stamps, t)
        if i >= len(truth) or t - stamps[0] < SETTLE_S:
            continue
        pairs.append((record["p"], truth[i][1:4]))
    if not pairs:
        return f"{run}: no overlapping samples"
    est = np.array([p for p, _ in pairs])
    tru = np.array([t for _, t in pairs])
    horizontal = np.hypot(est[:, 0] - tru[:, 0], est[:, 1] - tru[:, 1])
    vertical = est[:, 2] - tru[:, 2] - REST_OFFSET_M
    origin = tru[0, :2]
    a = tru[:, :2] - origin
    b = est[:, :2] - origin
    cross = (a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]).sum()
    dot = (a * b).sum()
    yaw = math.atan2(cross, dot)
    scale = math.hypot(cross, dot) / (a * a).sum()
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)], [math.sin(yaw), math.cos(yaw)]])
    remainder = b - (a @ rotation.T) * scale
    path = np.linalg.norm(np.diff(tru[:, :2], axis=0), axis=1).sum()
    return (f"{run}: n={len(pairs)} path {path:.0f} m | horizontal p50 {np.percentile(horizontal, 50):.2f} "
            f"p95 {np.percentile(horizontal, 95):.2f} max {horizontal.max():.2f} m | vertical |p50| "
            f"{np.percentile(np.abs(vertical), 50):.2f} |p95| {np.percentile(np.abs(vertical), 95):.2f} "
            f"min {vertical.min():+.2f} max {vertical.max():+.2f} m | frame turn {math.degrees(yaw):+.2f} deg "
            f"scale {scale:.4f} remainder rms {np.hypot(remainder[:, 0], remainder[:, 1]).std():.2f} m")


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__.split("\n")[0], file=sys.stderr)
        return 2
    for run in sys.argv[1:]:
        try:
            print(error_of(run))
        except OSError as error:
            print(f"{run}: {error}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
