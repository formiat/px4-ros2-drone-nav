#!/usr/bin/env python3
"""vertical_error.py RUN...: the lidar-inertial estimate's vertical error against the truth over a flight (specification
K27, the root goal of 2026-10-24 part 1): the estimate's z (lio_estimate.csv) less the true z (gz_pose.csv) less the
frame's rest offset (base_link 0.24 m above the model's origin, K22, read as the median of the first five seconds),
after the first ten seconds: min, 5th percentile, median, 95th percentile, max; the error every ten seconds with the
true position; the error's growth per ten metres of path over the flight's four quarters; and the horizontal error
(the estimate against the truth in x and y) at the end and at most, for comparison. One block a run; "python3 -I" on
the host, from the repository's root."""
import csv
import sys

import numpy as np


def load(run):
    est = np.array([[float(v) for v in r[:4]] for r in csv.reader(open(f"log/runs/{run}/lio_estimate.csv"))
                    if len(r) >= 4 and r[0][:1].isdigit()])
    truth = np.array([[float(v) for v in r[:4]] for r in csv.reader(open(f"log/runs/{run}/gz_pose.csv")) if len(r) >= 4])
    return est, truth


def main():
    for run in sys.argv[1:]:
        est, truth = load(run)
        t = est[:, 0]
        tx, ty, tz = (np.interp(t, truth[:, 0], truth[:, k]) for k in (1, 2, 3))
        rest = np.median((est[:, 3] - tz)[t < t[0] + 5.0])
        z = est[:, 3] - tz - rest
        horizontal = np.hypot(est[:, 1] - tx, est[:, 2] - ty)
        moving = t > t[0] + 10.0
        path = np.concatenate([[0.0], np.cumsum(np.hypot(np.diff(tx), np.diff(ty)))])
        print(f"{run}: rest offset {rest:+.3f} m; vertical error min {z[moving].min():+.2f} p5 {np.percentile(z[moving], 5):+.2f} "
              f"median {np.median(z[moving]):+.2f} p95 {np.percentile(z[moving], 95):+.2f} max {z[moving].max():+.2f} m; "
              f"horizontal error end {horizontal[-1]:.2f} max {horizontal.max():.2f} m; path {path[-1]:.0f} m")
        marks = []
        for mark in np.arange(0.0, t[-1] - t[0], 10.0):
            i = min(int(np.searchsorted(t, t[0] + mark)), len(t) - 1)
            marks.append(f"{mark:3.0f}s ({tx[i]:5.1f},{ty[i]:5.1f},{tz[i]:4.1f}) {z[i]:+.2f}")
        print("  " + " | ".join(marks))
        quarters = []
        for q in range(4):
            lo, hi = q * path[-1] / 4.0, (q + 1) * path[-1] / 4.0
            inside = (path >= lo) & (path < hi)
            if inside.sum() > 2:
                growth = (z[inside][-1] - z[inside][0]) / max(1.0e-6, path[inside][-1] - path[inside][0]) * 10.0
                quarters.append(f"q{q + 1} ({lo:.0f}-{hi:.0f} m) {growth:+.3f} m/10 m")
        print("  growth per ten metres of path: " + "; ".join(quarters))


if __name__ == "__main__":
    main()
