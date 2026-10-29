#!/usr/bin/env python3
"""flight_diff.py RUN_A RUN_B [--threshold-m 0.10]: how far two flights part (roadmap item 36, Р2.4): the true position
of each (`log/runs/RUN/gz_pose.csv`, the simulation clock) compared at the same simulation stamps (the later start's
stamps, the other flight interpolated), after both have left the pad: the largest and the 95th-percentile distance, the
first stamp at which they part by more than the threshold, and each flight's result line. "The same metres" is a
largest distance within the threshold over the whole flight and one mission result.
"""
from __future__ import annotations

import argparse
import csv
import re
import sys
from pathlib import Path

import numpy as np


def truth_of(run: str) -> np.ndarray:
    rows = [[float(v) for v in row] for row in csv.reader(open(Path("log/runs") / run / "gz_pose.csv"))
            if row and row[0][0].isdigit()]
    return np.array(rows)[:, :4]


def result_of(run: str) -> str:
    log = (Path("log/runs") / run / "ros_drone_nav.log").read_text(errors="ignore")
    found = re.search(r"MISSION_RESULT success=(\w+) reason='([^']*)'", log)
    return f"{found.group(1)} ({found.group(2)})" if found else "no result"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("run_a")
    parser.add_argument("run_b")
    parser.add_argument("--threshold-m", type=float, default=0.10)
    args = parser.parse_args()
    a = truth_of(args.run_a)
    b = truth_of(args.run_b)
    start = max(a[0, 0], b[0, 0])
    end = min(a[-1, 0], b[-1, 0])
    stamps = b[(b[:, 0] >= start) & (b[:, 0] <= end), 0]
    if len(stamps) < 2:
        print(f"{args.run_a} and {args.run_b} share no stamps")
        return 1
    a_at = np.stack([np.interp(stamps, a[:, 0], a[:, i]) for i in (1, 2, 3)], axis=1)
    b_at = np.stack([np.interp(stamps, b[:, 0], b[:, i]) for i in (1, 2, 3)], axis=1)
    distance = np.linalg.norm(a_at - b_at, axis=1)
    parted = np.nonzero(distance > args.threshold_m)[0]
    first = f"{stamps[parted[0]]:.2f} s" if len(parted) else "never"
    print(f"{args.run_a} vs {args.run_b}: {len(stamps)} stamps over {stamps[0]:.1f} to {stamps[-1]:.1f} s | distance max "
          f"{distance.max():.3f} m p95 {np.percentile(distance, 95):.3f} m | first beyond {args.threshold_m:.2f} m: {first} | "
          f"results {result_of(args.run_a)} / {result_of(args.run_b)} | spans {a[-1, 0] - a[0, 0]:.0f} s / {b[-1, 0] - b[0, 0]:.0f} s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
