#!/usr/bin/env python3
"""rtf_watchdog.py RUN [--lidar] [--factor F]: whether the flight RUN is being flown under a load that will void it
(specification A4, A7), read while it flies. The resource sampler writes the simulator's real-time factor once a
second (resources.csv); the watchdog takes the median of the last 30 samples after the flight has got going (the
first sample at or above the floor) and compares it with the host verdict's floor, 0.82 on the stereo set or 0.97 on
the 3D lidar of the factor the run asked for. Prints "WATCHDOG ok ..." and exits 0, or "WATCHDOG slow ..." and exits
1; a run that has not got going yet, or has fewer than 30 samples since, is "WATCHDOG waiting" and exits 0. Nothing
of the stack reads it: the launchers (fly_until_valid.sh, record_until_pass.sh) stop a slow flight and fly it again."""

from __future__ import annotations

import argparse
import csv
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import host_verdict  # noqa: E402

WINDOW_SAMPLES = 30


def factors(path: Path) -> list[float]:
    """The simulator's real-time factor of each sampled second, in order."""
    try:
        with path.open(newline="") as stream:
            rows = list(csv.DictReader(stream))
    except OSError:
        return []
    by_stamp = {}
    for row in rows:
        try:
            by_stamp[float(row["stamp_s"])] = float(row["real_time_factor"])
        except (KeyError, ValueError):
            continue
    return [value for _, value in sorted(by_stamp.items()) if value == value]


def judge(values: list[float], floor: float) -> tuple[str, float | None]:
    """("ok" | "slow" | "waiting", the median of the last window) for the factors sampled so far."""
    started = next((index for index, value in enumerate(values) if value >= floor), None)
    if started is None or len(values) - started < WINDOW_SAMPLES:
        return "waiting", None
    median = statistics.median(values[-WINDOW_SAMPLES:])
    return ("slow" if median < floor else "ok"), median


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run")
    parser.add_argument("--lidar", action="store_true")
    parser.add_argument("--factor", type=float, default=1.0,
                        help="the real-time factor the run asked for (REAL_TIME_FACTOR)")
    args = parser.parse_args()
    floor = host_verdict.MINIMUM_REAL_TIME_FACTOR_P50["lidar" if args.lidar else "stereo_tof"] * args.factor
    state, median = judge(factors(Path(f"log/runs/{args.run}/resources.csv")), floor)
    detail = f"median {median:.2f} over the last {WINDOW_SAMPLES} s against {floor:.2f}" if median is not None else ""
    print(f"WATCHDOG {state} {args.run} {detail}".rstrip())
    return 1 if state == "slow" else 0


if __name__ == "__main__":
    sys.exit(main())
