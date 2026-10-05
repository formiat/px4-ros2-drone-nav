#!/usr/bin/env python3
"""host_verdict.py RUN...: whether a flight counts, by the host it was flown on (specification A7).

A flight counts when the simulator kept its profile's share (0.82 at the median on the stereo set, 0.97 on the 3D
lidar) of the real-time factor the run asked for (REAL_TIME_FACTOR in the manifest, 1.0 when absent) and the resource
sampler's one-second record has no gap of 2 s or more (a frozen host, A6). The speed is
measured on the simulation clock and survives a slow host; what does not is the flight itself: a simulator slower than
the onboard loop flies the vehicle with a faster computer than it has, and a host that stalls the loop crashes it.
Prints "HOST valid ..." or "HOST VOID ..." per run and exits 1 when any run is void."""

from __future__ import annotations

import csv
import json
import statistics
import sys

MINIMUM_REAL_TIME_FACTOR_P50 = {"stereo_tof": 0.82, "lidar": 0.97}
MAXIMUM_SAMPLER_GAP_S = 2.0


def verdict(profile: str, real_time_factor_p50: float | None, largest_gap_s: float,
            requested_factor: float = 1.0) -> list[str]:
    """Why the flight does not count; empty when it does."""
    reasons = []
    floor = MINIMUM_REAL_TIME_FACTOR_P50[profile] * requested_factor
    if real_time_factor_p50 is None:
        reasons.append("no real-time factor in the resource record")
    elif real_time_factor_p50 < floor:
        reasons.append(f"real-time factor {real_time_factor_p50:.2f} at p50 under {floor:.2f}")
    if largest_gap_s >= MAXIMUM_SAMPLER_GAP_S:
        reasons.append(f"the host froze for {largest_gap_s:.1f} s")
    return reasons


def inspect(run: str) -> list[str]:
    overrides = json.load(open(f"log/runs/{run}/manifest.json")).get("effective_overrides", {})
    profile = "lidar" if overrides.get("NAVIGATION_SENSOR_PROFILE") == "lidar" else "stereo_tof"
    rows = list(csv.DictReader(open(f"log/runs/{run}/resources.csv")))
    stamps = sorted({float(row["stamp_s"]) for row in rows})
    gap = max((b - a for a, b in zip(stamps, stamps[1:])), default=0.0)
    # The median over the whole record, which every flight has: the mission check prints its own only for a flight
    # that reached its goal.
    factors = {}
    for row in rows:
        try:
            factors[float(row["stamp_s"])] = float(row["real_time_factor"])
        except (KeyError, ValueError):
            continue
    finite = [value for value in factors.values() if value == value]
    median = statistics.median(finite) if finite else None
    return verdict(profile, median, gap, float(overrides.get("REAL_TIME_FACTOR", 1.0)))


def main() -> int:
    void = False
    for run in sys.argv[1:]:
        reasons = inspect(run)
        void = void or bool(reasons)
        print(f"HOST {'VOID' if reasons else 'valid'} {run}" + (": " + "; ".join(reasons) if reasons else ""))
    return 1 if void else 0


if __name__ == "__main__":
    sys.exit(main())
