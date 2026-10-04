#!/usr/bin/env python3
"""host_verdict.py RUN...: whether a flight counts, by the host it was flown on (specification A7).

A flight counts when the simulator kept its profile's real-time factor (0.82 at the median on the stereo set, 0.97 on
the 3D lidar) and the resource sampler's one-second record has no gap of 2 s or more (a frozen host, A6). The speed is
measured on the simulation clock and survives a slow host; what does not is the flight itself: a simulator slower than
the onboard loop flies the vehicle with a faster computer than it has, and a host that stalls the loop crashes it.
Prints "HOST valid ..." or "HOST VOID ..." per run and exits 1 when any run is void."""

from __future__ import annotations

import csv
import json
import re
import sys

MINIMUM_REAL_TIME_FACTOR_P50 = {"stereo_tof": 0.82, "lidar": 0.97}
MAXIMUM_SAMPLER_GAP_S = 2.0


def verdict(profile: str, real_time_factor_p50: float | None, largest_gap_s: float) -> list[str]:
    """Why the flight does not count; empty when it does."""
    reasons = []
    floor = MINIMUM_REAL_TIME_FACTOR_P50[profile]
    if real_time_factor_p50 is None:
        reasons.append("no real-time factor in the check's output")
    elif real_time_factor_p50 < floor:
        reasons.append(f"real-time factor {real_time_factor_p50:.2f} at p50 under {floor:.2f}")
    if largest_gap_s >= MAXIMUM_SAMPLER_GAP_S:
        reasons.append(f"the host froze for {largest_gap_s:.1f} s")
    return reasons


def inspect(run: str) -> list[str]:
    overrides = json.load(open(f"log/runs/{run}/manifest.json")).get("effective_overrides", {})
    profile = "lidar" if overrides.get("NAVIGATION_SENSOR_PROFILE") == "lidar" else "stereo_tof"
    found = re.search(r"real-time factor is ([\d.]+) at p50",
                      open(f"log/tools/run_{run}.log", errors="ignore").read())
    stamps = sorted({float(row["stamp_s"])
                     for row in csv.DictReader(open(f"log/runs/{run}/resources.csv"))})
    gap = max((b - a for a, b in zip(stamps, stamps[1:])), default=0.0)
    return verdict(profile, float(found.group(1)) if found else None, gap)


def main() -> int:
    void = False
    for run in sys.argv[1:]:
        reasons = inspect(run)
        void = void or bool(reasons)
        print(f"HOST {'VOID' if reasons else 'valid'} {run}" + (": " + "; ".join(reasons) if reasons else ""))
    return 1 if void else 0


if __name__ == "__main__":
    sys.exit(main())
