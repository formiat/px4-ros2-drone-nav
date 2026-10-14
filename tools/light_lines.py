#!/usr/bin/env python3
"""light_lines.py [LINE...]: the light's own reading against a line (roadmap item 29, specification K12): for each
line, the longest stretch the moderate flicker (F1, 1000 seeds) keeps the light under it, and when the severe failure
and the light lost are first under it for 3.5 s (seconds after take-off), from the injector's own schedules."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
from carried_light import schedule  # noqa: E402

LONGEST_OUTAGE_S = 3.5


def stretches(outages, line):
    """Stretches (start, length) the share stays under the line: each outage is a trapezoid, the share down from one
    to its floor, at the floor, and back up, each ramp linear; consecutive outages do not touch."""
    out = []
    for outage in outages:
        if outage.floor >= line:
            continue
        below = (line - outage.floor) / (1.0 - outage.floor)
        start = outage.start_s + outage.ramp_down_s * (1.0 - below)
        length = outage.ramp_down_s * below + outage.dark_s + outage.ramp_up_s * below
        out.append((start, length))
    return out


def main():
    lines = [float(value) for value in sys.argv[1:]] or [0.1, 0.2, 0.35, 0.5]
    print("| line | moderate: longest under it (1000 seeds) | 99th percentile | severe: judged at | lost: judged at |")
    print("|---|---|---|---|---|")
    for line in lines:
        longest = []
        for seed in range(1000):
            runs = stretches(schedule("moderate", seed, 600.0), line)
            longest.append(max((length for _, length in runs), default=0.0))
        longest.sort()
        judged = {}
        for profile in ("severe", "lost"):
            runs = stretches(schedule(profile, 0, 600.0), line)
            judged[profile] = next((start + LONGEST_OUTAGE_S for start, length in runs
                                    if length >= LONGEST_OUTAGE_S), float("nan"))
        print(f"| {line:g} | {longest[-1]:.2f} s | {longest[989]:.2f} s | {judged['severe']:.1f} s | "
              f"{judged['lost']:.1f} s |")


if __name__ == "__main__":
    main()
