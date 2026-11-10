#!/usr/bin/env python3
"""speed_binding.py RUN...: what binds a flight's speed, by the share of its path and of its time (roadmap item 39
stage 5): for every limiter of the speed policy (mppi/mppi_track.jsonl, the planned ticks) the metres flown and the
seconds spent while it was the active one, the median reference it gave there and the median true speed; then the
flight's mean speed, the sensor's measured and frame ranges at the median, and the planner's tick. The cruise limiter
is the profile's own ceiling; the others are the stack holding the vehicle under it. Host."""
from __future__ import annotations

import json
import statistics
import sys
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[1]


def binding(run: str) -> str:
    track = REPOSITORY / "log/runs" / run / "mppi/mppi_track.jsonl"
    rows = [json.loads(line) for line in track.open() if line.strip()]
    rows = [row for row in rows if row.get("planning_state") == "planned"]
    if len(rows) < 2:
        return f"{run}: no planned ticks"
    metres: dict[str, float] = {}
    seconds: dict[str, float] = {}
    reference: dict[str, list[float]] = {}
    actual: dict[str, list[float]] = {}
    for row, following in zip(rows, rows[1:]):
        dt = min((following["stamp_ns"] - row["stamp_ns"]) * 1e-9, 0.2)
        step = sum((a - b) ** 2 for a, b in zip(following["p"], row["p"])) ** 0.5
        name = row["limiter"]
        metres[name] = metres.get(name, 0.0) + step
        seconds[name] = seconds.get(name, 0.0) + dt
        reference.setdefault(name, []).append(row["reference_speed_mps"])
        actual.setdefault(name, []).append(row["actual_speed_mps"])
    path = sum(metres.values())
    time = sum(seconds.values())
    lines = [f"{run}: {path:.0f} m in {time:.0f} s planned, mean {path / time:.2f} m/s; "
             f"sensor measured range p50 {statistics.median(r.get('sensor_measured_range_m', -1) for r in rows):.1f} m, "
             f"frame range p50 {statistics.median(r.get('sensor_frame_range_m', -1) for r in rows):.1f} m, "
             f"evidence age p50 {statistics.median(r.get('sensor_evidence_age_s', 0) for r in rows):.2f} s"]
    for name in sorted(metres, key=lambda key: -metres[key]):
        lines.append(f"  {name:20s} path {metres[name] / path:5.1%} time {seconds[name] / time:5.1%} "
                     f"reference p50 {statistics.median(reference[name]):5.2f} true p50 "
                     f"{statistics.median(actual[name]):5.2f} m/s")
    return "\n".join(lines)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for name in sys.argv[1:]:
        print(binding(name))
