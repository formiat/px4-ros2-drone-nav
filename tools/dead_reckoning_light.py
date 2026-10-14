#!/usr/bin/env python3
"""dead_reckoning_light.py RUN...: every dead-reckoning episode of a camera flight while the planner plans: when, how
long, the share of its configured range the frame alone granted through it (sensor_frame_range_m over 6.4 m, the
measure of K12 and of the dark hold, specification K19), whether the offboard held level (DEAD_RECKONING_HOLD), and
the carried light's own share then (carried_light.csv, an evaluation record the vehicle never reads)."""
from __future__ import annotations

import csv
import json
import sys

import numpy as np

from log_window import events

GUARANTEED_RANGE_M = 6.4
BLIND_SHARE = 0.35


def episodes(run: str):
    found = events(run, ("DEAD_RECKONING started=", "DEAD_RECKONING_HOLD", "VEHICLE_LANDED"))
    starts, spans, holds = None, [], []
    for sim, text in found:
        if "started=true" in text:
            starts = sim
        elif "started=false" in text and starts is not None:
            spans.append((starts, sim))
            starts = None
        elif "DEAD_RECKONING_HOLD" in text:
            holds.append(sim)
    if starts is not None:
        landed = [s for s, text in found if "VEHICLE_LANDED" in text and s > starts]
        spans.append((starts, landed[0] if landed else starts + 10.0))
    return spans, holds


def light_share(run: str):
    try:
        rows = [(float(r[0]), float(r[1])) for r in csv.reader(open(f"log/runs/{run}/carried_light.csv"))
                if r and r[0] != "sim_s"]
    except OSError:
        return None
    return np.array(rows) if rows else None


def report(run: str) -> list[str]:
    ticks = [json.loads(line) for line in open(f"log/runs/{run}/mppi/mppi_track.jsonl")]
    spans, holds = episodes(run)
    light = light_share(run)
    lines = []
    for begin, end in spans:
        inside = [t for t in ticks if begin <= t["stamp_ns"] * 1e-9 <= end + 0.05
                  and t.get("planning_state") == "planned" and t.get("sensor_frame_range_m", -1) >= 0]
        shares = [t["sensor_frame_range_m"] / GUARANTEED_RANGE_M for t in inside]
        held = any(begin - 0.05 <= h <= end + 0.05 for h in holds)
        lit = "-"
        if light is not None:
            within = light[(light[:, 0] >= begin - 1.0) & (light[:, 0] <= end)]
            lit = f"{within[:, 1].min():.2f}" if len(within) else f"{light[np.searchsorted(light[:, 0], begin) - 1, 1]:.2f}"
        if shares:
            frame = f"frame share {min(shares):.2f} to {max(shares):.2f} over {len(shares)} planned ticks"
            blind = sum(s < BLIND_SHARE for s in shares)
            frame += f", {blind} under the blind line"
        else:
            frame = "no planned tick"
        lines.append(f"{run}: dead reckoning {begin:.2f} to {end:.2f} s ({end - begin:.2f} s); {frame}; "
                     f"hold {'yes' if held else 'no'}; the light's own share (evaluation) at least {lit}")
    return lines or [f"{run}: no dead reckoning"]


if __name__ == "__main__":
    for run in sys.argv[1:]:
        for line in report(run):
            print(line, flush=True)
