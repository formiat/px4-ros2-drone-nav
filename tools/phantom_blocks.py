#!/usr/bin/env python3
"""phantom_blocks.py RUN...: blocked routes and their cells against the truth (roadmap item 18's phantoms, the
technical-debt goal's part 5). A blocked episode is a run of the controller's ticks held by the
blocked route's law (mppi_track.jsonl, limiter blocked_route) no more than 1 s apart; one that lasts 5 s or more
is counted. Each cell the trajectory validator names in it (TRAJECTORY_COLLISION_CELLS) is looked up in the 0.25 m
truth grid (log/tools/truth25): a cell with no truth occupied within 0.5 m is a phantom. Truth is the evaluation's
alone; nothing of it reaches the vehicle."""
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, "scripts")
from validate_static_cooperative_scenario import Occupancy3D  # noqa: E402

TRUTH = Occupancy3D.load(Path("log/tools/truth25/urban_r025.occupancy3d"))
OFFSETS = [(dx, dy, dz) for dx in (-2, -1, 0, 1, 2) for dy in (-2, -1, 0, 1, 2) for dz in (-2, -1, 0, 1, 2)]


def truth_near(point) -> bool:
    for dx, dy, dz in OFFSETS:
        cell = TRUTH.world_to_cell((point[0] + 0.25 * dx, point[1] + 0.25 * dy, point[2] + 0.25 * dz))
        if cell is not None and TRUTH.occupied(cell):
            return True
    return False


def stamp(line: str) -> float:
    return float(re.search(r"\[(\d{10}\.\d+)\]", line).group(1))


def report(run: str) -> dict:
    lines = open(f"log/runs/{run}/ros_drone_nav.log", errors="ignore").read().splitlines()
    releases = [stamp(l) for l in lines if "ROUTE_EXECUTION3D" in l and "effect=request_route_release" in l]
    import json
    track = [json.loads(l) for l in open(f"log/runs/{run}/mppi/mppi_track.jsonl")]
    blocked = [r["stamp_ns"] * 1e-9 for r in track if r.get("limiter") == "blocked_route"]
    # The track is stamped on the simulation clock, the log on the wall: the offset from the tick lines.
    ticks = [(stamp(l), int(m.group(1))) for l in lines if "PRODUCTION_MPPI_TICK tick=" in l
             for m in [re.search(r"PRODUCTION_MPPI_TICK tick=(\d+)", l)]]
    by_tick = {r["tick"]: r["stamp_ns"] * 1e-9 for r in track}
    offsets = [wall - by_tick[tick] for wall, tick in ticks if tick in by_tick]
    offset = float(np.median(offsets)) if offsets else 0.0
    releases = [t - offset for t in releases]
    cells = []
    for l in lines:
        if "TRAJECTORY_COLLISION_CELLS" in l:
            points = [tuple(float(v) for v in m.split(","))
                      for m in re.findall(r"\(([-\d.]+,[-\d.]+,[-\d.]+)\)", l.split("cells=", 1)[-1])]
            cells.append((stamp(l) - offset, points))
    episodes = []
    for t in blocked:
        if episodes and t - episodes[-1][1] <= 1.0:
            episodes[-1][1] = t
        else:
            episodes.append([t, t])
    long_episodes = [(a, b) for a, b in episodes if b - a >= 5.0]
    phantom = total = 0
    phantom_long = total_long = 0
    for t, points in cells:
        in_long = any(a - 0.5 <= t <= b + 0.5 for a, b in long_episodes)
        for point in points:
            near = truth_near(point)
            total += 1
            phantom += not near
            if in_long:
                total_long += 1
                phantom_long += not near
    durations = sorted(round(b - a, 1) for a, b in long_episodes)
    print(f"{run}: releases {len(releases)}, episodes {len(episodes)}, of 5 s or more {len(long_episodes)} "
          f"{durations}; cells named {total}, phantoms {phantom}"
          + (f" ({100.0 * phantom / total:.0f} %)" if total else "")
          + f"; in the long episodes {total_long} cells, phantoms {phantom_long}")
    return {"long": len(long_episodes), "phantom_long": phantom_long, "cells_long": total_long}


if __name__ == "__main__":
    totals = {"long": 0, "phantom_long": 0, "cells_long": 0}
    for argument in sys.argv[1:]:
        if Path(f"log/runs/{argument}/ros_drone_nav.log").is_file():
            for key, value in report(argument).items():
                totals[key] += value
    print(f"ALL: long episodes {totals['long']}, their cells {totals['cells_long']}, phantoms {totals['phantom_long']}")
