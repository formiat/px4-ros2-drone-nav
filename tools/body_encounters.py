#!/usr/bin/env python3
"""body_encounters.py RUN [RUN...]: a flight's meetings with the walking bodies of scripts/moving_bodies.py, by the
truth (roadmap item 21): one row a body. Host.

From log/runs/RUN: the bodies' true places (moving_bodies.csv), the vehicle's (gz_pose.csv), the stack's log
(ros_drone_nav.log) and the obstacle memory's snapshots (raw_snapshot_3d_revision_*.json; the flight needs
RAW_SNAPSHOT_BOUNDS_M round the walk for them).

- the closest approach: its time, the distance from the vehicle's centre to the body's surface (an upright cylinder
  0.5 m across, 1.7 m tall), split into the horizontal gap and the height over the body's top, the vehicle's speed
  there and its largest over the five seconds before;
- the closing speed, the fastest the distance fell over those five seconds, and the stopping margin, the least over
  them of the distance less the vehicle's radius (0.35 m) less the way a stop needs at the closing speed
  (v * 0.3 s + v^2 / (2 * 4 m/s^2), the reaction of a planner tick and the profile's acceleration);
- what the planner did while the body was within 10 m: trajectories found in collision and stops published;
- the body's trace in the memory: in every snapshot, the occupied cells inside the volume the walk sweeps (the
  walk's rectangle, the body's radius and a cell wider, the body's height) that lie farther than 0.6 m from where
  the body stood at the snapshot's stamp: cells of a wall where there is free floor. The largest count, the count in
  the last snapshot and how long after the vehicle was last within 10 m of the body that snapshot was taken;
- a contact with the body, from the crash detector's line.
"""
from __future__ import annotations

import bisect
import csv
import json
import math
import re
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))

from survey_coverage import load_memory  # noqa: E402

BODY_RADIUS_M, BODY_HEIGHT_M = 0.25, 1.7
VEHICLE_RADIUS_M = 0.35
REACTION_S, DECELERATION_MPS2 = 0.3, 4.0
NEAR_M = 10.0
APART_M = 0.6
STAMP = re.compile(r"\[(\d+\.\d+)\]")


def stop_way_m(speed_mps: float) -> float:
    """The way a stop needs at a closing speed: the reaction and the braking."""
    speed = max(0.0, speed_mps)
    return speed * REACTION_S + speed * speed / (2.0 * DECELERATION_MPS2)


def body_distance(vehicle: tuple[float, float, float], body: tuple[float, float, float]) -> tuple[float, float, float]:
    """The distance from a point to the body's surface, its horizontal gap and its height over the body's top."""
    gap = max(0.0, math.hypot(vehicle[0] - body[0], vehicle[1] - body[1]) - BODY_RADIUS_M)
    over = vehicle[2] - (body[2] + BODY_HEIGHT_M)
    vertical = max(0.0, over, body[2] - vehicle[2])
    return math.hypot(gap, vertical), gap, over


def read_bodies(run: Path) -> dict[int, list[tuple[float, float, float, float]]]:
    bodies: dict[int, list[tuple[float, float, float, float]]] = {}
    with (run / "moving_bodies.csv").open() as stream:
        for row in csv.DictReader(stream):
            bodies.setdefault(int(row["index"]), []).append(
                (float(row["sim_s"]), float(row["x"]), float(row["y"]), float(row["z_base"])))
    return bodies


def read_truth(run: Path) -> list[tuple[float, float, float, float]]:
    with (run / "gz_pose.csv").open() as stream:
        return [(float(row[0]), float(row[1]), float(row[2]), float(row[3])) for row in csv.reader(stream)]


def at(track: list[tuple[float, float, float, float]], stamps: list[float], t_s: float):
    index = min(len(track) - 1, max(0, bisect.bisect_left(stamps, t_s)))
    return track[index]


def meeting(truth, body) -> dict[str, float]:
    """The closest approach and the five seconds before it."""
    stamps = [row[0] for row in body]
    samples = []
    for t_s, x, y, z in truth:
        if not stamps[0] <= t_s <= stamps[-1]:
            continue
        place = at(body, stamps, t_s)
        distance, gap, over = body_distance((x, y, z), place[1:])
        samples.append((t_s, distance, gap, over, x, y, z))
    closest = min(samples, key=lambda sample: sample[1])
    before = [sample for sample in samples if closest[0] - 5.0 <= sample[0] <= closest[0]]
    # Speeds over half a second: the truth is sampled at 50 Hz.
    step = max(1, int(0.5 / max(1.0e-3, (before[-1][0] - before[0][0]) / max(1, len(before) - 1))))
    speeds, closings, margins = [], [], []
    for a, b in zip(before, before[step:]):
        dt = b[0] - a[0]
        if dt <= 0.0:
            continue
        speeds.append(math.dist(a[4:7], b[4:7]) / dt)
        closing = (a[1] - b[1]) / dt
        closings.append(closing)
        margins.append(b[1] - VEHICLE_RADIUS_M - stop_way_m(closing))
    speed_at = speeds[-1] if speeds else 0.0
    return {"t_s": closest[0], "distance_m": closest[1], "gap_m": closest[2], "over_m": closest[3],
            "speed_mps": speed_at, "speed_before_mps": max(speeds, default=0.0),
            "closing_mps": max(closings, default=0.0), "margin_m": min(margins, default=float("nan")),
            "near": [sample[0] for sample in samples if sample[1] <= NEAR_M]}


TICK = re.compile(r"\[(\d+\.\d+)\] \[production_mppi_node\]: PRODUCTION_MPPI_TICK tick=(\d+) ")


def clock_anchors(log: str, track: Path) -> list[tuple[float, float]]:
    """The wall's stamps of the log against the simulation's clock: the planner's logged ticks, which its track
    holds by their simulation stamps."""
    ticks: dict[int, float] = {}
    with track.open() as stream:
        for line in stream:
            row = json.loads(line)
            ticks[int(row["tick"])] = row["stamp_ns"] * 1.0e-9
    anchors = [(float(found.group(1)), ticks[int(found.group(2))])
               for found in TICK.finditer(log) if int(found.group(2)) in ticks]
    return sorted(anchors)


def sim_time(anchors: list[tuple[float, float]], wall_s: float) -> float:
    """A log line's wall stamp on the simulation's clock, by the logged tick before it."""
    index = max(0, bisect.bisect_right([anchor[0] for anchor in anchors], wall_s) - 1)
    return anchors[index][1] + (wall_s - anchors[index][0])


def planner_events(run: Path, near: list[float]) -> tuple[int, int, bool]:
    """Trajectories found in collision and stops published while the body was near (-1 where the flight's records
    do not place the log's lines on the simulation's clock), and a contact with a body."""
    log = (run / "ros_drone_nav.log").read_text(errors="replace")
    contact = bool(re.search(r"VEHICLE_DESTROYED[^\n]*obstacle_collision='walking_body", log))
    if not near:
        return 0, 0, contact
    track = run / "mppi" / "mppi_track.jsonl"
    anchors = clock_anchors(log, track) if track.exists() else []
    if not anchors:
        return -1, -1, contact
    collisions = stops = 0
    low, high = near[0] - 1.0, near[-1] + 1.0
    for line in log.splitlines():
        if "TRAJECTORY_COLLISION_REPLAN" not in line and "STOP_EXECUTION published=true" not in line:
            continue
        stamp = STAMP.search(line)
        if stamp is None:
            continue
        if low <= sim_time(anchors, float(stamp.group(1))) <= high:
            collisions += "TRAJECTORY_COLLISION_REPLAN" in line
            stops += "STOP_EXECUTION published=true" in line
    return collisions, stops, contact


def swept_volume(body) -> tuple[float, ...]:
    xs, ys = [row[1] for row in body], [row[2] for row in body]
    pad = BODY_RADIUS_M + 0.25
    return (min(xs) - pad, min(ys) - pad, body[0][3], max(xs) + pad, max(ys) + pad, body[0][3] + BODY_HEIGHT_M)


def trace(run: Path, body, near: list[float]) -> tuple[int, int, float, int]:
    """The body's trace over the snapshots: the largest count of wall cells where the body is not, the count in
    the last snapshot, how long after the last time near that snapshot was taken, and the snapshots read."""
    volume = swept_volume(body)
    stamps = [row[0] for row in body]
    counts = []
    for path in sorted(run.glob("raw_snapshot_3d_revision_*.json")):
        document = json.loads(path.read_text())
        stamp = document["header"]["stamp"]
        t_s = stamp["sec"] + 1.0e-9 * stamp["nanosec"]
        # A snapshot that does not hold the walk says nothing of it (the flight's RAW_SNAPSHOT_BOUNDS_M).
        held = document["selection"]["bounds_m"]
        if held is not None and not (all(held[axis] <= volume[axis] for axis in range(3))
                                     and all(held[axis + 3] >= volume[axis + 3] for axis in range(3))):
            continue
        try:
            _, res, origin, _, occupied = load_memory(path, volume)
        except ValueError:
            continue
        if not occupied.size:
            continue
        place = at(body, stamps, t_s)
        cells = np.argwhere(occupied)
        centres = origin + (cells + 0.5) * res
        apart = np.hypot(centres[:, 0] - place[1], centres[:, 1] - place[2]) > APART_M
        counts.append((t_s, int(apart.sum())))
    if not counts:
        return -1, -1, float("nan"), 0
    counts.sort()
    last_near = near[-1] if near else float("nan")
    return max(count for _, count in counts), counts[-1][1], counts[-1][0] - last_near, len(counts)


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    print("| run | body | closest at s | distance m (gap, over the top) | speed there / before m/s | closing m/s | "
          "stop margin m | collisions found, stops | trace cells most / last (s after, snapshots) | contact |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    for name in sys.argv[1:]:
        run = Path(name) if "/" in name else Path("log/runs") / name
        truth = read_truth(run)
        for index, body in sorted(read_bodies(run).items()):
            met = meeting(truth, body)
            collisions, stops, contact = planner_events(run, met["near"])
            most, last, after_s, snapshots = trace(run, body, met["near"])
            print(f"| {run.name} | {index} | {met['t_s']:.1f} | {met['distance_m']:.2f} ({met['gap_m']:.2f}, "
                  f"{met['over_m']:+.2f}) | {met['speed_mps']:.2f} / {met['speed_before_mps']:.2f} | "
                  f"{met['closing_mps']:.2f} | {met['margin_m']:+.2f} | {collisions}, {stops} | "
                  + (f"{most} / {last} ({after_s:.0f}, {snapshots})" if snapshots else "no snapshot holds the walk")
                  + f" | {'yes' if contact else 'no'} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
