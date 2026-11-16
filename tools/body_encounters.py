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
- how long the body stood in a field of the navigating sensors over the ten seconds before the closest approach
  less its last one, by the truth (the stereo set: the pair's guaranteed field, 5.2 m, 60 degrees either side of
  the heading and 52.4 up and down, or the thermal core's, 19 m, 28.5 and 22.2 degrees; the lidar set: 14 m all
  round; walls are not read, a body behind a corner counts as seen). A body that stood in no field for a second of
  those nine came from where the vehicle had no sensor, and the flight's margin measures nothing of the stack
  (r3696: out of a side passage behind the vehicle's shoulder, 66 to 72 degrees off its heading, nothing read by
  the memory and no warm region in the core's frames, 0.63 m under the vehicle at 1.66 m/s);
- what the planner did while the body was within 10 m: trajectories found in collision and stops published;
- the body's trace in the memory: in every snapshot, the occupied cells whose centres lie inside the volume the
  walk sweeps (the walk's rectangle, the body's radius and a tenth of a metre of the range's noise wider; from 0.5 m
  over the floor to the body's height) that lie farther than 0.6 m from where the body stood at the snapshot's
  stamp: an occupancy where there is free air. The memory is laid in the estimate's frame and the walk is the
  truth's: the cells are brought to the truth's frame by the estimate's error at the snapshot's stamp (the
  planner's track against the truth), and the half metre over the floor holds the floor's rows and what that
  error drifted by since the rows were laid. Read without it, the floor itself was the "trace" wherever the
  estimate stood 0.4 to 0.56 m over the truth (r3590, r3614, r3624, r3630, r3640: every "stale" cell in the row at
  1.38 m, the floor's own in the memory's frame). A wall the walk comes within
  a quarter of a metre of lies outside the volume; the floor's rows under it do too. The cells ahead of the body
  along its walk that the memory's sweep lays between scans (specification K66: the body's cylinder and a person's
  reach, carried along its velocity for 1.5 s, occupied by no measurement and lifted before every scan) are the
  sweep's, not a trace: counted as a trace they read "stale" by the age of the body's pass before (r3484, r3485,
  r3491: every stale cell 1.1 to 2.2 m ahead of the body). So are the cells of the places the memory itself kept at
  the snapshot's stamp, read off the flight's log (the FOLLOWED_BODIES line of the scan before the stamp): about a
  body it reads whose walk is not measured, a person's walk in the sweep's time on every side; along the measured
  velocity of one that walks and of a cluster seen to step within that walk of the vehicle (r3569: every stale cell
  3.3 to 3.5 m ahead of the body, the rim of the place kept every way); an unseen body keeps its place every way
  too (specification K66 since cbea5dff; r3656: 48 cells of it read stale). The largest count over the
  snapshots, the count in the last one, the stale cells over the snapshots (trace cells the body left longer ago
  than the memory's moving layer lives, 3 s, and the stamps' slack: a trace the layer has not taken back,
  specification K65; the slack is half a second for the lidar set and a second for the stereo set, whose last
  return on a voxel comes up to 0.2 s after the body's middle is 0.6 m off it and whose scan reaches the memory up to
  0.3 s after its frame, the layer dropping the voxel at the first scan after its 3 s, 0.3 s later at most: r3691,
  thirty voxels where the body turned, 3.6 to 3.9 s old in one snapshot) and the oldest trace cell's age: the longest, over the snapshots, since the body last
  stood within 0.6 m of a cell still occupied in that snapshot. A body walking back and forth comes back to every
  cell of its walk within its period, so the age never says more than the period does. A stale cell is one the
  memory could have looked at since: at some time before the snapshot the cell stood in the field of the vehicle's
  range sensors (the pair's guaranteed field about the heading, the lidar's 14 m all round) while the body had been
  away from it for longer than the layer lives. A trace cell the vehicle was never so placed for is counted apart, the last figure: by the
  memory's own rule (I2) what was not looked at again is not taken back (r3656: three cells a body left at the
  edge of the pair's reach, the vehicle flying away);
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
NOISE_M, OVER_FLOOR_M = 0.1, 0.5
LAYER_LIFE_S, STAMP_SLACK_S = 3.0, 0.5
SWEEP_S, REACH_M, SWEEP_SPEED_SLACK = 1.5, 0.5, 1.2
WALK_MPS, VEHICLES_OWN_M, PLACE_HOLDS_M, RISE_WITHIN_M = 1.5, 1.0, 0.75, 10.0
STEREO_REACH_M, LIDAR_REACH_M = 6.4, 14.0
# The fields (range, half-angle either side of the heading, half-angle up and down; the launch's sensor profile and
# specification K64): the pair's guaranteed one, the thermal core's within the memory's integration range, the
# lidar's guaranteed range all round. A body is seen in the pair's or the core's; a voxel is looked at in the
# pair's (the core measures no range).
PAIR_FIELD = (5.2, math.radians(60.0), math.radians(52.4))
CORE_FIELD = (19.0, math.radians(28.5), math.radians(22.2))
LIDAR_FIELD = (LIDAR_REACH_M, math.pi, math.pi / 2.0)
SEEN_OVER_S, LAST_S = 10.0, 1.0
STEREO_SLACK_S = 1.0
STAMP = re.compile(r"\[(\d+\.\d+)\]")
READ = re.compile(r"\[(\d+\.\d+)\] \[obstacle_memory_3d_node\]: FOLLOWED_BODIES followed=\d+ stepping=\d+ nearest=(.*)")
READ_BODY = re.compile(r"\[(-?[\d.]+) (-?[\d.]+) -?[\d.]+ r([\d.]+) h[\d.]+ v(-?[\d.]+),(-?[\d.]+) n\d+ a[\d.]+ d[\d.]+ "
                       r"(warm|step|size) u([\d.]+)\]")


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


def read_headings(run: Path) -> list[tuple[float, float]]:
    """The vehicle's true heading over the flight (the pose's quaternion, w first), a row for each of read_truth."""
    headings = []
    with (run / "gz_pose.csv").open() as stream:
        for row in csv.reader(stream):
            w, x, y, z = (float(value) for value in row[4:8])
            headings.append((float(row[0]), math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))))
    return headings


def in_a_field(vehicle, heading_rad: float, point, fields) -> bool:
    """Whether a point stands in one of the fields about the vehicle's heading. Walls are not read: what stands
    behind a corner counts as in the field."""
    dx, dy, dz = point[0] - vehicle[0], point[1] - vehicle[1], point[2] - vehicle[2]
    range_m = math.sqrt(dx * dx + dy * dy + dz * dz)
    bearing = (math.atan2(dy, dx) - heading_rad + math.pi) % (2.0 * math.pi) - math.pi
    elevation = math.atan2(dz, math.hypot(dx, dy))
    return any(range_m <= reach and abs(bearing) <= side and abs(elevation) <= up for reach, side, up in fields)


def seen_s(truth, headings, body, closest_s: float, fields) -> float:
    """How long the body's middle stood in a field over the ten seconds before the closest approach less its
    last one."""
    stamps = [row[0] for row in body]
    seen = 0.0
    previous = None
    for (t_s, x, y, z), (_, heading) in zip(truth, headings):
        if not closest_s - SEEN_OVER_S <= t_s <= closest_s - LAST_S:
            continue
        place = at(body, stamps, t_s)
        if previous is not None and in_a_field((x, y, z), heading,
                                               (place[1], place[2], place[3] + BODY_HEIGHT_M / 2.0), fields):
            seen += t_s - previous
        previous = t_s
    return seen


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


def walk_at(body, stamps: list[float], t_s: float) -> tuple[float, float]:
    """The body's velocity at t, from the truth's rows either side of it."""
    index = min(max(bisect.bisect_left(stamps, t_s), 1), len(body) - 1)
    before, after = body[index - 1], body[index]
    dt = after[0] - before[0]
    return ((after[1] - before[1]) / dt, (after[2] - before[2]) / dt) if dt > 0.0 else (0.0, 0.0)


def in_the_sweep(centres, place, walk: tuple[float, float], resolution_m: float):
    """The cells the memory's sweep may hold ahead of a body at `place` walking at `walk` (K66)."""
    speed = math.hypot(*walk)
    if speed < 1.0e-6:
        return np.zeros(len(centres), dtype=bool)
    along = ((centres[:, 0] - place[1]) * walk[0] + (centres[:, 1] - place[2]) * walk[1]) / speed
    across = np.abs((centres[:, 0] - place[1]) * walk[1] - (centres[:, 1] - place[2]) * walk[0]) / speed
    wide = BODY_RADIUS_M + REACH_M + NOISE_M + resolution_m
    return (along > 0.0) & (along <= SWEEP_SPEED_SLACK * SWEEP_S * speed + wide) & (across <= wide)


def read_places(run: Path) -> list[tuple[float, list[tuple[float, float, float, float, float, str, float]]]]:
    """What the memory read at every scan that read anything, on the simulation's clock: each body's or stepping
    cluster's place, radius and velocity, the word of its reading (a body read by returns, `warm`, or laid by its
    size alone, `size`; a cluster seen to step, `step`), and how long it was unseen."""
    track = run / "mppi" / "mppi_track.jsonl"
    log = (run / "ros_drone_nav.log").read_text(errors="replace")
    anchors = clock_anchors(log, track) if track.exists() else []
    if not anchors:
        return []
    places = []
    for found in READ.finditer(log):
        places.append((sim_time(anchors, float(found.group(1))),
                       [(float(x), float(y), float(radius), float(vx), float(vy), word, float(unseen))
                        for x, y, radius, vx, vy, word, unseen in READ_BODY.findall(found.group(2))]))
    return sorted(places)


def in_a_kept_place(centres, read, vehicle: tuple[float, float], resolution_m: float):
    """The cells the memory holds occupied by no measurement about what it read (K66): the place of a body whose
    walk is not measured, a person's walk in the sweep's time wide on every side and standing within 0.75 m of it,
    with the stretch to the vehicle while the body is within 10 m of it (a body laid by its size keeps that alone); the way of one that walks, and
    of a cluster seen to step within that walk of the vehicle, along its measured velocity."""
    kept = np.zeros(len(centres), dtype=bool)
    for x, y, radius, vx, vy, word, unseen_s in read:
        wide = radius + REACH_M + resolution_m
        speed = math.hypot(vx, vy)
        body = word != "step"
        to_vehicle = (vehicle[0] - x, vehicle[1] - y)
        apart_m = math.hypot(*to_vehicle)
        if unseen_s > 0.0 or speed <= 0.0:
            if body and (word != "size" or apart_m <= RISE_WITHIN_M):
                # Within the rise's distance the place is the stretch to the vehicle too.
                along = np.zeros(len(centres))
                if 0.0 < apart_m <= RISE_WITHIN_M:
                    along = np.clip(((centres[:, 0] - x) * to_vehicle[0] + (centres[:, 1] - y) * to_vehicle[1])
                                    / apart_m, 0.0, apart_m) / apart_m
                kept |= np.hypot(centres[:, 0] - x - along * to_vehicle[0],
                                 centres[:, 1] - y - along * to_vehicle[1]) <= (
                                     wide + WALK_MPS * SWEEP_S + PLACE_HOLDS_M)
            continue
        if not body and apart_m > (
                radius + REACH_M + WALK_MPS * SWEEP_S + VEHICLES_OWN_M + resolution_m):
            continue
        along = ((centres[:, 0] - x) * vx + (centres[:, 1] - y) * vy) / speed
        across = ((centres[:, 0] - x) * vy - (centres[:, 1] - y) * vx) / speed
        kept |= np.hypot(along - np.clip(along, 0.0, speed * SWEEP_S), across) <= wide
    return kept


def swept_volume(body) -> tuple[float, ...]:
    xs, ys = [row[1] for row in body], [row[2] for row in body]
    pad = BODY_RADIUS_M + NOISE_M
    return (min(xs) - pad, min(ys) - pad, body[0][3] + OVER_FLOOR_M, max(xs) + pad, max(ys) + pad,
            body[0][3] + BODY_HEIGHT_M)


def trace_age_s(body, stamps: list[float], centre: tuple[float, float], t_s: float) -> float:
    """How long before t the body last stood within APART_M of a cell's centre; the whole log when never."""
    for row in reversed(body[:bisect.bisect_right(stamps, t_s)]):
        if math.hypot(row[1] - centre[0], row[2] - centre[1]) <= APART_M:
            return t_s - row[0]
    return t_s - stamps[0]


def read_estimate(run: Path) -> list[tuple[float, float, float, float]]:
    """The vehicle's place by its own estimate (the planner's track), on the simulation's clock: the frame the
    memory's cells are laid in."""
    track = run / "mppi" / "mppi_track.jsonl"
    if not track.exists():
        return []
    rows = []
    with track.open() as stream:
        for line in stream:
            row = json.loads(line)
            rows.append((row["stamp_ns"] * 1.0e-9, *row["p"]))
    return rows


def sensor_reach_m(run: Path) -> float:
    """How far the flight's set is held to see: the stereo pair's reach (`sensor_reach_m` of the launch) or the
    lidar's guaranteed detection range (`guaranteed_lidar_detection_range_m`)."""
    log = run / "ros_drone_nav.log"
    with log.open(errors="replace") as stream:
        for line in stream:
            if "[stereo_depth_node]" in line:
                return STEREO_REACH_M
    return LIDAR_REACH_M


def looked_since_left(body, stamps, truth, truth_stamps, headings, centre, t_s: float, fields,
                      slack_s: float = STAMP_SLACK_S) -> bool:
    """Whether, since the body last left a cell before t, the cell stood in the field of the vehicle's range
    sensors while the body had been away from it for longer than the layer lives: the memory could look at the
    place the body had left. Within the sensors' range and outside their field is no look (r3695: a voxel 7.4 m
    behind the turned vehicle's shoulder). A pacing body comes back to the cell; a look before its last visit says nothing of what that
    visit left (r3665: a cell read stale 110 s after the vehicle had gone 25 m on, by a look before four visits)."""
    away_since = None
    looked = False
    for row in body[:bisect.bisect_right(stamps, t_s)]:
        if math.hypot(row[1] - centre[0], row[2] - centre[1]) <= APART_M:
            away_since = None
            looked = False
            continue
        if away_since is None:
            away_since = row[0]
        if looked or row[0] - away_since <= LAYER_LIFE_S + slack_s:
            continue
        vehicle = at(truth, truth_stamps, row[0])
        looked = in_a_field(vehicle[1:4], at(headings, truth_stamps, row[0])[1], centre, fields)
    return looked


def trace(run: Path, body, near: list[float], truth=None, headings=None) -> tuple[int, int, int, float, int, int]:
    """The body's trace over the snapshots: the largest count of occupied cells where the body is not, the count in
    the last snapshot, the stale cells over the snapshots, the oldest trace cell's age, and the snapshots read."""
    volume = swept_volume(body)
    stamps = [row[0] for row in body]
    places = read_places(run) if truth else []
    place_stamps = [row[0] for row in places]
    truth_stamps = [row[0] for row in truth] if truth else []
    estimate = read_estimate(run) if truth else []
    estimate_stamps = [row[0] for row in estimate]
    counts = []
    stale = 0
    unlooked = 0
    oldest_s = 0.0
    stereo = bool(truth) and sensor_reach_m(run) == STEREO_REACH_M
    fields = (PAIR_FIELD,) if stereo else (LIDAR_FIELD,)
    slack_s = STEREO_SLACK_S if stereo else STAMP_SLACK_S
    for path in sorted(run.glob("raw_snapshot_3d_revision_*.json")):
        document = json.loads(path.read_text())
        stamp = document["header"]["stamp"]
        t_s = stamp["sec"] + 1.0e-9 * stamp["nanosec"]
        # A snapshot that does not hold the walk says nothing of it (the flight's RAW_SNAPSHOT_BOUNDS_M).
        held = document["selection"]["bounds_m"]
        if held is not None and not (all(held[axis] <= volume[axis] for axis in range(3))
                                     and all(held[axis + 3] >= volume[axis + 3] for axis in range(3))):
            continue
        # The memory is laid in the estimate's frame, the walk is the truth's: the cells are read where the
        # estimate's error at the snapshot's stamp puts the walk, and brought back to the truth's frame.
        offset = np.zeros(3)
        if estimate:
            believed, true = at(estimate, estimate_stamps, t_s), at(truth, truth_stamps, t_s)
            offset = np.array(believed[1:4]) - np.array(true[1:4])
        shifted = tuple(volume[axis] + offset[axis % 3] + (-res_pad if axis < 3 else res_pad)
                        for axis in range(6) for res_pad in (0.25,))
        try:
            _, res, origin, _, occupied = load_memory(path, shifted)
        except ValueError:
            continue
        if not occupied.size:
            continue
        place = at(body, stamps, t_s)
        cells = np.argwhere(occupied)
        laid = origin + (cells + 0.5) * res
        centres = laid - offset
        inside = np.all((centres >= volume[:3]) & (centres <= volume[3:]), axis=1)
        apart = inside & (np.hypot(centres[:, 0] - place[1], centres[:, 1] - place[2]) > APART_M)
        apart &= ~in_the_sweep(centres, place, walk_at(body, stamps, t_s), res)
        scan = bisect.bisect_right(place_stamps, t_s) - 1
        if scan >= 0 and t_s - place_stamps[scan] <= STAMP_SLACK_S:
            vehicle = at(estimate, estimate_stamps, t_s) if estimate else at(truth, truth_stamps, t_s)
            apart &= ~in_a_kept_place(laid, places[scan][1], (vehicle[1], vehicle[2]), res)
        counts.append((t_s, int(apart.sum())))
        for centre in centres[apart]:
            age_s = trace_age_s(body, stamps, (centre[0], centre[1]), t_s)
            if age_s <= LAYER_LIFE_S + slack_s:
                continue
            if truth and headings and not looked_since_left(body, stamps, truth, truth_stamps, headings,
                                                            tuple(centre), t_s, fields, slack_s):
                unlooked += 1
                continue
            stale += 1
            oldest_s = max(oldest_s, age_s)
    if not counts:
        return -1, -1, -1, float("nan"), 0, 0
    counts.sort()
    return max(count for _, count in counts), counts[-1][1], stale, oldest_s, len(counts), unlooked


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 2
    print("| run | body | closest at s | distance m (gap, over the top) | speed there / before m/s | closing m/s | "
          "stop margin m | collisions found, stops | trace cells most / last (stale, oldest s, snapshots) | contact | "
          "in a field s |")
    print("|---|---|---|---|---|---|---|---|---|---|---|")
    for name in sys.argv[1:]:
        run = Path(name) if "/" in name else Path("log/runs") / name
        truth = read_truth(run)
        headings = read_headings(run)
        seen_in = (PAIR_FIELD, CORE_FIELD) if sensor_reach_m(run) == STEREO_REACH_M else (LIDAR_FIELD,)
        for index, body in sorted(read_bodies(run).items()):
            met = meeting(truth, body)
            collisions, stops, contact = planner_events(run, met["near"])
            most, last, stale, oldest_s, snapshots, unlooked = trace(run, body, met["near"], truth, headings)
            print(f"| {run.name} | {index} | {met['t_s']:.1f} | {met['distance_m']:.2f} ({met['gap_m']:.2f}, "
                  f"{met['over_m']:+.2f}) | {met['speed_mps']:.2f} / {met['speed_before_mps']:.2f} | "
                  f"{met['closing_mps']:.2f} | {met['margin_m']:+.2f} | {collisions}, {stops} | "
                  + (f"{most} / {last} ({stale}, {oldest_s:.1f}, {snapshots}; {unlooked})" if snapshots
                     else "no snapshot holds the walk")
                  + f" | {'yes' if contact else 'no'} | {seen_s(truth, headings, body, met['t_s'], seen_in):.1f} |")
    return 0


if __name__ == "__main__":
    sys.exit(main())
