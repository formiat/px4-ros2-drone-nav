#!/usr/bin/env python3
"""unlooked_bodies.py [--grid=PATH] RUN [RUN...]: every body the obstacle memory read and then took off in a
flight, and whether a sensor could look at where it stood while it went unseen (roadmap item 21). Host; one row
a body taken off.

A RUN is a run's name under log/runs or a directory that holds a run's record. From it: the memory's own account
(ros_drone_nav.log: the FOLLOWED_BODIES line of every scan that holds a body, each body's place, its height and
the time it has gone unseen), the vehicle's true pose (gz_pose.csv) and the true bodies (moving_bodies.csv). The
log's lines are stamped by the wall clock: they are brought to the simulation's by the LIDAR3D_MEMORY lines, which
carry both (one a second, the stamps between them interpolated).

A body is taken off when the next scan's line holds no read body within 1.5 m of where this scan kept one unseen
(or no line follows for 0.6 s: a scan with no body writes none). Over the scans it stood unseen through, by the
truth, a sensor could look at its place when the whole column (its foot, its middle and its top) stood in the
sensor's field, within the range the sensor reads a body at, with no occupied cell of the truth grid on the ray:
the stereo set's pair (5.2 m, 60 degrees either side of the heading and 52.4 up and down) or its thermal core
(19 m, 28.5 and 22.2 degrees); the lidar set's lidar (19 m all round). The time-of-flight modules (2.8 m) are not
counted. The estimate's error is not removed: the place is the memory's, the pose the truth's.

The verdict: `looked` when a sensor could look for a tenth of a second at least; `far` when none could and the
place stood farther than 10 m from the vehicle whenever it was unseen (specification K66: a place is not kept
beyond that); `UNLOOKED` when none could within 10 m: the body was taken off by the clock, not by a look. Beside
it: how long it stood unseen, for how much of that each sensor could look and what kept it from the place the rest
of the time (`range`; `aside`, off its field across; `under` or `over`, off it up or down; `wall`, an occupied cell
of the truth on the ray), the place's distance and bearing
off the heading and the vehicle's speed and its course off the heading at the last scan that showed the body, the
nearest and the farthest the vehicle stood from the place while it was unseen, and where the nearest true body
stood from the place when it was taken off. `--grid=PATH` names another world's truth grid (the urban 0.25 m grid
of log/tools/truth25 otherwise). The batch prints its start, its total and its count."""
import bisect, csv, math, re, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, 'scripts')
from validate_static_cooperative_scenario import CHUNK_SIZE, Occupancy3D

GRID = Path('log/tools/truth25/urban_r025.occupancy3d')
RUNS = []
for argument in sys.argv[1:]:
    if argument.startswith('--grid='):
        GRID = Path(argument.split('=', 1)[1])
    else:
        RUNS.append(Path(argument) if '/' in argument else Path('log/runs') / argument)
if not RUNS:
    sys.exit(__doc__)
OCC = Occupancy3D.load(GRID)
B = OCC.bounds
RES = B.resolution_m
KEPT_WITHIN_M = 10.0
# (name, range m, half field across rad, half field up and down rad); None: all round.
PAIR = ('pair', 5.2, math.radians(60.0), math.radians(52.4))
CORE = ('core', 19.0, math.radians(28.5), math.radians(22.2))
LIDAR = ('lidar', 19.0, None, None)
WALL = re.compile(r'\[(\d+\.\d+)\] \[obstacle_memory_3d_node\]: (\w+)')
BODY = re.compile(r'\[(-?[\d.]+) (-?[\d.]+) (-?[\d.]+) r([\d.]+) h([\d.]+) v(-?[\d.]+),(-?[\d.]+) n(\d+) a([\d.]+) '
                  r'd([\d.]+) (\w+) u([\d.]+)\]')
STAMP = re.compile(r'stamp_ns=(\d+)')


def occupied(point):
    i = math.floor((point[0] - B.origin_x_m) / RES)
    j = math.floor((point[1] - B.origin_y_m) / RES)
    k = math.floor((point[2] - B.origin_z_m) / RES)
    words = OCC.chunks.get((i // CHUNK_SIZE, j // CHUNK_SIZE, k // CHUNK_SIZE))
    if words is None:
        return False
    bit = i % CHUNK_SIZE + CHUNK_SIZE * (j % CHUNK_SIZE + CHUNK_SIZE * (k % CHUNK_SIZE))
    return (words[bit // 64] >> (bit % 64)) & 1 == 1


def clear_ray(eye, point):
    """No occupied truth cell between the eye and the point, the last 0.6 m (the body's own place) left out."""
    delta = point - eye
    length = float(np.linalg.norm(delta))
    steps = int((length - 0.6) / (0.5 * RES))
    return not any(occupied(eye + delta * (step * 0.5 * RES / length)) for step in range(1, steps + 1))


def hidden(sensor, pose, column):
    """What keeps the sensor, on the vehicle at this true pose, from the whole column: None when it could look,
    else the first of `range`, `aside` (off the field across), `under` or `over` (off it up or down), `wall`."""
    _, reach, across, updown = sensor
    eye = np.array(pose[1:4])
    for point in column:
        delta = point - eye
        if float(np.linalg.norm(delta)) > reach:
            return 'range'
        if across is not None:
            bearing = wrap(math.atan2(delta[1], delta[0]) - pose[4])
            # The field is a pyramid: the height is read against the distance along the heading.
            ahead = math.hypot(delta[0], delta[1]) * math.cos(bearing)
            if ahead <= 0.0 or abs(bearing) > across:
                return 'aside'
            if abs(math.atan2(delta[2], ahead)) > updown:
                return 'under' if delta[2] < 0.0 else 'over'
        if not clear_ray(eye, point):
            return 'wall'
    return None


def wrap(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def read(run):
    truth = []
    for row in csv.reader((run / 'gz_pose.csv').open()):
        try:
            t, x, y, z, qw, qx, qy, qz = (float(v) for v in row[:8])
        except ValueError:
            continue
        truth.append((t, x, y, z, math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))))
    walkers = {}
    if (run / 'moving_bodies.csv').exists():
        for row in csv.DictReader((run / 'moving_bodies.csv').open()):
            walkers.setdefault(row['index'], []).append((float(row['sim_s']), float(row['x']), float(row['y'])))
    clock = []          # (wall s, simulation s) of the lines that carry both
    lines = []          # (wall s, the read bodies of a scan: x, y, z, height, unseen s)
    cameras = False
    for line in (run / 'ros_drone_nav.log').open(errors='replace'):
        cameras = cameras or '[stereo_depth_node]' in line
        found = WALL.search(line)
        if not found:
            continue
        if found.group(2) == 'LIDAR3D_MEMORY' and STAMP.search(line):
            clock.append((float(found.group(1)), int(STAMP.search(line).group(1)) * 1e-9))
        elif found.group(2) == 'FOLLOWED_BODIES':
            lines.append((float(found.group(1)),
                          [(float(m[1]), float(m[2]), float(m[3]), float(m[5]), float(m[12]))
                           for m in BODY.finditer(line) if m[11] != 'step']))
    walls = [pair[0] for pair in clock]

    def simulation(wall):
        at = min(max(bisect.bisect_left(walls, wall), 1), len(clock) - 1)
        (w0, s0), (w1, s1) = clock[at - 1], clock[at]
        return s0 + (wall - w0) * (s1 - s0) / (w1 - w0) if w1 > w0 else s0

    scans = [(simulation(wall), held) for wall, held in lines] if len(clock) >= 2 else []
    return truth, walkers, scans, cameras


def report(run):
    truth, walkers, scans, cameras = read(run)
    stamps = [sample[0] for sample in truth]
    tracks = [([sample[0] for sample in track], track) for track in walkers.values()]

    def pose_at(t):
        return truth[min(len(truth) - 1, bisect.bisect_left(stamps, t))]

    sensors = (PAIR, CORE) if cameras else (LIDAR,)
    rows = []
    for index, (t, held) in enumerate(scans):
        follows = index + 1 < len(scans) and scans[index + 1][0] - t <= 0.6
        after = scans[index + 1][1] if follows else []
        for x, y, z, height, unseen in held:
            if unseen <= 0.0 or any(math.hypot(x - other[0], y - other[1]) <= 1.5 for other in after):
                continue
            # The scans it stood unseen through, back to the first of its stand at this place.
            first = index
            while first > 0 and any(math.hypot(x - o[0], y - o[1]) <= 0.05 and o[4] > 0.0 for o in scans[first - 1][1]):
                first -= 1
            taken_off = scans[index + 1][0] if follows else t + 0.15
            shown = pose_at(scans[max(first - 1, 0)][0])
            column = [np.array((x, y, level)) for level in (z - 0.5 * height + 0.1, z, z + 0.5 * height - 0.1)]
            looked = {sensor[0]: 0.0 for sensor in sensors}
            kept_by = {}
            any_s = total = 0.0
            apart = []
            for step in range(first, index + 1):
                end = scans[step + 1][0] if step < index else taken_off
                span = end - scans[step][0]
                pose = pose_at(end)
                reasons = {sensor[0]: hidden(sensor, pose, column) for sensor in sensors}
                seen_by = [name for name, reason in reasons.items() if reason is None]
                for name in seen_by:
                    looked[name] += span
                for name, reason in reasons.items():
                    if reason is not None:
                        kept_by[(name, reason)] = kept_by.get((name, reason), 0.0) + span
                any_s += span if seen_by else 0.0
                total += span
                apart.append(math.hypot(x - pose[1], y - pose[2]))
            later = pose_at(shown[0] + 0.5)
            vx, vy = (later[1] - shown[1]) / 0.5, (later[2] - shown[2]) / 0.5
            speed = math.hypot(vx, vy)
            walker = min((math.hypot(track[min(len(track) - 1, bisect.bisect_left(times, taken_off))][1] - x,
                                     track[min(len(track) - 1, bisect.bisect_left(times, taken_off))][2] - y)
                          for times, track in tracks), default=None)
            verdict = 'looked' if any_s >= 0.1 else ('far' if min(apart) > KEPT_WITHIN_M else 'UNLOOKED')
            why = ' '.join(f'{name} {reason} {seconds:.1f}' for (name, reason), seconds in sorted(kept_by.items()))
            rows.append((verdict, taken_off, x, y, z, total, any_s, looked, why,
                         math.hypot(x - shown[1], y - shown[2]),
                         wrap(math.atan2(y - shown[2], x - shown[1]) - shown[4]), speed,
                         wrap(math.atan2(vy, vx) - shown[4]) if speed > 0.2 else None,
                         min(apart), max(apart), walker))
    counts = {verdict: sum(1 for row in rows if row[0] == verdict) for verdict in ('looked', 'far', 'UNLOOKED')}
    print(f'{run.name}: {"stereo set" if cameras else "lidar set"}, {len(scans)} scans with a body,'
          f' {len(rows)} bodies taken off: looked {counts["looked"]}, far {counts["far"]},'
          f' UNLOOKED {counts["UNLOOKED"]}')
    for verdict, t, x, y, z, total, any_s, looked, why, flat, bearing, speed, course, near, far, walker in rows:
        by = ' '.join(f'{name} {seconds:.1f}' for name, seconds in looked.items())
        print(f'  {verdict:8s} {t:7.1f} s  place {x:6.1f} {y:6.1f} {z:4.1f}  unseen {total:4.1f} s,'
              f' a sensor could look {any_s:4.1f} s ({by}), kept from it: {why or "nothing"};'
              f' the vehicle {near:.1f} to {far:.1f} m off;'
              f' last shown {flat:4.1f} m at {math.degrees(bearing):+4.0f} deg, {speed:.1f} m/s'
              + (f' course {math.degrees(course):+4.0f} deg' if course is not None else ' at rest')
              + (f'; true body {walker:.1f} m from the place' if walker is not None else ''))
    return counts


started = time.time()
print(f'unlooked_bodies: {len(RUNS)} flights, started {time.strftime("%F %T %z")}')
totals = {'looked': 0, 'far': 0, 'UNLOOKED': 0}
for done, run in enumerate(RUNS, 1):
    for verdict, count in report(run).items():
        totals[verdict] += count
    print(f'  done {done} of {len(RUNS)} in {time.time() - started:.0f} s')
print(f'bodies taken off: looked {totals["looked"]}, far {totals["far"]}, UNLOOKED {totals["UNLOOKED"]}'
      ' (within 10 m, no sensor able to look at the place while it was unseen)')
