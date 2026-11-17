#!/usr/bin/env python3
"""wall_by_body.py RUN [--body X,Y] [--wall-beyond Y] [--from T]: the wall a warm body stands by, in the memory's
snapshots of a flight (roadmap item 21; specification K65: a wall beside which a warm body stands stays a wall).

The flight carries a body that stands (MOVING_BODIES with a walk of no length, scripts/moving_bodies.py) a step
from a wall of the corridor, and the memory's snapshots (RAW_SNAPSHOT_BOUNDS_M). For each snapshot after T seconds
that holds occupied cells there, the tool reads the wall's side of the body (every cell south of --wall-beyond,
default the corridor's south wall by the body of the measurement of 2026-11-17 at 40,-18.1) in the truth's frame
(the memory's cells brought there by the estimate's error at the snapshot's stamp, as tools/body_encounters.py
does), and prints the widest run of columns, 0.25 m each, within 1.5 m of the body along the wall, that hold no
occupied cell on a level between 0.1 and 1.7 m over the body's floor: the hole in the wall at the body's height.
The body's own cells count with the wall's (no sensor sees the wall right behind it).
With -v the wall is drawn level by level from the floor to 4.75 m over it.

    MOVING_BODIES="40,-18.1,40,-18.1,1.0,1.2" RAW_SNAPSHOT_BOUNDS_M="-33,-26,0,60,42.5,18" \
        tools/fly_until_valid.sh [--lidar] N 4
    tools/wall_by_body.py rN --from 250

r3793 (the stereo set, the flight code of 01107340): a hole 1.5 m wide in every snapshot after the pass; r3795
(81a31584, a column that stands over the tallest body is a surface): none. The lidar set none in both (r3794,
r3796).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))

import body_encounters as be  # noqa: E402

STEP_M = 0.25
ALONG_M = 6.0
BESIDE_M = 1.5


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run")
    parser.add_argument("--body", default="40,-18.1", help="where the body stands, x,y in the world's frame")
    parser.add_argument("--wall-beyond", type=float, default=-17.6,
                        help="the wall's side of the body: every cell with y under this")
    parser.add_argument("--from", dest="t_from", type=float, default=0.0)
    parser.add_argument("-v", action="store_true")
    args = parser.parse_args()
    run = Path(args.run) if "/" in args.run else Path("log/runs") / args.run
    body_x, body_y = (float(value) for value in args.body.split(","))
    truth = be.read_truth(run)
    truth_stamps = [row[0] for row in truth]
    estimate = be.read_estimate(run)
    estimate_stamps = [row[0] for row in estimate]
    floor_m = be.read_bodies(run)[0][0][3]
    snapshots = []
    for path in run.glob("raw_snapshot_3d_revision_*.json"):
        stamp = json.loads(path.read_text())["header"]["stamp"]
        t_s = stamp["sec"] + 1.0e-9 * stamp["nanosec"]
        if t_s < args.t_from:
            continue
        offset = np.zeros(3)
        if estimate:
            offset = (np.array(be.at(estimate, estimate_stamps, t_s)[1:4])
                      - np.array(be.at(truth, truth_stamps, t_s)[1:4]))
        box = (body_x - ALONG_M, body_y - 3.0, floor_m, body_x + ALONG_M, body_y + 3.0, floor_m + 4.75)
        try:
            _, res, origin, _, occupied = be.load_memory(path, tuple(box[axis] + offset[axis % 3] for axis in range(6)))
        except ValueError:
            continue
        cells = np.argwhere(occupied)
        if len(cells):
            snapshots.append((t_s, origin + (cells + 0.5) * res - offset, offset))
    columns = np.arange(body_x - ALONG_M + STEP_M / 2.0, body_x + ALONG_M, STEP_M)
    for t_s, centres, offset in sorted(snapshots, key=lambda snapshot: snapshot[0]):
        wall = centres[centres[:, 1] < args.wall_beyond]
        vehicle = be.at(truth, truth_stamps, t_s)
        widest = 0
        rows = []
        for level in np.arange(floor_m + 4.625, floor_m, -STEP_M):
            held = [bool(((np.abs(wall[:, 0] - x) < STEP_M / 2.0 + 1.0e-3)
                          & (np.abs(wall[:, 2] - level) < STEP_M / 2.0 + 1.0e-3)).any()) for x in columns]
            rows.append((level, held))
            if floor_m + 0.1 < level < floor_m + 1.7:
                gap = 0
                for x, hit in zip(columns, held):
                    gap = 0 if hit or abs(x - body_x) > BESIDE_M else gap + 1
                    widest = max(widest, gap)
        print(f"{run.name} t {t_s:.0f} s, the vehicle at ({vehicle[1]:.1f}, {vehicle[2]:.1f}, {vehicle[3]:.1f}), "
              f"the estimate {offset[1]:+.2f} m off across: the widest hole beside the body {widest * STEP_M:.2f} m")
        if args.v:
            for level, held in rows:
                print(f"   z {level:.2f} " + "".join("#" if hit else "." for hit in held))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
