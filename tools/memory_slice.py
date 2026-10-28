#!/usr/bin/env python3
"""A slice of a raw Occupancy3D snapshot as text, round a point.

memory_slice.py SNAPSHOT.json AXIS VALUE X,Y,Z [HALF_M] [STEP_M]

AXIS is x, y or z: the slice is the plane AXIS = VALUE, drawn HALF_M (12 m unless given) either side of the point's
other two coordinates every STEP_M (0.5 m): # occupied, . observed free, a blank unknown, ? outside the snapshot's
selection, S the point. Rows run from the larger coordinate down. Roadmap item 32 on the lidar (K49) read the heat ball
in the memory this way (r2292: no mark between the vehicle and the source; r2294).
"""
import sys

import numpy as np

from closure import load


def main():
    path, axis, value, point = sys.argv[1], sys.argv[2], float(sys.argv[3]), sys.argv[4]
    half = float(sys.argv[5]) if len(sys.argv) > 5 else 12.0
    step = float(sys.argv[6]) if len(sys.argv) > 6 else 0.5
    centre = np.array([float(v) for v in point.split(",")])
    _, res, origin, lo, occ, obs = load(path)
    fixed = "xyz".index(axis)
    across, down = [i for i in range(3) if i != fixed]
    print(f"{axis} = {value}: rows {'xyz'[down]} from {centre[down] + half:.1f} down, "
          f"columns {'xyz'[across]} from {centre[across] - half:.1f}")
    for row in np.arange(centre[down] + half, centre[down] - half - 1e-9, -step):
        line = ""
        for column in np.arange(centre[across] - half, centre[across] + half + 1e-9, step):
            at = np.zeros(3)
            at[fixed], at[across], at[down] = value, column, row
            cell = tuple(np.floor((at - origin) / res).astype(int) - lo)
            if not all(0 <= cell[i] < occ.shape[i] for i in range(3)):
                line += "?"
            elif abs(column - centre[across]) < step / 2 and abs(row - centre[down]) < step / 2:
                line += "S"
            else:
                line += "#" if occ[cell] else ("." if obs[cell] else " ")
        print(f"{row:6.1f} {line}")


if __name__ == "__main__":
    main()
