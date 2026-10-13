#!/usr/bin/env python3
"""Where a raw Occupancy3D snapshot closes the way between two points.

closure.py SNAPSHOT.json SX,SY,SZ GX,GY,GZ [BODY_M]

Unknown space is free (it is to the planner). The clearance of a voxel is its
distance to the nearest occupied voxel; the widest path between the points is
found by a binary search on the clearance, and the voxels at the bottleneck
are reported. Only the snapshot's selection is known: beyond it nothing is.
"""
import json
import sys

import numpy as np
from scipy import ndimage


def load(path):
    d = json.load(open(path))
    g = d["grid"]
    res = float(g["resolution_m"])
    origin = np.array(g["origin_m"], dtype=float)
    cs = int(g["chunk_size_cells"])
    b = d["selection"]["bounds_m"]
    lo = np.floor((np.array(b[:3]) - origin) / res).astype(int)
    hi = np.ceil((np.array(b[3:]) - origin) / res).astype(int)
    shape = hi - lo
    occ = np.zeros(shape, dtype=bool)
    obs = np.zeros(shape, dtype=bool)
    for chunk in d["chunks"]:
        cx, cy, cz = chunk["index"]
        for name, target in (("observed_words_hex", obs), ("occupied_words_hex", occ)):
            for wi, word in enumerate(chunk[name]):
                w = int(word, 16)
                while w:
                    low = w & -w
                    bit = low.bit_length() - 1
                    li = wi * 64 + bit
                    x = cx * cs + li % cs - lo[0]
                    y = cy * cs + (li // cs) % cs - lo[1]
                    z = cz * cs + li // (cs * cs) - lo[2]
                    if 0 <= x < shape[0] and 0 <= y < shape[1] and 0 <= z < shape[2]:
                        target[x, y, z] = True
                    w ^= low
    return d, res, origin, lo, occ, obs


def main():
    path, start, goal = sys.argv[1], sys.argv[2], sys.argv[3]
    body = float(sys.argv[4]) if len(sys.argv) > 4 else 0.55
    d, res, origin, lo, occ, obs = load(path)
    to_cell = lambda p: tuple((np.floor((np.array(p) - origin) / res).astype(int) - lo))
    to_point = lambda c: origin + (np.array(c) + lo + 0.5) * res
    s = to_cell([float(v) for v in start.split(",")])
    g = to_cell([float(v) for v in goal.split(",")])
    for c in (s, g):
        if not all(0 <= c[i] < occ.shape[i] for i in range(3)):
            sys.exit(f"point {c} outside the selection {occ.shape}")
    clearance = ndimage.distance_transform_edt(~occ, sampling=res)
    structure = ndimage.generate_binary_structure(3, 3)
    print(f"revision {d['revision']} occupied {occ.sum()} observed {obs.sum()} "
          f"start clearance {clearance[s]:.2f} goal clearance {clearance[g]:.2f}")

    def connected(r):
        labels, _ = ndimage.label(clearance >= r, structure=structure)
        return labels[s] != 0 and labels[s] == labels[g], labels

    lo_r, hi_r = 0.0, min(clearance[s], clearance[g])
    if not connected(lo_r + 1e-9)[0]:
        print("not connected even at zero clearance inside the selection")
        return
    for _ in range(14):
        mid = 0.5 * (lo_r + hi_r)
        if connected(mid)[0]:
            lo_r = mid
        else:
            hi_r = mid
    print(f"widest path clearance {lo_r:.2f} m (body {body:.2f}: "
          f"{'open' if lo_r >= body else 'CLOSED'})")
    # The bottleneck: voxels of clearance near the widest path's that join the
    # start's component above it to the goal's.
    _, labels_above = connected(hi_r)
    start_label, goal_label = labels_above[s], labels_above[g]
    band = (clearance >= lo_r - 1e-6) & (clearance < hi_r)
    grown_s = ndimage.binary_dilation(labels_above == start_label, structure, iterations=2)
    grown_g = ndimage.binary_dilation(labels_above == goal_label, structure, iterations=2)
    pinch = np.argwhere(band & grown_s & grown_g)
    if len(pinch) == 0:
        pinch = np.argwhere(band & grown_s)
    points = np.array([to_point(c) for c in pinch[:2000]])
    if len(points):
        print(f"bottleneck voxels {len(pinch)}: centroid {np.round(points.mean(0), 2)} "
              f"span {np.round(points.min(0), 1)}..{np.round(points.max(0), 1)}")
    if lo_r < body:
        comp = labels_above == start_label if lo_r < body else None
        labels_body, _ = ndimage.label(clearance >= body, structure=structure)
        region = np.argwhere(labels_body == labels_body[s]) if labels_body[s] else np.empty((0, 3))
        if len(region):
            pts = np.array([to_point(c) for c in region[:: max(1, len(region) // 5000)]])
            print(f"start's component at the body's clearance: {len(region)} voxels, "
                  f"span {np.round(pts.min(0), 1)}..{np.round(pts.max(0), 1)}")


if __name__ == "__main__":
    main()
