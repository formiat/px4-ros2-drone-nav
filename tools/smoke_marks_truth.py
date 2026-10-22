#!/usr/bin/env python3
"""smoke_marks_truth.py RUN...: every region whose marks the depth node laid (the SMOKE_FACE_MARKS lines of a run's
ros_drone_nav.log, specification K44) projected into the world by its centroid and rim depth in the left camera
(HFOV 120 deg) at the true pose of the moment, and judged against the run's dense smoke boxes (sigma 0.5/m or more,
from the manifest): TRUE within 1.5 m of a box, false otherwise. One line a region: the run, the simulation time, the
vehicle's true position, the face's position, its distance to the nearest box, the verdict and the region's summary
(reason, cells, centroid, rim hits, depth, spread, trend, see-through, rays laid). On the host, python3 -I, from the
repository's root. Roadmap item 18, the smoke face detector."""
import bisect
import csv
import json
import math
import re
import sys
from pathlib import Path

FX = 640.0 / math.tan(math.radians(60.0))
CX, CY = 639.5, 479.5


def boxes(run):
    manifest = json.loads(Path(f"log/runs/{run}/manifest.json").read_text())
    zones = manifest.get("effective_overrides", {}).get("SMOKE_ZONES", "")
    out = []
    for item in zones.split(";"):
        values = [float(a) for a in item.split(",") if a]
        if len(values) == 7 and values[6] >= 0.5:
            out.append(values)
    return out


def distance_to_box(point, box):
    d = [max(abs(point[i] - box[i]) - box[3 + i] / 2.0, 0.0) for i in range(3)]
    return math.sqrt(sum(a * a for a in d))


def poses(run):
    rows = []
    for r in csv.reader(open(f"log/runs/{run}/gz_pose.csv")):
        if len(r) >= 9:
            rows.append([float(a) for a in r[:9]])
    rows.sort(key=lambda r: r[8])
    return rows


def yaw_of(q):
    w, x, y, z = q
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def main():
    for run in sys.argv[1:]:
        rows = poses(run)
        walls = [r[8] for r in rows]
        dense = boxes(run)
        for line in open(f"log/runs/{run}/ros_drone_nav.log"):
            if "SMOKE_FACE_MARKS" not in line:
                continue
            m = re.search(r"\[(\d+\.\d+)\].*SMOKE_FACE_MARKS rays=(\d+)(.*)$", line)
            k = min(bisect.bisect_left(walls, float(m.group(1))), len(rows) - 1)
            r = rows[k]
            yaw = yaw_of(r[4:8])
            for s in m.group(3).split():
                f = re.match(r"(\d+)@(\d+),(\d+):(\w+):rim(\d+)@([0-9.]+)\+([0-9.]+):trend([0-9.]+):free(\d+)", s)
                if not f:
                    continue
                cells, col, row, reason, rim, depth, spread, trend, free = f.groups()
                cells, col, row, depth = int(cells), int(col), int(row), float(depth)
                left = -(4 * col - CX) / FX * depth
                up = -(4 * row - CY) / FX * depth
                px = r[1] + math.cos(yaw) * depth - math.sin(yaw) * left
                py = r[2] + math.sin(yaw) * depth + math.cos(yaw) * left
                pz = r[3] + up
                dist = min((distance_to_box((px, py, pz), b) for b in dense), default=99.0)
                truth = "TRUE" if dist <= 1.5 else "false"
                print(f"{run} t={r[0]:6.1f} veh=({r[1]:5.1f},{r[2]:5.1f},{r[3]:4.1f}) "
                      f"face=({px:5.1f},{py:5.1f},{pz:4.1f}) d_box={dist:4.1f} {truth} {reason} cells={cells} "
                      f"col={col} row={row} rim={rim} depth={depth:.2f} spread={spread} trend={trend} "
                      f"see={int(free) / cells * 100:.2f}% rays={m.group(2)}")


if __name__ == "__main__":
    main()
