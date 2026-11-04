#!/usr/bin/env python3
"""magnetic_heading.py RUN [RUN...]: the simulated magnetometer's heading against the truth (roadmap item 39 stage 3,
item 40). The field the camera estimator logs (VIO_MAGNETOMETER, body axes, simulation clock) is turned into the
world by the true attitude (gz_pose.csv); a consistent field has a constant horizontal direction there, whatever the
vehicle's heading. For each axis convention the line gives that direction's mean, its scatter (standard deviation
and the 95th percentile of the deviation), its drift over the flight and its correlation with the true heading;
the convention whose direction stays put is the sensor's. Host."""
import csv
import math
import re
import sys
from pathlib import Path

import numpy as np

CONVENTIONS = {
    "body FLU": lambda m: m,
    "body FRD": lambda m: np.array([m[0], -m[1], -m[2]]),
    "world ENU": None,  # the field already in the world: no rotation by the attitude
}


def quaternion_matrix(w, x, y, z):
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def main() -> int:
    for run in sys.argv[1:]:
        directory = Path("log/runs") / run
        readings = []
        for line in open(directory / "ros_drone_nav.log", errors="ignore"):
            match = re.search(r"VIO_MAGNETOMETER stamp_s=([0-9.]+) field_ut=\(([-0-9.]+),([-0-9.]+),([-0-9.]+)\)", line)
            if match:
                readings.append([float(v) for v in match.groups()])
        truth = np.array([[float(v) for v in row[:8]] for row in csv.reader(open(directory / "gz_pose.csv"))
                          if row and row[0][:1].isdigit()])
        if not readings or not len(truth):
            print(f"{run}: no readings or no truth")
            continue
        readings = np.array(readings)
        readings = readings[(readings[:, 0] > truth[0, 0]) & (readings[:, 0] < truth[-1, 0])]
        # The truth's attitude interpolated to each reading's stamp: the nearest row, up to 12 ms away, turned
        # the field by the vehicle's rates and read 0.47 degrees of scatter where the sensor has 0.03 (r2842).
        quaternions = truth[:, 4:8].copy()
        for row in range(1, len(quaternions)):
            if np.dot(quaternions[row], quaternions[row - 1]) < 0.0:
                quaternions[row] = -quaternions[row]
        attitude = np.stack([np.interp(readings[:, 0], truth[:, 0], quaternions[:, k]) for k in range(4)], axis=1)
        attitude /= np.linalg.norm(attitude, axis=1, keepdims=True)
        true_yaw = np.array([math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)) for w, x, y, z in attitude])
        print(f"{run}: {len(readings)} readings over {readings[-1, 0] - readings[0, 0]:.0f} s, "
              f"field {np.linalg.norm(readings[:, 1:], axis=1).mean():.1f} uT, "
              f"true heading spans {math.degrees(np.ptp(np.unwrap(true_yaw))):.0f} deg")
        for name, convert in CONVENTIONS.items():
            directions = []
            for (stamp, mx, my, mz), (w, x, y, z) in zip(readings, attitude):
                body = np.array([mx, my, mz])
                if convert is None:
                    world = body
                else:
                    world = quaternion_matrix(w, x, y, z) @ convert(body)
                directions.append(math.atan2(world[1], world[0]))
            directions = np.unwrap(np.array(directions))
            mean = float(np.mean(directions))
            deviation = np.degrees(directions - mean)
            drift = np.polyfit(readings[:, 0] - readings[0, 0], deviation, 1)[0] * (readings[-1, 0] - readings[0, 0])
            correlation = np.corrcoef(np.unwrap(true_yaw), directions)[0, 1] if np.std(true_yaw) > 0 else float("nan")
            print(f"  {name:9s}: direction {math.degrees(wrap(mean)):7.2f} deg, scatter sd {np.std(deviation):6.2f} "
                  f"p95 {np.percentile(np.abs(deviation), 95):6.2f} deg, drift over the flight {drift:+6.2f} deg, "
                  f"correlation with the true heading {correlation:+.2f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
