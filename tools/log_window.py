#!/usr/bin/env python3
"""log_window.py RUN T0 T1 [PATTERN...]: the ROS log lines of a run between two simulation times (s), with the
simulation time in front; by default the camera estimator's, the offboard's landing and the mission's events.
The wall stamps of the ROS log are put on the simulation clock through the true pose record (gz_pose.csv)."""
from __future__ import annotations

import re
import sys

import numpy as np

DEFAULT_PATTERNS = ("VISUAL_INERTIAL_ODOMETRY", "DEAD_RECKONING_LANDING", "GOAL_UNREACHABLE", "VEHICLE_",
                    "MISSION_RESULT", "PLANNER_HEALTH", "EXECUTION_REVOCATION")


def events(run: str, patterns=DEFAULT_PATTERNS, t0: float = 0.0, t1: float = float("inf")):
    """(simulation time, message) of every log line of RUN holding one of the patterns, in order."""
    clock = np.loadtxt(f"log/runs/{run}/gz_pose.csv", delimiter=",", usecols=(0, 8))
    found = []
    with open(f"log/runs/{run}/ros_drone_nav.log", errors="ignore") as log:
        for line in log:
            if not any(p in line for p in patterns):
                continue
            stamp = re.search(r"\[(\d{10}\.\d+)\]", line)
            if not stamp:
                continue
            sim = float(np.interp(float(stamp.group(1)), clock[:, 1], clock[:, 0]))
            if t0 <= sim <= t1:
                found.append((sim, line.split("]: ", 1)[-1].strip()))
    return found


if __name__ == "__main__":
    for sim, text in events(sys.argv[1], tuple(sys.argv[4:]) or DEFAULT_PATTERNS,
                            float(sys.argv[2]), float(sys.argv[3])):
        print(f"{sim:7.2f} {text[:230]}")
