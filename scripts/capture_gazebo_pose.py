#!/usr/bin/env python3
"""Record the true world pose of one Gazebo model (simulation time, position,
quaternion, wall time of reception) from the world's pose/info topic, for the
position-estimate check of the headless mission check. The wall time is what a
log-stamped estimate is held against when the simulation runs slower than the
wall clock. The file is rewritten atomically every few seconds.

  capture_gazebo_pose.py OUTPUT.csv --world WORLD --model MODEL
"""

from __future__ import annotations

import argparse
import csv
import os
import threading
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output")
    parser.add_argument("--world", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--save-period-s", type=float, default=2.0)
    args = parser.parse_args()

    rows: list[tuple[float, ...]] = []
    lock = threading.Lock()

    def on_poses(message: Pose_V) -> None:
        for pose in message.pose:
            if pose.name != args.model:
                continue
            stamp = message.header.stamp.sec + message.header.stamp.nsec * 1e-9
            with lock:
                rows.append((stamp, pose.position.x, pose.position.y, pose.position.z,
                             pose.orientation.w, pose.orientation.x, pose.orientation.y,
                             pose.orientation.z, time.time()))
            break

    node = Node()
    topic = f"/world/{args.world}/pose/info"
    if not node.subscribe(Pose_V, topic, on_poses):
        print(f"subscribe failed: {topic}", flush=True)
        return 1
    print(f"subscribed {topic} model={args.model}", flush=True)
    saved = 0
    while True:
        time.sleep(args.save_period_s)
        with lock:
            snapshot = list(rows)
        if len(snapshot) == saved:
            continue
        temporary = args.output + ".tmp"
        with open(temporary, "w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerows(snapshot)
        os.replace(temporary, args.output)
        saved = len(snapshot)
        print(f"poses {saved} last {snapshot[-1][:4]}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
