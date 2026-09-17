#!/usr/bin/env python3
"""Record the lidar-inertial odometry estimate of one flight, in the map frame,
for the mission check's comparison with the true pose. The file is rewritten
atomically every few seconds so an interrupted flight still leaves a record.

  capture_lidar_inertial_estimate.py OUTPUT.csv [--topic T]
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import threading

import rclpy
from rclpy.node import Node


class EstimateCapture(Node):
    def __init__(self, output: str, topic: str, save_period_s: float) -> None:
        super().__init__("lidar_inertial_estimate_capture")
        from geometry_msgs.msg import PoseStamped
        self._output = output
        self._lock = threading.Lock()
        self._rows: list[tuple[float, float, float, float, float, int]] = []
        self.create_subscription(PoseStamped, topic, self._on_pose, 10)
        self.create_timer(save_period_s, self.save)

    def _on_pose(self, message) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        q = message.pose.orientation
        yaw = math.atan2(2.0 * q.w * q.z, 1.0 - 2.0 * q.z * q.z)
        healthy = 1 if message.header.frame_id == "map" else 0
        with self._lock:
            self._rows.append((stamp, message.pose.position.x, message.pose.position.y,
                               message.pose.position.z, yaw, healthy))

    def save(self) -> None:
        with self._lock:
            rows = list(self._rows)
        temporary = self._output + ".tmp"
        with open(temporary, "w", newline="", encoding="utf-8") as stream:
            csv.writer(stream).writerows(rows)
        os.replace(temporary, self._output)
        print(f"estimates {len(rows)}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output")
    parser.add_argument("--topic", default="/drone_city_nav/lidar_inertial_odometry/pose")
    parser.add_argument("--save-period-s", type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = EstimateCapture(args.output, args.topic, args.save_period_s)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.save()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
