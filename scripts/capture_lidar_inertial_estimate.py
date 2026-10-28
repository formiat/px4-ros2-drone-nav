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
        self._rows: list[tuple[float, float, float, float, float, int, float, float]] = []
        self.create_subscription(PoseStamped, topic, self._on_pose, 10)
        # The autopilot's attitude beside the estimate (px4_attitude.csv next to the output: the sample's stamp in
        # seconds, roll, pitch, yaw in NED): the level the estimator's registration is held to since 2026-10-25.
        self._attitude_rows: list[tuple[float, float, float, float]] = []
        try:
            from px4_msgs.msg import VehicleAttitude
            from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
            qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE)
            self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self._on_attitude, qos)
        except ImportError:
            pass
        self.create_timer(save_period_s, self.save)

    def _on_pose(self, message) -> None:
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        q = message.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        # Roll and pitch too (the root goal of 2026-10-24, part 1): the estimate's
        # tilt against the truth is what its vertical drift is read by.
        roll = math.atan2(2.0 * (q.w * q.x + q.y * q.z), 1.0 - 2.0 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x))))
        healthy = 1 if message.header.frame_id == "map" else 0
        with self._lock:
            self._rows.append((stamp, message.pose.position.x, message.pose.position.y,
                               message.pose.position.z, yaw, healthy, roll, pitch))

    def _on_attitude(self, message) -> None:
        w, x, y, z = (float(v) for v in message.q)
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        with self._lock:
            self._attitude_rows.append((message.timestamp_sample * 1e-6, roll, pitch, yaw))

    def save(self) -> None:
        with self._lock:
            rows = list(self._rows)
            attitude_rows = list(self._attitude_rows)
            if attitude_rows:
                # Beside the estimate, never in its way: a failure here is logged and the estimate is still saved.
                attitude_path = os.path.join(os.path.dirname(self._output), "px4_attitude.csv")
                try:
                    with open(attitude_path + ".tmp", "w", newline="", encoding="utf-8") as stream:
                        csv.writer(stream).writerows(attitude_rows)
                    os.replace(attitude_path + ".tmp", attitude_path)
                except OSError as error:
                    print(f"attitude record not saved: {error}", flush=True)
            temporary = self._output + ".tmp"
            with open(temporary, "w", newline="", encoding="utf-8") as stream:
                csv.writer(stream).writerows(rows)
            os.replace(temporary, self._output)
        print(f"estimates {len(rows)} attitude {len(attitude_rows)}", flush=True)


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
