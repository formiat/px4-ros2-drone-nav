#!/usr/bin/env python3
"""Record the offboard trajectory setpoints and the autopilot's local position of one
flight, for the controller-dynamics checks of the headless mission check.

Both streams are PX4's own (the setpoint is what the offboard adapter sends, the
local position is the estimate it reads), in PX4's local NED frame and its clock;
the checks align and read them. The file is rewritten atomically every few seconds
so an interrupted flight still leaves a usable record.

  capture_tracking_setpoints.py OUTPUT.npz [--setpoint-topic T] [--local-position-topic T]
"""

from __future__ import annotations

import argparse
import os
import threading

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy


class TrackingCapture(Node):
    def __init__(self, output: str, setpoint_topic: str, local_position_topic: str,
                 save_period_s: float) -> None:
        super().__init__("tracking_setpoint_capture")
        from px4_msgs.msg import TrajectorySetpoint, VehicleLocalPosition

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10,
                         durability=DurabilityPolicy.VOLATILE)
        self._output = output
        self._lock = threading.Lock()
        self._setpoints: list[list[float]] = []
        self._positions: list[list[float]] = []
        self.create_subscription(TrajectorySetpoint, setpoint_topic, self._on_setpoint, qos)
        self.create_subscription(VehicleLocalPosition, local_position_topic,
                                 self._on_local_position, qos)
        self.create_timer(save_period_s, self.save)

    def _on_setpoint(self, message) -> None:
        with self._lock:
            self._setpoints.append([float(message.timestamp), *message.position,
                                    *message.velocity])

    def _on_local_position(self, message) -> None:
        with self._lock:
            self._positions.append([float(message.timestamp),
                                    float(message.timestamp_sample), message.x, message.y,
                                    message.z, message.vx, message.vy, message.vz])

    def save(self) -> None:
        with self._lock:
            setpoints = np.array(self._setpoints, dtype=np.float64).reshape(-1, 7)
            positions = np.array(self._positions, dtype=np.float64).reshape(-1, 8)
        temporary = self._output + ".tmp.npz"
        np.savez_compressed(temporary, sp=setpoints, lp=positions)
        os.replace(temporary, self._output)
        print(f"setpoints {len(setpoints)} positions {len(positions)}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output")
    parser.add_argument("--setpoint-topic", default="/fmu/in/trajectory_setpoint")
    parser.add_argument("--local-position-topic",
                        default="/fmu/out/vehicle_local_position_v1")
    parser.add_argument("--save-period-s", type=float, default=5.0)
    args = parser.parse_args()
    rclpy.init()
    node = TrackingCapture(args.output, args.setpoint_topic, args.local_position_topic,
                           args.save_period_s)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # The launch tears the context down when the flight ends.
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
