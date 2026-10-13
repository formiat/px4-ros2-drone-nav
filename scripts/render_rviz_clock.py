#!/usr/bin/env python3
"""render_rviz_clock.py OUTPUT: the clock a rendered flight's RViz views run on, as the bag played back
publishes it: (wall clock, simulation clock) of every tenth /clock message, which times the views' frames
(scripts/render_flight_video.py). Evaluation only."""
import csv
import sys
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("render_rviz_clock")
    stream = open(sys.argv[1], "w", newline="")
    writer = csv.writer(stream)
    count = [0]

    def on_clock(message: Clock) -> None:
        count[0] += 1
        if count[0] % 10 == 0:
            writer.writerow([f"{time.time():.4f}",
                             f"{message.clock.sec + 1.0e-9 * message.clock.nanosec:.4f}"])
            stream.flush()

    node.create_subscription(Clock, "/clock", on_clock,
                             QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    stream.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
