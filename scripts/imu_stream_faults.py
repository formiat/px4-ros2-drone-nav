#!/usr/bin/env python3
"""The autopilot's IMU stream failing beside the flight, an evaluation component
nobody tells the vehicle about (specification K28; the register's entry on a hole
of seconds, r720 and r723, where another task froze the host). It relays the
autopilot's IMU samples to the topic the camera estimator reads, and on a fixed
schedule drops them for a while, as a stalled host or a lost link does
(IMU_FAULTS=holes). Its schedule never leaves this process; no production node
reads it (the contract test holds it there); the vehicle meets a hole only as
samples that do not come. The samples are relayed as they arrive, their bytes
untouched; the schedule runs on their own stamps, the simulation clock.
"""
from __future__ import annotations

import argparse
import csv
import sys
import threading
import time
from dataclasses import dataclass
from pathlib import Path

RELAY_SUFFIX = "/relayed"


@dataclass(frozen=True)
class ImuHole:
    start_s: float
    length_s: float

    @property
    def end_s(self) -> float:
        return self.start_s + self.length_s


def schedule(profile: str) -> list[ImuHole]:
    """The holes of a flight: two seconds a minute in, four seconds two minutes
    in, both longer than the filter's unaided second, the first within what the
    autopilot's own estimate carries, the second the length r723 left."""
    if profile == "holes":
        return [ImuHole(60.0, 2.0), ImuHole(120.0, 4.0)]
    raise ValueError(f"unknown IMU fault profile '{profile}'")


def hole_at(holes: list[ImuHole], t_s: float) -> ImuHole | None:
    for hole in holes:
        if hole.start_s <= t_s < hole.end_s:
            return hole
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--topic", default="/fmu/out/sensor_combined")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    import rclpy
    from px4_msgs.msg import SensorCombined
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    holes = schedule(args.profile)
    with args.output.with_suffix(".schedule.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["start_s", "length_s"])
        for hole in holes:
            writer.writerow([f"{hole.start_s:.3f}", f"{hole.length_s:.3f}"])
    rclpy.init()
    node = Node("imu_stream_faults")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                     durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=100)
    publisher = node.create_publisher(SensorCombined, args.topic + RELAY_SUFFIX, qos)
    log = args.output.open("w", newline="")
    writer = csv.writer(log)
    writer.writerow(["stamp_s", "action"])
    lock = threading.Lock()
    dropped = [0]

    def relay(message: SensorCombined) -> None:
        # The autopilot runs in lockstep on the simulation clock: its stamp,
        # microseconds, is the simulation time.
        stamp_s = message.timestamp * 1.0e-6
        if hole_at(holes, stamp_s) is not None:
            with lock:
                dropped[0] += 1
                if dropped[0] % 25 == 1:
                    writer.writerow([f"{stamp_s:.3f}", "dropped"])
            return
        publisher.publish(message)

    node.create_subscription(SensorCombined, args.topic, relay, qos)
    print(f"IMU_STREAM_FAULTS profile={args.profile} holes={len(holes)}", flush=True)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            with lock:
                log.flush()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
