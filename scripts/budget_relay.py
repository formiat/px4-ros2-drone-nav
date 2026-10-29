#!/usr/bin/env python3
"""The onboard budget as a relay (roadmap item 36, specification K53), an evaluation process nobody tells the
vehicle about. Each output of the onboard loop (the stereo depth's returns, the estimator's odometry to the
autopilot, the memory's delta and snapshot and its latest scan, the planner's horizon) passes through this relay,
which holds it until it is as old, on the simulation clock, as the host of record measured that output to be at
its consumer: an estimate, a memory update or a horizon reaches its consumer as late as it would on board (K25),
whatever the host does to the nodes. At a real-time factor of 1.0 the outputs are already that old when they
arrive, so the relay changes nothing; under a simulator that waits for the loop, it is what keeps the budget
honest. The bytes pass untouched. BUDGET_RELAY=measure listens only and records every output's age at the relay;
BUDGET_RELAY=on relays (the consumers read the `/budgeted` topics, the estimators publish their odometry to the
relay: `launch/budget_relay.py`). The ages are the medians K53 keeps; they are never turned down by hand.
"""
from __future__ import annotations

import argparse
import csv
import struct
import sys
import threading
from collections import deque
from dataclasses import dataclass
from pathlib import Path

BUDGETED_SUFFIX = "/budgeted"
UNBUDGETED_ODOMETRY = "/drone_city_nav/vehicle_visual_odometry/unbudgeted"
ODOMETRY = "/fmu/in/vehicle_visual_odometry"


@dataclass(frozen=True)
class Relayed:
    name: str
    source: str  # the topic the producer publishes to
    target: str  # the topic the consumer reads
    message: str  # "package/msg/Type"
    qos: str  # "sensor", "best_effort_transient", "reliable_transient", "reliable2", "best_effort10"
    header_first: bool  # a std_msgs/Header leads the message; else PX4's timestamp_sample
    age_ms: dict[str, float]  # the median age at the consumer by sensor profile (K53), ms


# The ages measured on the host of record (K53): the relay holds each output until it is this old.
RELAYED = (
    Relayed("depth_points", "/stereo_depth/points", "/stereo_depth/points" + BUDGETED_SUFFIX,
            "sensor_msgs/msg/PointCloud2", "sensor", True, {"stereo_tof": 154.0}),
    Relayed("odometry", UNBUDGETED_ODOMETRY, ODOMETRY, "px4_msgs/msg/VehicleOdometry", "best_effort10", False,
            {"stereo_tof": 65.0, "lidar": 25.0}),
    Relayed("memory_delta", "/drone_city_nav/raw_obstacle_delta_3d",
            "/drone_city_nav/raw_obstacle_delta_3d" + BUDGETED_SUFFIX, "drone_city_nav/msg/RawObstacleDelta3D",
            "best_effort_transient", True, {"stereo_tof": 43.0, "lidar": 78.0}),
    Relayed("memory_snapshot", "/drone_city_nav/raw_obstacle_snapshot_3d",
            "/drone_city_nav/raw_obstacle_snapshot_3d" + BUDGETED_SUFFIX, "drone_city_nav/msg/RawObstacleSnapshot3D",
            "reliable_transient", True, {"stereo_tof": 43.0, "lidar": 78.0}),
    Relayed("latest_scan", "/drone_city_nav/latest_sensor_obstacle_scan",
            "/drone_city_nav/latest_sensor_obstacle_scan" + BUDGETED_SUFFIX,
            "drone_city_nav/msg/LatestSensorObstacleScan", "sensor", True, {"stereo_tof": 43.0, "lidar": 78.0}),
    Relayed("horizon", "/drone_city_nav/mppi/execution_horizon",
            "/drone_city_nav/mppi/execution_horizon" + BUDGETED_SUFFIX, "drone_city_nav/msg/MppiTrajectoryHorizon",
            "reliable2", True, {"stereo_tof": 23.0, "lidar": 26.0}),
)


def stamp_ns_of(data: bytes, header_first: bool) -> int:
    """The message's stamp read off its serialized bytes: after the 4-byte CDR encapsulation a leading
    std_msgs/Header starts with int32 seconds and uint32 nanoseconds; PX4's VehicleOdometry starts with uint64
    timestamp then uint64 timestamp_sample in microseconds (8-byte alignment counted from the payload's start)."""
    if header_first:
        sec, nsec = struct.unpack_from("<iI", data, 4)
        return sec * 1_000_000_000 + nsec
    _, sample_us = struct.unpack_from("<QQ", data, 4)
    return sample_us * 1_000


def percentiles(values: list[float]) -> tuple[float, float, float]:
    ordered = sorted(values)
    return (ordered[len(ordered) // 2], ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))], ordered[-1])


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--mode", choices=("on", "measure"), required=True)
    parser.add_argument("--profile", default="stereo_tof")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary-period-s", type=float, default=10.0)
    args = parser.parse_args()

    import rclpy
    from rclpy.node import Node
    from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                           qos_profile_sensor_data)
    from rosidl_runtime_py.utilities import get_message

    def qos_of(kind: str) -> QoSProfile:
        if kind == "sensor":
            return qos_profile_sensor_data
        reliability = ReliabilityPolicy.RELIABLE if kind.startswith("reliable") else ReliabilityPolicy.BEST_EFFORT
        durability = (DurabilityPolicy.TRANSIENT_LOCAL if kind.endswith("transient")
                      else DurabilityPolicy.VOLATILE)
        depth = {"reliable2": 2, "best_effort10": 10}.get(kind, 1)
        return QoSProfile(reliability=reliability, durability=durability,
                          history=HistoryPolicy.KEEP_LAST, depth=depth)

    rclpy.init()
    node = Node("budget_relay", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    relayed = [r for r in RELAYED if args.profile in r.age_ms]
    log = args.output.open("w", newline="")
    writer = csv.writer(log)
    writer.writerow(["sim_s", "output", "samples", "age_p50_ms", "age_p95_ms", "age_max_ms", "held_p50_ms"])
    lock = threading.Lock()
    held: dict[str, deque] = {r.name: deque() for r in relayed}
    ages: dict[str, list[float]] = {r.name: [] for r in relayed}
    holds: dict[str, list[float]] = {r.name: [] for r in relayed}
    publishers = {}
    last_summary = [0.0]

    def now_ns() -> int:
        return node.get_clock().now().nanoseconds

    def release() -> None:
        now = now_ns()
        with lock:
            for r in relayed:
                queue = held[r.name]
                while queue and queue[0][0] <= now:
                    due, data, received = queue.popleft()
                    publishers[r.name].publish(data)
                    holds[r.name].append((now - received) * 1.0e-6)
            if (now - last_summary[0]) * 1.0e-9 >= args.summary_period_s:
                last_summary[0] = now
                for r in relayed:
                    if ages[r.name]:
                        p50, p95, top = percentiles(ages[r.name])
                        hold = percentiles(holds[r.name])[0] if holds[r.name] else 0.0
                        writer.writerow([f"{now * 1.0e-9:.3f}", r.name, len(ages[r.name]), f"{p50:.1f}",
                                         f"{p95:.1f}", f"{top:.1f}", f"{hold:.1f}"])
                        ages[r.name].clear()
                        holds[r.name].clear()
                log.flush()

    def receiver(r: Relayed):
        def on_message(data: bytes) -> None:
            received = now_ns()
            age_ms = (received - stamp_ns_of(data, r.header_first)) * 1.0e-6
            with lock:
                ages[r.name].append(age_ms)
                if args.mode == "on":
                    due = max(received, stamp_ns_of(data, r.header_first) + int(r.age_ms[args.profile] * 1.0e6))
                    held[r.name].append((due, data, received))
            if args.mode == "on":
                release()
        return on_message

    for r in relayed:
        message_type = get_message(r.message)
        # In measure mode the consumers read the producers directly: listen there, publish nothing.
        source = r.source if args.mode == "on" else (r.target if r.name == "odometry" else r.source)
        node.create_subscription(message_type, source, receiver(r), qos_of(r.qos), raw=True)
        if args.mode == "on":
            publishers[r.name] = node.create_publisher(message_type, r.target, qos_of(r.qos))
    node.create_timer(0.002, release)
    print(f"BUDGET_RELAY mode={args.mode} profile={args.profile} outputs={[r.name for r in relayed]} "
          f"ages_ms={[r.age_ms[args.profile] for r in relayed]}", flush=True)
    from rclpy.executors import ExternalShutdownException
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        with lock:
            for r in relayed:
                if ages[r.name]:
                    p50, p95, top = percentiles(ages[r.name])
                    writer.writerow([f"{now_ns() * 1.0e-9:.3f}", r.name, len(ages[r.name]), f"{p50:.1f}",
                                     f"{p95:.1f}", f"{top:.1f}", "0.0"])
            log.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
