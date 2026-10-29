#!/usr/bin/env python3
"""The autopilot's IMU stream failing beside the flight, an evaluation component
nobody tells the vehicle about (specification K28; the register's entry on a hole
of seconds, r720 and r723, where another task froze the host). It relays the
autopilot's IMU samples to the topic the camera estimator reads, and on a fixed
schedule drops them for a while, as a stalled host or a lost link does
(IMU_FAULTS=holes), or, as the real vehicle's imperfection (roadmap item 35, K52,
IMU_FAULTS=jitter), jitters their stamps and drops short runs of them as the NTU
VIRAL eee_01 recording shows a VN100 doing in flight: a period of 5.06 ms with a
standard deviation of 2.6 ms (p99 6.1 ms) and gaps over 15 ms 2.4 times a minute,
the largest 22 ms, all seeded. Its schedule never leaves this process; no
production node reads it (the contract test holds it there); the vehicle meets a
hole only as samples that do not come. The samples' bytes are untouched but for
the jitter profile's stamp; the schedule runs on their own stamps, the simulation
clock.
"""
from __future__ import annotations

import argparse
import csv
import random
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


JITTER_STDDEV_S = 0.00257  # NTU eee_01: the IMU period's standard deviation
JITTER_CLIP_S = 0.0061  # its 99th percentile deviation
GAPS_PER_MINUTE = 2.4  # gaps over 15 ms a minute
GAP_SHORTEST_S = 0.015
GAP_LONGEST_S = 0.0223  # the largest gap recorded
LEAST_SPACING_S = 0.0005  # the stamps stay monotonic


def schedule(profile: str, seed: int = 0, horizon_s: float = 7200.0) -> list[ImuHole]:
    """The holes of a flight. 'holes': two seconds a minute in, four seconds two
    minutes in, both longer than the filter's unaided second, the first within what
    the autopilot's own estimate carries, the second the length r723 left. 'jitter':
    the recording's short gaps, 2.4 a minute of 15 to 22 ms at seeded moments."""
    if profile == "holes":
        return [ImuHole(60.0, 2.0), ImuHole(120.0, 4.0)]
    if profile == "jitter":
        rng = random.Random(f"imu_jitter_gaps:{seed}")
        holes = []
        t_s = 0.0
        while True:
            t_s += rng.expovariate(GAPS_PER_MINUTE / 60.0)
            if t_s >= horizon_s:
                return holes
            holes.append(ImuHole(round(t_s, 4), round(rng.uniform(GAP_SHORTEST_S, GAP_LONGEST_S), 4)))
    raise ValueError(f"unknown IMU fault profile '{profile}'")


def jittered_stamp_us(stamp_us: int, previous_us: int, rng: random.Random) -> int:
    """The sample's stamp as a real driver's clock would have written it: the
    recording's jitter, clipped at its 99th percentile, the stamps kept monotonic."""
    jitter_s = min(max(rng.gauss(0.0, JITTER_STDDEV_S), -JITTER_CLIP_S), JITTER_CLIP_S)
    jittered = stamp_us + int(round(jitter_s * 1.0e6))
    return max(jittered, previous_us + int(LEAST_SPACING_S * 1.0e6))


def hole_at(holes: list[ImuHole], t_s: float) -> ImuHole | None:
    for hole in holes:
        if hole.start_s <= t_s < hole.end_s:
            return hole
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--topic", default="/fmu/out/sensor_combined")
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    import rclpy
    from px4_msgs.msg import SensorCombined
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    holes = schedule(args.profile, args.seed)
    jitter = random.Random(f"imu_jitter_stamps:{args.seed}") if args.profile == "jitter" else None
    previous_us = [0]
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
                if dropped[0] % 25 == 1 or jitter is not None:
                    writer.writerow([f"{stamp_s:.3f}", "dropped"])
            return
        if jitter is not None:
            message.timestamp = jittered_stamp_us(int(message.timestamp), previous_us[0], jitter)
            previous_us[0] = int(message.timestamp)
        publisher.publish(message)

    node.create_subscription(SensorCombined, args.topic, relay, qos)
    print(f"IMU_STREAM_FAULTS profile={args.profile} seed={args.seed} holes={len(holes)}", flush=True)
    from rclpy.executors import ExternalShutdownException
    # The flight's end stops the process from outside: a shutdown, not a fault.
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            with lock:
                log.flush()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    log.close()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
