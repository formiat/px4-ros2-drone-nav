#!/usr/bin/env python3
"""Roadmap item 40: the slow disturbance of the magnetic field a vehicle reads, beside the flight (MAGNETIC_DISTURBANCE).

An evaluation component nobody tells the vehicle about: it relays the vehicle's magnetometer from the simulator to
the topic the camera process reads, and adds to each axis a first-order Gauss-Markov process stepped on the readings'
own stamps (the simulation clock), seeded by the run: the field a vehicle reads beside its own currents and a
building's steel. A scenario of its own, as smoke is, not the vehicle's imperfections profile (K52 holds the
sensor): what it measures is where the magnetometer's heading stops helping. Profile `ntu`: the NTU VIRAL eee_01
vehicle's (tools/imperfections_sources.py: its VN-100 against the flight's FAST-LIO attitude, the heading's
deviation with the heading taken out), 1.06 uT standing, a correlation time of 9.3 s. gz-sensors' magnetometer
applies its noise without the step's time, so a dynamic bias in the model never moves: the walk is this relay's.
The field is in the gauss the sensor reports.
"""

from __future__ import annotations

import argparse
import csv
import math
import random
import sys
import threading
import time
from pathlib import Path

RELAY_SUFFIX = "/relayed"
STANDING_GAUSS = 1.06e-2  # NTU eee_01: 1.06 uT
CORRELATION_S = 9.3  # NTU eee_01


class Walk:
    """Three first-order Gauss-Markov processes, one an axis, stepped on the stamps given."""

    def __init__(self, seed: int, standing: float = STANDING_GAUSS, correlation_s: float = CORRELATION_S) -> None:
        self.rng = random.Random(f"magnetometer:{seed}")
        self.standing = standing
        self.correlation_s = correlation_s
        self.value = [self.rng.gauss(0.0, standing) for _ in range(3)]
        self.stamp_s: float | None = None

    def at(self, stamp_s: float) -> list[float]:
        if self.stamp_s is not None and stamp_s > self.stamp_s:
            decay = math.exp(-(stamp_s - self.stamp_s) / self.correlation_s)
            spread = self.standing * math.sqrt(1.0 - decay * decay)
            self.value = [decay * v + self.rng.gauss(0.0, spread) for v in self.value]
        if self.stamp_s is None or stamp_s > self.stamp_s:
            self.stamp_s = stamp_s
        return self.value


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--profile", choices=("ntu",), required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    from gz.msgs10.magnetometer_pb2 import Magnetometer
    from gz.transport13 import Node, SubscribeOptions

    topic = f"/world/{args.world}/model/{args.model}/link/base_link/sensor/magnetometer_sensor/magnetometer"
    walk = Walk(args.seed)
    node = Node()
    publisher = node.advertise(topic + RELAY_SUFFIX, Magnetometer)
    lock = threading.Lock()
    log = args.output.open("w", newline="")
    writer = csv.writer(log)
    writer.writerow(["stamp_s", "x_gauss", "y_gauss", "z_gauss"])
    last_logged = [-1.0]

    def relay(data: bytes, _info) -> None:
        message = Magnetometer()
        message.ParseFromString(data)
        stamp_s = message.header.stamp.sec + 1.0e-9 * message.header.stamp.nsec
        with lock:
            x, y, z = walk.at(stamp_s)
            if stamp_s - last_logged[0] >= 0.5:
                writer.writerow([f"{stamp_s:.3f}", f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])
                last_logged[0] = stamp_s
        message.field_tesla.x += x
        message.field_tesla.y += y
        message.field_tesla.z += z
        publisher.publish(message)

    if not node.subscribe_raw(topic, relay, "gz.msgs.Magnetometer", SubscribeOptions()):
        print(f"cannot subscribe to {topic}", file=sys.stderr)
        return 1
    print(f"MAGNETOMETER_DISTURBANCE profile={args.profile} seed={args.seed} standing_gauss={STANDING_GAUSS:g} "
          f"correlation_s={CORRELATION_S:g}", flush=True)
    try:
        while True:
            time.sleep(1.0)
            with lock:
                log.flush()
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
