#!/usr/bin/env python3
"""The lidar's sweeps delivered with a real driver's cadence spread (roadmap item 35, specification K52), an
evaluation component nobody tells the vehicle about. It relays the simulated lidar's clouds to the topic the bridge
reads and delays each one by a seeded draw from the spread the NTU VIRAL eee_01 recording shows (an OS1-16: a period
of 100 ms with a standard deviation of 9 ms, p1 71 ms, p99 106 ms): a normal of mean 9 ms and deviation 9 ms,
clipped to 0 to 30 ms. The cloud's bytes and its stamp are untouched: the sweep was taken when it was taken, it
arrives late, as a real one does. The relay never leaves this process; no production node reads it.
"""
from __future__ import annotations

import argparse
import csv
import random
import sys
import threading
import time
from collections import deque
from pathlib import Path

RELAY_SUFFIX = "/relayed"
DELAY_MEAN_S = 0.009  # NTU eee_01: the sweep period's spread, 9 ms
DELAY_STDDEV_S = 0.009
DELAY_MAX_S = 0.030


def delay_s(rng: random.Random) -> float:
    """One sweep's delivery delay from the recording's spread."""
    return min(max(rng.gauss(DELAY_MEAN_S, DELAY_STDDEV_S), 0.0), DELAY_MAX_S)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    from gz.msgs10.pointcloud_packed_pb2 import PointCloudPacked
    from gz.transport13 import Node, SubscribeOptions

    rng = random.Random(f"lidar_sweep_jitter:{args.seed}")
    topic = f"/world/{args.world}/model/{args.model}/link/link/sensor/lidar_3d_v1/scan/points"
    node = Node()
    publisher = node.advertise(topic + RELAY_SUFFIX, PointCloudPacked)
    queue: deque[tuple[float, bytes, float]] = deque()
    lock = threading.Lock()
    wake = threading.Condition(lock)
    log = args.output.open("w", newline="")
    writer = csv.writer(log)
    writer.writerow(["arrival_wall_s", "delay_s"])
    count = [0]

    def relay(data: bytes, _info) -> None:
        held = delay_s(rng)
        with lock:
            queue.append((time.monotonic() + held, data, held))
            wake.notify()

    def deliver() -> None:
        while True:
            with lock:
                while not queue:
                    wake.wait()
                due, data, held = queue[0]
                now = time.monotonic()
                if now < due:
                    wake.wait(timeout=due - now)
                    continue
                queue.popleft()
            publisher.publish_raw(data, "gz.msgs.PointCloudPacked")
            count[0] += 1
            if count[0] % 50 == 1:
                with lock:
                    writer.writerow([f"{time.time():.3f}", f"{held:.4f}"])
                    log.flush()

    threading.Thread(target=deliver, daemon=True).start()
    if not node.subscribe_raw(topic, relay, "gz.msgs.PointCloudPacked", SubscribeOptions()):
        print(f"cannot subscribe to {topic}", file=sys.stderr)
        return 1
    print(f"LIDAR_SWEEP_JITTER seed={args.seed} mean_ms={DELAY_MEAN_S * 1e3:.0f} "
          f"stddev_ms={DELAY_STDDEV_S * 1e3:.0f} max_ms={DELAY_MAX_S * 1e3:.0f}", flush=True)
    try:
        while True:
            time.sleep(1.0)
    except KeyboardInterrupt:
        return 0


if __name__ == "__main__":
    sys.exit(main())
