#!/usr/bin/env python3
"""Roadmap item 17 stage 5: the camera stream's failures, beside the flight.

An evaluation component nobody tells the vehicle about: it relays the stereo
pair's images from the simulator to the topics the vehicle's camera driver
reads, and on a seeded schedule drops them for a while or hands them over
late, both cameras alike, as a camera link that loses or stalls its frames
does (STREAM_FAULTS, seeded by STREAM_FAULT_SEED). Its schedule never leaves
this process; no production node reads it (the contract test holds it there);
the vehicle meets a failure only as frames that do not come, or come old.

The frames are relayed as they arrive, their bytes untouched; the schedule
runs on their own stamps, the simulation clock.
"""

from __future__ import annotations

import argparse
import csv
import random
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path

RELAY_SUFFIX = "/relayed"


@dataclass(frozen=True)
class StreamFault:
    start_s: float
    length_s: float
    delay_s: float  # zero: every frame of the stretch dropped

    @property
    def end_s(self) -> float:
        return self.start_s + self.length_s


def schedule(profile: str, seed: int, horizon_s: float) -> list[StreamFault]:
    """The faults of a flight up to `horizon_s` of simulation time."""
    rng = random.Random(f"{profile}:{seed}")
    faults: list[StreamFault] = []
    t_s = 0.0
    while True:
        if profile == "moderate":
            # Every 5 to 30 s the link either loses 0.2 to 1 s of frames or
            # stalls them 0.1 to 0.5 s for 1 to 3 s.
            t_s += rng.uniform(5.0, 30.0)
            if rng.random() < 0.5:
                fault = StreamFault(t_s, rng.uniform(0.2, 1.0), 0.0)
            else:
                fault = StreamFault(t_s, rng.uniform(1.0, 3.0), rng.uniform(0.1, 0.5))
        else:
            raise ValueError(f"unknown camera stream fault profile '{profile}'")
        if fault.start_s >= horizon_s:
            return faults
        faults.append(fault)
        t_s = fault.end_s


def fault_at(faults: list[StreamFault], t_s: float) -> StreamFault | None:
    for fault in faults:
        if fault.start_s <= t_s < fault.end_s:
            return fault
        if fault.start_s > t_s:
            return None
    return None


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--model", required=True)
    parser.add_argument("--profile", required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    from gz.msgs10.image_pb2 import Image
    from gz.transport13 import Node, SubscribeOptions

    faults = schedule(args.profile, args.seed, 7200.0)
    with args.output.with_suffix(".schedule.csv").open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["start_s", "length_s", "delay_s"])
        for fault in faults:
            writer.writerow([f"{fault.start_s:.3f}", f"{fault.length_s:.3f}",
                             f"{fault.delay_s:.3f}"])
    node = Node()
    lock = threading.Lock()
    log = args.output.open("w", newline="")
    writer = csv.writer(log)
    writer.writerow(["side", "stamp_s", "action"])
    prefix = (f"/world/{args.world}/model/{args.model}/link/stereo_tof_link/sensor")
    held: dict[str, deque] = {}
    publishers = {}
    for side in ("left", "right"):
        topic = f"{prefix}/stereo_{side}/image"
        publishers[side] = node.advertise(topic + RELAY_SUFFIX, Image)
        held[side] = deque()

        def relay(data: bytes, _info, side=side) -> None:
            stamp_s = stamp_of(data)
            with lock:
                queue = held[side]
                queue.append((stamp_s, data))
                while queue:
                    first_s, first = queue[0]
                    late = fault_at(faults, first_s)
                    if late is not None and late.delay_s == 0.0:
                        queue.popleft()
                        writer.writerow([side, f"{first_s:.3f}", "dropped"])
                        continue
                    delay_s = late.delay_s if late is not None else 0.0
                    if stamp_s - first_s + 1.0e-6 < delay_s:
                        break
                    queue.popleft()
                    publishers[side].publish_raw(first, "gz.msgs.Image")
                    if delay_s > 0.0:
                        writer.writerow([side, f"{first_s:.3f}", f"late_{delay_s:.3f}"])

        if not node.subscribe_raw(topic, relay, "gz.msgs.Image", SubscribeOptions()):
            print(f"cannot subscribe to {topic}", file=sys.stderr)
            return 1
    print(f"CAMERA_STREAM_FAULTS profile={args.profile} seed={args.seed} faults={len(faults)}",
          flush=True)
    try:
        while True:
            time.sleep(1.0)
            with lock:
                log.flush()
    except KeyboardInterrupt:
        return 0


def stamp_of(data: bytes) -> float:
    """The frame's own stamp, read off the head of its serialized message:
    field 1 of gz.msgs.Image is its header, whose field 1 is the stamp
    (seconds, field 1, and nanoseconds, field 2, both varints)."""

    def varint(position: int) -> tuple[int, int]:
        value = 0
        shift = 0
        while position < len(data):
            byte = data[position]
            position += 1
            value |= (byte & 0x7F) << shift
            if not byte & 0x80:
                return value, position
            shift += 7
        return value, position

    if len(data) < 2 or data[0] != 0x0A:
        return 0.0
    _, position = varint(1)
    if position >= len(data) or data[position] != 0x0A:
        return 0.0
    stamp_length, position = varint(position + 1)
    end = position + stamp_length
    seconds = 0
    nanoseconds = 0
    while position < end:
        key = data[position]
        value, position = varint(position + 1)
        if key == 0x08:
            seconds = value
        elif key == 0x10:
            nanoseconds = value
    return seconds + nanoseconds * 1.0e-9


if __name__ == "__main__":
    sys.exit(main())
