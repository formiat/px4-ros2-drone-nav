#!/usr/bin/env python3
"""Roadmap item 21: warm bodies that walk through the location, beside the flight.

A body is a model of a person's size, an upright cylinder 0.5 m across and
1.7 m tall, textured, its surface tagged 307 K for the thermal core (the
world's temperatures are tags, not physics; the core reads 305.8 K of it, 17.8
over an untagged frame, specification K64). It walks back and forth between
two points of a floor at a constant speed, where on the walk it starts drawn
from the seed.

    MOVING_BODIES="x1,y1,x2,y2,z,speed;..."  (the world's frame, metres;
    z a height at or above the floor at the first point; speed m/s)

Each body is created in the running world a little above its floor, falls
onto it and stands there under gravity; from then on its pose is set ten
times a second on the simulation clock along its walk at the height it came
to rest at, so the walk has to lie on one level floor. The pair, the ring
and the lidar see it as any body, and the vehicle's contact sensor reports a
touch as any contact (a crash, the owner's rule of 2026-09-29).

This is an evaluation component nobody tells the vehicle about: the walks and
their phases never leave this process, and no production node reads them
(the contract test holds them here). The bodies' true places go to the run's
directory, ten times a second, for the measurement after the flight
(tools/body_encounters.py).
"""

from __future__ import annotations

import argparse
import csv
import math
import multiprocessing
import random
import struct
import sys
import time
import zlib
from dataclasses import dataclass
from pathlib import Path

DIAMETER_M = 0.5
HEIGHT_M = 1.7
MASS_KG = 70.0
KELVIN = 307.0
DROP_M = 0.3
# Ten times a second, the sensors' own rate (the pair 7.5 Hz, the lidar 10): a body set oftener moves no
# differently between their frames. (A cost of the pose service to the simulator's speed was read on 2026-11-06
# from four flights and withdrawn the same day: the host carried other work, and three flights in a row on a
# quieter host read 0.67 without a body, 0.81 with a walking one and 0.90 with a standing one, r2995 to r2997.)
SET_RATE_HZ = 10.0
LOG_PERIOD_S = 0.1


@dataclass(frozen=True)
class Walk:
    """A walk back and forth between two points of a floor."""

    x1: float
    y1: float
    x2: float
    y2: float
    z: float
    speed_mps: float
    phase: float  # where on the walk it starts: 0 at the first point, 0.5 at the second, 1 back at the first

    @property
    def length_m(self) -> float:
        return math.hypot(self.x2 - self.x1, self.y2 - self.y1)

    def state(self, t_s: float) -> tuple[float, float, float, float]:
        """The body's place and velocity (x, y, vx, vy) t seconds into its walk."""
        length = self.length_m
        if length <= 0.0 or self.speed_mps <= 0.0:
            return self.x1, self.y1, 0.0, 0.0
        along = (self.phase * 2.0 * length + self.speed_mps * t_s) % (2.0 * length)
        direction = 1.0 if along <= length else -1.0
        share = (along if along <= length else 2.0 * length - along) / length
        ux, uy = (self.x2 - self.x1) / length, (self.y2 - self.y1) / length
        return (self.x1 + share * (self.x2 - self.x1), self.y1 + share * (self.y2 - self.y1),
                direction * self.speed_mps * ux, direction * self.speed_mps * uy)


def parse_walks(text: str, seed: int) -> list[Walk]:
    """The walks of MOVING_BODIES, each one's phase drawn from the seed."""
    generator = random.Random(seed)
    walks = []
    for item in filter(None, (part.strip() for part in text.split(";"))):
        values = [float(value) for value in item.split(",")]
        if len(values) != 6:
            raise ValueError(f"a body is x1,y1,x2,y2,z,speed: {item!r}")
        if not 0.0 <= values[5] <= 3.0:
            raise ValueError(f"a body walks at 0 to 3 m/s: {item!r}")
        walks.append(Walk(*values, phase=generator.random()))
    return walks


def texture_png(path: Path, seed: int) -> None:
    """A random grey texture, clothes to the pair's matcher: a plain body is a surface the matcher cannot match."""
    generator = random.Random(seed)
    size, cell = 256, 8
    coarse = [[generator.random() for _ in range(size // cell)] for _ in range(size // cell)]
    rows = []
    for v in range(size):
        row = bytearray([0])
        for u in range(size):
            grey = int(40 + 180 * (0.6 * coarse[v // cell][u // cell] + 0.4 * generator.random()))
            row += bytes((grey, grey, grey))
        rows.append(bytes(row))

    def chunk(kind: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data) & 0xFFFFFFFF)

    path.write_bytes(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", size, size, 8, 2, 0, 0, 0))
                     + chunk(b"IDAT", zlib.compress(b"".join(rows), 6)) + chunk(b"IEND", b""))


def body_name(index: int) -> str:
    return f"warm_body_{index}"


def body_sdf(index: int, x: float, y: float, z_base: float, texture: Path) -> str:
    """The body's model: dynamic, so that its pose can be set and its touch is a contact."""
    radius = DIAMETER_M / 2.0
    lateral = MASS_KG * (3.0 * radius * radius + HEIGHT_M * HEIGHT_M) / 12.0
    axial = MASS_KG * radius * radius / 2.0
    cylinder = f"<cylinder><radius>{radius}</radius><length>{HEIGHT_M}</length></cylinder>"
    return f"""<?xml version="1.0"?>
<sdf version="1.10"><model name="{body_name(index)}"><static>false</static>
<pose>{x} {y} {z_base + HEIGHT_M / 2.0} 0 0 0</pose>
<link name="link">
  <inertial><mass>{MASS_KG}</mass><inertia><ixx>{lateral}</ixx><iyy>{lateral}</iyy><izz>{axial}</izz>
    <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz></inertia></inertial>
  <collision name="body"><geometry>{cylinder}</geometry></collision>
  <visual name="body"><geometry>{cylinder}</geometry>
    <material><diffuse>1 1 1 1</diffuse><pbr><metal><albedo_map>{texture}</albedo_map></metal></pbr></material>
    <plugin filename="gz-sim-thermal-system" name="gz::sim::systems::Thermal"><temperature>{KELVIN}</temperature></plugin>
  </visual></link></model></sdf>"""


def listen(world: str, sim_s, heights) -> None:
    """The simulation's clock and the bodies' heights, read for the walking process."""
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    node = Node()

    def on_clock(message: Clock) -> None:
        sim_s.value = message.sim.sec + 1.0e-9 * message.sim.nsec

    def on_poses(message: Pose_V) -> None:
        for pose in message.pose:
            if pose.name.startswith("warm_body_"):
                index = int(pose.name.rsplit("_", 1)[1])
                if index < len(heights):
                    heights[index] = pose.position.z

    node.subscribe(Clock, f"/world/{world}/clock", on_clock)
    node.subscribe(Pose_V, f"/world/{world}/pose/info", on_poses)
    while True:
        time.sleep(1.0)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--world", required=True)
    parser.add_argument("--bodies", required=True)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    walks = parse_walks(args.bodies, args.seed)
    with args.output.with_suffix(".schedule.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["index", "x1", "y1", "x2", "y2", "z", "speed_mps", "phase"])
        for index, walk in enumerate(walks):
            writer.writerow([index, walk.x1, walk.y1, walk.x2, walk.y2, walk.z, walk.speed_mps, f"{walk.phase:.6f}"])
    if not walks:
        return 0

    # The simulation's clock and the bodies' heights are read in a process of
    # its own: a subscription's callback and a service request in one process
    # wait on each other in the Python bindings until the request times out
    # (a pose set every 0.3 s instead of every 0.02, measured on the bench).
    sim_s = multiprocessing.Value("d", -1.0)
    heights = multiprocessing.Array("d", [float("nan")] * len(walks))
    listener = multiprocessing.Process(target=listen, args=(args.world, sim_s, heights), daemon=True)
    listener.start()

    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node

    node = Node()

    def now() -> float | None:
        value = sim_s.value
        return value if value >= 0.0 else None

    texture = args.output.with_suffix(".texture.png").resolve()
    texture_png(texture, args.seed)
    # The create service answers once the world runs: tried for a minute.
    for index, walk in enumerate(walks):
        x, y, _, _ = walk.state(0.0)
        factory = EntityFactory()
        factory.sdf = body_sdf(index, x, y, walk.z + DROP_M, texture)
        factory.name = body_name(index)
        factory.allow_renaming = False
        for _ in range(12):
            ok, reply = node.request(f"/world/{args.world}/create", factory, EntityFactory, Boolean, 5000)
            if ok and reply.data:
                break
            time.sleep(1.0)
        else:
            raise RuntimeError(f"the body {index} could not be created in the world")
    # Each body falls onto its floor and stands: two seconds of the simulation's clock.
    while now() is None:
        time.sleep(0.05)
    start_s = now() + 2.0
    while now() < start_s:
        time.sleep(0.05)
    rest = {index: heights[index] if math.isfinite(heights[index]) else walk.z + HEIGHT_M / 2.0
            for index, walk in enumerate(walks)}
    for index, walk in enumerate(walks):
        print(f"MOVING_BODY index={index} walk=({walk.x1:.2f},{walk.y1:.2f})-({walk.x2:.2f},{walk.y2:.2f}) "
              f"speed_mps={walk.speed_mps:.2f} phase={walk.phase:.3f} floor_m={rest[index] - HEIGHT_M / 2.0:.3f}",
              flush=True)

    pose = Pose()
    pose.orientation.w = 1.0
    with args.output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sim_s", "index", "x", "y", "z_base", "vx", "vy"])
        logged_s = -1.0
        try:
            while True:
                t_s = now()
                log = t_s - logged_s >= LOG_PERIOD_S
                for index, walk in enumerate(walks):
                    x, y, vx, vy = walk.state(t_s - start_s)
                    pose.name = body_name(index)
                    pose.position.x, pose.position.y, pose.position.z = x, y, rest[index]
                    node.request(f"/world/{args.world}/set_pose", pose, Pose, Boolean, 200)
                    if log:
                        writer.writerow([f"{t_s:.3f}", index, f"{x:.3f}", f"{y:.3f}",
                                         f"{rest[index] - HEIGHT_M / 2.0:.3f}", f"{vx:.3f}", f"{vy:.3f}"])
                if log:
                    logged_s = t_s
                    stream.flush()
                time.sleep(1.0 / SET_RATE_HZ)
        except KeyboardInterrupt:
            pass
    return 0


if __name__ == "__main__":
    sys.exit(main())
