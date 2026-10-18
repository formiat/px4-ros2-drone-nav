#!/usr/bin/env python3
"""The world half of a flight rendered afterwards (part 6 of the technical-debt goal).

Run inside the simulation container beside a Gazebo server that holds the
flight's world, paused, and the recording's Gazebo window. The vehicle model
is put into the world at the flight's first true pose and then moved through
the run's true pose record (gz_pose.csv) on the wall clock at the given rate,
its carried light set as the run's light record (carried_light.csv) says. Each
pose set is written to the clock record (wall clock, simulation clock, x, y, z),
which times the window's frames as the flight's own pose record times a live
recording's. Evaluation only: nothing of it reaches a flight.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from carried_light import NOMINAL_INTENSITY, light_message  # noqa: E402

TICK_S = 1.0 / 48.0


def true_poses(run_directory: Path) -> list[tuple[float, ...]]:
    """(simulation clock, x, y, z, qw, qx, qy, qz) of every true pose, in order."""
    rows = []
    with (run_directory / "gz_pose.csv").open(newline="") as stream:
        for row in csv.reader(stream):
            if len(row) >= 8:
                rows.append(tuple(float(value) for value in row[:8]))
    rows.sort()
    return rows


def light_shares(run_directory: Path) -> list[tuple[float, float]]:
    """(simulation clock, share of the nominal intensity) the light was set to."""
    shares = []
    try:
        with (run_directory / "carried_light.csv").open(newline="") as stream:
            for row in csv.DictReader(stream):
                if row.get("accepted") == "1":
                    shares.append((float(row["sim_s"]), float(row["share"])))
    except OSError:
        pass
    return shares


def pose_at(poses: list[tuple[float, ...]], times: list[float], t_s: float) -> tuple[float, ...]:
    """The pose at `t_s`: the position between the two records around it, the
    orientation of the nearer one."""
    index = min(max(bisect.bisect_left(times, t_s), 1), len(poses) - 1)
    before, after = poses[index - 1], poses[index]
    span = after[0] - before[0]
    share = min(max((t_s - before[0]) / span, 0.0), 1.0) if span > 0.0 else 0.0
    position = [before[i] + share * (after[i] - before[i]) for i in (1, 2, 3)]
    orientation = (before if share < 0.5 else after)[4:8]
    return (*position, *orientation)


def share_at(shares: list[tuple[float, float]], t_s: float) -> float:
    index = bisect.bisect_right([s for s, _ in shares], t_s) - 1
    return shares[index][1] if index >= 0 else 1.0


# Smoke in the world half of the recording (K26): the model's zones as boxes
# a viewer sees through, the denser the greyer (the overlay's rule), lit by
# a glow of their own in proportion (the location has no light, and a lit
# box is black), drawn on
# a visibility layer no sensor of the vehicle renders (the lidar's own
# visuals' layer, bit 25), so that the picture shows the smoke and the
# vehicle still sees it only through the smoke model.
SMOKE_VISIBILITY_FLAGS = 33554432


def run_smoke_zones(run_directory: Path) -> list[tuple[tuple[float, ...], tuple[float, ...], float]]:
    """The run's SMOKE_ZONES as the manifest recorded them, or none."""
    manifest = json.loads((run_directory / "manifest.json").read_text())
    text = manifest.get("effective_overrides", {}).get("SMOKE_ZONES", "")
    if not text.strip():
        return []
    from evaluation_overlay import smoke_zones
    return smoke_zones(text)


def smoke_box_sdf(index: int, zone: tuple[tuple[float, ...], tuple[float, ...], float]) -> str:
    """A static model of one visual box, no collision, the zone's place and
    size, grey at the overlay's opacity for its density."""
    from evaluation_overlay import SMOKE, smoke_alpha
    (x, y, z), (sx, sy, sz), sigma = zone
    alpha = smoke_alpha(sigma)
    colour = " ".join(f"{channel:.2f}" for channel in SMOKE)
    # Measured on a bench (tools/smoke_box_bench.py): under the
    # vehicle's own light an emissive of 0.2 with the transparency of 1/m
    # smoke is a grey veil the wall shows through, 0.62 all but a wall, a
    # material without emissive or lighting invisible in the dark.
    glow = 0.1 + 0.3 * alpha
    emissive = " ".join(f"{glow * channel / SMOKE[0]:.2f}" for channel in SMOKE)
    return (f'<?xml version="1.0"?><sdf version="1.9"><model name="smoke_zone_{index}">'
            f"<static>true</static><pose>{x} {y} {z} 0 0 0</pose><link name=\"link\">"
            f'<visual name="smoke"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>'
            f"<material><ambient>{colour} {alpha:.2f}</ambient><diffuse>{colour} {alpha:.2f}</diffuse>"
            f"<specular>0 0 0 1</specular><emissive>{emissive} 1</emissive></material>"
            f"<transparency>{1.0 - alpha:.2f}</transparency>"
            f"<visibility_flags>{SMOKE_VISIBILITY_FLAGS}</visibility_flags><cast_shadows>false</cast_shadows>"
            f"</visual></link></model></sdf>")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-directory", type=Path, required=True)
    parser.add_argument("--world", required=True)
    parser.add_argument("--model", default="x500_lidar_3d_0")
    parser.add_argument("--model-uri", default="model://x500_lidar_3d")
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--begin-s", type=float, required=True)
    parser.add_argument("--end-s", type=float, required=True)
    parser.add_argument("--go-file", type=Path, required=True)
    parser.add_argument("--clock-output", type=Path, required=True)
    args = parser.parse_args()

    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.entity_factory_pb2 import EntityFactory
    from gz.msgs10.light_pb2 import Light
    from gz.msgs10.pose_pb2 import Pose
    from gz.transport13 import Node

    poses = true_poses(args.run_directory)
    times = [pose[0] for pose in poses]
    shares = light_shares(args.run_directory)
    node = Node()

    def set_pose(pose: tuple[float, ...]) -> bool:
        message = Pose()
        message.name = args.model
        message.position.x, message.position.y, message.position.z = pose[0:3]
        (message.orientation.w, message.orientation.x, message.orientation.y,
         message.orientation.z) = pose[3:7]
        ok, reply = node.request(f"/world/{args.world}/set_pose", message, Pose, Boolean, 1000)
        return bool(ok and reply.data)

    def set_light(share: float) -> bool:
        ok, reply = node.request(f"/world/{args.world}/light_config",
                                 light_message(Light(), NOMINAL_INTENSITY * share), Light,
                                 Boolean, 1000)
        return bool(ok and reply.data)

    first = pose_at(poses, times, args.begin_s)
    factory = EntityFactory()
    factory.sdf_filename = args.model_uri
    factory.name = args.model
    factory.allow_renaming = False
    factory.pose.position.x, factory.pose.position.y, factory.pose.position.z = first[0:3]
    (factory.pose.orientation.w, factory.pose.orientation.x, factory.pose.orientation.y,
     factory.pose.orientation.z) = first[3:7]
    for _ in range(60):
        ok, reply = node.request(f"/world/{args.world}/create", factory, EntityFactory,
                                 Boolean, 5000)
        if ok and reply.data:
            break
        time.sleep(1.0)
    else:
        print("RENDER_WORLD none: the model could not be created", flush=True)
        return 1
    print(f"RENDER_WORLD model {args.model} at the flight's {args.begin_s:.2f} s", flush=True)
    zones = run_smoke_zones(args.run_directory)
    for index, zone in enumerate(zones):
        box = EntityFactory()
        box.sdf = smoke_box_sdf(index, zone)
        box.name = f"smoke_zone_{index}"
        box.allow_renaming = False
        ok, reply = node.request(f"/world/{args.world}/create", box, EntityFactory, Boolean, 5000)
        if not (ok and reply.data):
            print(f"RENDER_WORLD smoke zone {index} could not be created", flush=True)
    if zones:
        print(f"RENDER_WORLD {len(zones)} smoke zones drawn", flush=True)
    set_pose(first)
    applied = share_at(shares, args.begin_s)
    set_light(applied)

    while not args.go_file.exists():
        time.sleep(0.05)
    started = time.time()
    with args.clock_output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        t_s = args.begin_s
        while t_s <= args.end_s:
            t_s = args.begin_s + (time.time() - started) * args.rate
            pose = pose_at(poses, times, t_s)
            set_pose(pose)
            share = share_at(shares, t_s)
            if share != applied and set_light(share):
                applied = share
            writer.writerow([f"{time.time():.4f}", f"{t_s:.4f}", f"{pose[0]:.3f}",
                             f"{pose[1]:.3f}", f"{pose[2]:.3f}"])
            time.sleep(max(0.0, started + (t_s - args.begin_s) / args.rate + TICK_S - time.time()))
    print(f"RENDER_WORLD done at the flight's {args.end_s:.2f} s", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
