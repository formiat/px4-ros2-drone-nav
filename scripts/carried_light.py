#!/usr/bin/env python3
"""Roadmap item 17 stage 5: the carried light, its battery and its failures.

The simulation of the light the camera vehicle carries, beside the flight.
It sets the vehicle model's spot light through Gazebo's `light_config`
service on the simulation clock and writes every change to the run's
directory. Two things move the light:

- its battery, when the scenario gives it one (LIGHT_BATTERY_S, seconds of
  light at launch; without it the light never runs out and no charge is
  published): its charge, which the vehicle knows as any airframe knows its
  batteries, drains at one constant rate whatever the light does, is the one
  thing of the light published on ROS (/carried_light/charge_s), and at
  zero the light goes out (specification F7, F8);
- its failures, injected (LIGHT_FAULTS, seeded by LIGHT_FAULT_SEED): an
  evaluation component nobody tells the vehicle about. Their schedule never
  leaves this process, and no production node reads it (the contract test
  holds it there); the vehicle learns of a failure only from its frames. The
  share of the nominal intensity the light is set to goes out for the viewer
  alone, on /evaluation/light_share, which the RViz overlay draws and no
  production node subscribes to.

Two regimes, decided by the project owner on 2026-09-27:

- moderate, the norm of every flight: short and frequent dimming, its dark
  stretches never long enough to reach the "unreliable" judgment;
- severe, a scenario of its own: outages growing longer and more frequent
  until the vehicle judges its light unreliable, and staying that bad while
  it flies home.

Every outage ramps down and back up over seconds, never a switch; the
shortest ramp the parameters allow is one the flights fly. Outages arrive
every 1 s to 1 min and last 1 s to 1 min (the owner's range, 2026-09-20).
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

# The spot light of models/stereo_tof_v1 as the model states it; the service
# replaces the whole light, so every property is sent with the intensity.
LIGHT_NAME = "carried_light"
NOMINAL_INTENSITY = 2.0
LIGHT_RANGE_M = 25.0
ATTENUATION_CONSTANT = 0.25
ATTENUATION_LINEAR = 0.0
ATTENUATION_QUADRATIC = 0.06
SPOT_INNER_ANGLE_RAD = 2.1
SPOT_OUTER_ANGLE_RAD = 2.4
SPOT_FALLOFF = 1.0
SPECULAR = 0.1

SHORTEST_RAMP_S = 0.2
UPDATE_PERIOD_S = 0.1


@dataclass(frozen=True)
class Outage:
    start_s: float
    ramp_down_s: float
    dark_s: float
    ramp_up_s: float
    floor: float  # the share of the nominal intensity left at the bottom

    @property
    def end_s(self) -> float:
        return self.start_s + self.ramp_down_s + self.dark_s + self.ramp_up_s

    def share(self, t_s: float) -> float:
        """The share of the nominal intensity at simulation time `t_s`."""
        into = t_s - self.start_s
        if into <= 0.0 or t_s >= self.end_s:
            return 1.0
        if into < self.ramp_down_s:
            return 1.0 - (1.0 - self.floor) * into / self.ramp_down_s
        into -= self.ramp_down_s
        if into < self.dark_s:
            return self.floor
        into -= self.dark_s
        return self.floor + (1.0 - self.floor) * into / self.ramp_up_s


def schedule(profile: str, seed: int, horizon_s: float) -> list[Outage]:
    """The outages of a flight up to `horizon_s` of simulation time."""
    rng = random.Random(f"{profile}:{seed}")
    outages: list[Outage] = []
    t_s = 0.0
    count = 0
    while True:
        if profile == "moderate":
            t_s += rng.uniform(5.0, 30.0)
            dark_s = rng.uniform(1.0, 2.0)
            floor = rng.uniform(0.0, 0.5)
        elif profile == "severe":
            # Worsening from the twentieth second to a plateau: the gaps
            # shrink to 8 s and the outages grow to 4.5 s of dark, each to
            # the dark, past the 4 s of outage at which the vehicle judges its
            # light, about a minute in, and inside what its hold survives in
            # a doorway. The estimator's dead reckoning drifts 0.3 to 1.1 m
            # an outage and the drift adds up, 1 to 1.4 m over the sixteen
            # outages of a way home begun 200 m out (r795 to r797), where a
            # door leaves half a metre; a failure that grows without bound
            # lands the vehicle wherever it is (r789, r790).
            t_s += 20.0 if count == 0 else max(8.0, 15.0 * 0.8**count)
            dark_s = min(4.5, 2.0 * 1.4**count)
            floor = 0.0
        elif profile == "lost":
            # The harshest the owner's range allows, a scenario of its own:
            # from the forty-fifth second, in flight, outages of a minute to
            # the dark a second apart, on the shortest ramps: the light is
            # all but gone, and the vehicle has only to stay whole.
            t_s += 45.0 if count == 0 else 1.0
            dark_s = 60.0
            floor = 0.0
        else:
            raise ValueError(f"unknown light fault profile '{profile}'")
        shortest = profile == "lost"
        outage = Outage(
            start_s=t_s,
            ramp_down_s=SHORTEST_RAMP_S if shortest else rng.uniform(SHORTEST_RAMP_S, 2.0),
            dark_s=dark_s,
            ramp_up_s=SHORTEST_RAMP_S if shortest else rng.uniform(SHORTEST_RAMP_S, 2.0),
            floor=floor,
        )
        if outage.start_s >= horizon_s:
            return outages
        outages.append(outage)
        t_s = outage.end_s
        count += 1


def share_at(outages: list[Outage], t_s: float) -> float:
    return min((outage.share(t_s) for outage in outages), default=1.0)


AIRBORNE_RISE_M = 1.0


def scheduled_share(outages, sim_s: float, airborne_s: float | None) -> float:
    """The light the schedule leaves: its clock starts when the vehicle is
    airborne, a metre over where it stood. An outage on the pad met the
    estimator in its first seconds, the autopilot reset its position by 3.4 m
    and the flight never left (r924)."""
    return 1.0 if airborne_s is None else share_at(outages, sim_s - airborne_s)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--world", required=True)
    parser.add_argument("--profile", default="none",
                        choices=("none", "moderate", "severe", "lost"))
    parser.add_argument("--battery-s", type=float)
    parser.add_argument("--model", default="x500_lidar_3d_0")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--horizon-s", type=float, default=3600.0)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()

    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.clock_pb2 import Clock
    from gz.msgs10.light_pb2 import Light
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    import rclpy
    from std_msgs.msg import Float64

    outages = ([] if args.profile == "none"
               else schedule(args.profile, args.seed, args.horizon_s))
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with (args.output.with_suffix(".schedule.csv")).open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["start_s", "ramp_down_s", "dark_s", "ramp_up_s", "floor"])
        for outage in outages:
            writer.writerow([f"{outage.start_s:.3f}", f"{outage.ramp_down_s:.3f}",
                             f"{outage.dark_s:.3f}", f"{outage.ramp_up_s:.3f}",
                             f"{outage.floor:.3f}"])

    rclpy.init()
    ros_node = rclpy.create_node("carried_light")
    charge_pub = ros_node.create_publisher(Float64, "/carried_light/charge_s", 1)
    share_pub = ros_node.create_publisher(Float64, "/evaluation/light_share", 1)
    node = Node()
    clock = {"s": None}
    lock = threading.Lock()

    def on_clock(message: Clock) -> None:
        with lock:
            clock["s"] = message.sim.sec + 1.0e-9 * message.sim.nsec

    node.subscribe(Clock, f"/world/{args.world}/clock", on_clock)
    position = {"xyz": None}

    def on_poses(message: Pose_V) -> None:
        for pose in message.pose:
            if pose.name == args.model:
                with lock:
                    position["xyz"] = (pose.position.x, pose.position.y, pose.position.z)
                break

    node.subscribe(Pose_V, f"/world/{args.world}/pose/info", on_poses)
    service = f"/world/{args.world}/light_config"

    def request(intensity: float) -> bool:
        light = Light()
        light.name = LIGHT_NAME
        light.type = Light.SPOT
        light.intensity = intensity
        light.diffuse.r = light.diffuse.g = light.diffuse.b = light.diffuse.a = 1.0
        light.specular.r = light.specular.g = light.specular.b = SPECULAR
        light.specular.a = 1.0
        light.range = LIGHT_RANGE_M
        light.attenuation_constant = ATTENUATION_CONSTANT
        light.attenuation_linear = ATTENUATION_LINEAR
        light.attenuation_quadratic = ATTENUATION_QUADRATIC
        light.spot_inner_angle = SPOT_INNER_ANGLE_RAD
        light.spot_outer_angle = SPOT_OUTER_ANGLE_RAD
        light.spot_falloff = SPOT_FALLOFF
        light.direction.x = 1.0
        ok, reply = node.request(service, light, Light, Boolean, 1000)
        return bool(ok and reply.data)

    with args.output.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["sim_s", "share", "accepted"])
        applied = None
        published_s = -1.0
        ground_z = None
        airborne_s = None
        while True:
            with lock:
                t_s = clock["s"]
            if t_s is not None:
                charge_s = (float("inf") if args.battery_s is None
                            else max(0.0, args.battery_s - t_s))
                if args.battery_s is not None and t_s - published_s >= 1.0:
                    charge_pub.publish(Float64(data=charge_s))
                    published_s = t_s
                with lock:
                    xyz = position["xyz"]
                if xyz is not None and airborne_s is None:
                    ground_z = xyz[2] if ground_z is None else ground_z
                    if xyz[2] - ground_z >= AIRBORNE_RISE_M:
                        airborne_s = t_s
                        print(f"airborne at sim {t_s:.3f} s", flush=True)
                share = scheduled_share(outages, t_s, airborne_s) if charge_s > 0.0 else 0.0
                share = round(share, 3)
                if share != applied:
                    accepted = request(NOMINAL_INTENSITY * share)
                    writer.writerow([f"{t_s:.3f}", f"{share:.3f}", int(accepted)])
                    stream.flush()
                    if accepted:
                        applied = share
                        share_pub.publish(Float64(data=share))
            time.sleep(UPDATE_PERIOD_S)


if __name__ == "__main__":
    sys.exit(main())
