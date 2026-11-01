#!/usr/bin/env python3
"""The satellite signal failing beside the flight (roadmap item 39, stages 1 and
2), an evaluation component nobody tells the vehicle about: it relays the
autopilot's fixes (`/fmu/out/vehicle_gps_position`, px4_msgs SensorGps) to
the topic the visual-inertial estimator reads on the hybrid profile
(`/drone_city_nav/simulation/gnss`), and on a schedule given as
GNSS_FAULTS plays the table's variants on them, every one counted from the
moment the vehicle is airborne and seeded by the run's number:

  variant@start:duration[:magnitude] ; ...

  lost           no fix for the duration (nothing relayed)
  flap           the fix lost and back every two seconds, the loss first
  jam            no fix flagged: fix_type none, a reported error of 50 m, the
                 jamming state detected, the position noised by that error
  degraded       an honest fix with a large, varying error: gaussian noise of
                 the magnitude (default 10 m), the reported error that figure
  walk           the fix drifts from the truth at the magnitude (default
                 1 m/s) in a seeded direction, the reported error small
  jump           the fix leaps by the magnitude (default 100 m) at once
  replay         the fix of magnitude seconds ago (default 10 s), consistent
                 with itself but late
  bad_velocity   the fix's velocity scaled by the magnitude (default 0: none)
  bad_time       the fix's clock frozen at the fault's start
  return_offset  lost for the duration, then back at a constant offset of the
                 magnitude (default 50 m) for the rest of the flight

The schedule never leaves this process; no production node reads it. The
fix's bytes are untouched but for what the variant says. A row of what was
relayed every second goes to --output (the evidence).
"""
from __future__ import annotations

import argparse
import csv
import math
import random
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

INPUT_TOPIC = "/fmu/out/vehicle_gps_position"
OUTPUT_TOPIC = "/drone_city_nav/simulation/gnss"
VARIANTS = ("lost", "flap", "jam", "degraded", "walk", "jump", "replay", "bad_velocity",
            "bad_time", "return_offset")
DEFAULT_MAGNITUDE = {"degraded": 10.0, "walk": 1.0, "jump": 100.0, "replay": 10.0,
                     "bad_velocity": 0.0, "return_offset": 50.0, "jam": 50.0}
EARTH_RADIUS_M = 6378137.0
FIX_TYPE_NONE = 1
FIX_TYPE_3D = 3
JAMMING_STATE_DETECTED = 3


@dataclass(frozen=True)
class Fault:
    variant: str
    start_s: float
    duration_s: float
    magnitude: float

    @property
    def end_s(self) -> float:
        return self.start_s + self.duration_s

    def active(self, t_s: float) -> bool:
        return self.start_s <= t_s < self.end_s


@dataclass
class Fix:
    """The fields of a fix the variants touch, in plain numbers."""
    timestamp_us: int
    latitude_deg: float
    longitude_deg: float
    altitude_m: float
    vel_n_mps: float
    vel_e_mps: float
    vel_d_mps: float
    eph_m: float
    epv_m: float
    fix_type: int
    jamming_state: int = 0
    dropped: bool = False


def parse_faults(text: str) -> list[Fault]:
    faults: list[Fault] = []
    for item in [part.strip() for part in text.split(";") if part.strip()]:
        try:
            variant, rest = item.split("@", 1)
            fields = rest.split(":")
            start_s = float(fields[0])
            duration_s = float(fields[1])
            magnitude = float(fields[2]) if len(fields) > 2 else DEFAULT_MAGNITUDE.get(variant, 0.0)
        except (ValueError, IndexError) as error:
            raise ValueError(f"GNSS_FAULTS item '{item}' is not variant@start:duration[:magnitude]") from error
        if variant not in VARIANTS:
            raise ValueError(f"GNSS_FAULTS variant '{variant}' is not one of {', '.join(VARIANTS)}")
        if start_s < 0.0 or duration_s <= 0.0:
            raise ValueError(f"GNSS_FAULTS item '{item}' needs a start >= 0 and a duration > 0")
        faults.append(Fault(variant, start_s, duration_s, magnitude))
    return faults


def offset_fix(fix: Fix, north_m: float, east_m: float) -> None:
    """Move a fix by metres on the flat earth about its own latitude."""
    to_degrees = 180.0 / math.pi / EARTH_RADIUS_M
    fix.latitude_deg += north_m * to_degrees
    fix.longitude_deg += east_m * to_degrees / math.cos(math.radians(fix.latitude_deg))


@dataclass
class FaultPlayer:
    faults: list[Fault]
    rng: random.Random
    directions: dict[int, float] = field(default_factory=dict)  # the seeded direction of a fault
    frozen_timestamp_us: dict[int, int] = field(default_factory=dict)
    history: list[tuple[float, Fix]] = field(default_factory=list)  # (t_s, fix) for the replay
    returned_offset: tuple[float, float] | None = None  # return_offset, after its outage

    def direction(self, index: int) -> float:
        if index not in self.directions:
            self.directions[index] = self.rng.uniform(0.0, 2.0 * math.pi)
        return self.directions[index]

    def play(self, t_s: float, fix: Fix) -> Fix:
        """The fix as relayed at t_s seconds since the vehicle was airborne (negative before)."""
        self.history.append((t_s, Fix(**fix.__dict__)))
        while self.history and self.history[0][0] < t_s - 60.0:
            self.history.pop(0)
        for index, fault in enumerate(self.faults):
            if fault.variant == "return_offset" and t_s >= fault.end_s and self.returned_offset is None:
                heading = self.direction(index)
                self.returned_offset = (fault.magnitude * math.cos(heading),
                                        fault.magnitude * math.sin(heading))
            if not fault.active(t_s):
                continue
            if fault.variant == "lost" or fault.variant == "return_offset":
                fix.dropped = True
            elif fault.variant == "flap":
                if int((t_s - fault.start_s) // 2.0) % 2 == 0:  # the loss first: the fault begins with it
                    fix.dropped = True
            elif fault.variant == "jam":
                fix.fix_type = FIX_TYPE_NONE
                fix.eph_m = max(fix.eph_m, fault.magnitude)
                fix.epv_m = max(fix.epv_m, fault.magnitude)
                fix.jamming_state = JAMMING_STATE_DETECTED
                offset_fix(fix, self.rng.gauss(0.0, fault.magnitude), self.rng.gauss(0.0, fault.magnitude))
            elif fault.variant == "degraded":
                offset_fix(fix, self.rng.gauss(0.0, fault.magnitude), self.rng.gauss(0.0, fault.magnitude))
                fix.altitude_m += self.rng.gauss(0.0, fault.magnitude)
                fix.eph_m = max(fix.eph_m, fault.magnitude)
                fix.epv_m = max(fix.epv_m, fault.magnitude)
            elif fault.variant == "walk":
                heading = self.direction(index)
                walked_m = fault.magnitude * (t_s - fault.start_s)
                offset_fix(fix, walked_m * math.cos(heading), walked_m * math.sin(heading))
                fix.vel_n_mps += fault.magnitude * math.cos(heading)
                fix.vel_e_mps += fault.magnitude * math.sin(heading)
            elif fault.variant == "jump":
                heading = self.direction(index)
                offset_fix(fix, fault.magnitude * math.cos(heading), fault.magnitude * math.sin(heading))
            elif fault.variant == "replay":
                old = next((past for past_t, past in self.history if past_t >= t_s - fault.magnitude), None)
                if old is not None:
                    stamp = fix.timestamp_us
                    fix.__dict__.update(old.__dict__)
                    fix.timestamp_us = stamp
                    fix.dropped = False
            elif fault.variant == "bad_velocity":
                fix.vel_n_mps *= fault.magnitude
                fix.vel_e_mps *= fault.magnitude
                fix.vel_d_mps *= fault.magnitude
            elif fault.variant == "bad_time":
                if index not in self.frozen_timestamp_us:
                    self.frozen_timestamp_us[index] = fix.timestamp_us
                fix.timestamp_us = self.frozen_timestamp_us[index]
        if self.returned_offset is not None and not fix.dropped:
            offset_fix(fix, *self.returned_offset)
        return fix

    def active_names(self, t_s: float) -> str:
        names = [fault.variant for fault in self.faults if fault.active(t_s)]
        if self.returned_offset is not None:
            names.append("returned_offset")
        return "+".join(names) or "none"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--faults", default="")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--input", default=INPUT_TOPIC)
    parser.add_argument("--output-topic", default=OUTPUT_TOPIC)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    faults = parse_faults(args.faults)
    player = FaultPlayer(faults, random.Random(args.seed))

    import rclpy
    from rclpy.executors import ExternalShutdownException
    from px4_msgs.msg import SensorGps, VehicleLandDetected
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    rclpy.init()
    node = rclpy.create_node("gnss_fault_relay")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=50)
    publisher = node.create_publisher(SensorGps, args.output_topic, qos)
    lock = threading.Lock()
    airborne_since_us: list[int | None] = [None]
    relayed = [0, 0]  # relayed, dropped
    rows: list[list[str]] = []
    last_row_s = [None]

    def on_landed(message: VehicleLandDetected) -> None:
        with lock:
            if not message.landed and airborne_since_us[0] is None:
                airborne_since_us[0] = int(message.timestamp)
                node.get_logger().info(f"GNSS_FAULT_RELAY airborne_at_us={message.timestamp} faults={args.faults!r}")

    def on_fix(message: SensorGps) -> None:
        with lock:
            t_s = (float(message.timestamp - airborne_since_us[0]) * 1.0e-6
                   if airborne_since_us[0] is not None else -1.0e9)
            fix = Fix(int(message.timestamp), float(message.latitude_deg), float(message.longitude_deg),
                      float(message.altitude_msl_m), float(message.vel_n_m_s), float(message.vel_e_m_s),
                      float(message.vel_d_m_s), float(message.eph), float(message.epv), int(message.fix_type),
                      int(message.jamming_state))
            played = player.play(t_s, fix) if t_s > -1.0e8 else fix
            if not played.dropped:
                out = SensorGps()
                out.timestamp = played.timestamp_us
                out.timestamp_sample = message.timestamp_sample
                out.device_id = message.device_id
                out.latitude_deg = played.latitude_deg
                out.longitude_deg = played.longitude_deg
                out.altitude_msl_m = played.altitude_m
                out.altitude_ellipsoid_m = message.altitude_ellipsoid_m + (played.altitude_m - message.altitude_msl_m)
                out.s_variance_m_s = message.s_variance_m_s
                out.c_variance_rad = message.c_variance_rad
                out.fix_type = played.fix_type
                out.eph = played.eph_m
                out.epv = played.epv_m
                out.hdop = message.hdop
                out.vdop = message.vdop
                out.noise_per_ms = message.noise_per_ms
                out.automatic_gain_control = message.automatic_gain_control
                out.jamming_state = played.jamming_state
                out.jamming_indicator = message.jamming_indicator
                out.spoofing_state = message.spoofing_state
                out.system_error = message.system_error
                out.vel_m_s = message.vel_m_s
                out.vel_n_m_s = played.vel_n_mps
                out.vel_e_m_s = played.vel_e_mps
                out.vel_d_m_s = played.vel_d_mps
                out.cog_rad = message.cog_rad
                out.vel_ned_valid = message.vel_ned_valid
                out.timestamp_time_relative = message.timestamp_time_relative
                out.time_utc_usec = message.time_utc_usec
                out.satellites_used = message.satellites_used
                out.heading = message.heading
                out.heading_offset = message.heading_offset
                out.heading_accuracy = message.heading_accuracy
                out.rtcm_injection_rate = message.rtcm_injection_rate
                out.selected_rtcm_instance = message.selected_rtcm_instance
                out.rtcm_crc_failed = message.rtcm_crc_failed
                out.rtcm_msg_used = message.rtcm_msg_used
                publisher.publish(out)
                relayed[0] += 1
            else:
                relayed[1] += 1
            second = math.floor(t_s) if t_s > -1.0e8 else None
            if second is not None and second != last_row_s[0]:
                last_row_s[0] = second
                rows.append([f"{t_s:.1f}", player.active_names(t_s), str(relayed[0]), str(relayed[1]),
                             f"{played.latitude_deg:.7f}", f"{played.longitude_deg:.7f}",
                             f"{played.altitude_m:.2f}", str(played.fix_type), f"{played.eph_m:.1f}"])

    node.create_subscription(VehicleLandDetected, "/fmu/out/vehicle_land_detected", on_landed, qos)
    node.create_subscription(SensorGps, args.input, on_fix, qos)
    node.get_logger().info(f"GNSS_FAULT_RELAY ready input={args.input} output={args.output_topic} "
                           f"faults={[f.__dict__ for f in faults]} seed={args.seed}")

    def save() -> None:
        with lock:
            snapshot = list(rows)
        temporary = args.output.with_suffix(args.output.suffix + ".part")
        with temporary.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(["t_airborne_s", "active", "relayed", "dropped", "latitude_deg", "longitude_deg",
                             "altitude_m", "fix_type", "eph_m"])
            writer.writerows(snapshot)
        temporary.replace(args.output)

    last_save = time.monotonic()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.5)
            if time.monotonic() - last_save > 2.0:
                save()
                last_save = time.monotonic()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
