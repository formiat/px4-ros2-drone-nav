#!/usr/bin/env python3
"""airframe_envelope.py --out DIR [SPEED...] | --out DIR --report: the pilot's script of tools/airframe_envelope.sh
(roadmap item 39 stage 5) and its report.

The script arms the autopilot's SITL vehicle in offboard, climbs to 30 m and, for every SPEED, three times: commands
the horizontal velocity SPEED (out along north, back along south, so the runs stay over the start), waits until the
speed is held (within 0.3 m/s of it for 3 s, or 30 s at the most: a speed the airframe does not reach is recorded as
what it reached), then commands a velocity of nought and waits for rest (under 0.3 m/s). The setpoints are the
stack's kind: a velocity in the plane with the height held by position. Every sample of the autopilot's local
position and attitude goes to samples.csv with the phase it belongs to.

--report reads samples.csv and motors.txt (the simulator's motor commands, `gz topic -e`) and writes summary.txt:
a row a run and the medians a speed. Runs in the dev container (rclpy, px4_msgs)."""
from __future__ import annotations

import argparse
import csv
import math
import statistics
import sys
import time
from pathlib import Path

HEIGHT_M = 30.0
HOLD_TOLERANCE_MPS = 0.3
HOLD_S = 3.0
REACH_TIMEOUT_S = 30.0
REST_MPS = 0.3
REST_TIMEOUT_S = 30.0
MOTOR_TOP_RADPS = 1000.0   # the x500 model's maxRotVelocity


def fly(out: Path, speeds: list[float]) -> int:
    import rclpy
    from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint, VehicleAttitude, VehicleCommand,
                              VehicleLocalPosition)
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, durability=DurabilityPolicy.VOLATILE,
                     history=HistoryPolicy.KEEP_LAST, depth=5)
    nan = float("nan")

    class Pilot(Node):
        def __init__(self) -> None:
            super().__init__("airframe_envelope")
            self.mode_pub = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
            self.setpoint_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
            self.command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", qos)
            self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self.on_position, qos)
            self.create_subscription(VehicleAttitude, "/fmu/out/vehicle_attitude", self.on_attitude, qos)
            self.position = None
            self.tilt_deg = 0.0
            self.velocity_north = 0.0     # the commanded velocity along north
            self.phase = "boot"
            self.samples = csv.writer((out / "samples.csv").open("w", newline=""))
            self.samples.writerow(["t_s", "x", "y", "z", "vx", "vy", "vz", "tilt_deg", "phase", "commanded_mps",
                                   "wall_s"])
            self.create_timer(0.05, self.tick)

        def on_attitude(self, message) -> None:
            w, x, y, z = message.q
            # The angle between the body's down axis and the world's.
            self.tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, 1.0 - 2.0 * (x * x + y * y)))))

        def on_position(self, message) -> None:
            self.position = message
            self.samples.writerow([f"{message.timestamp * 1e-6:.3f}", f"{message.x:.3f}", f"{message.y:.3f}",
                                   f"{message.z:.3f}", f"{message.vx:.3f}", f"{message.vy:.3f}", f"{message.vz:.3f}",
                                   f"{self.tilt_deg:.2f}", self.phase, f"{self.velocity_north:.2f}",
                                   f"{time.time():.3f}"])

        def stamp(self) -> int:
            return int(self.get_clock().now().nanoseconds / 1000)

        def command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
            message = VehicleCommand()
            message.command = command
            message.param1 = param1
            message.param2 = param2
            message.target_system = 1
            message.target_component = 1
            message.source_system = 1
            message.source_component = 1
            message.from_external = True
            message.timestamp = self.stamp()
            self.command_pub.publish(message)

        def tick(self) -> None:
            mode = OffboardControlMode()
            mode.position = self.phase in ("boot", "climb")
            mode.velocity = self.phase not in ("boot", "climb")
            mode.timestamp = self.stamp()
            self.mode_pub.publish(mode)
            setpoint = TrajectorySetpoint()
            if self.phase in ("boot", "climb"):
                setpoint.position = [0.0, 0.0, -HEIGHT_M]
                setpoint.velocity = [nan, nan, nan]
            else:
                setpoint.position = [nan, nan, -HEIGHT_M]
                setpoint.velocity = [self.velocity_north, 0.0, nan]
            setpoint.acceleration = [nan, nan, nan]
            setpoint.yaw = 0.0
            setpoint.timestamp = self.stamp()
            self.setpoint_pub.publish(setpoint)

    rclpy.init()
    pilot = Pilot()

    def spin(seconds: float) -> None:
        end = pilot.get_clock().now().nanoseconds + int(seconds * 1e9)
        while pilot.get_clock().now().nanoseconds < end:
            rclpy.spin_once(pilot, timeout_sec=0.02)

    def speed() -> float:
        return math.hypot(pilot.position.vx, pilot.position.vy)

    def now() -> float:
        return pilot.position.timestamp * 1e-6

    print("waiting for the autopilot's position", flush=True)
    for _ in range(600):
        spin(0.1)
        if pilot.position is not None and pilot.position.xy_valid:
            break
    else:
        print("no valid local position from the autopilot", flush=True)
        return 1
    spin(1.0)
    for _ in range(40):                      # offboard and armed, asked until the vehicle climbs
        pilot.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        pilot.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        pilot.phase = "climb"
        spin(0.5)
        if -pilot.position.z > 1.0:
            break
    else:
        print("the vehicle did not take off", flush=True)
        return 1
    for _ in range(600):
        spin(0.1)
        if abs(-pilot.position.z - HEIGHT_M) < 0.5 and speed() < 0.3:
            break
    spin(3.0)
    total = len(speeds) * 3
    done = 0
    print(f"runs start: {total} runs", flush=True)
    for commanded in speeds:
        for repeat in range(3):
            direction = 1.0 if repeat % 2 == 0 else -1.0
            if pilot.position.x > 150.0:
                direction = -1.0
            if pilot.position.x < -150.0:
                direction = 1.0
            pilot.phase = f"reach:{commanded:g}:{repeat}"
            pilot.velocity_north = direction * commanded
            started = now()
            held_since = None
            while now() - started < REACH_TIMEOUT_S:
                spin(0.05)
                if abs(speed() - commanded) <= HOLD_TOLERANCE_MPS:
                    held_since = held_since if held_since is not None else now()
                    if now() - held_since >= HOLD_S:
                        break
                else:
                    held_since = None
            pilot.phase = f"brake:{commanded:g}:{repeat}"
            pilot.velocity_north = 0.0
            started = now()
            while now() - started < REST_TIMEOUT_S and speed() > REST_MPS:
                spin(0.02)
            pilot.phase = "rest"
            spin(4.0)
            done += 1
            print(f"runs {done}/{total}: {commanded:g} m/s repeat {repeat + 1}", flush=True)
    pilot.phase = "land"
    pilot.command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
    spin(2.0)
    rclpy.shutdown()
    return 0


def motor_shares(path: Path) -> list[tuple[float, float]]:
    """(wall seconds, the fastest motor's share of its top speed): the wrapper's record of the motor commands."""
    if not path.is_file():
        return []
    shares = []
    for line in path.read_text().splitlines():
        parts = line.split()
        if len(parts) == 2:
            shares.append((float(parts[0]), float(parts[1]) / MOTOR_TOP_RADPS))
    return shares


def report(out: Path) -> int:
    rows = list(csv.DictReader((out / "samples.csv").open()))
    motors = motor_shares(out / "motors.txt")
    motor_times = [t for t, _ in motors]

    def motor_peak(start: float, end: float) -> float:
        values = [share for t, share in motors if start <= t <= end]
        return max(values) if values else float("nan")

    def motor_mean(start: float, end: float) -> float:
        values = [share for t, share in motors if start <= t <= end]
        return statistics.fmean(values) if values else float("nan")

    phases: dict[str, list[dict]] = {}
    for row in rows:
        phases.setdefault(row["phase"], []).append(row)
    lines = ["speed m/s | run | reached m/s | time to it s | distance to it m | tilt held deg | motors held | "
             "brake distance m | brake time s | mean decel m/s2 | peak decel m/s2 | brake tilt deg | motors peak | "
             "height lost m"]
    by_speed: dict[float, list[tuple]] = {}
    for name in phases:
        if not name.startswith("reach:"):
            continue
        _, commanded, repeat = name.split(":")
        reach = phases[name]
        brake = phases.get(f"brake:{commanded}:{repeat}", [])
        if len(reach) < 5 or len(brake) < 5:
            continue
        t = [float(r["t_s"]) for r in reach]
        v = [math.hypot(float(r["vx"]), float(r["vy"])) for r in reach]
        x = [float(r["x"]) for r in reach]
        last = [vi for ti, vi in zip(t, v) if ti >= t[-1] - 2.0]
        reached = statistics.fmean(last)
        first = next((i for i, vi in enumerate(v) if vi >= 0.97 * reached), len(v) - 1)
        tilt_held = statistics.fmean(float(r["tilt_deg"]) for r in reach if float(r["t_s"]) >= t[-1] - 2.0)
        bt = [float(r["t_s"]) for r in brake]
        bv = [math.hypot(float(r["vx"]), float(r["vy"])) for r in brake]
        bx = [float(r["x"]) for r in brake]
        bz = [-float(r["z"]) for r in brake]
        distance = abs(bx[-1] - bx[0])
        duration = bt[-1] - bt[0]
        peak = 0.0
        for i in range(len(bt)):
            j = next((k for k in range(i, len(bt)) if bt[k] - bt[i] >= 0.5), None)
            if j is not None:
                peak = max(peak, (bv[i] - bv[j]) / (bt[j] - bt[i]))
        wall = [float(r["wall_s"]) for r in reach]
        brake_wall = [float(r["wall_s"]) for r in brake]
        held_from = next(w for w, ti in zip(wall, t) if ti >= t[-1] - 2.0)
        row = (float(commanded), int(repeat) + 1, reached, t[first] - t[0], abs(x[first] - x[0]), tilt_held,
               motor_mean(held_from, wall[-1]), distance, duration,
               bv[0] ** 2 / (2.0 * distance) if distance > 0 else float("nan"), peak,
               max(float(r["tilt_deg"]) for r in brake), motor_peak(brake_wall[0], brake_wall[-1]), bz[0] - min(bz))
        by_speed.setdefault(float(commanded), []).append(row)
        lines.append(" | ".join(f"{value:.2f}" if isinstance(value, float) else str(value) for value in row))
    lines.append("")
    lines.append("medians a speed:")
    for commanded in sorted(by_speed):
        runs = by_speed[commanded]
        medians = [statistics.median(run[k] for run in runs) for k in range(2, 14)]
        lines.append(f"{commanded:g} m/s over {len(runs)} runs | " + " | ".join(f"{value:.2f}" for value in medians))
    (out / "summary.txt").write_text("\n".join(lines) + "\n")
    print("\n".join(lines))
    print(f"motor samples {len(motor_times)}; samples {len(rows)} -> {out / 'summary.txt'}")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--report", action="store_true")
    parser.add_argument("speeds", nargs="*", type=float)
    args = parser.parse_args()
    if args.report:
        return report(args.out)
    return fly(args.out, args.speeds or [5.0, 10.0, 12.0, 15.0, 20.0])


if __name__ == "__main__":
    sys.exit(main())
