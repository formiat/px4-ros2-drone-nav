#!/usr/bin/env python3
"""The pilot of a simulated flight, an evaluation component (roadmap item 27 block 1,
specification K29). The production nodes never arm the vehicle or switch its mode:
the pilot does, with the commands a radio or a ground station sends PX4. At the
start it puts each vehicle in offboard and arms it once the offboard streams its
setpoints; afterwards it never arms again. With --external-mode it flies the
stack's own mode (specification K91) as a pilot flies a mode of PX4's: it arms
in Takeoff, holds once the autopilot has taken off (1.5 m over where it was
armed and the climb stopped), and after two seconds of Hold selects the stack's
mode;
an external mode cannot take off by itself (PX4's takeoff is the flight tasks',
which run in no external mode), and one selected before the autopilot's takeoff
has ended keeps that takeoff's wish to take off (vehicle_constraints, the flight
tasks'), which the autopilot never clears in an external mode: on the ground at
the mission's end it spooled up again and again and never disarmed (r4199,
r4201). Hold, a flight task holding where the vehicle is, publishes no wish. A flight may carry a schedule of the
pilot's and the operator's actions (PILOT_EVENTS), on the autopilot's clock (the
simulation's, in lockstep):

  hold@T      the pilot switches to Hold (PX4 AUTO_LOITER)
  posctl@T    the pilot switches to Position mode
  land@T      the pilot switches to Land (PX4 AUTO_LAND)
  kill@T      the pilot kills the motors (a forced disarm in flight)
  rc_loss@T   the pilot's radio falls silent (with --rc it is heard from the start)
  cmd:stop@T, cmd:land@T, cmd:home@T, cmd:goal:X,Y,Z@T  the operator's commands
              on the mission's command channel
  fault:stop:NODE@T, fault:resume:NODE@T  an injected failure of an onboard
              process, for the failsafe ladder (specification K90): the flight's
              own process whose executable is NODE is stopped where it stands
              (SIGSTOP) and let go on (SIGCONT), as a hung computer does it

Every action and every change of each vehicle's mode and arming lands in the
output CSV with its stamp; the vehicle is told nothing else.
"""
from __future__ import annotations

import argparse
import csv
import os
import signal
import sys
from dataclasses import dataclass
from pathlib import Path

# PX4's custom modes (main mode, sub mode) for VEHICLE_CMD_DO_SET_MODE.
MODES = {"offboard": (6, 0), "hold": (4, 3), "land": (4, 6), "posctl": (3, 0), "takeoff": (4, 2)}
OFFBOARD_NAV_STATE = 14
TAKEOFF_NAV_STATE = 17
# The autopilot's takeoff has ended: over the height a climb has begun by and
# no longer climbing, where PX4's Takeoff holds (MIS_TAKEOFF_ALT, 2.5 m; r4198:
# the wish to take off cleared at 2.3 m and 0.26 m/s, the climb under 0.05 m/s
# 3 s later). Off the ground by the land detector alone, the motors spooling
# up before the climb read as stopped (r4200, the mode selected at 2 s).
TAKEN_OFF_M = 1.5
CLIMB_STOPPED_MPS = 0.1
LOITER_NAV_STATE = 4
HOLD_BEFORE_THE_MODE_S = 2.0
# PX4's external modes: nav states 23 to 30, selected as AUTO (4) with the
# sub modes 11 to 18 (commander px4_custom_mode.h, v1.17.0).
FIRST_EXTERNAL_NAV_STATE = 23
LAST_EXTERNAL_NAV_STATE = 30
FIRST_EXTERNAL_SUB_MODE = 11


def external_nav_state(can_set_nav_states_mask: int) -> int | None:
    """The first external mode PX4 lets be selected: the stack's, the one registered."""
    for nav_state in range(FIRST_EXTERNAL_NAV_STATE, LAST_EXTERNAL_NAV_STATE + 1):
        if can_set_nav_states_mask & (1 << nav_state):
            return nav_state
    return None
FORCE_DISARM_MAGIC = 21196.0
OPERATOR_COMMANDS = ("stop", "land", "home")
FAULT_SIGNALS = {"stop": signal.SIGSTOP, "resume": signal.SIGCONT}


def signal_processes(executable: str, signal_number: int) -> list[int]:
    """Signals every process of this flight whose executable is named so; its pids."""
    pids = []
    for entry in os.listdir("/proc"):
        if not entry.isdigit() or int(entry) == os.getpid():
            continue
        try:
            with open(f"/proc/{entry}/cmdline", "rb") as stream:
                program = stream.read().split(b"\0")[0].decode(errors="replace")
        except OSError:
            continue
        if os.path.basename(program) == executable:
            try:
                os.kill(int(entry), signal_number)
                pids.append(int(entry))
            except OSError:
                continue
    return pids


@dataclass(frozen=True)
class Event:
    at_s: float
    action: str  # a mode, "kill", "rc_loss" or "cmd:<operator command>"


def parse_events(text: str) -> list[Event]:
    events = []
    for item in filter(None, (part.strip() for part in text.split(";"))):
        action, _, at = item.rpartition("@")
        if not action:
            raise ValueError(f"an event needs ACTION@SECONDS: '{item}'")
        at_s = float(at)
        if action.startswith("cmd:"):
            command = action[4:]
            if command.startswith("goal:"):
                values = command[5:].split(",")
                if len(values) != 3:
                    raise ValueError(f"a goal needs X,Y,Z: '{item}'")
                [float(value) for value in values]
            elif command not in OPERATOR_COMMANDS:
                raise ValueError(f"unknown operator command '{command}'")
        elif action.startswith("fault:"):
            parts = action.split(":")
            if len(parts) != 3 or parts[1] not in FAULT_SIGNALS or not parts[2]:
                raise ValueError(f"a fault needs fault:stop:NODE or fault:resume:NODE: '{item}'")
        elif action not in ("hold", "posctl", "land", "kill", "rc_loss"):
            raise ValueError(f"unknown pilot action '{action}'")
        events.append(Event(at_s, action))
    return sorted(events, key=lambda event: event.at_s)


def operator_text(action: str) -> str:
    """The operator's command as the mission reads it: "goal X Y Z" for a goal."""
    command = action[4:]
    if command.startswith("goal:"):
        return "goal " + " ".join(command[5:].split(","))
    return command


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--namespaces", default="",
                        help="the vehicles' PX4 namespaces, space separated; empty for one")
    parser.add_argument("--events", default="")
    parser.add_argument("--rc", action="store_true",
                        help="a radio heard from the start (needed for rc_loss)")
    parser.add_argument("--operator-topic", default="/drone_city_nav/operator_command")
    parser.add_argument("--external-mode", action="store_true",
                        help="select the stack's own PX4 mode instead of offboard (K91)")
    parser.add_argument("--output", type=Path, required=True)
    # Roadmap item 36: under the step controller the pilot acts inside the step
    # protocol (its ticks on the controller's steps, then its acknowledgement),
    # so that its arming and its events fall on the same simulation stamps in
    # every flight; the timers above are not started then.
    parser.add_argument("--step-protocol", action="store_true")
    args = parser.parse_args()
    events = parse_events(args.events)
    import rclpy
    from px4_msgs.msg import (ManualControlSetpoint, VehicleCommand, VehicleLocalPosition,
                              VehicleStatus)
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    rclpy.init()
    node = Node("sim_pilot")
    px4_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         durability=DurabilityPolicy.VOLATILE,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
    operator = node.create_publisher(
        String, args.operator_topic,
        QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL,
                   history=HistoryPolicy.KEEP_LAST, depth=10))
    log = args.output.open("w", newline="")
    writer = csv.writer(log)
    writer.writerow(["clock_s", "vehicle", "what", "detail"])

    class Vehicle:
        def __init__(self, index: int, namespace: str) -> None:
            prefix = f"/{namespace}/fmu" if namespace else "/fmu"
            self.name = namespace or "px4"
            self.system = index + 1
            self.status: VehicleStatus | None = None
            self.ground_z: float | None = None  # NED, where the vehicle stood
            self.taken_off = False  # over TAKEN_OFF_M above it, the climb stopped
            self.hold_since_s: float | None = None  # Hold after the takeoff, since
            self.started = False  # armed in offboard once: never armed again
            self.commands = node.create_publisher(
                VehicleCommand, f"{prefix}/in/vehicle_command", px4_qos)
            self.radio = node.create_publisher(
                ManualControlSetpoint, f"{prefix}/in/manual_control_input", px4_qos)
            node.create_subscription(VehicleStatus, f"{prefix}/out/vehicle_status_v1",
                                     self.on_status, px4_qos)
            if args.external_mode:
                node.create_subscription(VehicleLocalPosition,
                                         f"{prefix}/out/vehicle_local_position_v1",
                                         self.on_local_position, px4_qos)

        def on_local_position(self, position: VehicleLocalPosition) -> None:
            armed = (self.status is not None
                     and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED)
            if not armed:
                self.ground_z = position.z
            self.taken_off = (self.ground_z is not None
                              and position.z < self.ground_z - TAKEN_OFF_M
                              and abs(position.vz) < CLIMB_STOPPED_MPS)

        def on_status(self, status: VehicleStatus) -> None:
            previous = self.status
            self.status = status
            armed = status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            if previous is None or previous.nav_state != status.nav_state:
                record(self.name, "nav_state", str(status.nav_state))
            if previous is None or previous.arming_state != status.arming_state:
                record(self.name, "armed" if armed else "disarmed",
                       f"reason={status.latest_arming_reason if armed else status.latest_disarming_reason}")
            if armed and status.nav_state == self.start_nav_state() and not self.started:
                self.started = True
                record(self.name, "started",
                       "armed in the stack's mode" if args.external_mode else "armed in offboard")

        def start_nav_state(self) -> int | None:
            if not args.external_mode:
                return OFFBOARD_NAV_STATE
            return external_nav_state(self.status.can_set_nav_states_mask) if self.status else None

        def command(self, command: int, param1: float = 0.0, param2: float = 0.0,
                    param3: float = 0.0) -> None:
            message = VehicleCommand()
            message.timestamp = int(node.get_clock().now().nanoseconds / 1000)
            message.command = command
            message.param1, message.param2, message.param3 = param1, param2, param3
            message.target_system = self.system
            message.target_component = 1
            message.source_system = 255  # a ground station's system id
            message.source_component = 190
            message.from_external = True
            self.commands.publish(message)

        def set_mode(self, mode: str) -> None:
            if mode == "external":
                nav_state = self.start_nav_state()
                main_mode, sub_mode = 4, FIRST_EXTERNAL_SUB_MODE + nav_state - FIRST_EXTERNAL_NAV_STATE
                self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, float(main_mode),
                             float(sub_mode))
                return
            main_mode, sub_mode = MODES[mode]
            self.command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, float(main_mode),
                         float(sub_mode))

    def clock_s() -> float:
        stamps = [vehicle.status.timestamp for vehicle in vehicles if vehicle.status]
        return max(stamps) * 1.0e-6 if stamps else 0.0

    def record(vehicle: str, what: str, detail: str = "") -> None:
        writer.writerow([f"{clock_s():.3f}", vehicle, what, detail])
        log.flush()
        print(f"SIM_PILOT clock_s={clock_s():.3f} vehicle={vehicle} {what} {detail}",
              flush=True)

    vehicles = [Vehicle(index, namespace)
                for index, namespace in enumerate(args.namespaces.split() or [""])]
    radio_on = args.rc
    pending = list(events)

    def start_tick() -> None:
        # The pilot's start: offboard first (PX4 takes it once the stream of
        # setpoints is there), then the arming; never again once started.
        for vehicle in vehicles:
            status = vehicle.status
            if status is None or vehicle.started:
                continue
            start = vehicle.start_nav_state()
            if start is None:
                continue  # the stack's mode is not listed yet
            armed = status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            if args.external_mode:
                # Takeoff, armed in it; the stack's mode once the autopilot's
                # takeoff has ended (PX4's Takeoff holds at its height and stays).
                if not armed and status.nav_state != TAKEOFF_NAV_STATE:
                    vehicle.set_mode("takeoff")
                elif not armed:
                    vehicle.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
                elif status.nav_state == LOITER_NAV_STATE:
                    vehicle.hold_since_s = vehicle.hold_since_s or clock_s()
                    if clock_s() - vehicle.hold_since_s >= HOLD_BEFORE_THE_MODE_S:
                        vehicle.set_mode("external")
                elif vehicle.taken_off:
                    vehicle.set_mode("hold")
                continue
            if status.nav_state != start:
                vehicle.set_mode("offboard")
            elif status.arming_state != VehicleStatus.ARMING_STATE_ARMED:
                vehicle.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)

    def radio_tick() -> None:
        if not radio_on:
            return
        for vehicle in vehicles:
            sticks = ManualControlSetpoint()
            sticks.timestamp = int(node.get_clock().now().nanoseconds / 1000)
            sticks.valid = True
            sticks.data_source = ManualControlSetpoint.SOURCE_MAVLINK_0
            vehicle.radio.publish(sticks)

    def event_tick() -> None:
        nonlocal radio_on
        now_s = clock_s()
        while pending and now_s >= pending[0].at_s:
            event = pending.pop(0)
            if event.action.startswith("cmd:"):
                text = operator_text(event.action)
                operator.publish(String(data=text))
                record("operator", "command", text)
                continue
            if event.action.startswith("fault:"):
                _, what, executable = event.action.split(":")
                pids = signal_processes(executable, FAULT_SIGNALS[what])
                record("fault", what, f"{executable} pids={' '.join(map(str, pids)) or 'none'}")
                continue
            for vehicle in vehicles:
                if event.action == "kill":
                    vehicle.command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0,
                                    FORCE_DISARM_MAGIC)
                elif event.action == "rc_loss":
                    radio_on = False
                else:
                    vehicle.set_mode(event.action)
                record(vehicle.name, "pilot", event.action)

    if args.step_protocol:
        from std_msgs.msg import UInt64
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

        step_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
        acknowledgement = node.create_publisher(UInt64, "/drone_city_nav/step_ack/pilot", step_qos)

        def on_step(step: UInt64) -> None:
            # The 20 ms steps: the events every 5 (100 ms), the start every 50 (1 s), the radio every 3 (60 ms).
            index = int(step.data)
            if index % 5 == 0:
                event_tick()
            if index % 50 == 0:
                start_tick()
            if index % 3 == 0:
                radio_tick()
            acknowledgement.publish(step)

        node.create_subscription(UInt64, "/drone_city_nav/step", on_step, step_qos)
    else:
        node.create_timer(1.0, start_tick)
        node.create_timer(0.05, radio_tick)
        node.create_timer(0.1, event_tick)
    print(f"SIM_PILOT vehicles={len(vehicles)} events={len(events)} rc={args.rc}", flush=True)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    log.close()
    node.destroy_node()
    rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
