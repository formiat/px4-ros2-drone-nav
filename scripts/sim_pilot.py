#!/usr/bin/env python3
"""The pilot of a simulated flight, an evaluation component (roadmap item 27 block 1,
specification K29). The production nodes never arm the vehicle or switch its mode:
the pilot does, with the commands a radio or a ground station sends PX4. At the
start it puts each vehicle in offboard and arms it once the offboard streams its
setpoints; afterwards it never arms again. A flight may carry a schedule of the
pilot's and the operator's actions (PILOT_EVENTS), on the autopilot's clock (the
simulation's, in lockstep):

  hold@T      the pilot switches to Hold (PX4 AUTO_LOITER)
  posctl@T    the pilot switches to Position mode
  land@T      the pilot switches to Land (PX4 AUTO_LAND)
  kill@T      the pilot kills the motors (a forced disarm in flight)
  rc_loss@T   the pilot's radio falls silent (with --rc it is heard from the start)
  cmd:stop@T, cmd:land@T, cmd:home@T, cmd:goal:X,Y,Z@T  the operator's commands
              on the mission's command channel

Every action and every change of each vehicle's mode and arming lands in the
output CSV with its stamp; the vehicle is told nothing else.
"""
from __future__ import annotations

import argparse
import csv
import sys
from dataclasses import dataclass
from pathlib import Path

# PX4's custom modes (main mode, sub mode) for VEHICLE_CMD_DO_SET_MODE.
MODES = {"offboard": (6, 0), "hold": (4, 3), "land": (4, 6), "posctl": (3, 0)}
OFFBOARD_NAV_STATE = 14
FORCE_DISARM_MAGIC = 21196.0
OPERATOR_COMMANDS = ("stop", "land", "home")


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
    parser.add_argument("--output", type=Path, required=True)
    # Roadmap item 36: under the step controller the pilot acts inside the step
    # protocol (its ticks on the controller's steps, then its acknowledgement),
    # so that its arming and its events fall on the same simulation stamps in
    # every flight; the timers above are not started then.
    parser.add_argument("--step-protocol", action="store_true")
    args = parser.parse_args()
    events = parse_events(args.events)
    import rclpy
    from px4_msgs.msg import ManualControlSetpoint, VehicleCommand, VehicleStatus
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
            self.started = False  # armed in offboard once: never armed again
            self.commands = node.create_publisher(
                VehicleCommand, f"{prefix}/in/vehicle_command", px4_qos)
            self.radio = node.create_publisher(
                ManualControlSetpoint, f"{prefix}/in/manual_control_input", px4_qos)
            node.create_subscription(VehicleStatus, f"{prefix}/out/vehicle_status_v1",
                                     self.on_status, px4_qos)

        def on_status(self, status: VehicleStatus) -> None:
            previous = self.status
            self.status = status
            armed = status.arming_state == VehicleStatus.ARMING_STATE_ARMED
            if previous is None or previous.nav_state != status.nav_state:
                record(self.name, "nav_state", str(status.nav_state))
            if previous is None or previous.arming_state != status.arming_state:
                record(self.name, "armed" if armed else "disarmed",
                       f"reason={status.latest_arming_reason if armed else status.latest_disarming_reason}")
            if armed and status.nav_state == OFFBOARD_NAV_STATE and not self.started:
                self.started = True
                record(self.name, "started", "armed in offboard")

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
            if status.nav_state != OFFBOARD_NAV_STATE:
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
