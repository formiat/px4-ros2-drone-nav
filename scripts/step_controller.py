#!/usr/bin/env python3
"""The simulator waits for the onboard loop (roadmap item 36, specification K53): a simulation process that pauses
the Gazebo world once the onboard nodes are up and then advances it one step at a time (STEP_MS of simulation
time, the world's 4 ms iterations), after every step telling the nodes the step's number on /drone_city_nav/step
and waiting for each expected node's acknowledgement on /drone_city_nav/step_ack/<name> (the node sends it once it
has processed everything the step brought it: `step_acknowledger.hpp`). Whatever the host does, the world does not
move on before the loop has finished with the step: two flights with the same seeds fly the same metres on any
host, as slowly as the host needs. A node that does not acknowledge within the timeout is named in the log and the
world moves on (the flight is then not a deterministic one, and the log says so). Before the step is announced the
controller waits for the step's sensor outputs (--sensors: a topic and its period in ms, the stamp read from the
message; a sensor is waited for while its latest stamp plus its period lies on or before the step's end, the
lidar's sweep every 100 ms, the autopilot's stream through its bridge every 10 ms), since the simulator's sensors
publish off the render thread after the world's step has
returned and a node that reads the step first acknowledges it before the scan (the twins of 2026-10-30 parted on the
memory's first scan, a frame apart). The CSV carries, every 50 steps, the steps, the wait for the acknowledgements
(wall ms, median and 95th percentile), the wait for the sensors, the timeouts and their nodes, and the wall speed
(simulation seconds per wall second).
"""
from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

WORLD_ITERATION_S = 0.004


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--world", required=True)
    parser.add_argument("--expected", required=True, help="comma-separated acknowledgement names")
    parser.add_argument("--step-ms", type=int, default=20)
    parser.add_argument("--timeout-s", type=float, default=2.0)
    parser.add_argument("--startup-wait-s", type=float, default=240.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sensors", default="",
                        help="comma-separated TOPIC:PERIOD_MS of the sensor outputs a step waits for")
    args = parser.parse_args()

    import rclpy
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.transport13 import Node as GzNode
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
    from rosidl_runtime_py.utilities import get_message
    from std_msgs.msg import UInt64

    expected = [name for name in args.expected.split(",") if name]
    iterations = max(1, round(args.step_ms / (WORLD_ITERATION_S * 1000.0)))
    step_ns = iterations * int(WORLD_ITERATION_S * 1.0e9)
    rclpy.init()
    node = Node("step_controller", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    step_pub = node.create_publisher(UInt64, "/drone_city_nav/step", qos)
    acknowledged: dict[str, int] = {name: -1 for name in expected}
    ack_waits: dict[str, list[float]] = {name: [] for name in expected}
    asked_at = [0.0]

    def listener(name: str):
        def on_ack(message: UInt64) -> None:
            if int(message.data) > acknowledged[name]:
                ack_waits[name].append((time.monotonic() - asked_at[0]) * 1.0e3)
            acknowledged[name] = max(acknowledged[name], int(message.data))
        return on_ack

    for name in expected:
        node.create_subscription(UInt64, f"/drone_city_nav/step_ack/{name}", listener(name), qos)
    # The sensors' latest stamps, from the message's header or the autopilot's timestamp (microseconds, the
    # simulation clock under px4_clock_is_ros_clock).
    sensors: dict[str, int] = {}
    latest_ns: dict[str, int] = {}

    def watch(topic: str, period_ms: str) -> None:
        sensors[topic] = int(float(period_ms) * 1.0e6)
        latest_ns[topic] = -1

        def on_message(message) -> None:
            if hasattr(message, "header"):
                stamp = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            else:
                stamp = int(message.timestamp) * 1_000
            latest_ns[topic] = max(latest_ns[topic], stamp)

        topic_types = dict(node.get_topic_names_and_types()).get(topic)
        if not topic_types:
            print(f"STEP_CONTROLLER sensor {topic} has no type yet; waited for by its publisher", flush=True)
        message_type = get_message(topic_types[0]) if topic_types else None
        if message_type is not None:
            node.create_subscription(message_type, topic, on_message, qos_profile_sensor_data)

    pending_sensors = [entry for entry in args.sensors.split(",") if entry]

    def sensors_due(target: int) -> list[str]:
        """The sensors whose stream stands a period or more behind the step's end: a sensor's latest stamp plus
        its period on or before the end means the step's output is still to come (a stream whose stamps are not
        on the step's grid, the autopilot's at 100 Hz through its bridge, is a period behind at most)."""
        return [topic for topic, period in sensors.items()
                if (latest_ns[topic] if latest_ns[topic] >= 0 else 0) + period <= target]

    gz = GzNode()
    service = f"/world/{args.world}/control"

    def control(**fields) -> bool:
        request = WorldControl()
        for key, value in fields.items():
            setattr(request, key, value)
        ok, response = gz.request(service, request, WorldControl, Boolean, 2000)
        return bool(ok and response.data)

    def sim_ns() -> int:
        return node.get_clock().now().nanoseconds

    log = args.output.open("w", newline="")
    writer = csv.writer(log)
    writer.writerow(["sim_s", "steps", "wait_p50_ms", "wait_p95_ms", "sensor_wait_p95_ms", "sensor_timeouts",
                     "stalls", "runaways", "timeouts", "missing", "wall_speed", "slowest"])
    # The world stays paused from the start (the runtime starts it without -r); said again here for a world
    # that runs. The stack is up when every expected node publishes its acknowledgement.
    if not control(pause=True):
        print("STEP_CONTROLLER the world did not pause", flush=True)
        return 1
    started = time.monotonic()
    while time.monotonic() - started < args.startup_wait_s:
        rclpy.spin_once(node, timeout_sec=0.2)
        missing = [name for name in expected if node.count_publishers(f"/drone_city_nav/step_ack/{name}") == 0]
        if not missing:
            break
    else:
        print(f"STEP_CONTROLLER no acknowledgement publisher from {missing} within {args.startup_wait_s:.0f} s",
              flush=True)
        return 1
    for entry in pending_sensors:
        topic, _, period_ms = entry.partition(":")
        for _ in range(100):
            if dict(node.get_topic_names_and_types()).get(topic):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        watch(topic, period_ms)
    print(f"STEP_CONTROLLER paused at sim {sim_ns() * 1e-9:.3f} s; stepping {iterations} iterations "
          f"({args.step_ms} ms) a step, waiting for {expected}, the sensors {sorted(sensors)}", flush=True)
    index = 0
    waits: list[float] = []
    sensor_waits: list[float] = []
    sensor_timeouts = 0
    stalls = 0
    runaways = 0  # the world found running on its own (the autopilot's bridge unpauses it at its start)
    expected_ns = sim_ns()
    timeouts = 0
    missing_names: dict[str, int] = {}
    window_started = time.monotonic()
    window_sim_started = sim_ns()
    try:
        while rclpy.ok():
            if sim_ns() > expected_ns + step_ns // iterations:
                # The world ran on its own since the last step: pause it again and step from where it is.
                runaways += 1
                control(pause=True)
                rclpy.spin_once(node, timeout_sec=0.05)
            target_ns = sim_ns() + step_ns
            expected_ns = target_ns
            # pause=True in the same request: the message's default (false) would set the world running.
            if not control(pause=True, multi_step=iterations):
                stalls += 1
            # The world's iterations done: the clock has reached the step's end.
            deadline = time.monotonic() + 5.0
            while sim_ns() < target_ns and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.001)
            if sim_ns() < target_ns:
                stalls += 1
            # The step's sensor outputs, before the nodes hear of the step.
            sensors_asked = time.monotonic()
            deadline = sensors_asked + 2.0
            while sensors_due(target_ns) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.0005)
            sensor_waits.append((time.monotonic() - sensors_asked) * 1.0e3)
            if sensors_due(target_ns):
                sensor_timeouts += 1
            index += 1
            asked = time.monotonic()
            asked_at[0] = asked
            step_pub.publish(UInt64(data=index))
            deadline = asked + args.timeout_s
            while any(acknowledged[name] < index for name in expected) and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.0005)
            late = [name for name in expected if acknowledged[name] < index]
            waits.append((time.monotonic() - asked) * 1.0e3)
            if late:
                timeouts += 1
                for name in late:
                    missing_names[name] = missing_names.get(name, 0) + 1
            if index % 50 == 0:
                ordered = sorted(waits)
                wall = time.monotonic() - window_started
                speed = (sim_ns() - window_sim_started) * 1.0e-9 / wall if wall > 0 else 0.0
                medians = {name: sorted(values)[len(values) // 2] for name, values in ack_waits.items() if values}
                slowest = max(medians, key=medians.get) if medians else ""
                sensor_ordered = sorted(sensor_waits)
                writer.writerow([f"{sim_ns() * 1e-9:.3f}", index, f"{ordered[len(ordered) // 2]:.1f}",
                                 f"{ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]:.1f}",
                                 f"{sensor_ordered[min(len(sensor_ordered) - 1, int(0.95 * len(sensor_ordered)))]:.1f}",
                                 sensor_timeouts, stalls, runaways,
                                 timeouts,
                                 ";".join(f"{k}:{v}" for k, v in sorted(missing_names.items())), f"{speed:.3f}",
                                 f"{slowest}:{medians[slowest]:.0f}" if slowest else ""])
                log.flush()
                waits.clear()
                sensor_waits.clear()
                for values in ack_waits.values():
                    values.clear()
                window_started = time.monotonic()
                window_sim_started = sim_ns()
    except KeyboardInterrupt:
        pass
    finally:
        control(pause=False)
        log.flush()
    return 0


if __name__ == "__main__":
    sys.exit(main())
