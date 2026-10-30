#!/usr/bin/env python3
"""The simulator waits for the onboard loop (roadmap item 36, specification K53): a simulation process that pauses
the Gazebo world once the onboard nodes are up and then advances it one step at a time (STEP_MS of simulation
time, the world's 4 ms iterations), after every step telling the nodes the step's number on /drone_city_nav/step
and waiting for each expected node's acknowledgement on /drone_city_nav/step_ack/<name> (the node sends it once it
has processed everything the step brought it: `step_acknowledger.hpp`). Whatever the host does, the world does not
move on before the loop has finished with the step: two flights with the same seeds fly the same metres on any
host, as slowly as the host needs. A node that does not acknowledge within the timeout is named in the log and the
world moves on (the flight is then not a deterministic one, and the log says so). The CSV carries, every 50 steps,
the steps, the wait for the acknowledgements (wall ms, median and 95th percentile), the timeouts and their nodes,
and the wall speed (simulation seconds per wall second).
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
    args = parser.parse_args()

    import rclpy
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.world_control_pb2 import WorldControl
    from gz.transport13 import Node as GzNode
    from rclpy.node import Node
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import UInt64

    expected = [name for name in args.expected.split(",") if name]
    iterations = max(1, round(args.step_ms / (WORLD_ITERATION_S * 1000.0)))
    step_ns = iterations * int(WORLD_ITERATION_S * 1.0e9)
    rclpy.init()
    node = Node("step_controller", parameter_overrides=[rclpy.parameter.Parameter("use_sim_time", value=True)])
    qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=10)
    step_pub = node.create_publisher(UInt64, "/drone_city_nav/step", qos)
    acknowledged: dict[str, int] = {name: -1 for name in expected}

    def listener(name: str):
        def on_ack(message: UInt64) -> None:
            acknowledged[name] = max(acknowledged[name], int(message.data))
        return on_ack

    for name in expected:
        node.create_subscription(UInt64, f"/drone_city_nav/step_ack/{name}", listener(name), qos)
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
    writer.writerow(["sim_s", "steps", "wait_p50_ms", "wait_p95_ms", "stalls", "runaways", "timeouts", "missing",
                     "wall_speed"])
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
    print(f"STEP_CONTROLLER paused at sim {sim_ns() * 1e-9:.3f} s; stepping {iterations} iterations "
          f"({args.step_ms} ms) a step, waiting for {expected}", flush=True)
    index = 0
    waits: list[float] = []
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
            if not control(multi_step=iterations):
                stalls += 1
            # The world's iterations done: the clock has reached the step's end.
            deadline = time.monotonic() + 5.0
            while sim_ns() < target_ns and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.001)
            if sim_ns() < target_ns:
                stalls += 1
            index += 1
            step_pub.publish(UInt64(data=index))
            asked = time.monotonic()
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
                writer.writerow([f"{sim_ns() * 1e-9:.3f}", index, f"{ordered[len(ordered) // 2]:.1f}",
                                 f"{ordered[min(len(ordered) - 1, int(0.95 * len(ordered)))]:.1f}", stalls, runaways,
                                 timeouts,
                                 ";".join(f"{k}:{v}" for k, v in sorted(missing_names.items())), f"{speed:.3f}"])
                log.flush()
                waits.clear()
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
