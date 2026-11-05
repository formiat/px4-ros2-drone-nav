#!/usr/bin/env python3
"""replay_compare.py FLIGHT_BAG REPLAY_BAG: the replayed stack's outputs against the flight's (roadmap item 27 block
6, tools/replay_stack.sh): the estimators' poses matched by their stamps (the distance's median, 95th percentile
and largest over the matched pairs, and how many of either side found no match), the trajectory setpoints matched by
their stamps (the nearest within 5 ms: the setpoint carries the stamp of its own tick, whose phase the replay's
timer does not share), the share equal in position, velocity and yaw, the largest differences, the mission's
events in order (the first that differs). The verdict against the replay's contract: the matched poses' 95th
percentile within 0.05 m. Run in the dev container (rosbag2_py)."""

from __future__ import annotations

import math
import sys
from pathlib import Path

POSE_TOPICS = ("/drone_city_nav/visual_inertial_odometry/pose", "/drone_city_nav/lidar_inertial_odometry/pose")
SETPOINT_TOPIC = "/fmu/in/trajectory_setpoint"
EVENTS_TOPIC = "/drone_city_nav/mission_events"
POSE_TOLERANCE_M = 0.05


def read_bag(path: Path, topics: tuple[str, ...]) -> dict[str, list[tuple[bytes, object]]]:
    """Each topic's messages in order: the serialized bytes and the message."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="mcap"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {entry.name: get_message(entry.type) for entry in reader.get_all_topics_and_types()}
    reader.set_filter(rosbag2_py.StorageFilter(topics=[t for t in topics if t in types]))
    messages: dict[str, list[tuple[bytes, object]]] = {topic: [] for topic in topics}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        messages[topic].append((data, deserialize_message(data, types[topic])))
    return messages


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def match_by_key(flight: list, replay: list, key) -> tuple[list[tuple], int, int]:
    """The pairs that share a key, the flight's unmatched, the replay's unmatched (a repeated key keeps its first)."""
    flight_by_key = {}
    for item in flight:
        flight_by_key.setdefault(key(item), item)
    replay_by_key = {}
    for item in replay:
        replay_by_key.setdefault(key(item), item)
    pairs = [(flight_by_key[k], replay_by_key[k]) for k in flight_by_key if k in replay_by_key]
    return pairs, len(flight_by_key) - len(pairs), len(replay_by_key) - len(pairs)


def percentile(values: list[float], share: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    return ordered[min(len(ordered) - 1, int(round(share * (len(ordered) - 1))))]


def distance_stats(distances: list[float]) -> str:
    if not distances:
        return "no matched pairs"
    return (f"p50 {percentile(distances, 0.5):.3f} p95 {percentile(distances, 0.95):.3f} "
            f"max {max(distances):.3f} m over {len(distances)} pairs")


def compare_poses(flight: list, replay: list) -> tuple[str, bool]:
    pairs, flight_alone, replay_alone = match_by_key(flight, replay, lambda item: stamp_ns(item[1]))
    distances = [math.dist((a[1].pose.position.x, a[1].pose.position.y, a[1].pose.position.z),
                           (b[1].pose.position.x, b[1].pose.position.y, b[1].pose.position.z)) for a, b in pairs]
    p95 = percentile(distances, 0.95)
    ok = bool(distances) and p95 <= POSE_TOLERANCE_M
    # The distance's median a minute of the flight: where the two part.
    by_minute: dict[int, list[float]] = {}
    for (a, _), d in zip(pairs, distances):
        by_minute.setdefault(stamp_ns(a[1]) // 60_000_000_000, []).append(d)
    minutes = " ".join(f"{percentile(by_minute[m], 0.5):.2f}" for m in sorted(by_minute))
    return (f"{distance_stats(distances)}; unmatched flight {flight_alone} replay {replay_alone}"
            f" ({'within' if ok else 'beyond'} {POSE_TOLERANCE_M:.2f} m at p95); by minute {minutes}"), ok


def vector_difference(a, b) -> float:
    """The distance over the finite components; infinite where one side leaves a component unset and the other not."""
    pairs = [(float(x), float(y)) for x, y in zip(a, b)]
    if any(math.isfinite(x) != math.isfinite(y) for x, y in pairs):
        return math.inf
    return math.sqrt(sum((x - y) ** 2 for x, y in pairs if math.isfinite(x)))


SETPOINT_MATCH_US = 9_000  # under half the offboard's 20 ms tick: the replay's phase is its own
SETPOINT_EQUAL = 1.0e-6


def match_nearest(flight: list, replay: list, key, within: int) -> tuple[list[tuple], int, int]:
    """Each flight item with the replay item nearest by key within the span, each replay item taken once."""
    ordered = sorted(replay, key=key)
    keys = [key(item) for item in ordered]
    import bisect
    taken = set()
    pairs = []
    for item in flight:
        k = key(item)
        i = bisect.bisect_left(keys, k)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(ordered) and j not in taken and abs(keys[j] - k) <= within:
                if best is None or abs(keys[j] - k) < abs(keys[best] - k):
                    best = j
        if best is not None:
            taken.add(best)
            pairs.append((item, ordered[best]))
    return pairs, len(flight) - len(pairs), len(replay) - len(pairs)


def yaw_difference(a, b) -> float:
    if math.isfinite(a) != math.isfinite(b):
        return math.inf
    return abs(math.remainder(a - b, 2.0 * math.pi)) if math.isfinite(a) else 0.0


def compare_setpoints(flight: list, replay: list) -> str:
    pairs, flight_alone, replay_alone = match_nearest(flight, replay, lambda item: int(item[1].timestamp),
                                                      SETPOINT_MATCH_US)
    if not pairs:
        return f"no matched setpoints; unmatched flight {flight_alone} replay {replay_alone}"
    positions = [vector_difference(a[1].position, b[1].position) for a, b in pairs]
    velocities = [vector_difference(a[1].velocity, b[1].velocity) for a, b in pairs]
    yaws = [yaw_difference(float(a[1].yaw), float(b[1].yaw)) for a, b in pairs]
    equal = sum(1 for p, v, y in zip(positions, velocities, yaws) if max(p, v, y) <= SETPOINT_EQUAL)
    return (f"{equal} of {len(pairs)} matched setpoints equal in position, velocity and yaw "
            f"({100.0 * equal / len(pairs):.1f} percent); position difference p95 {percentile(positions, 0.95):.3f} "
            f"max {max(positions):.3f} m, velocity p95 {percentile(velocities, 0.95):.3f} max {max(velocities):.3f} "
            f"m/s, yaw p95 {percentile(yaws, 0.95):.4f} rad; unmatched flight {flight_alone} replay {replay_alone}")


def compare_events(flight: list[str], replay: list[str]) -> str:
    first = next((i for i, (a, b) in enumerate(zip(flight, replay)) if a != b), None)
    if first is None and len(flight) == len(replay):
        return f"{len(flight)} events, the same in order"
    if first is None:
        return f"{min(len(flight), len(replay))} events the same, then the flight has {len(flight)} and the replay {len(replay)}"
    return f"event {first + 1} differs: flight '{flight[first][:80]}' replay '{replay[first][:80]}'"


def main() -> int:
    flight_path, replay_path = Path(sys.argv[1]), Path(sys.argv[2])
    topics = (*POSE_TOPICS, SETPOINT_TOPIC, EVENTS_TOPIC)
    flight, replay = read_bag(flight_path, topics), read_bag(replay_path, topics)
    verdict = True
    for topic in POSE_TOPICS:
        if not flight[topic] and not replay[topic]:
            continue
        line, ok = compare_poses(flight[topic], replay[topic])
        verdict = verdict and ok
        print(f"{'OK' if ok else 'FAIL'}: {topic}: {line}")
    print(f"NOTE: {SETPOINT_TOPIC}: {compare_setpoints(flight[SETPOINT_TOPIC], replay[SETPOINT_TOPIC])}")
    print(f"NOTE: {EVENTS_TOPIC}: "
          f"{compare_events([m.data for _, m in flight[EVENTS_TOPIC]], [m.data for _, m in replay[EVENTS_TOPIC]])}")
    print("REPLAY " + ("PASS" if verdict else "FAIL"))
    return 0 if verdict else 1


if __name__ == "__main__":
    raise SystemExit(main())
