#!/usr/bin/env python3
"""bag_imu_losses.py BAG: the autopilot's raw IMU topic in a stack bag, a stream of 4 ms: every longer step (a
sample the bag lacks) with the counts of the raw and the relayed samples 0.2 s around it. A relayed copy that holds
more than the raw topic there received the sample the recorder did not: lost between the agent and the recorder.
In the container."""
import sys

RAW, RELAYED = "/fmu/out/sensor_combined", "/fmu/out/sensor_combined/relayed"
PERIOD_US, AROUND_US = 4000, 200_000


def stamps(bag: str) -> dict[str, list[int]]:
    import rosbag2_py
    from px4_msgs.msg import SensorCombined
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[RAW, RELAYED]))
    out: dict[str, list[int]] = {RAW: [], RELAYED: []}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        out[topic].append(deserialize_message(data, SensorCombined).timestamp)
    return out


def losses(raw: list[int], relayed: list[int]) -> list[tuple[int, int, int, int]]:
    """Every step of the raw stream longer than its period: the stamp before it, the step, the two counts around."""
    found = []
    for before, after in zip(raw, raw[1:]):
        if after - before > PERIOD_US:
            low, high = before - AROUND_US, before + AROUND_US
            found.append((before, after - before, sum(1 for v in raw if low < v <= high),
                          sum(1 for v in relayed if low < v <= high)))
    return found


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    topics = stamps(sys.argv[1])
    raw, relayed = topics[RAW], topics[RELAYED]
    print(f"raw {len(raw)} relayed {len(relayed)} samples")
    for before, step, n_raw, n_relayed in losses(raw, relayed):
        print(f"raw step {step} us after {before * 1.0e-6:.3f} s: within 0.2 s raw {n_raw} relayed {n_relayed}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
