#!/usr/bin/env python3
"""replay_first_poses.py FLIGHT_BAG REPLAY_BAG [COUNT]: the first poses of the stereo estimate in a flight's stack
bag and in its replay's, at full precision, with the largest difference of each pair: whether two estimates part at
the declared pose or after it (r2939: 3e-8 rad of heading from the frame after the declaration). In the container."""
import sys

TOPIC = "/drone_city_nav/visual_inertial_odometry/pose"


def poses(bag: str, count: int) -> list:
    import rosbag2_py
    from geometry_msgs.msg import PoseStamped
    from rclpy.serialization import deserialize_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
    reader.set_filter(rosbag2_py.StorageFilter(topics=[TOPIC]))
    out = []
    while reader.has_next() and len(out) < count:
        _, data, _ = reader.read_next()
        out.append(deserialize_message(data, PoseStamped))
    return out


def values(message) -> tuple:
    pose = message.pose
    return (pose.position.x, pose.position.y, pose.position.z,
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)


def main() -> int:
    if len(sys.argv) < 3:
        print(__doc__)
        return 2
    count = int(sys.argv[3]) if len(sys.argv) > 3 else 8
    for flight, replay in zip(poses(sys.argv[1], count), poses(sys.argv[2], count)):
        a, b = values(flight), values(replay)
        stamp = flight.header.stamp.sec + flight.header.stamp.nanosec * 1.0e-9
        print(f"{stamp:.3f} equal={a == b} largest_difference={max(abs(p - q) for p, q in zip(a, b)):.3e}")
        print("   flight", " ".join(f"{v:.12f}" for v in a))
        print("   replay", " ".join(f"{v:.12f}" for v in b))
    return 0


if __name__ == "__main__":
    sys.exit(main())
