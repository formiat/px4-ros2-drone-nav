#!/usr/bin/env python3
"""render_camera_frames.py OUTPUT_DIRECTORY [--topic T]: the vehicle's own left frame, as the bag
replays it after the smoke relay, kept for the recording's inset: every frame a quarter
scale (every fourth row and column of the mono8 frame) as raw bytes, OUTPUT_DIRECTORY/NNNNNN.raw,
and a line of OUTPUT_DIRECTORY/frames.csv: index, the frame's stamp in seconds of simulation,
width, height. The recorder (record_flight_video.py) pastes the latest frame at or before each
moment of the flight into the world half of the follow view."""
import argparse
import csv
import sys
from pathlib import Path

SCALE = 4


def quarter(data: bytes, width: int, height: int) -> tuple[bytes, int, int]:
    """Every SCALE-th row and column of a mono8 frame."""
    import numpy
    frame = numpy.frombuffer(data, dtype=numpy.uint8)[: width * height].reshape(height, width)
    small = numpy.ascontiguousarray(frame[::SCALE, ::SCALE])
    return small.tobytes(), small.shape[1], small.shape[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_directory", type=Path)
    parser.add_argument("--topic", default="/stereo/left/image")
    args = parser.parse_args()
    args.output_directory.mkdir(parents=True, exist_ok=True)

    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import Image

    rclpy.init()
    node = rclpy.create_node("render_camera_frames")
    table = (args.output_directory / "frames.csv").open("w", newline="")
    writer = csv.writer(table)
    count = 0

    def on_image(message: Image) -> None:
        nonlocal count
        if message.encoding != "mono8":
            return
        data, width, height = quarter(bytes(message.data), message.width, message.height)
        (args.output_directory / f"{count:06d}.raw").write_bytes(data)
        stamp_s = message.header.stamp.sec + message.header.stamp.nanosec * 1.0e-9
        writer.writerow([count, f"{stamp_s:.4f}", width, height])
        table.flush()
        count += 1

    node.create_subscription(Image, args.topic, on_image, qos_profile_sensor_data)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        table.close()
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
