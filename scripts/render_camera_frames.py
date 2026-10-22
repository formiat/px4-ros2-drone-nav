#!/usr/bin/env python3
"""render_camera_frames.py OUTPUT_DIRECTORY [--topic T]: the vehicle's own left frame, as the bag
replays it after the smoke relay, kept for the recording's inset: every frame at half
scale (each pixel the mean of a 2 x 2 block of the mono8 frame) as raw bytes, OUTPUT_DIRECTORY/NNNNNN.raw,
and a line of OUTPUT_DIRECTORY/frames.csv: index, the frame's stamp in seconds of simulation,
width, height. The recorder (record_flight_video.py) pastes the latest frame at or before each
moment of the flight into the world half of the follow view."""
import argparse
import csv
import sys
from pathlib import Path

SCALE = 2


def reduced(data: bytes, width: int, height: int) -> tuple[bytes, int, int]:
    """A mono8 frame at 1/SCALE scale, each pixel the mean of a SCALE x SCALE
    block (the pair's noise, A12, averaged rather than sampled)."""
    import numpy
    frame = numpy.frombuffer(data, dtype=numpy.uint8)[: width * height].reshape(height, width)
    rows, columns = height // SCALE, width // SCALE
    blocks = frame[: rows * SCALE, : columns * SCALE].reshape(rows, SCALE, columns, SCALE)
    small = blocks.mean(axis=(1, 3)).astype(numpy.uint8)
    return small.tobytes(), columns, rows


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
        data, width, height = reduced(bytes(message.data), message.width, message.height)
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
