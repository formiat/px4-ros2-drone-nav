#!/usr/bin/env python3
"""dds_frames.py pub|sub SECONDS: the pair's frames over the middleware between two processes, as the camera process
publishes them (sensor data QoS, 1280 x 960 RGB, 7.5 Hz, left and right): the subscriber counts what arrives."""
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

role, seconds = sys.argv[1], float(sys.argv[2])
rclpy.init()
node = Node(f"dds_frames_{role}")
if role == "pub":
    pubs = [node.create_publisher(Image, f"/stereo/{side}/image", qos_profile_sensor_data) for side in ("left", "right")]
    frame = Image()
    frame.width, frame.height, frame.encoding, frame.step = 1280, 960, "rgb8", 1280 * 3
    frame.data = bytes(1280 * 960 * 3)
    start = time.time()
    sent = 0
    while time.time() - start < seconds:
        frame.header.stamp = node.get_clock().now().to_msg()
        for pub in pubs:
            pub.publish(frame)
        sent += 2
        time.sleep(1.0 / 7.5)
    print(f"PUB sent {sent}", flush=True)
else:
    counts = {"left": 0, "right": 0}

    def make(side):
        def on_frame(_msg):
            counts[side] += 1
        return on_frame

    subs = [node.create_subscription(Image, f"/stereo/{side}/image", make(side), qos_profile_sensor_data)
            for side in ("left", "right")]
    start = time.time()
    while time.time() - start < seconds:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"SUB received left {counts['left']} right {counts['right']}", flush=True)
node.destroy_node()
rclpy.shutdown()
