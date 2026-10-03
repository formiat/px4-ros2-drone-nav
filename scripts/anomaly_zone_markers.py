#!/usr/bin/env python3
"""The zones that fail the carried light (ANOMALY_ZONES), drawn in RViz.

A picture for the person watching, nothing else: each zone as its dark core,
the sphere its falloff reaches and a label, on `/evaluation/anomaly_zones`.
It is an evaluation component like the zones themselves
(`scripts/carried_light.py`): no production node subscribes to the topic, and
a contract test holds it so. The zones are in the world frame, which the
`map` frame coincides with.
"""

from __future__ import annotations

import argparse
import sys

TOPIC = "/evaluation/anomaly_zones"
FRAME = "map"


def spheres(zones):
    """(x, y, z, diameter, rgba, namespace) of every sphere drawn: the core
    all but opaque, the falloff's reach faint."""
    drawn = []
    for x, y, z, radius, falloff in zones:
        drawn.append((x, y, z, 2.0 * radius, (0.05, 0.0, 0.1, 0.85), "core"))
        drawn.append((x, y, z, 2.0 * (radius + falloff), (0.5, 0.2, 0.9, 0.12), "falloff"))
    return drawn


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zones", default="")
    args = parser.parse_args()

    from carried_light import zones_from

    zones = zones_from(args.zones)
    if not zones:
        return 0

    import rclpy
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from visualization_msgs.msg import Marker, MarkerArray

    rclpy.init()
    node = rclpy.create_node("anomaly_zone_markers")
    publisher = node.create_publisher(
        MarkerArray, TOPIC,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    array = MarkerArray()
    for index, (x, y, z, diameter, rgba, namespace) in enumerate(spheres(zones)):
        marker = Marker()
        marker.header.frame_id = FRAME
        marker.ns = namespace
        marker.id = index
        marker.type = Marker.SPHERE
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = x, y, z
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = diameter
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = rgba
        array.markers.append(marker)
    for index, (x, y, z, radius, _) in enumerate(zones):
        label = Marker()
        label.header.frame_id = FRAME
        label.ns = "label"
        label.id = index
        label.type = Marker.TEXT_VIEW_FACING
        label.pose.position.x, label.pose.position.y = x, y
        label.pose.position.z = z + radius + 1.0
        label.pose.orientation.w = 1.0
        label.scale.z = 1.2
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = "light fails here"
        array.markers.append(label)
    # Latched for an RViz that starts late, and repeated for one that lost it.
    node.create_timer(2.0, lambda: publisher.publish(array))
    publisher.publish(array)
    rclpy.spin(node)
    return 0


if __name__ == "__main__":
    sys.exit(main())
