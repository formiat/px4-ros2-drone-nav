#!/usr/bin/env python3
"""Apply typed spectator target changes to the Gazebo GUI follow camera."""

from __future__ import annotations

import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from drone_city_nav.msg import SpectatorTarget
from gazebo_gui_control import configure_follow_camera


class GazeboSpectatorFollow(Node):
    def __init__(self, *, world: str, offset: str, wait_s: int) -> None:
        super().__init__("gazebo_spectator_follow")
        self._world = world
        self._offset = offset
        self._wait_s = wait_s
        self._current_model = ""
        spectator_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            SpectatorTarget,
            "/drone_city_nav/spectator_target",
            self._on_target,
            spectator_qos,
        )

    def _on_target(self, target: SpectatorTarget) -> None:
        if not target.gazebo_model or target.gazebo_model == self._current_model:
            return
        model = target.gazebo_model
        self.get_logger().info(
            f"Applying Gazebo spectator target vehicle_id='{target.vehicle_id}' "
            f"model='{model}' epoch={target.mission_epoch}"
        )
        configure_follow_camera(
            world=self._world,
            target=model,
            offset_text=self._offset,
            wait_s=self._wait_s,
            tracking_sample_duration_s=0.25,
            tracking_confirmation_attempts=1,
            required_confirmations=1,
        )
        self._current_model = model


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--world", default="generated_city")
    parser.add_argument("--offset", default="-12 0 6")
    parser.add_argument("--wait-s", type=int, default=15)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = GazeboSpectatorFollow(
        world=args.world,
        offset=args.offset,
        wait_s=max(1, args.wait_s),
    )
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
