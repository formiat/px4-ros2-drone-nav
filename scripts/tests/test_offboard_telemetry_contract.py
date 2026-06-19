#!/usr/bin/env python3
"""Static checks for offboard telemetry log fields."""

from __future__ import annotations

import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
OFFBOARD_NODE = REPO_ROOT / "drone_city_nav/src/px4_offboard_node.cpp"


class OffboardTelemetryContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.offboard_text = OFFBOARD_NODE.read_text(encoding="utf-8")

    def test_telemetry_logs_attitude_at_runtime_rate(self) -> None:
        self.assertIn("telemetry_log_period_s", self.offboard_text)
        self.assertIn("Drone telemetry:", self.offboard_text)
        self.assertIn("attitude[valid=%s age_s=%.2f", self.offboard_text)
        self.assertIn("roll=%.3frad pitch=%.3frad yaw=%.3frad", self.offboard_text)
        self.assertIn("roll_deg=%.1f pitch_deg=%.1f yaw_deg=%.1f", self.offboard_text)
        self.assertIn("tilt_deg=%.1f", self.offboard_text)

    def test_offboard_node_subscribes_to_px4_attitude(self) -> None:
        self.assertIn("#include <px4_msgs/msg/vehicle_attitude.hpp>", self.offboard_text)
        self.assertIn('"px4_vehicle_attitude_topic"', self.offboard_text)
        self.assertIn("create_subscription<px4_msgs::msg::VehicleAttitude>", self.offboard_text)
        self.assertIn("quaternionToEuler(msg.q)", self.offboard_text)

    def test_runtime_configs_expose_attitude_topic(self) -> None:
        for relative_path in (
            "drone_city_nav/config/urban_mvp.yaml",
            "drone_city_nav/config/real_drone_template.yaml",
        ):
            with self.subTest(relative_path=relative_path):
                text = (REPO_ROOT / relative_path).read_text(encoding="utf-8")
                self.assertIn(
                    "px4_vehicle_attitude_topic: /fmu/out/vehicle_attitude", text
                )


if __name__ == "__main__":
    unittest.main()
