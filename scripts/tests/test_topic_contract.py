#!/usr/bin/env python3
"""Static tests for raw/prohibited/debug obstacle topic contracts."""

from __future__ import annotations

import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]


def read(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text(encoding="utf-8")


class TopicContractTest(unittest.TestCase):
    def test_runtime_files_do_not_reference_removed_inflated_memory_topic(self) -> None:
        checked_paths = [
            "drone_city_nav/config/urban_mvp.yaml",
            "drone_city_nav/config/real_drone_template.yaml",
            "drone_city_nav/rviz/city_nav_debug.rviz",
            "scripts/record_debug_bag.sh",
            "README.md",
            "CONTRIBUTING.md",
            "docs/MVP_SIMULATION.md",
        ]

        for relative_path in checked_paths:
            with self.subTest(relative_path=relative_path):
                text = read(relative_path)
                self.assertNotIn("obstacle_memory_inflated_grid", text)
                self.assertNotIn("/drone_city_nav/occupancy_grid", text)
                self.assertNotIn("inflated_obstacle_points", text)

    def test_runtime_configs_use_prohibited_grid_contract(self) -> None:
        for relative_path in (
            "drone_city_nav/config/urban_mvp.yaml",
            "drone_city_nav/config/real_drone_template.yaml",
        ):
            with self.subTest(relative_path=relative_path):
                text = read(relative_path)
                self.assertIn(
                    "prohibited_grid_topic: /drone_city_nav/prohibited_grid", text
                )
                self.assertNotIn("occupancy_grid_topic:", text)
                self.assertIn("memory_occupied_value: 100", text)
                self.assertNotIn("memory_occupied_threshold:", text)

    def test_node_sources_use_prohibited_grid_parameter_name(self) -> None:
        checked_paths = [
            "drone_city_nav/src/planner_node_config.cpp",
            "drone_city_nav/src/planner_node.cpp",
            "drone_city_nav/src/px4_offboard_node.cpp",
            "drone_city_nav/src/lidar_debug_node.cpp",
        ]

        for relative_path in checked_paths:
            with self.subTest(relative_path=relative_path):
                text = read(relative_path)
                self.assertIn("prohibited_grid", text)
                self.assertNotIn('"occupancy_grid_topic"', text)

    def test_debug_bag_records_raw_memory_and_final_prohibited_grid(self) -> None:
        text = read("scripts/record_debug_bag.sh")

        self.assertIn("/drone_city_nav/obstacle_memory_grid", text)
        self.assertIn("/drone_city_nav/prohibited_grid", text)
        self.assertNotIn("/drone_city_nav/obstacle_memory_inflated_grid", text)
        self.assertNotIn("/drone_city_nav/occupancy_grid", text)

    def test_sensor_hit_depth_names_are_not_legacy_obstacle_depth_names(self) -> None:
        checked_paths = [
            "drone_city_nav/config/urban_mvp.yaml",
            "drone_city_nav/config/real_drone_template.yaml",
            "drone_city_nav/include/drone_city_nav/obstacle_memory.hpp",
            "drone_city_nav/include/drone_city_nav/current_lidar_overlay.hpp",
            "drone_city_nav/src/obstacle_memory.cpp",
            "drone_city_nav/src/current_lidar_overlay.cpp",
            "drone_city_nav/src/planner_node_config.cpp",
        ]

        for relative_path in checked_paths:
            with self.subTest(relative_path=relative_path):
                text = read(relative_path)
                self.assertIn("sensor_hit_depth", text)
                self.assertNotIn("hit_obstacle_depth", text)
                self.assertNotIn("current_lidar_obstacle_depth", text)


if __name__ == "__main__":
    unittest.main()
