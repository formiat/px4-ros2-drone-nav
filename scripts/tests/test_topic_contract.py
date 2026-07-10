#!/usr/bin/env python3
"""Static tests for raw/prohibited/debug obstacle topic contracts."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]

NODE_RUNTIME_SOURCE_PATHS = (
    "drone_city_nav/src/planner_node.cpp",
    "drone_city_nav/src/planner_node.hpp",
    "drone_city_nav/src/planner_node_config.cpp",
    "drone_city_nav/src/planner_node_inputs.cpp",
    "drone_city_nav/src/planner_node_lifecycle.cpp",
    "drone_city_nav/src/planner_node_publish.cpp",
    "drone_city_nav/src/planner_node_runtime.cpp",
    "drone_city_nav/src/px4_offboard_node.cpp",
    "drone_city_nav/src/px4_offboard_node.hpp",
    "drone_city_nav/src/px4_offboard_node_control.cpp",
    "drone_city_nav/src/px4_offboard_node_inputs.cpp",
    "drone_city_nav/src/px4_offboard_node_lifecycle.cpp",
    "drone_city_nav/src/px4_offboard_node_telemetry.cpp",
    "drone_city_nav/src/px4_offboard_node_trajectory.cpp",
    "drone_city_nav/src/lidar_debug_node.cpp",
    "drone_city_nav/src/lidar_debug_node.hpp",
    "drone_city_nav/src/lidar_debug_node_callbacks.cpp",
    "drone_city_nav/src/lidar_debug_node_config.cpp",
    "drone_city_nav/src/lidar_debug_node_lifecycle.cpp",
    "drone_city_nav/src/lidar_debug_node_points.cpp",
    "drone_city_nav/src/lidar_debug_node_snapshot.cpp",
    "drone_city_nav/src/lidar_debug_snapshot_pipeline.cpp",
)


def read(relative_path: str) -> str:
    return (REPO_ROOT / relative_path).read_text(encoding="utf-8")


class TopicContractTest(unittest.TestCase):
    def test_runtime_files_do_not_reference_removed_inflated_memory_topic(self) -> None:
        checked_paths = [
            "drone_city_nav/config/urban_mvp.yaml",
            "drone_city_nav/rviz/city_nav_debug.rviz",
            "scripts/record_debug_bag.sh",
            "README.md",
            "CONTRIBUTING.md",
            "docs/obstacle_mapping.md",
            "docs/navigation_pipeline.md",
        ]

        for relative_path in checked_paths:
            with self.subTest(relative_path=relative_path):
                text = read(relative_path)
                self.assertNotIn("obstacle_memory_inflated_grid", text)
                self.assertNotIn("/drone_city_nav/occupancy_grid", text)
                self.assertNotIn("inflated_obstacle_points", text)

    def test_runtime_configs_use_prohibited_grid_contract(self) -> None:
        for relative_path in ("drone_city_nav/config/urban_mvp.yaml",):
            with self.subTest(relative_path=relative_path):
                text = read(relative_path)
                self.assertIn(
                    "prohibited_grid_topic: /drone_city_nav/prohibited_grid", text
                )
                self.assertNotIn("occupancy_grid_topic:", text)
                self.assertIn("memory_occupied_value: 100", text)
                self.assertNotIn("memory_occupied_threshold:", text)

    def test_runtime_config_ros_parameters_are_nested_for_ros_parser(self) -> None:
        for relative_path in ("drone_city_nav/config/urban_mvp.yaml",):
            with self.subTest(relative_path=relative_path):
                lines = read(relative_path).splitlines()
                for index, line in enumerate(lines):
                    if "ros__parameters:" not in line:
                        continue
                    self.assertTrue(
                        line.startswith("  ros__parameters:"),
                        f"{relative_path}:{index + 1} must indent ros__parameters "
                        "with exactly two spaces",
                    )
                    self.assertFalse(
                        line.startswith("    ros__parameters:"),
                        f"{relative_path}:{index + 1} is accepted by YAML but "
                        "rejected by the ROS params parser",
                    )

    def test_node_sources_use_prohibited_grid_parameter_name(self) -> None:
        combined_text = "\n".join(
            read(relative_path) for relative_path in NODE_RUNTIME_SOURCE_PATHS
        )
        self.assertIn("prohibited_grid", combined_text)

        for relative_path in NODE_RUNTIME_SOURCE_PATHS:
            with self.subTest(relative_path=relative_path):
                text = read(relative_path)
                self.assertNotIn('"occupancy_grid_topic"', text)

    def test_debug_bag_records_raw_memory_and_final_prohibited_grid(self) -> None:
        text = read("scripts/record_debug_bag.sh")

        self.assertIn("/drone_city_nav/obstacle_memory_grid", text)
        self.assertIn("/drone_city_nav/prohibited_grid", text)
        self.assertIn("/drone_city_nav/raw_memory_obstacle_points", text)
        self.assertIn("/drone_city_nav/prohibited_obstacle_points", text)
        self.assertIn("/drone_city_nav/static_building_markers", text)
        self.assertIn("/drone_city_nav/known_passage_markers", text)
        self.assertNotIn("/drone_city_nav/obstacle_memory_inflated_grid", text)
        self.assertNotIn("/drone_city_nav/occupancy_grid", text)

    def test_known_passage_marker_contract_is_wired_for_debugging(self) -> None:
        yaml_text = read("drone_city_nav/config/urban_mvp.yaml")
        rviz_text = read("drone_city_nav/rviz/city_nav_debug.rviz")
        bag_text = read("scripts/record_debug_bag.sh")

        self.assertIn("known_passages_enabled: true", yaml_text)
        self.assertIn(
            "known_passages_path: worlds/known_passages.passages3d", yaml_text
        )
        self.assertIn(
            "known_passage_markers_topic: /drone_city_nav/known_passage_markers",
            yaml_text,
        )
        self.assertIn(
            "static_building_markers_topic: /drone_city_nav/static_building_markers",
            yaml_text,
        )
        self.assertIn("/drone_city_nav/static_building_markers", rviz_text)
        self.assertIn("/drone_city_nav/static_building_markers", bag_text)
        self.assertIn("/drone_city_nav/known_passage_markers", rviz_text)
        self.assertIn("/drone_city_nav/known_passage_markers", bag_text)

    def test_rviz_uses_gazebo_aligned_map_transform(self) -> None:
        launch_text = read("drone_city_nav/launch/city_nav.launch.py")
        rviz_text = read("drone_city_nav/rviz/city_nav_debug.rviz")

        self.assertIn("gazebo_aligned_map_tf", launch_text)
        self.assertIn("static_transform_publisher", launch_text)
        self.assertIn('"gazebo_map"', launch_text)
        self.assertIn('"map"', launch_text)
        self.assertIn("This transform is intentional", launch_text)
        self.assertIn("Fixed Frame: gazebo_map", rviz_text)
        self.assertIn("Reference Frame: gazebo_map", rviz_text)
        self.assertIn("Target Frame: gazebo_map", rviz_text)

    def test_known_passage_annotations_are_not_legacy_physical_models(self) -> None:
        passage_text = read("drone_city_nav/worlds/known_passages.passages3d")
        sdf_text = read("drone_city_nav/worlds/generated_city.sdf")
        static_map_text = read("drone_city_nav/worlds/generated_city.map2d")

        structure_ids = sorted(
            set(
                re.findall(
                    r"^structure\s+(building_with_passage_\d+)\s",
                    passage_text,
                    re.M,
                )
            )
        )
        self.assertGreater(len(structure_ids), 0)
        for structure_id in structure_ids:
            with self.subTest(structure_id=structure_id):
                self.assertNotIn(f'<model name="{structure_id}">', sdf_text)
                self.assertNotIn(structure_id, static_map_text)
        connector_ids = re.findall(
            r'<model name="(physical_building_connector_\d+_\d+)">', sdf_text
        )
        self.assertEqual(
            connector_ids,
            [
                "physical_building_connector_03_11",
                "physical_building_connector_04_12",
                "physical_building_connector_05_13",
                "physical_building_connector_06_14",
            ],
        )
        for connector_id in connector_ids:
            with self.subTest(connector_id=connector_id):
                model_match = re.search(
                    rf'<model name="{connector_id}">(.*?)</model>', sdf_text, re.S
                )
                self.assertIsNotNone(model_match)
                model_text = model_match.group(1)
                self.assertIn('<link name="lower_mass">', model_text)
                self.assertIn('<link name="upper_mass">', model_text)
                self.assertIn(
                    "<pose>0.00 0.00 7.25 0.00 0.00 0.00</pose>", model_text
                )
                self.assertIn(
                    "<pose>0.00 0.00 24.75 0.00 0.00 0.00</pose>", model_text
                )
                self.assertIn("<size>24.00 30.00 14.50</size>", model_text)
                self.assertIn("<size>24.00 30.00 6.50</size>", model_text)
        self.assertNotIn("known_passage_test_gate", passage_text)
        self.assertNotIn("known_passage_test_gate", sdf_text)

    def test_known_passage_openings_cover_wide_altitude_range(self) -> None:
        passage_text = read("drone_city_nav/worlds/known_passages.passages3d")

        opening_values = sorted(
            (float(center_z), float(width), float(height))
            for center_z, width, height in re.findall(
                r"^opening\s+\S+\s+\S+\s+[-+0-9.]+\s+[-+0-9.]+"
                r"\s+([-+0-9.]+)\s+[-+0-9.]+\s+[-+0-9.]+"
                r"\s+([-+0-9.]+)\s+([-+0-9.]+)\s",
                passage_text,
                re.M,
            )
        )

        self.assertEqual(
            opening_values,
            [
                (5.0, 7.0, 7.0),
                (10.0, 7.0, 7.0),
                (15.0, 7.0, 7.0),
                (20.0, 7.0, 7.0),
                (25.0, 7.0, 7.0),
            ],
        )

    def test_lidar_hit_depth_preprocessing_is_removed(self) -> None:
        checked_paths = [
            "drone_city_nav/config/urban_mvp.yaml",
            "drone_city_nav/include/drone_city_nav/obstacle_memory.hpp",
            "drone_city_nav/include/drone_city_nav/current_lidar_overlay.hpp",
            "drone_city_nav/include/drone_city_nav/lidar_projection.hpp",
            "drone_city_nav/include/drone_city_nav/lidar_snapshot_writer.hpp",
            "drone_city_nav/src/obstacle_memory.cpp",
            "drone_city_nav/src/current_lidar_overlay.cpp",
            "drone_city_nav/src/lidar_projection.cpp",
            "drone_city_nav/src/lidar_snapshot_writer.cpp",
            "drone_city_nav/src/planner_node_config.cpp",
        ]

        for relative_path in checked_paths:
            with self.subTest(relative_path=relative_path):
                text = read(relative_path)
                self.assertNotIn("sensor_hit_" "depth", text)
                self.assertNotIn("hit_obstacle_" "depth", text)
                self.assertNotIn("current_lidar_obstacle_" "depth", text)
                self.assertNotIn("depth_" "endpoint", text)
                self.assertNotIn("depth_" "end", text)


if __name__ == "__main__":
    unittest.main()
