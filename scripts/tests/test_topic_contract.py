#!/usr/bin/env python3
"""Static tests for raw/prohibited/debug obstacle topic contracts."""

from __future__ import annotations

import re
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]

BUILDING_DIFFUSE = "0.48 0.50 0.53 1"
PASSAGE_LOWER_DIFFUSE = "0.56 0.49 0.43 1"
PASSAGE_UPPER_DIFFUSE = "0.43 0.47 0.55 1"

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
    def test_docs_do_not_describe_removed_passage_sensor_policy(self) -> None:
        forbidden_terms = (
            "passage_traversal_sensor_policy",
            "passage_traversal_activation_margin",
            "passage_traversal_lookahead_margin",
            "passage_traversal_opening_corridor",
            "passage_traversal_expected_wall_margin",
            "passage_sensor_policy",
            "ignore_expected_walls",
            "emergency_blocker",
            "expected-wall",
        )

        for path in sorted((REPO_ROOT / "docs").glob("*.md")):
            text = path.read_text(encoding="utf-8")
            for term in forbidden_terms:
                with self.subTest(path=path, term=term):
                    self.assertNotIn(term, text)

    def test_known_static_lidar_classifier_config_replaces_proximity_policy(
        self,
    ) -> None:
        yaml_text = read("drone_city_nav/config/urban_mvp.yaml")
        closer_tolerances = re.findall(
            r"^\s+known_static_lidar_hit_closer_range_tolerance_m:\s*([0-9.]+)\s*$",
            yaml_text,
            re.M,
        )
        farther_tolerances = re.findall(
            r"^\s+known_static_lidar_hit_farther_range_tolerance_m:\s*([0-9.]+)\s*$",
            yaml_text,
            re.M,
        )
        ground_enabled = re.findall(
            r"^\s+ground_lidar_rejection_enabled:\s*(true|false)\s*$",
            yaml_text,
            re.M,
        )
        ground_altitudes = re.findall(
            r"^\s+ground_lidar_altitude_m:\s*([-0-9.]+)\s*$", yaml_text, re.M
        )
        ground_closer = re.findall(
            r"^\s+ground_lidar_closer_range_tolerance_m:\s*([0-9.]+)\s*$",
            yaml_text,
            re.M,
        )
        ground_farther = re.findall(
            r"^\s+ground_lidar_farther_range_tolerance_m:\s*([0-9.]+)\s*$",
            yaml_text,
            re.M,
        )

        self.assertEqual(closer_tolerances, ["0.5", "0.5"])
        self.assertEqual(farther_tolerances, ["1.5", "1.5"])
        self.assertEqual(ground_enabled, ["true", "true"])
        self.assertEqual(ground_altitudes, ["0.05", "0.05"])
        self.assertEqual(ground_closer, ["0.5", "0.5"])
        self.assertEqual(ground_farther, ["1.5", "1.5"])
        self.assertNotIn("lidar_mapping_tilt_cutoff", yaml_text)
        self.assertNotIn("known_static_lidar_hit_range_tolerance_m", yaml_text)
        self.assertNotIn("passage_traversal_sensor_policy", yaml_text)
        self.assertNotIn("passage_traversal_activation_margin", yaml_text)
        self.assertNotIn("passage_traversal_lookahead_margin", yaml_text)
        self.assertNotIn("passage_traversal_expected_wall_margin", yaml_text)

    def test_ground_rejection_has_one_shared_formula_and_no_tilt_cutoff(self) -> None:
        source_paths = sorted((REPO_ROOT / "drone_city_nav" / "src").glob("*.cpp"))
        source_text = "\n".join(path.read_text(encoding="utf-8") for path in source_paths)

        self.assertEqual(
            source_text.count("(config.ground_altitude_m - origin.z) / direction.z"),
            1,
        )
        self.assertNotRegex(
            source_text,
            r"tilt.{0,40}(suspend|disable|ignore)|"
            r"(suspend|disable|ignore).{0,40}tilt",
        )

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
        self.assertIn("/drone_city_nav/obstacle_memory_provenance", text)
        self.assertIn("/drone_city_nav/obstacle_memory_snapshot", text)
        self.assertIn("/drone_city_nav/prohibited_grid", text)
        self.assertIn("/drone_city_nav/raw_memory_obstacle_points", text)
        self.assertIn("/drone_city_nav/prohibited_obstacle_points", text)
        self.assertIn("/drone_city_nav/static_building_markers", text)
        self.assertIn("/drone_city_nav/known_passage_markers", text)
        self.assertNotIn("/drone_city_nav/obstacle_memory_inflated_grid", text)
        self.assertNotIn("/drone_city_nav/occupancy_grid", text)

        yaml_text = read("drone_city_nav/config/urban_mvp.yaml")
        self.assertEqual(
            yaml_text.count(
                "obstacle_memory_provenance_topic: "
                "/drone_city_nav/obstacle_memory_provenance"
            ),
            1,
        )
        self.assertEqual(
            yaml_text.count(
                "obstacle_memory_snapshot_topic: "
                "/drone_city_nav/obstacle_memory_snapshot"
            ),
            2,
        )

        planner_lifecycle = read("drone_city_nav/src/planner_node_lifecycle.cpp")
        self.assertIn("memory_snapshot_sub_", planner_lifecycle)
        self.assertIn("ObstacleMemorySnapshot", planner_lifecycle)
        self.assertNotIn("memory_provenance_sub_", planner_lifecycle)
        self.assertNotIn("memory_grid_sub_", planner_lifecycle)

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
        top_down_rviz_text = read("drone_city_nav/rviz/city_nav_debug_top_down.rviz")

        self.assertIn("gazebo_aligned_map_tf", launch_text)
        self.assertIn("static_transform_publisher", launch_text)
        self.assertIn('"gazebo_map"', launch_text)
        self.assertIn('"map"', launch_text)
        self.assertIn("This transform is intentional", launch_text)
        self.assertIn("Fixed Frame: gazebo_map", rviz_text)
        self.assertIn("Reference Frame: gazebo_map", rviz_text)
        self.assertIn("Target Frame: drone_follow", rviz_text)
        self.assertIn("rviz_drone_follow_tf_enabled", launch_text)
        self.assertIn("Target Frame: gazebo_map", top_down_rviz_text)

    def test_rviz_follow_frame_is_visualization_only_and_opt_out_by_env(
        self,
    ) -> None:
        launch_text = read("drone_city_nav/launch/city_nav.launch.py")
        runner_text = read("scripts/run_drone_nav_sim.sh")
        offboard_text = read("drone_city_nav/src/px4_offboard_node_trajectory.cpp")
        rviz_text = read("drone_city_nav/rviz/city_nav_debug.rviz")

        self.assertIn("drone_follow", rviz_text)
        self.assertIn("ENABLE_RVIZ_FOLLOW_CAMERA:-true", runner_text)
        self.assertIn("city_nav_debug_top_down.rviz", runner_text)
        self.assertIn("rviz_drone_follow_tf_enabled", launch_text)
        self.assertIn("visualization-only frame", offboard_text)

    def test_known_passage_annotations_are_not_legacy_or_static_map_obstacles(
        self,
    ) -> None:
        passage_text = read("drone_city_nav/worlds/known_passages.passages3d")
        sdf_text = read("drone_city_nav/worlds/generated_city.sdf")
        static_map_text = read("drone_city_nav/worlds/generated_city.map2d")

        self.assertNotIn("building_with_passage", passage_text)
        self.assertNotIn("known_passage_test_gate", passage_text)
        self.assertNotIn("known_passage_test_gate", sdf_text)

        for structure_id in re.findall(r"^structure\s+(\S+)\s", passage_text, re.M):
            with self.subTest(structure_id=structure_id):
                self.assertNotIn(structure_id, static_map_text)

    def test_known_passage_annotations_match_physical_connectors(self) -> None:
        passage_text = read("drone_city_nav/worlds/known_passages.passages3d")
        sdf_text = read("drone_city_nav/worlds/generated_city.sdf")

        connector_ids = re.findall(
            r'<model name="(physical_building_connector_\d+_\d+)">', sdf_text
        )
        self.assertEqual(
            connector_ids,
            [
                "physical_building_connector_22_23",
                "physical_building_connector_04_12",
                "physical_building_connector_06_14",
            ],
        )

        structure_values = {
            structure_id: tuple(float(value) for value in values)
            for structure_id, *values in re.findall(
                r"^structure\s+(\S+)\s+([-+0-9.]+)\s+([-+0-9.]+)"
                r"\s+([-+0-9.]+)\s+([-+0-9.]+)\s+([-+0-9.]+)"
                r"\s+([-+0-9.]+)$",
                passage_text,
                re.M,
            )
        }
        self.assertEqual(set(structure_values), set(connector_ids))

        opening_values = {
            structure_id: (opening_id, tuple(float(value) for value in values))
            for structure_id, opening_id, *values in re.findall(
                r"^opening\s+(\S+)\s+(\S+)\s+([-+0-9.]+)\s+([-+0-9.]+)"
                r"\s+([-+0-9.]+)\s+([-+0-9.]+)\s+([-+0-9.]+)"
                r"\s+([-+0-9.]+)\s+([-+0-9.]+)\s+([-+0-9.]+)"
                r"\s+([-+0-9.]+)\s+([-+0-9.]+)\s+([-+0-9.]+)"
                r"\s+([-+0-9.]+)$",
                passage_text,
                re.M,
            )
        }
        self.assertEqual(set(opening_values), set(connector_ids))

        expected_vertical_geometry = {
            "physical_building_connector_22_23": {
                "center_z": 5.0,
                "min_z": 1.5,
                "max_z": 8.5,
                "lower_pose_z": 0.75,
                "lower_size_z": 1.5,
                "upper_pose_z": 20.25,
                "upper_size_z": 23.5,
                "sdf_size_x": 30.0,
                "sdf_size_y": 24.0,
                "structure_size_x": 24.0,
                "structure_size_y": 30.0,
                "normal_x": 1.0,
                "normal_y": 0.0,
            },
            "physical_building_connector_04_12": {
                "center_z": 15.0,
                "min_z": 11.5,
                "max_z": 18.5,
                "lower_pose_z": 5.75,
                "lower_size_z": 11.5,
                "upper_pose_z": 25.25,
                "upper_size_z": 13.5,
                "sdf_size_x": 24.0,
                "sdf_size_y": 30.0,
                "structure_size_x": 30.0,
                "structure_size_y": 24.0,
                "normal_x": 0.0,
                "normal_y": 1.0,
            },
            "physical_building_connector_06_14": {
                "center_z": 25.0,
                "min_z": 21.5,
                "max_z": 28.5,
                "lower_pose_z": 10.75,
                "lower_size_z": 21.5,
                "upper_pose_z": 30.25,
                "upper_size_z": 3.5,
                "sdf_size_x": 24.0,
                "sdf_size_y": 30.0,
                "structure_size_x": 30.0,
                "structure_size_y": 24.0,
                "normal_x": 0.0,
                "normal_y": 1.0,
            },
        }

        for connector_id in connector_ids:
            with self.subTest(connector_id=connector_id):
                model_match = re.search(
                    rf'<model name="{connector_id}">(.*?)</model>', sdf_text, re.S
                )
                self.assertIsNotNone(model_match)
                model_text = model_match.group(1)
                self.assertIn('<link name="lower_mass">', model_text)
                self.assertIn('<link name="upper_mass">', model_text)
                expected_vertical = expected_vertical_geometry[connector_id]
                self.assertIn(
                    (
                        f"<pose>0.00 0.00 {expected_vertical['lower_pose_z']:.2f} "
                        "0.00 0.00 0.00</pose>"
                    ),
                    model_text,
                )
                self.assertIn(
                    (
                        f"<pose>0.00 0.00 {expected_vertical['upper_pose_z']:.2f} "
                        "0.00 0.00 0.00</pose>"
                    ),
                    model_text,
                )
                self.assertIn(
                    (
                        f"<size>{expected_vertical['sdf_size_x']:.2f} "
                        f"{expected_vertical['sdf_size_y']:.2f} "
                        f"{expected_vertical['lower_size_z']:.2f}</size>"
                    ),
                    model_text,
                )
                self.assertIn(
                    (
                        f"<size>{expected_vertical['sdf_size_x']:.2f} "
                        f"{expected_vertical['sdf_size_y']:.2f} "
                        f"{expected_vertical['upper_size_z']:.2f}</size>"
                    ),
                    model_text,
                )
                model_pose = re.search(r"<pose>(.*?)</pose>", model_text)
                self.assertIsNotNone(model_pose)
                gazebo_x, gazebo_y, *_ = (
                    float(value) for value in model_pose.group(1).split()
                )
                # The source SDF is written in Gazebo visual coordinates, while
                # the planner/static map uses the legacy swapped X/Y convention.
                # Keep known passage annotations in planner map coordinates so
                # RViz overlays and trajectory planning line up with the real
                # connector buildings after the gazebo_aligned_map transform.
                map_x = gazebo_y + 135.0
                map_y = gazebo_x + 225.0

                self.assertEqual(
                    structure_values[connector_id],
                    (
                        map_x,
                        map_y,
                        expected_vertical["structure_size_x"],
                        expected_vertical["structure_size_y"],
                        0.0,
                        32.0,
                    ),
                )
                opening_id, opening = opening_values[connector_id]
                self.assertTrue(opening_id.startswith("connector_"))
                self.assertEqual(
                    opening,
                    (
                        map_x,
                        map_y,
                        expected_vertical["center_z"],
                        expected_vertical["normal_x"],
                        expected_vertical["normal_y"],
                        30.0,
                        7.0,
                        24.0,
                        expected_vertical["min_z"],
                        expected_vertical["max_z"],
                        18.0,
                        18.0,
                    ),
                )

    def test_city_building_passage_geometry_and_colors_are_normalized(self) -> None:
        sdf_text = read("drone_city_nav/worlds/generated_city.sdf")
        static_map_text = read("drone_city_nav/worlds/generated_city.map2d")
        config_text = read("drone_city_nav/config/urban_mvp.yaml")

        building_models = re.findall(
            r'<model name="(manhattan_building_\d+)">(.*?)</model>', sdf_text, re.S
        )
        self.assertEqual(len(building_models), 40)
        for building_id, model_text in building_models:
            with self.subTest(building_id=building_id):
                pose_match = re.search(r"<pose>(.*?)</pose>", model_text)
                self.assertIsNotNone(pose_match)
                self.assertEqual(float(pose_match.group(1).split()[2]), 16.0)
                self.assertEqual(
                    re.findall(r"<size>(.*?)</size>", model_text),
                    ["24.00 24.00 32.00", "24.00 24.00 32.00"],
                )
                self.assertEqual(
                    re.findall(r"<diffuse>(.*?)</diffuse>", model_text),
                    [BUILDING_DIFFUSE],
                )

        static_building_heights = [
            float(height)
            for height in re.findall(
                r"^rect\s+building_\d+\s+(?:[-+0-9.]+\s+){4}([-+0-9.]+)$",
                static_map_text,
                re.M,
            )
        ]
        self.assertEqual(static_building_heights, [32.0] * 40)
        self.assertIn("uniform_building_height_m: 32.0", config_text)

        connector_models = re.findall(
            r'<model name="(physical_building_connector_\d+_\d+)">(.*?)</model>',
            sdf_text,
            re.S,
        )
        self.assertEqual(len(connector_models), 3)
        for connector_id, model_text in connector_models:
            with self.subTest(connector_id=connector_id):
                lower_match = re.search(
                    r'<link name="lower_mass">(.*?)</link>', model_text, re.S
                )
                upper_match = re.search(
                    r'<link name="upper_mass">(.*?)</link>', model_text, re.S
                )
                self.assertIsNotNone(lower_match)
                self.assertIsNotNone(upper_match)
                self.assertEqual(
                    re.findall(r"<diffuse>(.*?)</diffuse>", lower_match.group(1)),
                    [PASSAGE_LOWER_DIFFUSE],
                )
                self.assertEqual(
                    re.findall(r"<diffuse>(.*?)</diffuse>", upper_match.group(1)),
                    [PASSAGE_UPPER_DIFFUSE],
                )

                upper_pose_match = re.search(
                    r"<pose>0\.00 0\.00 ([-+0-9.]+) ", upper_match.group(1)
                )
                upper_size_match = re.search(
                    r"<size>[-+0-9.]+ [-+0-9.]+ ([-+0-9.]+)</size>",
                    upper_match.group(1),
                )
                self.assertIsNotNone(upper_pose_match)
                self.assertIsNotNone(upper_size_match)
                self.assertEqual(
                    float(upper_pose_match.group(1))
                    + 0.5 * float(upper_size_match.group(1)),
                    32.0,
                )

    def test_known_passage_openings_cover_wide_altitude_range(self) -> None:
        passage_text = read("drone_city_nav/worlds/known_passages.passages3d")

        opening_values = sorted(
            (float(center_z), float(width), float(height), float(min_z), float(max_z))
            for center_z, width, height, min_z, max_z in re.findall(
                r"^opening\s+\S+\s+\S+\s+[-+0-9.]+\s+[-+0-9.]+"
                r"\s+([-+0-9.]+)\s+[-+0-9.]+\s+[-+0-9.]+"
                r"\s+([-+0-9.]+)\s+([-+0-9.]+)\s+[-+0-9.]+"
                r"\s+([-+0-9.]+)\s+([-+0-9.]+)\s",
                passage_text,
                re.M,
            )
        )

        self.assertEqual(
            opening_values,
            [
                (5.0, 30.0, 7.0, 1.5, 8.5),
                (15.0, 30.0, 7.0, 11.5, 18.5),
                (25.0, 30.0, 7.0, 21.5, 28.5),
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
