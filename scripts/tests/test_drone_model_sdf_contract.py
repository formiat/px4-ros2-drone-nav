#!/usr/bin/env python3
"""Static contract tests for local Gazebo SDF models."""

from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from math import hypot
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
WRAPPER_SDF = REPO_ROOT / "drone_city_nav/models/x500_lidar_2d/model.sdf"
LIDAR_SDF = REPO_ROOT / "drone_city_nav/models/lidar_2d_v2/model.sdf"
NAV_CONFIG = REPO_ROOT / "drone_city_nav/config/urban_mvp.yaml"
WORLD_SDF = REPO_ROOT / "drone_city_nav/worlds/generated_city.sdf"
PASSAGES_3D = REPO_ROOT / "drone_city_nav/worlds/known_passages.passages3d"

GZ_VISIBILITY_ALL = 0x0FFFFFFF
PASSAGE_MASS_VISIBILITY_FLAG = 0x08000000
LIDAR_VISIBILITY_MASK = GZ_VISIBILITY_ALL & ~PASSAGE_MASS_VISIBILITY_FLAG


def parse_sdf(path: Path) -> ET.Element:
    return ET.parse(path).getroot()


def element_names(root: ET.Element, tag: str) -> set[str]:
    return {
        element.attrib["name"]
        for element in root.iter(tag)
        if "name" in element.attrib
    }


class DroneModelSdfContractTest(unittest.TestCase):
    def test_wrapper_uses_upstream_base_uri_and_local_lidar_uri(self) -> None:
        root = parse_sdf(WRAPPER_SDF)
        uris = [element.text for element in root.iter("uri")]

        self.assertEqual(["x500", "model://lidar_2d_v2"], uris)

    def test_wrapper_contains_visibility_marker_link_and_joint(self) -> None:
        root = parse_sdf(WRAPPER_SDF)

        self.assertIn("visibility_marker_link", element_names(root, "link"))
        self.assertIn("VisibilityMarkerJoint", element_names(root, "joint"))

    def test_wrapper_owns_yellow_visibility_visuals(self) -> None:
        root = parse_sdf(WRAPPER_SDF)
        visuals = element_names(root, "visual")

        self.assertIn("yellow_body_plate", visuals)
        self.assertIn("yellow_arm_x", visuals)
        self.assertIn("yellow_arm_y", visuals)
        self.assertIn("yellow_ground_projection_beam", visuals)
        self.assertIn("yellow_ground_projection_disc", visuals)

    def test_wrapper_loads_physical_contact_system(self) -> None:
        root = parse_sdf(WRAPPER_SDF)
        plugin = next(
            element
            for element in root.iter("plugin")
            if element.attrib.get("name") == "drone_city_nav::DroneContactSystem"
        )

        self.assertEqual("libdrone_contact_system.so", plugin.attrib["filename"])
        self.assertEqual(
            "/drone_city_nav/drone_contacts", plugin.findtext("topic")
        )

    def test_lidar_model_keeps_sensor_and_no_drone_visibility_visuals(self) -> None:
        root = parse_sdf(LIDAR_SDF)
        visuals = element_names(root, "visual")
        sensors = {
            element.attrib.get("type")
            for element in root.iter("sensor")
            if element.attrib.get("name") == "lidar_2d_v2"
        }

        self.assertIn("gpu_lidar", sensors)
        self.assertFalse(
            any(name.startswith("yellow_") for name in visuals),
            f"lidar model must not own drone visibility visuals: {sorted(visuals)}",
        )

    def test_gpu_lidar_ignores_only_physical_connector_mass_visuals(self) -> None:
        lidar_root = parse_sdf(LIDAR_SDF)
        world_root = parse_sdf(WORLD_SDF)
        sensor = next(
            element
            for element in lidar_root.iter("sensor")
            if element.attrib.get("name") == "lidar_2d_v2"
        )
        lidar_mask = int(sensor.findtext("ray/visibility_mask", ""))

        self.assertEqual(LIDAR_VISIBILITY_MASK, lidar_mask)
        self.assertEqual(0, lidar_mask & PASSAGE_MASS_VISIBILITY_FLAG)

        flagged_visuals = [
            visual
            for visual in world_root.iter("visual")
            if int(visual.findtext("visibility_flags", "0"))
            & PASSAGE_MASS_VISIBILITY_FLAG
        ]
        self.assertEqual(12, len(flagged_visuals))

        connector_models = [
            model
            for model in world_root.iter("model")
            if model.attrib.get("name", "").startswith(
                "physical_building_connector_"
            )
        ]
        self.assertEqual(6, len(connector_models))
        self.assertEqual(
            {
                "physical_building_connector_04_12",
                "physical_building_connector_06_14",
                "physical_building_connector_11_19",
                "physical_building_connector_20_21",
                "physical_building_connector_22_23",
                "physical_building_connector_22_30",
            },
            {model.attrib["name"] for model in connector_models},
        )

        for model in connector_models:
            for mass_name in ("lower_mass", "upper_mass"):
                with self.subTest(
                    connector=model.attrib["name"],
                    mass=mass_name,
                ):
                    link = next(
                        link
                        for link in model.findall("link")
                        if link.attrib.get("name") == mass_name
                    )
                    visual = link.find("visual")
                    collision = link.find("collision")

                    self.assertIsNotNone(visual)
                    self.assertIsNotNone(collision)
                    visual_flags = int(visual.findtext("visibility_flags", ""))
                    self.assertEqual(PASSAGE_MASS_VISIBILITY_FLAG, visual_flags)
                    self.assertEqual(0, lidar_mask & visual_flags)

    def test_physical_connectors_match_known_passage_annotations(self) -> None:
        world_root = parse_sdf(WORLD_SDF)
        connector_models = {
            model.attrib["name"]: model
            for model in world_root.iter("model")
            if model.attrib.get("name", "").startswith(
                "physical_building_connector_"
            )
        }
        structures = {}
        openings = {}
        for raw_line in PASSAGES_3D.read_text(encoding="utf-8").splitlines():
            line = raw_line.partition("#")[0].strip()
            if not line:
                continue
            fields = line.split()
            if fields[0] == "structure":
                structures[fields[1]] = tuple(float(value) for value in fields[2:])
            elif fields[0] == "opening":
                openings[fields[1]] = (
                    fields[2],
                    tuple(float(value) for value in fields[3:]),
                )

        self.assertEqual(set(connector_models), set(structures))
        self.assertEqual(set(connector_models), set(openings))
        for connector, model in connector_models.items():
            with self.subTest(connector=connector):
                pose = [float(value) for value in model.findtext("pose", "").split()]
                center_x_m, center_y_m, size_x_m, size_y_m = structures[connector][
                    :4
                ]
                opening_id, opening = openings[connector]
                normal_x, normal_y = opening[3:5]
                opening_width_m = opening[5]
                opening_depth_m = opening[7]
                lower_mass = next(
                    link
                    for link in model.findall("link")
                    if link.attrib.get("name") == "lower_mass"
                )
                sdf_size = [
                    float(value)
                    for value in lower_mass.findtext(
                        "collision/geometry/box/size", ""
                    ).split()
                ]
                map_size_x_m = sdf_size[1]
                map_size_y_m = sdf_size[0]
                normal_length = hypot(normal_x, normal_y)
                self.assertGreater(normal_length, 0.0)
                lateral_x = -normal_y / normal_length
                lateral_y = normal_x / normal_length
                normal_x /= normal_length
                normal_y /= normal_length
                footprint_depth_m = (
                    abs(normal_x) * size_x_m + abs(normal_y) * size_y_m
                )
                footprint_width_m = (
                    abs(lateral_x) * size_x_m + abs(lateral_y) * size_y_m
                )

                self.assertAlmostEqual(center_y_m - 225.0, pose[0])
                self.assertAlmostEqual(center_x_m - 135.0, pose[1])
                self.assertAlmostEqual(map_size_x_m, size_x_m)
                self.assertAlmostEqual(map_size_y_m, size_y_m)
                self.assertAlmostEqual(1.0, normal_length)
                self.assertAlmostEqual(opening_depth_m, footprint_depth_m)
                self.assertAlmostEqual(opening_width_m, footprint_width_m)
                self.assertEqual(
                    f"{connector.removeprefix('physical_building_')}_opening",
                    opening_id,
                )

    def test_lidar_sensor_pose_matches_configured_full_extrinsic(self) -> None:
        wrapper_root = parse_sdf(WRAPPER_SDF)
        lidar_root = parse_sdf(LIDAR_SDF)
        include = next(
            element
            for element in wrapper_root.iter("include")
            if element.findtext("uri") == "model://lidar_2d_v2"
        )
        sensor = next(
            element
            for element in lidar_root.iter("sensor")
            if element.attrib.get("name") == "lidar_2d_v2"
        )
        include_pose = [float(value) for value in include.findtext("pose", "").split()]
        sensor_pose = [float(value) for value in sensor.findtext("pose", "").split()]

        self.assertEqual([0.12, 0.0, 0.26], include_pose[:3])
        self.assertEqual([0.0, 0.0, 0.055], sensor_pose[:3])
        config_text = NAV_CONFIG.read_text(encoding="utf-8")
        self.assertEqual(
            2,
            config_text.count(
                "lidar_extrinsic_translation_body_frd_m: [0.12, 0.0, -0.315]"
            ),
        )
        self.assertEqual(
            2,
            config_text.count(
                "lidar_extrinsic_quaternion_lidar_flu_to_body_frd: "
                "[0.0, 1.0, 0.0, 0.0]"
            ),
        )


if __name__ == "__main__":
    unittest.main()
