#!/usr/bin/env python3
"""Static contract tests for local Gazebo SDF models."""

from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
import json
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
WRAPPER_SDF = REPO_ROOT / "drone_city_nav/models/x500_lidar_2d/model.sdf"
LIDAR_SDF = REPO_ROOT / "drone_city_nav/models/lidar_2d_v2/model.sdf"
NAV_CONFIG = REPO_ROOT / "drone_city_nav/config/urban_mvp.yaml"
WORLD_SDF = REPO_ROOT / "drone_city_nav/worlds/generated_city.sdf"
WORLD_SPEC = REPO_ROOT / "drone_city_nav/worlds/canonical_city.world3d.json"

GZ_VISIBILITY_ALL = 0x0FFFFFFF
STATIC_CHANNEL_MASS_VISIBILITY_FLAG = 0x08000000
NO_STATIC_OCCLUDER_VISIBILITY_FLAG = 0x04000000
LIDAR_VISIBILITY_MASK = GZ_VISIBILITY_ALL & ~(
    STATIC_CHANNEL_MASS_VISIBILITY_FLAG | NO_STATIC_OCCLUDER_VISIBILITY_FLAG
)


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

    def test_static_lidar_default_excludes_connector_masses_and_occluders(self) -> None:
        lidar_root = parse_sdf(LIDAR_SDF)
        world_root = parse_sdf(WORLD_SDF)
        sensor = next(
            element
            for element in lidar_root.iter("sensor")
            if element.attrib.get("name") == "lidar_2d_v2"
        )
        lidar_mask = int(sensor.findtext("ray/visibility_mask", ""))

        self.assertEqual(LIDAR_VISIBILITY_MASK, lidar_mask)
        self.assertEqual(0, lidar_mask & STATIC_CHANNEL_MASS_VISIBILITY_FLAG)

        flagged_visuals = [
            visual
            for visual in world_root.iter("visual")
            if int(visual.findtext("visibility_flags", "0"))
            & STATIC_CHANNEL_MASS_VISIBILITY_FLAG
        ]
        self.assertGreater(len(flagged_visuals), 0)
        for visual in flagged_visuals:
            self.assertEqual(
                STATIC_CHANNEL_MASS_VISIBILITY_FLAG,
                int(visual.findtext("visibility_flags", "")),
            )
            self.assertEqual(0, lidar_mask & STATIC_CHANNEL_MASS_VISIBILITY_FLAG)

    def test_all_channels_have_collisionless_no_static_lidar_occluders(self) -> None:
        world_root = parse_sdf(WORLD_SDF)
        spec = json.loads(WORLD_SPEC.read_text(encoding="utf-8"))
        expected_names = {
            f"{channel['id']}_no_static_occluder" for channel in spec["channels"]
        }
        occluder_models = {
            model.attrib["name"]: model
            for model in world_root.iter("model")
            if model.attrib.get("name", "").endswith("_no_static_occluder")
        }

        self.assertEqual(expected_names, set(occluder_models))
        for channel in spec["channels"]:
            model_name = f"{channel['id']}_no_static_occluder"
            with self.subTest(channel=channel["id"]):
                occluders = [
                    link
                    for link in occluder_models[model_name].findall("link")
                    if link.attrib.get("name") == "no_static_lidar_occluder"
                ]
                self.assertEqual(1, len(occluders))
                occluder = occluders[0]
                visual = occluder.find("visual")

                self.assertIsNotNone(visual)
                self.assertIsNone(occluder.find("collision"))
                self.assertEqual("0.999", visual.findtext("transparency"))
                self.assertEqual(
                    NO_STATIC_OCCLUDER_VISIBILITY_FLAG,
                    int(visual.findtext("visibility_flags", "")),
                )
                self.assertEqual(
                    0,
                    LIDAR_VISIBILITY_MASK
                    & NO_STATIC_OCCLUDER_VISIBILITY_FLAG,
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
