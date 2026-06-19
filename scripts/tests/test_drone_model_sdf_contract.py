#!/usr/bin/env python3
"""Static contract tests for local Gazebo SDF models."""

from __future__ import annotations

import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
WRAPPER_SDF = REPO_ROOT / "drone_city_nav/models/x500_lidar_2d/model.sdf"
LIDAR_SDF = REPO_ROOT / "drone_city_nav/models/lidar_2d_v2/model.sdf"


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


if __name__ == "__main__":
    unittest.main()
