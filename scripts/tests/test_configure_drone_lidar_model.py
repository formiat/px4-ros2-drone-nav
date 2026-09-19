#!/usr/bin/env python3
"""Tests for runtime X500 lidar wrapper materialization."""

from __future__ import annotations

import shutil
import sys
import tempfile
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


SCRIPTS_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = SCRIPTS_DIR.parent
sys.path.insert(0, str(SCRIPTS_DIR))

import configure_drone_lidar_model as lidar_model  # noqa: E402


SOURCE_MODEL = REPO_ROOT / "drone_city_nav/models/x500_lidar_2d"


class ConfigureDroneLidarModelTest(unittest.TestCase):
    def test_materializes_3d_wrapper_with_consistent_model_names(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            destination = Path(temp_dir) / "x500_lidar_3d"
            shutil.copytree(SOURCE_MODEL, destination)

            sensor_model = lidar_model.configure_model(
                destination, "x500_lidar_3d", "3d"
            )

            sdf_root = ET.parse(destination / "model.sdf").getroot()
            config_root = ET.parse(destination / "model.config").getroot()
            self.assertEqual("lidar_3d_v1", sensor_model)
            self.assertEqual("x500_lidar_3d", sdf_root.find("model").attrib["name"])
            self.assertEqual("x500_lidar_3d", config_root.findtext("name"))
            self.assertIn(
                "model://lidar_3d_v1",
                [element.text for element in sdf_root.iter("uri")],
            )

    def test_camera_profile_adds_the_stereo_and_time_of_flight_set(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            destination = Path(temp_dir) / "x500_lidar_3d"
            shutil.copytree(SOURCE_MODEL, destination)

            lidar_model.configure_model(
                destination, "x500_lidar_3d", "3d", "stereo_tof"
            )

            model = ET.parse(destination / "model.sdf").getroot().find("model")
            self.assertIn(
                "model://stereo_tof_v1", [element.text for element in model.iter("uri")]
            )
            joint = model.find("joint[@name='CameraProfileJoint']")
            self.assertEqual("base_link", joint.findtext("parent"))
            self.assertEqual("stereo_tof_link", joint.findtext("child"))

    def test_camera_set_keeps_the_evaluation_depth_out_of_the_pair(self) -> None:
        sensors = {
            sensor.attrib["name"]: sensor.attrib["type"]
            for sensor in ET.parse(
                REPO_ROOT / "drone_city_nav/models/stereo_tof_v1/model.sdf"
            ).getroot().iter("sensor")
        }
        self.assertEqual(
            {
                "stereo_left": "camera",
                "stereo_right": "camera",
                "evaluation_depth_left": "depth_camera",
                "tof_up": "gpu_lidar",
                "tof_down": "gpu_lidar",
            },
            sensors,
        )

    def test_rejects_non_materialized_none_profile(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            destination = Path(temp_dir) / "x500_lidar_none"
            shutil.copytree(SOURCE_MODEL, destination)

            with self.assertRaisesRegex(ValueError, "unsupported"):
                lidar_model.configure_model(destination, "x500_lidar_none", "none")


if __name__ == "__main__":
    unittest.main()
