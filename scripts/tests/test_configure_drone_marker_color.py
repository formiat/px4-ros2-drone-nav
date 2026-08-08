#!/usr/bin/env python3
"""Tests for runtime role-specific Gazebo drone markers."""

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

import configure_drone_marker_color as marker_color  # noqa: E402


SOURCE_MODEL_DIRECTORY = REPO_ROOT / "drone_city_nav/models/x500_lidar_2d"
EVADER_MODEL_NAME = "x500_lidar_2d_evader"


class ConfigureDroneMarkerColorTest(unittest.TestCase):
    def configure_copy(self) -> tuple[int, ET.Element, ET.Element]:
        temporary_directory = tempfile.TemporaryDirectory()
        self.addCleanup(temporary_directory.cleanup)
        destination = Path(temporary_directory.name) / EVADER_MODEL_NAME
        shutil.copytree(SOURCE_MODEL_DIRECTORY, destination)

        configured_count = marker_color.configure_model(
            destination, EVADER_MODEL_NAME
        )
        sdf_root = ET.parse(destination / "model.sdf").getroot()
        config_root = ET.parse(destination / "model.config").getroot()
        return configured_count, sdf_root, config_root

    def test_configures_a_red_evader_variant(self) -> None:
        configured_count, sdf_root, config_root = self.configure_copy()
        model = sdf_root.find("model")

        self.assertIsNotNone(model)
        assert model is not None
        self.assertEqual(EVADER_MODEL_NAME, model.attrib["name"])
        self.assertEqual(EVADER_MODEL_NAME, config_root.findtext("name"))

        marker_link = model.find("./link[@name='visibility_marker_link']")
        self.assertIsNotNone(marker_link)
        assert marker_link is not None
        visuals = marker_link.findall("visual")
        self.assertEqual(len(visuals), configured_count)
        self.assertGreater(configured_count, 0)
        self.assertFalse(
            any(
                visual.attrib["name"].startswith(
                    marker_color.SOURCE_MARKER_PREFIX
                )
                for visual in visuals
            )
        )
        self.assertTrue(
            all(
                visual.attrib["name"].startswith(
                    marker_color.TARGET_MARKER_PREFIX
                )
                for visual in visuals
            )
        )

        for visual in visuals:
            with self.subTest(visual=visual.attrib["name"]):
                ambient = [
                    float(value)
                    for value in visual.findtext("material/ambient", "").split()
                ]
                diffuse = [
                    float(value)
                    for value in visual.findtext("material/diffuse", "").split()
                ]
                emissive = [
                    float(value)
                    for value in visual.findtext("material/emissive", "").split()
                ]
                self.assertEqual(list(marker_color.RED_SURFACE_RGB), ambient[:3])
                self.assertEqual(list(marker_color.RED_SURFACE_RGB), diffuse[:3])
                self.assertGreater(emissive[0], 0.0)
                self.assertEqual([0.0, 0.0], emissive[1:3])

    def test_rejects_a_model_without_canonical_marker_visuals(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            destination = Path(temp_dir) / EVADER_MODEL_NAME
            shutil.copytree(SOURCE_MODEL_DIRECTORY, destination)
            sdf_path = destination / "model.sdf"
            sdf_text = sdf_path.read_text(encoding="utf-8").replace(
                'name="yellow_', 'name="unrelated_'
            )
            sdf_path.write_text(sdf_text, encoding="utf-8")

            with self.assertRaisesRegex(RuntimeError, "no yellow_ marker visuals"):
                marker_color.configure_model(destination, EVADER_MODEL_NAME)


if __name__ == "__main__":
    unittest.main()
