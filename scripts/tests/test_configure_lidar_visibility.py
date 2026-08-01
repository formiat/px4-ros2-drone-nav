#!/usr/bin/env python3
"""Tests for runtime lidar visibility configuration."""

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

import configure_lidar_visibility as visibility  # noqa: E402


SOURCE_MODEL = REPO_ROOT / "drone_city_nav/models/lidar_2d_v2/model.sdf"


def read_mask(path: Path) -> int:
    root = ET.parse(path).getroot()
    sensor = next(
        element
        for element in root.iter("sensor")
        if element.attrib.get("name") == "lidar_2d_v2"
    )
    return int(sensor.findtext("ray/visibility_mask", ""))


class ConfigureLidarVisibilityTest(unittest.TestCase):
    def configure_copy(self, mode: str) -> tuple[int, int]:
        with tempfile.TemporaryDirectory() as temp_dir:
            destination = Path(temp_dir) / "model.sdf"
            shutil.copyfile(SOURCE_MODEL, destination)
            returned_mask = visibility.configure_model(destination, mode)
            return returned_mask, read_mask(destination)

    def test_static_mode_excludes_channel_masses_and_virtual_occluders(self) -> None:
        returned_mask, written_mask = self.configure_copy("static")

        self.assertEqual(visibility.STATIC_VISIBILITY_MASK, returned_mask)
        self.assertEqual(returned_mask, written_mask)
        self.assertEqual(
            0, returned_mask & visibility.STATIC_CHANNEL_MASS_VISIBILITY_FLAG
        )
        self.assertEqual(
            0, returned_mask & visibility.NO_STATIC_OCCLUDER_VISIBILITY_FLAG
        )

    def test_no_static_mode_sees_channel_masses_and_virtual_occluders(self) -> None:
        returned_mask, written_mask = self.configure_copy("no-static")

        self.assertEqual(visibility.GZ_VISIBILITY_ALL, returned_mask)
        self.assertEqual(returned_mask, written_mask)
        self.assertNotEqual(
            0, returned_mask & visibility.STATIC_CHANNEL_MASS_VISIBILITY_FLAG
        )
        self.assertNotEqual(
            0, returned_mask & visibility.NO_STATIC_OCCLUDER_VISIBILITY_FLAG
        )

    def test_unknown_mode_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "unsupported lidar visibility mode"):
            visibility.visibility_mask("unknown")


if __name__ == "__main__":
    unittest.main()
