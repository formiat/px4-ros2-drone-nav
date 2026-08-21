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
import gazebo_visibility as layers  # noqa: E402


SOURCE_MODEL = REPO_ROOT / "drone_city_nav/models/lidar_2d_v2/model.sdf"
SOURCE_MODEL_3D = REPO_ROOT / "drone_city_nav/models/lidar_3d_v1/model.sdf"


def read_sensor(path: Path) -> ET.Element:
    root = ET.parse(path).getroot()
    return next(
        element
        for element in root.iter("sensor")
        if element.attrib.get("type") == "gpu_lidar"
    )


def read_mask(path: Path) -> int:
    return int(read_sensor(path).findtext("ray/visibility_mask", ""))


def read_always_on(path: Path) -> bool:
    return read_sensor(path).findtext("always_on") == "true"


class ConfigureLidarVisibilityTest(unittest.TestCase):
    def configure_copy(self, mode: str) -> tuple[int, int]:
        with tempfile.TemporaryDirectory() as temp_dir:
            destination = Path(temp_dir) / "model.sdf"
            shutil.copyfile(SOURCE_MODEL, destination)
            returned_mask = visibility.configure_model(destination, mode)
            return returned_mask, read_mask(destination)

    def test_static_mode_excludes_passage_masses_and_virtual_occluders(self) -> None:
        returned_mask, written_mask = self.configure_copy("static")

        self.assertEqual(layers.STATIC_VISIBILITY_MASK, returned_mask)
        self.assertEqual(returned_mask, written_mask)
        self.assertEqual(
            0, returned_mask & layers.STATIC_PASSAGE_MASS_VISIBILITY_FLAG
        )
        self.assertEqual(
            0, returned_mask & layers.NO_STATIC_OCCLUDER_VISIBILITY_FLAG
        )

    def test_no_static_mode_sees_passage_masses_and_virtual_occluders(self) -> None:
        returned_mask, written_mask = self.configure_copy("no-static-2d")

        self.assertEqual(layers.SENSOR_VISIBLE_WORLD_MASK, returned_mask)
        self.assertEqual(returned_mask, written_mask)
        self.assertNotEqual(
            0, returned_mask & layers.STATIC_PASSAGE_MASS_VISIBILITY_FLAG
        )
        self.assertNotEqual(
            0, returned_mask & layers.NO_STATIC_OCCLUDER_VISIBILITY_FLAG
        )
        self.assertEqual(
            0, returned_mask & layers.DRONE_MARKER_VISIBILITY_FLAG
        )

    def test_no_static_3d_sees_physical_passage_but_not_virtual_occluder(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            destination = Path(temp_dir) / "model.sdf"
            shutil.copyfile(SOURCE_MODEL_3D, destination)

            returned_mask = visibility.configure_model(destination, "no-static-3d")

            self.assertEqual(layers.NO_STATIC_3D_VISIBILITY_MASK, returned_mask)
            self.assertEqual(returned_mask, read_mask(destination))
            self.assertNotEqual(
                0, returned_mask & layers.STATIC_PASSAGE_MASS_VISIBILITY_FLAG
            )
            self.assertEqual(
                0, returned_mask & layers.NO_STATIC_OCCLUDER_VISIBILITY_FLAG
            )
            self.assertEqual(
                0, returned_mask & layers.DRONE_MARKER_VISIBILITY_FLAG
            )
            self.assertNotEqual(
                0,
                returned_mask & layers.SENSOR_COLLISION_PROXY_VISIBILITY_FLAG,
            )

    def test_unknown_mode_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "unsupported lidar visibility mode"):
            visibility.visibility_mask("unknown")

    def test_sensor_can_be_disabled_without_changing_visibility_contract(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            destination = Path(temp_dir) / "model.sdf"
            shutil.copyfile(SOURCE_MODEL, destination)

            returned_mask = visibility.configure_model(
                destination, "static", enabled=False
            )

            self.assertEqual(layers.STATIC_VISIBILITY_MASK, returned_mask)
            self.assertFalse(read_always_on(destination))


if __name__ == "__main__":
    unittest.main()
