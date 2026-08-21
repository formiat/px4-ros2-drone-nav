#!/usr/bin/env python3
"""Tests for typed static and no-static environment preparation."""

from __future__ import annotations

import argparse
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path
from xml.etree import ElementTree as ET


REPOSITORY = Path(__file__).resolve().parents[2]
SCRIPTS = REPOSITORY / "scripts"
PREPARER_PATH = SCRIPTS / "prepare_environment_simulation.py"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
from gazebo_visibility import SENSOR_COLLISION_PROXY_VISIBILITY_FLAG

SPEC = importlib.util.spec_from_file_location(
    "prepare_environment_simulation", PREPARER_PATH
)
assert SPEC is not None and SPEC.loader is not None
PREPARER = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(PREPARER)


class RuntimeMapModeTest(unittest.TestCase):
    def test_no_static_rejects_static_only_arguments(self) -> None:
        with self.assertRaisesRegex(
            PREPARER.EnvironmentPreparationError,
            "--static-map requires --runtime-map-mode static",
        ):
            PREPARER.validate_runtime_map_arguments(
                argparse.Namespace(
                    runtime_map_mode="no-static",
                    static_map="r050",
                    rebuild_topology=False,
                )
            )

        with self.assertRaisesRegex(
            PREPARER.EnvironmentPreparationError,
            "--rebuild-topology requires --runtime-map-mode static",
        ):
            PREPARER.validate_runtime_map_arguments(
                argparse.Namespace(
                    runtime_map_mode="no-static",
                    static_map=None,
                    rebuild_topology=True,
                )
            )

    def test_no_static_environment_exports_empty_static_artifacts(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            environment_file = root / "environment.env"
            PREPARER.write_runtime_environment(
                environment_file,
                REPOSITORY,
                "test_world",
                root / "collision.sdf",
                root / "sensor.sdf",
                root / "gui.sdf",
                root / "source",
                "no-static",
                None,
                None,
                None,
            )

            environment = environment_file.read_text(encoding="utf-8")

        self.assertIn("export ENVIRONMENT_RUNTIME_MAP_MODE=no-static\n", environment)
        self.assertIn("export SIM_COLLISION_WORLD_SDF_PATH=", environment)
        self.assertIn("export SIM_SENSOR_WORLD_SDF_PATH=", environment)
        self.assertIn("export SIM_GUI_WORLD_SDF_PATH=", environment)
        self.assertIn("export STATIC_OCCUPANCY_3D_PATH=''\n", environment)
        self.assertIn("export STATIC_ESDF_3D_CACHE_PATH=''\n", environment)
        self.assertIn("export STATIC_FREE_SPACE_TOPOLOGY_3D_PATH=''\n", environment)


class LaunchPlatformMaterializationTest(unittest.TestCase):
    def test_platform_remains_physical_and_visible_to_mapping_lidar(self) -> None:
        platforms = [
            {
                "id": "departure",
                "center_sdf_m": (1.0, 2.0, 3.0),
                "size_sdf_m": (6.0, 6.0, 0.5),
            }
        ]
        for mode in (
            PREPARER.MaterializationMode.SENSOR,
            PREPARER.MaterializationMode.GUI,
        ):
            tree = ET.ElementTree(ET.fromstring("<sdf><world name='test'/></sdf>"))

            PREPARER.add_launch_platforms(tree, platforms, mode)

            model = tree.getroot().find("world/model")
            self.assertIsNotNone(model)
            assert model is not None
            self.assertIsNotNone(model.find("link/collision"))
            visual = model.find("link/visual")
            self.assertIsNotNone(visual)
            assert visual is not None
            if mode is PREPARER.MaterializationMode.SENSOR:
                self.assertEqual(
                    SENSOR_COLLISION_PROXY_VISIBILITY_FLAG,
                    int(visual.findtext("visibility_flags", "")),
                )
            else:
                self.assertIsNone(visual.find("visibility_flags"))


if __name__ == "__main__":
    unittest.main()
