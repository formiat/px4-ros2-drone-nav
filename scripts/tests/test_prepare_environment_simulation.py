#!/usr/bin/env python3
"""Tests for typed static and no-static environment preparation."""

from __future__ import annotations

import argparse
import importlib.util
import sys
import tempfile
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
SCRIPTS = REPOSITORY / "scripts"
PREPARER_PATH = SCRIPTS / "prepare_environment_simulation.py"
if str(SCRIPTS) not in sys.path:
    sys.path.insert(0, str(SCRIPTS))
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
                root / "gui.sdf",
                root / "source",
                "no-static",
                None,
                None,
                None,
            )

            environment = environment_file.read_text(encoding="utf-8")

        self.assertIn("export ENVIRONMENT_RUNTIME_MAP_MODE=no-static\n", environment)
        self.assertIn("export STATIC_OCCUPANCY_3D_PATH=''\n", environment)
        self.assertIn("export STATIC_ESDF_3D_CACHE_PATH=''\n", environment)
        self.assertIn("export STATIC_FREE_SPACE_TOPOLOGY_3D_PATH=''\n", environment)


if __name__ == "__main__":
    unittest.main()
