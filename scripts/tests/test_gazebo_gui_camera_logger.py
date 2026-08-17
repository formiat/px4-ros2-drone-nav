#!/usr/bin/env python3
"""Tests for Gazebo GUI camera pose logging helpers."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "gazebo_gui_camera_logger.py"
SPEC = importlib.util.spec_from_file_location("gazebo_gui_camera_logger", SCRIPT_PATH)
assert SPEC is not None
logger = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = logger
assert SPEC.loader is not None
SPEC.loader.exec_module(logger)


class GazeboGuiCameraLoggerTest(unittest.TestCase):
    def test_parses_identity_camera_pose(self) -> None:
        pose = logger.parse_camera_pose(
            {
                "position": {"x": 1.0, "y": 2.0, "z": 3.0},
                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0},
            }
        )

        self.assertEqual({"x": 1.0, "y": 2.0, "z": 3.0}, pose["position_m"])
        self.assertEqual({"x": 0.0, "y": 0.0, "z": -1.0}, pose["forward_direction"])

    def test_rotates_camera_forward_direction(self) -> None:
        pose = logger.parse_camera_pose(
            {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {
                    "x": 0.0,
                    "y": 0.7071067811865476,
                    "z": 0.0,
                    "w": 0.7071067811865476,
                },
            }
        )

        self.assertEqual({"x": -1.0, "y": 0.0, "z": 0.0}, pose["forward_direction"])

    def test_rejects_invalid_camera_log_interval(self) -> None:
        for value in ("0", "-1", "nan", "invalid"):
            with self.subTest(value=value):
                with self.assertRaises(logger.CameraLoggerError):
                    logger.parse_interval_s(value)


if __name__ == "__main__":
    unittest.main()
