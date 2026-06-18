#!/usr/bin/env python3
"""Tests for Gazebo GUI launch log validation."""

from __future__ import annotations

import importlib.util
import sys
import unittest
from pathlib import Path


SCRIPT_PATH = Path(__file__).resolve().parents[1] / "validate_gazebo_gui_launch_log.py"
SPEC = importlib.util.spec_from_file_location("gazebo_gui_launch_log", SCRIPT_PATH)
assert SPEC is not None
validator = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = validator
assert SPEC.loader is not None
SPEC.loader.exec_module(validator)


class GazeboGuiLaunchLogValidatorTest(unittest.TestCase):
    def test_valid_log_passes(self) -> None:
        result = validator.validate_log(
            "\n".join(
                [
                    "Gazebo stale cleanup: no conflicting Gazebo processes found",
                    "Gazebo world running command confirmed: world=generated_city",
                    (
                        "Gazebo GUI follow camera command accepted but state "
                        "confirmation is unavailable; continuing best-effort."
                    ),
                ]
            )
        )

        self.assertTrue(result.ok, result.errors)

    def test_missing_unpause_fails(self) -> None:
        result = validator.validate_log(
            "\n".join(
                [
                    "Gazebo stale cleanup: no conflicting Gazebo processes found",
                    "Gazebo GUI follow camera state confirmed: target=x500_lidar_2d_0",
                ]
            )
        )

        self.assertFalse(result.ok)
        self.assertIn("FAIL: Gazebo world unpause is confirmed", result.errors)

    def test_gui_config_override_fails(self) -> None:
        result = validator.validate_log(
            "\n".join(
                [
                    "Gazebo stale cleanup: no conflicting Gazebo processes found",
                    "Gazebo world running command confirmed: world=generated_city",
                    "Gazebo GUI follow camera state confirmed: target=x500_lidar_2d_0",
                    "gz sim -g --gui-config /tmp/gui.config",
                ]
            )
        )

        self.assertFalse(result.ok)
        self.assertIn("FAIL: Gazebo GUI config override is absent", result.errors)


if __name__ == "__main__":
    unittest.main()
