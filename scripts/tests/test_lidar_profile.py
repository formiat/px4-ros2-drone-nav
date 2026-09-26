#!/usr/bin/env python3
"""Tests for the navigation lidar profile and model identity resolution."""

from __future__ import annotations

import runpy
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SUPPORT = runpy.run_path(
    str(REPO_ROOT / "drone_city_nav/launch/lidar_profile.py")
)
DEFAULT_LIDAR_PROFILE = SUPPORT["DEFAULT_LIDAR_PROFILE"]
resolve_model_identity = SUPPORT["resolve_model_identity"]
validate_lidar_profile = SUPPORT["validate_lidar_profile"]


class LidarProfileTest(unittest.TestCase):
    def test_default_profile_is_3d(self) -> None:
        self.assertEqual("3d", DEFAULT_LIDAR_PROFILE)

    def test_supported_profile_is_normalized(self) -> None:
        self.assertEqual("3d", validate_lidar_profile(" 3D "))

    def test_3d_profile_keeps_the_scenario_model_identity(self) -> None:
        identity = ("gz_x500_lidar_3d", "x500_lidar_3d_0")

        self.assertEqual(identity, resolve_model_identity(*identity, "3d"))

    def test_other_profiles_are_rejected(self) -> None:
        for profile in ("2d", "none", "dual"):
            with self.subTest(profile=profile), self.assertRaisesRegex(
                ValueError, "lidar profile"
            ):
                validate_lidar_profile(profile)


if __name__ == "__main__":
    unittest.main()
