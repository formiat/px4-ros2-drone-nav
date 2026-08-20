#!/usr/bin/env python3
"""Tests for mutually exclusive lidar profile model resolution."""

from __future__ import annotations

import runpy
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SUPPORT = runpy.run_path(
    str(REPO_ROOT / "drone_city_nav/launch/lidar_profile.py")
)
resolve_model_identity = SUPPORT["resolve_model_identity"]
validate_lidar_profile = SUPPORT["validate_lidar_profile"]


class LidarProfileTest(unittest.TestCase):
    def test_supported_profiles_are_normalized(self) -> None:
        self.assertEqual("3d", validate_lidar_profile(" 3D "))

    def test_2d_and_none_preserve_legacy_model_identity(self) -> None:
        identity = ("gz_x500_lidar_2d", "x500_lidar_2d_0")

        self.assertEqual(identity, resolve_model_identity(*identity, "2d"))
        self.assertEqual(identity, resolve_model_identity(*identity, "none"))

    def test_3d_profile_resolves_standard_and_role_specific_models(self) -> None:
        self.assertEqual(
            ("gz_x500_lidar_3d", "x500_lidar_3d_0"),
            resolve_model_identity(
                "gz_x500_lidar_2d", "x500_lidar_2d_0", "3d"
            ),
        )
        self.assertEqual(
            ("gz_x500_lidar_3d_evader", "x500_lidar_3d_evader_3"),
            resolve_model_identity(
                "gz_x500_lidar_2d_evader",
                "x500_lidar_2d_evader_3",
                "3d",
            ),
        )

    def test_unknown_profile_and_incompatible_model_are_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "lidar profile"):
            validate_lidar_profile("dual")
        with self.assertRaisesRegex(ValueError, "compatible PX4 model"):
            resolve_model_identity("gz_x500", "x500_0", "3d")


if __name__ == "__main__":
    unittest.main()
