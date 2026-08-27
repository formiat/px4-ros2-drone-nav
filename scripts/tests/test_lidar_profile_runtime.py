#!/usr/bin/env python3
"""Runtime tests for the typed lidar profile selector."""

from __future__ import annotations

import os
import shlex
import subprocess
import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
RUNTIME = REPOSITORY / "scripts" / "lidar_profile_runtime.sh"


class LidarProfileRuntimeTest(unittest.TestCase):
    def resolve(self, profile: str | None) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        environment.pop("LIDAR_PROFILE", None)
        if profile is not None:
            environment["LIDAR_PROFILE"] = profile
        command = (
            "set -euo pipefail; "
            f"source {shlex.quote(str(RUNTIME))}; "
            "resolve_lidar_profile; "
            "printf '%s\\n' \"${lidar_profile}\""
        )
        return subprocess.run(
            ["bash", "-c", command],
            check=False,
            capture_output=True,
            text=True,
            env=environment,
        )

    def test_unset_and_empty_profiles_resolve_to_3d(self) -> None:
        for profile in (None, ""):
            with self.subTest(profile=profile):
                result = self.resolve(profile)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout.strip(), "3d")

    def test_each_typed_profile_can_be_selected_explicitly(self) -> None:
        for profile in ("none", "3d"):
            with self.subTest(profile=profile):
                result = self.resolve(profile)
                self.assertEqual(result.returncode, 0, result.stderr)
                self.assertEqual(result.stdout.strip(), profile)

    def test_unknown_profile_is_rejected(self) -> None:
        for profile in ("2d", "dual"):
            with self.subTest(profile=profile):
                result = self.resolve(profile)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn("LIDAR_PROFILE must be one of none or 3d", result.stderr)


if __name__ == "__main__":
    unittest.main()
