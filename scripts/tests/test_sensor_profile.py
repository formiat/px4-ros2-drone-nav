#!/usr/bin/env python3
"""Tests for the navigation sensor profiles and their defaults: cameras are
the default in the scripts, in both launches and in the Makefile targets."""

from __future__ import annotations

import os
import runpy
import shlex
import subprocess
import unittest
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
SUPPORT = runpy.run_path(str(REPO_ROOT / "drone_city_nav/launch/sensor_profile.py"))
RUNTIME = REPO_ROOT / "scripts" / "lidar_profile_runtime.sh"
validate_sensor_profiles = SUPPORT["validate_sensor_profiles"]
stereo_tof_topics = SUPPORT["stereo_tof_topics"]


class SensorProfileTest(unittest.TestCase):
    def resolve(self, **overrides: str) -> subprocess.CompletedProcess[str]:
        environment = os.environ.copy()
        for name in ("CAMERA_PROFILE", "NAVIGATION_SENSOR_PROFILE"):
            environment.pop(name, None)
        environment.update(overrides)
        command = (
            "set -euo pipefail; "
            f"source {shlex.quote(str(RUNTIME))}; "
            "resolve_camera_profile; "
            "printf '%s %s\\n' \"${camera_profile}\" \"${navigation_sensor_profile}\""
        )
        return subprocess.run(["bash", "-c", command], check=False,
                              capture_output=True, text=True, env=environment)

    def test_cameras_are_the_default_of_the_launch_support(self) -> None:
        self.assertEqual("stereo_tof", SUPPORT["DEFAULT_CAMERA_PROFILE"])
        self.assertEqual("stereo_tof", SUPPORT["DEFAULT_NAVIGATION_SENSOR_PROFILE"])

    def test_cameras_are_the_default_of_the_scripts(self) -> None:
        result = self.resolve()
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "stereo_tof stereo_tof")

    def test_the_lidar_profile_stays_available_on_request(self) -> None:
        result = self.resolve(CAMERA_PROFILE="none", NAVIGATION_SENSOR_PROFILE="lidar")
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertEqual(result.stdout.strip(), "none lidar")
        self.assertEqual(("none", "lidar"), validate_sensor_profiles(" None ", "LIDAR"))
        self.assertEqual(("stereo_tof", "lidar"),
                         validate_sensor_profiles("stereo_tof", "lidar"))

    def test_stereo_navigation_needs_the_camera_set(self) -> None:
        with self.assertRaises(ValueError):
            validate_sensor_profiles("none", "stereo_tof")
        with self.assertRaises(ValueError):
            validate_sensor_profiles("radar", "lidar")
        self.assertNotEqual(self.resolve(CAMERA_PROFILE="radar").returncode, 0)

    def test_each_vehicle_bridges_its_own_camera_set(self) -> None:
        arguments, remaps, depth = stereo_tof_topics("world", "x500_lidar_3d_2",
                                                     "/vehicles/civilian_2")
        # Only the time-of-flight clouds cross the bridge; the images are taken
        # from Gazebo inside the depth node's process.
        self.assertEqual(2, len(arguments))
        self.assertTrue(all("/model/x500_lidar_3d_2/link/stereo_tof_link/sensor/tof_"
                            in argument for argument in arguments))
        self.assertEqual(4, len(remaps))
        self.assertEqual(
            "/world/world/model/x500_lidar_3d_2/link/stereo_tof_link/sensor/"
            "stereo_left/image",
            depth["left_gazebo_image_topic"],
        )
        self.assertEqual("/vehicles/civilian_2/stereo_depth/points",
                         depth["returns_topic"])
        self.assertEqual("/vehicles/civilian_2/stereo/left/image",
                         depth["left_image_topic"])
        self.assertEqual("/vehicles/civilian_2/tof/down/points",
                         depth["tof_down_topic"])
        self.assertEqual("/stereo_depth/points",
                         stereo_tof_topics("world", "model")[2]["returns_topic"])

    def test_both_launches_and_the_makefile_default_to_the_profile(self) -> None:
        single = (REPO_ROOT / "drone_city_nav/launch/city_nav.launch.py").read_text()
        multi = (REPO_ROOT / "drone_city_nav/launch/multi_vehicle.launch.py").read_text()
        self.assertIn("default_value=DEFAULT_NAVIGATION_SENSOR_PROFILE", single)
        self.assertIn("default_value=DEFAULT_CAMERA_PROFILE", single)
        self.assertIn('default_value=_SENSOR_PROFILE_SUPPORT["DEFAULT_CAMERA_PROFILE"]',
                      multi)
        self.assertIn('"DEFAULT_NAVIGATION_SENSOR_PROFILE"', multi)
        self.assertIn("planner_params.update(_STEREO_TOF_OBSERVABILITY)", multi)
        self.assertIn("memory_params.update(_VISION_MEMORY_OVERRIDES)", multi)
        runner = (REPO_ROOT / "scripts/run_drone_nav_sim.sh").read_text()
        self.assertEqual(2, runner.count('camera_profile:="${camera_profile}"'))
        self.assertEqual(
            2, runner.count('navigation_sensor_profile:="${navigation_sensor_profile}"'))
        makefile = (REPO_ROOT / "Makefile").read_text()
        # Cameras need the textured world; both headless targets choose it.
        self.assertEqual(2, makefile.count('"$${CAMERA_PROFILE:-stereo_tof}" = none'))


if __name__ == "__main__":
    unittest.main()
