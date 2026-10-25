#!/usr/bin/env python3
"""Roadmap item 27 block 2: the onboard build. Everything that needs Gazebo or
SDFormat stands behind DRONE_CITY_NAV_ENABLE_SIMULATION, the Dockerfile has an
onboard stage without Gazebo (CUDA 13 on amd64, none on arm64), the
development image is its dev stage, and the Orin's CUDA architecture is
among the MPPI's."""

from __future__ import annotations

import re
import unittest
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parents[2]


class OnboardBuildContractTest(unittest.TestCase):
    def test_gazebo_stands_behind_the_option(self) -> None:
        cmake = (REPOSITORY / "drone_city_nav/CMakeLists.txt").read_text()
        self.assertIn("option(DRONE_CITY_NAV_ENABLE_SIMULATION", cmake)
        # Every line naming a Gazebo, SDFormat or ros_gz target or package sits
        # inside an if(DRONE_CITY_NAV_ENABLE_SIMULATION) block.
        depth: list[bool] = []
        for number, line in enumerate(cmake.splitlines(), 1):
            text = line.strip()
            if text.startswith("if("):
                depth.append("DRONE_CITY_NAV_ENABLE_SIMULATION" in text)
            elif text.startswith("endif("):
                depth.pop()
            elif re.search(r"gz-[a-z]+[0-9]+|sdformat14|ros_gz_interfaces", text) and not text.startswith("#"):
                self.assertTrue(any(depth), f"line {number} outside the option: {text}")
        self.assertIn('"75;86;87"', cmake)

    def test_the_onboard_stage_carries_no_gazebo(self) -> None:
        dockerfile = (REPOSITORY / "docker/Dockerfile").read_text()
        self.assertIn("FROM px4io/px4-dev:v1.17.0 AS dev", dockerfile)
        onboard = dockerfile[dockerfile.index("FROM ros:jazzy-ros-base AS onboard"):]
        self.assertNotIn("gz-", onboard)
        self.assertIn("cuda-compiler-13-0", onboard)
        self.assertIn("ARG CUDA=on", onboard)
        self.assertIn("--target dev", (REPOSITORY / "scripts/build_dev_image.sh").read_text())
        script = (REPOSITORY / "tools/onboard_image.sh").read_text()
        self.assertIn("-DDRONE_CITY_NAV_ENABLE_SIMULATION=OFF", script)
        self.assertIn("--arm64) arch=arm64; platform=linux/arm64; cuda=off", script)


if __name__ == "__main__":
    unittest.main()
