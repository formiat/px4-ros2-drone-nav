#!/usr/bin/env python3
"""Contract tests for static ESDF cache launch wiring."""

from __future__ import annotations

import unittest
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
PARAMETERS = REPO_ROOT / "drone_city_nav/config/urban_mvp.yaml"
MULTI_VEHICLE_LAUNCH = REPO_ROOT / "drone_city_nav/launch/multi_vehicle.launch.py"


class StaticEsdfCacheContractTest(unittest.TestCase):
    def test_cache_path_belongs_to_planner_parameters(self) -> None:
        with PARAMETERS.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        planner = document["production_mppi_node"]["ros__parameters"]
        visualization = document["world_visualization_node"]["ros__parameters"]
        self.assertEqual("", planner["static_esdf_3d_cache_path"])
        self.assertNotIn("static_esdf_3d_cache_path", visualization)

    def test_multi_vehicle_launch_forwards_cache_path_to_every_planner(self) -> None:
        source = MULTI_VEHICLE_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            '"static_esdf_3d_cache_path": static_esdf_cache_path', source
        )
        self.assertIn(
            'DeclareLaunchArgument("static_esdf_3d_cache_path"', source
        )


if __name__ == "__main__":
    unittest.main()
