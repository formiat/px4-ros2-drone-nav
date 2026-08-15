#!/usr/bin/env python3
"""Contract tests for the separate FreeSpaceTopology3D artifact."""

from __future__ import annotations

import unittest
from pathlib import Path

import yaml


REPO_ROOT = Path(__file__).resolve().parents[2]
PARAMETERS = REPO_ROOT / "drone_city_nav/config/urban_mvp.yaml"
MULTI_VEHICLE_LAUNCH = (
    REPO_ROOT / "drone_city_nav/launch/multi_vehicle.launch.py"
)
CITY_NAV_LAUNCH = REPO_ROOT / "drone_city_nav/launch/city_nav.launch.py"


class FreeSpaceTopologyContractTest(unittest.TestCase):
    def test_topology_path_belongs_only_to_the_planner(self) -> None:
        with PARAMETERS.open(encoding="utf-8") as stream:
            document = yaml.safe_load(stream)
        planner = document["production_mppi_node"]["ros__parameters"]
        visualization = document["world_visualization_node"]["ros__parameters"]
        self.assertEqual(
            "worlds/generated_city.topology3d",
            planner["static_free_space_topology_3d_path"],
        )
        self.assertNotIn("static_free_space_topology_3d_path", visualization)

    def test_launches_forward_an_explicit_topology_path(self) -> None:
        multi_vehicle_source = MULTI_VEHICLE_LAUNCH.read_text(encoding="utf-8")
        city_nav_source = CITY_NAV_LAUNCH.read_text(encoding="utf-8")
        self.assertIn(
            '"static_free_space_topology_3d_path": static_topology_path',
            multi_vehicle_source,
        )
        self.assertIn(
            'DeclareLaunchArgument(\n'
            '                "static_free_space_topology_3d_path"',
            multi_vehicle_source,
        )
        self.assertIn(
            "if not static_topology_path_override and not static_path_override:",
            multi_vehicle_source,
        )
        self.assertIn(
            '"static_free_space_topology_3d_path": (', city_nav_source
        )
        self.assertIn(
            'DeclareLaunchArgument(\n'
            '                "static_free_space_topology_3d_path"',
            city_nav_source,
        )
        self.assertIn(
            '{"static_free_space_topology_3d_path": ""}', city_nav_source
        )

    def test_raw_occupancy_does_not_own_topology(self) -> None:
        occupancy_header = (
            REPO_ROOT
            / "drone_city_nav/include/drone_city_nav/occupancy_grid_3d.hpp"
        ).read_text(encoding="utf-8")
        occupancy_source = (
            REPO_ROOT / "drone_city_nav/src/occupancy_grid_3d.cpp"
        ).read_text(encoding="utf-8")
        self.assertNotIn("portalGraph", occupancy_header)
        self.assertNotIn("DerivedPortalGraph", occupancy_header)
        self.assertNotIn("PassageTraversalEdge", occupancy_source)


if __name__ == "__main__":
    unittest.main()
