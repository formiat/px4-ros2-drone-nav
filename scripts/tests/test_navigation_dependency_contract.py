#!/usr/bin/env python3
"""Architectural include bans for controller-neutral navigation contracts."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
INCLUDE = REPOSITORY / "drone_city_nav" / "include" / "drone_city_nav"
CONTROLLER_NEUTRAL_HEADERS = (
    "compiled_trajectory_3d.hpp",
    "derived_clearance_3d.hpp",
    "esdf_query.hpp",
    "observed_esdf_3d.hpp",
    "persistent_dstar_lite_planner_3d.hpp",
    "route_3d.hpp",
    "route_planning_3d.hpp",
    "route_risk_annotation_3d.hpp",
    "static_route_extension.hpp",
    "swept_footprint.hpp",
    "trajectory_compiler_3d.hpp",
    "world_snapshot_3d.hpp",
)


class NavigationDependencyContractTest(unittest.TestCase):
    def test_world_planning_and_trajectory_contracts_do_not_depend_on_mppi(self) -> None:
        for name in CONTROLLER_NEUTRAL_HEADERS:
            with self.subTest(header=name):
                text = (INCLUDE / name).read_text(encoding="utf-8")
                self.assertNotIn("drone_city_nav/mppi/", text)
                self.assertNotIn("mppi::", text)


if __name__ == "__main__":
    unittest.main()
