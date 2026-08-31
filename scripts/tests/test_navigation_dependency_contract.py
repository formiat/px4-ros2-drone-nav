#!/usr/bin/env python3
"""Architectural include bans for controller-neutral navigation contracts."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
INCLUDE = REPOSITORY / "drone_city_nav" / "include" / "drone_city_nav"
SOURCE = REPOSITORY / "drone_city_nav" / "src"
CONTROLLER_NEUTRAL_HEADERS = (
    "compiled_trajectory_3d.hpp",
    "control_contracts_3d.hpp",
    "control_route_projection_3d.hpp",
    "derived_clearance_3d.hpp",
    "dynamic_handoff_validator_3d.hpp",
    "esdf_query.hpp",
    "finite_execution_path_3d.hpp",
    "finite_motion_horizon_3d.hpp",
    "motion_altitude_envelope_3d.hpp",
    "motion_dynamics_3d.hpp",
    "navigation_state_prediction.hpp",
    "observed_esdf_3d.hpp",
    "persistent_dstar_lite_planner_3d.hpp",
    "route_3d.hpp",
    "route_planning_3d.hpp",
    "route_risk_annotation_3d.hpp",
    "route_risk_policy_3d.hpp",
    "static_route_extension.hpp",
    "swept_footprint.hpp",
    "tracking_error_tube_handoff_3d.hpp",
    "trajectory_control_reference_3d.hpp",
    "trajectory_compiler_3d.hpp",
    "world_snapshot_3d.hpp",
)
EXECUTION_HEADERS = tuple(sorted(INCLUDE.glob("execution*.hpp"))) + (
    INCLUDE / "committed_execution_authority_3d.hpp",
)
# This ROS-free use case is intentionally the MPPI-side adapter that turns a
# controller result into neutral execution-plan transitions.
MPPI_EXECUTION_ADAPTERS = {
    SOURCE / "execution_horizon_assembler_3d.cpp",
    SOURCE / "execution_horizon_assembler_3d.hpp",
}
EXECUTION_IMPLEMENTATION = tuple(
    path
    for path in (
        *sorted(SOURCE.glob("execution*.cpp")),
        *sorted(SOURCE.glob("execution*.hpp")),
        SOURCE / "committed_execution_authority_3d.cpp",
    )
    if path not in MPPI_EXECUTION_ADAPTERS
)


class NavigationDependencyContractTest(unittest.TestCase):
    def test_world_planning_and_trajectory_contracts_do_not_depend_on_mppi(self) -> None:
        for name in CONTROLLER_NEUTRAL_HEADERS:
            with self.subTest(header=name):
                text = (INCLUDE / name).read_text(encoding="utf-8")
                self.assertNotIn("drone_city_nav/mppi/", text)
                self.assertNotIn("mppi::", text)

    def test_execution_contracts_and_implementation_do_not_depend_on_mppi(
        self,
    ) -> None:
        for path in EXECUTION_HEADERS + EXECUTION_IMPLEMENTATION:
            with self.subTest(path=path.relative_to(REPOSITORY)):
                text = path.read_text(encoding="utf-8")
                self.assertNotIn("drone_city_nav/mppi/", text)
                self.assertNotIn("mppi::", text)


if __name__ == "__main__":
    unittest.main()
