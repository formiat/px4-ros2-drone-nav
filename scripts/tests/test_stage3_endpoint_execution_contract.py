#!/usr/bin/env python3
"""Static regressions for typed route endpoints and emergency braking tails."""

from __future__ import annotations

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "drone_city_nav"
SOURCE = PACKAGE / "src"
MPPI_RUNTIME = SOURCE / "runtime"
ROS_RUNTIME = MPPI_RUNTIME / "ros"


class Stage3EndpointExecutionContractTest(unittest.TestCase):
    def test_continuation_speed_and_finite_boundary_are_separate_policies(self) -> None:
        contract = (SOURCE / "route_execution_contract_3d.cpp").read_text(
            encoding="utf-8"
        )
        parameterization = (SOURCE / "route_time_parameterization.cpp").read_text(
            encoding="utf-8"
        )
        execution = (ROS_RUNTIME / "production_mppi_node_execution.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("routeEndpointHasTerminalStop3D", parameterization)
        self.assertIn("routeEndpointUsesLocalBoundary3D", execution)
        self.assertRegex(
            contract,
            r"case RouteEndpointSemantics3D::kContinuation:\s*return false;",
        )
        self.assertRegex(
            contract,
            r"case RouteEndpointSemantics3D::kMissionStop:\s*return false;",
        )

    def test_planning_uses_certified_endpoint_semantics_not_goal_negation(self) -> None:
        planning = (MPPI_RUNTIME / "planning_cycle_coordinator_3d.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("activated_route->planned_endpoint_semantics", planning)
        self.assertIn(
            "routeEndpointHasTerminalStop3D(route_endpoint_semantics)", planning
        )
        self.assertNotIn("!route_reaches_mission_goal", planning)

if __name__ == "__main__":
    unittest.main()
