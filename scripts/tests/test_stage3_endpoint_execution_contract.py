#!/usr/bin/env python3
"""Static regressions for typed route endpoints and emergency braking tails."""

from __future__ import annotations

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "drone_city_nav"
SOURCE = PACKAGE / "src"


class Stage3EndpointExecutionContractTest(unittest.TestCase):
    def test_continuation_speed_and_finite_boundary_are_separate_policies(self) -> None:
        contract = (SOURCE / "route_execution_contract_3d.cpp").read_text(
            encoding="utf-8"
        )
        parameterization = (SOURCE / "route_time_parameterization.cpp").read_text(
            encoding="utf-8"
        )
        execution = (SOURCE / "production_mppi_node_execution.cpp").read_text(
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
        planning = (SOURCE / "production_mppi_node_planning_tick.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("activated_route->planned_endpoint_semantics", planning)
        self.assertIn(
            "routeEndpointHasTerminalStop3D(route_endpoint_semantics)", planning
        )
        endpoint_assignment = planning.split(
            ".route_endpoint_remaining_m =", maxsplit=1
        )[1].split(".route_constraint_speed_limit_mps", maxsplit=1)[0]
        self.assertNotIn("!route_reaches_mission_goal", endpoint_assignment)

    def test_raw_invalidation_can_only_publish_an_emergency_brake_tail(self) -> None:
        certification = (
            SOURCE / "execution_route_snapshot_3d_finite_execution.cpp"
        ).read_text(encoding="utf-8")
        transitions = (
            SOURCE / "execution_route_snapshot_3d_transitions.cpp"
        ).read_text(encoding="utf-8")

        raw_certification = certification.split(
            "if (certifies_raw_invalidation)", maxsplit=1
        )[1].split("if ((!static_mode", maxsplit=1)[0]
        self.assertIn(
            "certification.kind != FiniteExecutionKind3D::kEmergencyBrakeTail",
            raw_certification,
        )
        raw_transition = transitions.split(
            "case RouteLifecycleEventKind3D::kRawInvalidated:", maxsplit=1
        )[1].split("[[fallthrough]]", maxsplit=1)[0]
        self.assertRegex(
            raw_transition,
            r"retained_safe_execution->kind\s*!=\s*"
            r"FiniteExecutionKind3D::kEmergencyBrakeTail",
        )
        self.assertIn("next.state = BrakingPlan3D{", transitions)


if __name__ == "__main__":
    unittest.main()
