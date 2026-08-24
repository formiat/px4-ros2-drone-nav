#!/usr/bin/env python3
"""Static regressions for stateful route-strategy arbitration."""

from __future__ import annotations

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "drone_city_nav"
INCLUDE = PACKAGE / "include" / "drone_city_nav"
SOURCE = PACKAGE / "src"


class Stage5RouteStrategyContractTest(unittest.TestCase):
    def test_arbitrator_owns_reason_budget_hysteresis_and_return_lineage(self) -> None:
        header = (INCLUDE / "route_strategy_arbitrator_3d.hpp").read_text(
            encoding="utf-8"
        )
        implementation = (SOURCE / "route_strategy_arbitrator_3d.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("RouteStrategyLeaseReason3D reason", header)
        self.assertIn("RouteStrategyReturnLineage3D return_lineage", header)
        self.assertIn("double distance_budget_m", header)
        self.assertIn("direct_advantage_confirmations", header)
        self.assertIn("minimum_return_mission_progress_m", header)
        self.assertIn("RouteStrategyKind3D::kTopologyMission", implementation)
        self.assertIn("RouteStrategyKind3D::kObservationFrontier", implementation)
        self.assertIn("RouteStrategyKind3D::kTopologicalBacktrack", implementation)
        self.assertIn("directHasReleaseAdvantage", implementation)
        self.assertIn("retiredMatchesProposal", implementation)

    def test_selection_state_changes_only_after_publication_outcome(self) -> None:
        header = (INCLUDE / "route_strategy_arbitrator_3d.hpp").read_text(
            encoding="utf-8"
        )
        implementation = (SOURCE / "route_strategy_arbitrator_3d.cpp").read_text(
            encoding="utf-8"
        )
        guide = (SOURCE / "production_mppi_node_static_guide.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("state_without_commit", header)
        self.assertIn("state_after_commit", header)
        self.assertIn("recordOutcome", header)
        self.assertIn("pending_decision_sequence_", implementation)
        evaluation = guide.index("route_strategy_arbitrator_3d_.evaluate")
        activation = guide.index("commitRouteActivation3D")
        outcome = guide.index("route_strategy_arbitrator_3d_.recordOutcome")
        self.assertLess(evaluation, activation)
        self.assertLess(activation, outcome)
        self.assertIn("strategy_decision, certified_pending", guide[outcome:])

    def test_topology_candidate_carries_generation_bound_return_lineage(self) -> None:
        selection = (SOURCE / "production_mppi_route_selection.cpp").read_text(
            encoding="utf-8"
        )
        planning_header = (INCLUDE / "route_planning_3d.hpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("makeRouteStrategyReturnLineage3D", planning_header)
        self.assertIn("topology.plan.strategic_plan_id", selection)
        self.assertIn("topology.plan.topology_lineage_id", selection)
        self.assertIn("topology.plan.planned_on_revision", selection)
        self.assertIn("topology.plan.start_node.value", selection)
        self.assertIn("strategyLeaseReason(topology.plan)", selection)

    def test_runtime_configuration_and_diagnostics_are_wired(self) -> None:
        config = (PACKAGE / "config" / "urban_mvp.yaml").read_text(encoding="utf-8")
        node = (SOURCE / "production_mppi_node.cpp").read_text(encoding="utf-8")
        route_strategy = (
            SOURCE / "production_mppi_node_route_strategy.cpp"
        ).read_text(encoding="utf-8")
        guide = (SOURCE / "production_mppi_node_static_guide.cpp").read_text(
            encoding="utf-8"
        )

        self.assertIn("configureRouteStrategyArbitration()", node)
        for parameter in (
            "route_strategy_topology_mission_lease_budget_m",
            "route_strategy_observation_frontier_lease_budget_m",
            "route_strategy_topological_backtrack_lease_budget_m",
            "route_strategy_minimum_lease_commitment_m",
            "route_strategy_minimum_return_mission_progress_m",
            "route_strategy_direct_release_minimum_progress_advantage_m",
            "route_strategy_direct_release_minimum_progress_ratio_advantage",
            "route_strategy_direct_release_confirmation_count",
        ):
            self.assertIn(parameter, config)
            self.assertIn(parameter, route_strategy)
        self.assertIn("ROUTE_STRATEGY_ARBITRATION3D", guide)
        self.assertNotIn("selectRouteProposal3D(final_proposals", guide)


if __name__ == "__main__":
    unittest.main()
