#!/usr/bin/env python3
"""Static regressions for the single persistent 3D production planner."""

from __future__ import annotations

import pathlib
import unittest


ROOT = pathlib.Path(__file__).resolve().parents[2]
SOURCE = ROOT / "drone_city_nav" / "src"


class PersistentPlannerProductionContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.selection = (
            SOURCE / "production_mppi_route_selection.cpp"
        ).read_text(encoding="utf-8")
        cls.planning = (
            SOURCE / "production_mppi_node_route_planning.cpp"
        ).read_text(encoding="utf-8")
        cls.header = (SOURCE / "production_mppi_node.hpp").read_text(
            encoding="utf-8"
        )
        cls.artifacts = (
            SOURCE / "production_route_pipeline_artifacts_3d.hpp"
        ).read_text(encoding="utf-8")
        cls.execution_types = (
            SOURCE / "production_mppi_node_execution_types.hpp"
        ).read_text(encoding="utf-8")
        cls.transaction = (
            SOURCE / "production_planner_search_transaction_3d.hpp"
        ).read_text(encoding="utf-8")
        cls.materialization = (
            SOURCE / "production_mppi_route_materialization.cpp"
        ).read_text(encoding="utf-8")
        cls.time_refinement = (
            SOURCE / "persistent_dstar_lite_planner_3d_time_search.cpp"
        ).read_text(encoding="utf-8")
        cls.spatial_search = (
            SOURCE / "persistent_dstar_lite_planner_3d_search.cpp"
        ).read_text(encoding="utf-8")
        cls.diagnostics = (
            SOURCE / "production_mppi_node_diagnostics.cpp"
        ).read_text(encoding="utf-8")
        cls.diagnostics_format = (
            SOURCE / "production_mppi_node_diagnostics_format.hpp"
        ).read_text(encoding="utf-8")
        cls.route_intent = (
            ROOT
            / "drone_city_nav"
            / "include"
            / "drone_city_nav"
            / "route_planning_3d.hpp"
        ).read_text(encoding="utf-8")

    def test_selection_invokes_exactly_one_persistent_mission_planner(self) -> None:
        self.assertEqual(self.selection.count("persistent_planner_3d_->plan("), 1)
        self.assertIn(".mission_goal = mission_goal", self.selection)
        self.assertIn(
            ".mission_epoch = transaction.objective.mission_epoch", self.selection
        )
        self.assertIn("intent.id =", self.selection)
        self.assertIn("makeRouteIntentId3D(", self.selection)

    def test_competing_production_route_producers_are_absent(self) -> None:
        for legacy_producer in (
            "planRiskAwareLattice3D",
            "selectIncrementalTopologyRoute3D",
            "planLaunchSupportDeparture3D",
            "RouteIntentSource3D::kDirect",
            "RouteIntentSource3D::kTopology",
            "RouteIntentSource3D::kLaunchDeparture",
            "RouteIntentPurpose3D::kObservationFrontier",
            "RouteIntentPurpose3D::kTopologicalBacktrack",
        ):
            self.assertNotIn(legacy_producer, self.selection)
        for retired_contract in (
            "RouteIntentSource3D",
            "RouteIntentPurpose3D",
            "intent_target",
            "segment_target",
            "observation_stop_required",
        ):
            self.assertNotIn(retired_contract, self.route_intent)
        for retired_source in (
            "route_strategy_arbitrator_3d.cpp",
            "production_mppi_node_route_strategy.cpp",
            "production_mppi_node_pending_strategy.cpp",
            "risk_aware_lattice.cpp",
            "risk_aware_lattice_3d.cpp",
            "incremental_topological_planner_3d.cpp",
            "incremental_topology_graph_3d.cpp",
            "observation_frontier.cpp",
            "global_guide_candidate.cpp",
            "no_static_route_cycle.cpp",
        ):
            self.assertFalse((SOURCE / retired_source).exists())

    def test_incremental_search_is_resumed_without_discarding_newer_worlds(
        self,
    ) -> None:
        self.assertIn("planner_update.dispatch.continue_search", self.planning)
        self.assertIn("planner_update.improved_incumbent", self.planning)
        self.assertIn("if (!pending_route_planning_work_)", self.planning)
        self.assertIn(".transaction = transaction", self.planning)
        self.assertIn(
            "continuation_queued ? \"true\" : \"newer_world_pending\"",
            self.planning,
        )
        continuation = self.planning.index("planner_update.dispatch.continue_search")
        activation = self.planning.index("commitRouteActivation3D")
        self.assertLess(continuation, activation)

    def test_search_request_is_an_immutable_typed_transaction(self) -> None:
        self.assertIn("struct PlannerSearchTransaction3D", self.transaction)
        self.assertIn("std::shared_ptr<const WorldSnapshot3D> world", self.transaction)
        self.assertIn(
            "std::shared_ptr<const PersistentPlannerWorld3D> planner_world",
            self.transaction,
        )
        self.assertIn("StaticRouteSearchRequestIdentity request", self.transaction)
        self.assertIn("PlannerSearchContinuityBase3D", self.transaction)
        self.assertIn(
            "std::shared_ptr<const PlannerSearchTransaction3D> transaction",
            self.execution_types,
        )
        self.assertNotIn(
            "std::shared_ptr<const ProductionMppiPreparedEsdf> world",
            self.execution_types,
        )
        self.assertNotIn("ProductionMppiPreparedEsdf", self.header)
        self.assertNotIn("ProductionMppiPreparedEsdf", self.artifacts)
        for stage_type in (
            "MaterializedRoute3D",
            "ProductionCompiledRouteCandidate3D",
            "RouteAdmissionReport3D",
            "ProductionRouteActivationResult3D",
        ):
            self.assertIn(stage_type, self.artifacts)
        for retired_request_field in (
            "observed_planner_world",
            "route_search_planner_world",
            "bound_route_instance_id",
            "static_route_extension_request",
            "static_route_replan_request",
            "search_objective{}",
        ):
            self.assertNotIn(retired_request_field, self.artifacts)

    def test_execution_time_refinement_is_inside_the_single_planner(self) -> None:
        for contract in (
            "continueExecutionTimeSearch",
            "estimatedFlightStopAndTurnDelay3D",
            "requiresFlightStopAndTurn3D",
            "seedExecutionTimeIncumbent",
            "execution_time_refiner_.goal_cost_s_",
        ):
            self.assertIn(contract, self.time_refinement)
        self.assertIn("PathPostprocessor3D::shortcut", self.spatial_search)
        self.assertIn("context.time_profile(trial)", self.spatial_search)

    def test_successor_search_starts_at_certified_future_station(self) -> None:
        self.assertIn("search_base_stitch_station_m", self.selection)
        self.assertIn("sampleRoute3DAtStation(active_geometry, stitch_station_m)", self.selection)
        self.assertIn("search_start = stitch.position", self.selection)
        self.assertIn("search_velocity = velocityAtStitch", self.selection)
        self.assertIn("reason=future_stitch_beyond_certified_route", self.selection)

        deferred = self.selection.index("reason=future_stitch_beyond_certified_route")
        planner_call = self.selection.index("persistent_planner_3d_->plan(")
        self.assertLess(deferred, planner_call)

    def test_hard_validation_uses_physical_footprint_and_allows_unknown(self) -> None:
        self.assertIn("OccupiedCollisionWorld3D", self.selection)
        self.assertIn(".footprint = physical_footprint", self.selection)
        self.assertIn(".flight_envelope = flight_envelope", self.selection)
        self.assertIn("physical_footprint_config_", self.selection)
        self.assertNotIn("require_known_free_space", self.selection)
        self.assertNotIn("reject_invalid_esdf", self.selection)
        self.assertNotIn("static_route_tracking_margin_m", self.selection)

    def test_runtime_telemetry_reports_persistent_planner_evidence(self) -> None:
        self.assertIn("ProductionPersistentPlannerTelemetry3D", self.artifacts)
        for field in (
            "candidate.planner_input_status",
            "candidate.planner_progress",
            "plan.search_generation",
            "plan.repair_generation",
            "plan.changed_occupied_voxels",
            "plan.affected_lattice_states",
            "spatial_route.estimated_execution_time_s",
            "plan.execution_time_search_expansions",
            "plan.execution_time_search_objective_s",
            "plan.execution_time_search_complete",
            "plan.world_update_ms",
            "plan.search_ms",
            "plan.search_state_reused",
            "plan.occupied_world_unchanged",
            "plan.incumbent_retained",
        ):
            self.assertIn(field, self.materialization)
        self.assertIn(
            '" planner=persistent_dstar_lite_3d"', self.diagnostics_format
        )
        self.assertIn(
            "persistentPlannerJsonFields(planner_telemetry)", self.diagnostics
        )

    def test_runtime_snapshot_has_no_retired_route_pipeline_telemetry(self) -> None:
        for retired_field in (
            "topology_candidates",
            "topology_objective_cost",
            "global_guide_expansions",
            "global_guide_cost",
            "lattice_search_performed",
            "lattice_status",
            "lattice_termination",
            "lattice_frontier_candidates_considered",
            "lattice_successor_diagnostics",
            "lattice_3d_successor_diagnostics",
            "observation_route_replacement_status",
        ):
            self.assertNotIn(retired_field, self.header)
            self.assertNotIn(retired_field, self.diagnostics)


if __name__ == "__main__":
    unittest.main()
