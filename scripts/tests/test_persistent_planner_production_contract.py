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
        cls.guide = (
            SOURCE / "production_mppi_node_static_guide.cpp"
        ).read_text(encoding="utf-8")

    def test_selection_invokes_exactly_one_persistent_mission_planner(self) -> None:
        self.assertEqual(self.selection.count("persistent_planner_3d_->plan("), 1)
        self.assertIn(".mission_goal = mission_goal", self.selection)
        self.assertIn(
            ".mission_epoch = world.search_objective.mission_epoch", self.selection
        )
        self.assertIn(
            ".source = RouteIntentSource3D::kPersistentPlanner", self.selection
        )
        self.assertIn(
            ".purpose = RouteIntentPurpose3D::kMissionTransit", self.selection
        )

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
        for retired_source in (
            "route_strategy_arbitrator_3d.cpp",
            "production_mppi_node_route_strategy.cpp",
            "production_mppi_node_pending_strategy.cpp",
        ):
            self.assertFalse((SOURCE / retired_source).exists())

    def test_incremental_search_is_resumed_without_discarding_newer_worlds(
        self,
    ) -> None:
        self.assertIn("PersistentPlannerStatus3D::kSearchInProgress", self.guide)
        self.assertIn("if (!pending_guide_world_)", self.guide)
        self.assertIn("continuation_queued ? \"true\" : \"newer_world_pending\"", self.guide)
        continuation = self.guide.index("PersistentPlannerStatus3D::kSearchInProgress")
        activation = self.guide.index("commitRouteActivation3D")
        self.assertLess(continuation, activation)

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
        self.assertIn(".footprint = physical_footprint", self.selection)
        self.assertIn("physical_footprint_config_", self.selection)
        self.assertIn(".require_known_free_space = false", self.selection)
        self.assertIn(".reject_invalid_esdf = false", self.selection)
        self.assertNotIn("static_route_tracking_margin_m", self.selection)


if __name__ == "__main__":
    unittest.main()
