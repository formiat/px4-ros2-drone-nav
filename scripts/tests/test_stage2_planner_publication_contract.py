#!/usr/bin/env python3
"""Static contracts for Stage-2 planner publication transactions."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
SOURCE = REPOSITORY / "drone_city_nav" / "src"
INPUTS = SOURCE / "production_mppi_node_inputs.cpp"
RAW_INPUT = SOURCE / "production_mppi_node_raw_input.cpp"
PLANNING_TICK = SOURCE / "production_mppi_node_planning_tick.cpp"
PLANNING_TICK_FINALIZE = SOURCE / "production_mppi_node_planning_tick_finalize.cpp"
EXECUTION = SOURCE / "production_mppi_node_execution.cpp"
EXECUTION_PUBLICATION = SOURCE / "production_mppi_node_execution_publication.cpp"
EXECUTION_HOLDS = SOURCE / "production_mppi_node_execution_holds.cpp"
PLANNER_MISSION = SOURCE / "production_mppi_node_mission.cpp"


class Stage2PlannerPublicationContractTest(unittest.TestCase):
    def test_publication_linearizes_objective_and_latest_evidence(self) -> None:
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        finalization = PLANNING_TICK_FINALIZE.read_text(encoding="utf-8")
        execution = EXECUTION.read_text(encoding="utf-8")
        publication = (
            EXECUTION_PUBLICATION.read_text(encoding="utf-8")
            + EXECUTION_HOLDS.read_text(encoding="utf-8")
        )
        inputs = INPUTS.read_text(encoding="utf-8")
        raw_input = RAW_INPUT.read_text(encoding="utf-8")
        mission = PLANNER_MISSION.read_text(encoding="utf-8")

        finalization_capture = planning_tick.split(
            "finalizePlanningTick(ProductionMppiPlanningTickFinalization{",
            maxsplit=1,
        )[1]
        self.assertIn(".objective = objective", finalization_capture)
        execution_call = finalization.split(
            "ProductionMppiExecutionPublication execution = publishExecutionHorizon(",
            maxsplit=1,
        )[1]
        self.assertIn("input, result, *world,", execution_call)
        self.assertIn("route_execution, objective,", execution_call)

        execution_publication = execution.split(
            "ProductionMppiNode::publishExecutionHorizon", maxsplit=1
        )[1]
        self.assertIn("objective == nullptr", execution_publication)
        self.assertIn(
            "const Point3 mission_goal = objective->goal;", execution_publication
        )
        self.assertIn(".objective = objective", execution_publication)
        self.assertNotIn("navigationObjective()", execution_publication)
        self.assertNotIn("objective ? objective->goal", execution_publication)

        commit = publication.split(
            "ProductionMppiNode::commitAndPublishExecutionHorizon", maxsplit=1
        )[1].split(
            "ProductionMppiNode::commitExecutionSnapshotHorizon", maxsplit=1
        )[0]
        input_lock = commit.index("input_mutex_")
        request_generation = commit.index("requested_execution_revocation_.load")
        objective_currentness = commit.index(
            "navigation_objective_.load(std::memory_order_acquire) != cycle.objective"
        )
        input_freshness = commit.index("executionInputFreshAt")
        lidar_owner = commit.index("publication_lidar")
        lidar_freshness = commit.index("assessLatestLidarEvidenceFreshness3D")
        raw_currentness = commit.index("committed_world_current")
        snapshot_cas = commit.index("route_execution_manager_.publishPlan")
        wire_publication = commit.index("execution_horizon_pub_->publish")
        for barrier in (
            request_generation,
            objective_currentness,
            input_freshness,
            lidar_owner,
            lidar_freshness,
            raw_currentness,
        ):
            self.assertLess(input_lock, barrier)
            self.assertLess(barrier, snapshot_cas)
            self.assertLess(barrier, wire_publication)
        self.assertIn("kConfirmSnapshotUnchanged", commit)
        self.assertIn("snapshotLidarOwner(*publication_snapshot)", commit)
        self.assertNotIn("cycle.latest_lidar_evidence", commit)
        self.assertIn("publication_lidar->evidenceId()", commit)
        self.assertIn("current_lidar->evidenceId()", commit)
        self.assertIn("publication_lidar->contentFingerprint()", commit)
        self.assertIn("current_lidar->contentFingerprint()", commit)
        self.assertNotIn("publication_lidar != current_lidar", commit)
        self.assertIn("latest_lidar_evidence_identity_conflicted_", commit)
        self.assertIn("publication_now_ns", commit)

        self.assertNotIn("publishLegacyExecutionHorizon", publication)
        self.assertNotIn("legacy_execution_arbiter_", publication)
        hold_commit = publication.split(
            "ProductionMppiNode::publishPositionHold", maxsplit=1
        )[1].split(
            "ProductionMppiNode::publishNoExecutablePathHold", maxsplit=1
        )[0]
        self.assertLess(
            hold_commit.index("execution_evidence_commit_mutex_"),
            hold_commit.index("commitAndPublishExecutionHorizon"),
        )
        raw_update = raw_input.split(
            "void ProductionMppiNode::queueRawWorld3D(", maxsplit=1
        )[1].split(
            "void ProductionMppiNode::onMemoryStatus", maxsplit=1
        )[0]
        self.assertLess(
            raw_update.index("execution_evidence_commit_mutex_"),
            raw_update.index("latest_raw_world_3d_.store"),
        )

        objective_callback = inputs.split(
            "void ProductionMppiNode::onNavigationObjective", maxsplit=1
        )[1]
        pointer_check = objective_callback.index(
            "navigation_objective_.load(std::memory_order_acquire) != previous"
        )
        pointer_store = objective_callback.index(
            "navigation_objective_.store(objective, std::memory_order_release)"
        )
        lifecycle_commit = objective_callback.index(
            "tracking_line_of_sight_lifecycle_ = next_line_of_sight_lifecycle"
        )
        anchor_commit = objective_callback.index("objective_replan_anchor_ = goal")
        route_high_water = objective_callback.index(
            "minimum_tracking_route_sample_sequence_.store"
        )
        self.assertIn("input_mutex_, objective_replan_mutex_", objective_callback)
        self.assertNotIn(
            "tracking_line_of_sight_lifecycle_.reset()",
            objective_callback[:pointer_store],
        )
        self.assertLess(pointer_check, pointer_store)
        for side_effect in (lifecycle_commit, anchor_commit, route_high_water):
            self.assertLess(pointer_store, side_effect)

        mission_transaction = mission.split(
            "Goal capture is an irreversible mission transition", maxsplit=1
        )[1].split("if (!update.waypoint_completed)", maxsplit=1)[0]
        self.assertIn(
            "input_mutex_, objective_replan_mutex_", mission_transaction
        )
        mission_advance = mission_transaction.split(
            "mission_goal_ = mission_waypoint_sequence_->activeGoal();", maxsplit=1
        )[1]
        mission_store = mission_advance.index("navigation_objective_.store")
        self.assertLess(
            mission_store, mission_advance.index("requestExecutionRevocation")
        )
        self.assertLess(
            mission_store, mission_advance.index("objective_replan_anchor_")
        )


if __name__ == "__main__":
    unittest.main()
