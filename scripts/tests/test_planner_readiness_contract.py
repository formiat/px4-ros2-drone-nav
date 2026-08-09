#!/usr/bin/env python3
"""Static contracts for planner-world and intercept-start readiness."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
SOURCE = PACKAGE / "src"
INPUTS = SOURCE / "production_mppi_node_inputs.cpp"
ESDF = SOURCE / "production_mppi_node_esdf.cpp"
PLANNER = SOURCE / "production_mppi_node.cpp"
PLANNING_TICK = SOURCE / "production_mppi_node_planning_tick.cpp"
REFEREE = SOURCE / "intercept_mission_referee_node.cpp"
REFEREE_LIFECYCLE = SOURCE / "intercept_mission_referee_lifecycle.cpp"
REFEREE_SUPPORT = SOURCE / "intercept_referee_support.cpp"
ASSIGNMENT_COORDINATOR = SOURCE / "target_assignment_coordinator_node.cpp"
LAUNCH = PACKAGE / "launch" / "intercept.launch.py"


class PlannerReadinessContractTest(unittest.TestCase):
    def test_static_esdf_bootstrap_does_not_consume_lidar_snapshots(self) -> None:
        inputs = INPUTS.read_text(encoding="utf-8")
        esdf = ESDF.read_text(encoding="utf-8")

        self.assertIn("if (use_static_map_) {\n    return;\n  }", inputs)
        self.assertIn("requestStaticEsdfWork();", inputs)
        self.assertIn("vehicle_navigation_ready_", inputs)
        self.assertIn("pending_static_esdf_work_", esdf)
        self.assertIn("static_occupancy_3d_", esdf)
        self.assertIn("completeStaticEsdfWork(true)", esdf)

    def test_static_esdf_is_not_expired_by_lidar_time(self) -> None:
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")

        self.assertRegex(
            planning_tick,
            r"if \(esdf\.has_value\(\)\)\s*\{\s*"
            r"esdf_age_ms = use_static_map_\s*\?\s*0\.0",
        )

    def test_planners_publish_latched_world_readiness(self) -> None:
        planner = PLANNER.read_text(encoding="utf-8")
        launch = LAUNCH.read_text(encoding="utf-8")

        self.assertIn('"world_readiness_topic"', planner)
        self.assertIn("reliable().transient_local()", planner)
        self.assertIn('f"{prefix}/mppi/world_ready"', launch)
        planner_parameters = launch.split("planner_params =", maxsplit=1)[1].split(
            "offboard_params =", maxsplit=1
        )[0]
        self.assertIn(
            '"navigation_readiness_topic": f"{prefix}/navigation_ready"',
            planner_parameters,
        )

    def test_intercept_start_requires_all_worlds_and_target_tracks(self) -> None:
        referee = REFEREE.read_text(encoding="utf-8") + REFEREE_LIFECYCLE.read_text(
            encoding="utf-8"
        )
        referee_support = REFEREE_SUPPORT.read_text(encoding="utf-8")
        coordinator = ASSIGNMENT_COORDINATOR.read_text(encoding="utf-8")
        launch = LAUNCH.read_text(encoding="utf-8")

        self.assertIn("missionReady() const", referee)
        self.assertIn("interceptor_world_readiness_topics", referee_support)
        self.assertIn("target_world_readiness_topics", referee_support)
        self.assertIn("target_track_readiness_topics", referee_support)
        self.assertIn("std::ranges::all_of(interceptors_", referee)
        self.assertIn("publishReadiness(runtime, true)", coordinator)
        self.assertIn("interceptor_world_readiness_topics", launch)
        self.assertIn("target_world_readiness_topics", launch)
        self.assertIn("target_track_readiness_topics", launch)

    def test_coordinate_alignment_is_latched_as_a_startup_contract(self) -> None:
        referee = REFEREE.read_text(encoding="utf-8") + REFEREE_LIFECYCLE.read_text(
            encoding="utf-8"
        )
        referee_support = REFEREE_SUPPORT.read_text(encoding="utf-8")
        self.assertIn("latchStartupContract()", referee)
        self.assertIn("startup_failure_confirmed", referee)
        self.assertIn("runtime_residual=true", referee_support)
        self.assertIn("mission_blocked=false", referee_support)
        self.assertNotIn("truth_alignment_sample_aligned_", referee)


if __name__ == "__main__":
    unittest.main()
