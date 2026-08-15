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
EXECUTION = SOURCE / "production_mppi_node_execution.cpp"
OFFBOARD = SOURCE / "mppi_offboard_node.cpp"
OBSTACLE_MEMORY = SOURCE / "obstacle_memory_node.cpp"
HORIZON_MESSAGE = PACKAGE / "msg" / "MppiTrajectoryHorizon.msg"
SPEED_POLICY = PACKAGE / "include" / "drone_city_nav" / "mppi_speed_policy.hpp"
MPPI_REFERENCE = SOURCE / "mppi" / "mppi_reference.cpp"
MPPI_KERNELS = SOURCE / "mppi" / "mppi_engine_kernels.cuh"
FINITE_HORIZON = SOURCE / "mppi" / "mppi_finite_horizon.cpp"
FINITE_EXECUTION_PATH = SOURCE / "mppi" / "finite_execution_path.cpp"
REFEREE = SOURCE / "intercept_mission_referee_node.cpp"
REFEREE_LIFECYCLE = SOURCE / "intercept_mission_referee_lifecycle.cpp"
REFEREE_SUPPORT = SOURCE / "intercept_referee_support.cpp"
ASSIGNMENT_COORDINATOR = SOURCE / "target_assignment_coordinator_node.cpp"
LAUNCH = PACKAGE / "launch" / "multi_vehicle.launch.py"
MISSION_LAUNCH = PACKAGE / "launch" / "multi_vehicle_mission_launch.py"


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

    def test_missing_executable_route_holds_without_a_clearance_gate(self) -> None:
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        execution = EXECUTION.read_text(encoding="utf-8")
        offboard = OFFBOARD.read_text(encoding="utf-8")
        planner = PLANNER.read_text(encoding="utf-8")
        horizon_message = HORIZON_MESSAGE.read_text(encoding="utf-8")
        speed_policy = SPEED_POLICY.read_text(encoding="utf-8")
        finite_horizon = FINITE_HORIZON.read_text(encoding="utf-8")
        finite_execution_path = FINITE_EXECUTION_PATH.read_text(encoding="utf-8")

        self.assertIn("kNoExecutableRouteHold", planning_tick)
        self.assertIn("no_executable_route_hold_position_", planning_tick)
        self.assertIn("temporary_frontier_is_terminal", planning_tick)
        self.assertIn("ProductionMppiExecutionReason::kNoExecutableRoute", execution)
        self.assertIn(
            "ProductionMppiExecutionReason::kNoExecutableHorizon", execution
        )
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_ROUTE=4", horizon_message)
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_HORIZON=1", horizon_message)
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_ROUTE", offboard)
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_HORIZON", offboard)
        self.assertIn("no_executable_path_hold_position_", execution)
        self.assertIn("action=hold_no_executable_path", execution)
        self.assertIn("publish_position_hold", execution)
        self.assertNotIn("ProductionMppiExecutionMode::kBraking", execution)
        self.assertNotIn("EXECUTION_MODE_BRAKING", horizon_message)
        self.assertNotIn("dynamicStopRequested", offboard)
        self.assertIn("publishUnavailablePathHoldSetpoint", offboard)
        self.assertIn("plannedFinitePathCompleted", offboard)
        self.assertIn("buildMppiPathTrajectorySetpoint", offboard)
        self.assertIn("buildValidatedFiniteExecutionPath", execution)
        self.assertIn("buildFiniteHorizon", finite_execution_path)
        self.assertIn("finiteHorizonHasTerminalRestState", finite_horizon)
        self.assertIn(
            "finite_path_arrival_maximum_horizontal_deceleration_mps2", planner
        )
        self.assertIn("finite_horizon_config_", execution)
        self.assertIn("maximum_horizontal_deceleration_mps2", finite_horizon)
        self.assertIn("validateFiniteExecutionTrajectoryContinuation", execution)
        self.assertIn("validateFiniteExecutionPathContinuation", execution)
        self.assertIn("actual_state_validation.accepted()", execution)
        self.assertIn("rebased_from_actual=true", execution)
        self.assertIn("active_finite_execution_path_", execution)
        self.assertIn("FiniteExecutionPathTerminalBoundary", execution)
        self.assertIn("original_valid_until_ns", execution)
        self.assertIn("planned_horizon_without_terminal_rest_state", offboard)
        self.assertNotIn("route_free_", planning_tick)
        self.assertNotIn("temporary_frontier_continuation_ready", planning_tick)
        self.assertNotIn("route_endpoint_terminal_speed_mps", speed_policy)
        self.assertNotRegex(
            planning_tick,
            r"clearance[^\n]*kNoExecutableRouteHold|"
            r"kNoExecutableRouteHold[^\n]*clearance",
        )

    def test_cpu_and_cuda_enforce_dynamic_altitude_recoverability(self) -> None:
        reference = MPPI_REFERENCE.read_text(encoding="utf-8")
        kernels = MPPI_KERNELS.read_text(encoding="utf-8")

        self.assertIn("altitudeEnvelopeDynamicallyRecoverable", reference)
        self.assertIn("altitudeEnvelopeDynamicallyRecoverable", kernels)

    def test_latest_raw_lidar_validates_the_complete_finite_path(self) -> None:
        obstacle_memory = OBSTACLE_MEMORY.read_text(encoding="utf-8")
        inputs = INPUTS.read_text(encoding="utf-8")
        execution = EXECUTION.read_text(encoding="utf-8")
        finite_execution_path = FINITE_EXECUTION_PATH.read_text(encoding="utf-8")

        self.assertIn("publishLatestLidarObstacleScan", obstacle_memory)
        self.assertIn("onLatestLidarObstacleScan", inputs)
        self.assertIn("latest_lidar_obstacle_maximum_age_ms_", execution)
        self.assertIn("buildValidatedFiniteExecutionPath", execution)
        self.assertIn("validateCompleteFiniteExecutionPath", finite_execution_path)
        self.assertIn(
            "validateRawPointCloudSweptFootprint", finite_execution_path
        )
        self.assertIn(
            'return "latest_lidar_raw_collision"', finite_execution_path
        )
        self.assertRegex(
            finite_execution_path,
            r"for \(std::size_t index = 1U; index < points\.size\(\);",
        )
        self.assertNotIn("clearance_increasing", execution)

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
        launch = LAUNCH.read_text(encoding="utf-8") + MISSION_LAUNCH.read_text(
            encoding="utf-8"
        )

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
