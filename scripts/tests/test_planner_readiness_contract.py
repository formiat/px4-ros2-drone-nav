#!/usr/bin/env python3
"""Static contracts for planner-world and intercept-start readiness."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
SOURCE = PACKAGE / "src"
EXECUTION_RUNTIME = SOURCE / "execution"
MPPI_RUNTIME = SOURCE / "runtime"
ROS_RUNTIME = MPPI_RUNTIME / "ros"
CONFIG = PACKAGE / "config" / "urban_mvp.yaml"
OBSERVED_ESDF = ROS_RUNTIME / "production_mppi_node_observed_esdf.cpp"
PLANNER = ROS_RUNTIME / "production_mppi_node.cpp"
PLANNER_INTERFACES = ROS_RUNTIME / "production_mppi_node_interfaces.cpp"
PLANNER_CONFIG_HEADER = MPPI_RUNTIME / "production_mppi_config.hpp"
PLANNER_CONFIG_LOADER = ROS_RUNTIME / "production_mppi_config_ros.cpp"
PLANNING_TICK = ROS_RUNTIME / "production_mppi_node_planning_tick.cpp"
PLANNING_COORDINATOR = MPPI_RUNTIME / "planning_cycle_coordinator_3d.cpp"
PLANNING_COORDINATOR_TEST = (
    PACKAGE / "tests" / "planning_cycle_coordinator_3d_test.cpp"
)
STATIC_EXTENSION = ROS_RUNTIME / "production_mppi_node_static_extension.cpp"
EXECUTION = ROS_RUNTIME / "production_mppi_node_execution.cpp"
EXECUTION_ASSEMBLER = MPPI_RUNTIME / "execution_horizon_assembler_3d.cpp"
EXECUTION_PUBLICATION = ROS_RUNTIME / "production_mppi_node_execution_publication.cpp"
EXECUTION_HOLDS = ROS_RUNTIME / "production_mppi_node_execution_holds.cpp"
EXECUTION_RETENTION = ROS_RUNTIME / "production_mppi_node_execution_retention.cpp"
EXECUTION_RETENTION_SERVICE = SOURCE / "execution_supervisor_3d_retention.cpp"
EXECUTION_HOLD_SERVICE = SOURCE / "execution_supervisor_3d_hold.cpp"
EXECUTION_HOLD_TEST = PACKAGE / "tests" / "execution_supervisor_hold_3d_test.cpp"
ROUTE_EXECUTION = EXECUTION_RUNTIME / "production_mppi_route_execution.cpp"
OFFBOARD = SOURCE / "mppi_offboard_node.cpp"
OFFBOARD_NAMES = SOURCE / "mppi_offboard_node_names.hpp"
MISSION_MONITOR = SOURCE / "mission_monitor_node.cpp"
HORIZON_MESSAGE = PACKAGE / "msg" / "MppiTrajectoryHorizon.msg"
HORIZON_POINT_MESSAGE = PACKAGE / "msg" / "MppiHorizonPoint.msg"
CONTROL_FEEDBACK_MESSAGE = PACKAGE / "msg" / "MppiControlFeedback.msg"
HORIZON_CONTRACT_ROS = SOURCE / "execution_horizon_contract_ros.cpp"
SPEED_POLICY = PACKAGE / "include" / "drone_city_nav" / "mppi_speed_policy.hpp"
FINITE_HORIZON_HEADER = (
    PACKAGE / "include" / "drone_city_nav" / "control_contracts_3d.hpp"
)
STOPPING_CAPABILITY = (
    PACKAGE / "include" / "drone_city_nav" / "stopping_capability.hpp"
)
MPPI_REFERENCE = SOURCE / "mppi" / "mppi_reference.cpp"
MPPI_KERNELS = SOURCE / "mppi" / "mppi_engine_kernels.cuh"
FINITE_HORIZON = SOURCE / "finite_motion_horizon_3d.cpp"
FINITE_EXECUTION_PATH = SOURCE / "finite_execution_path_3d.cpp"
REFEREE = SOURCE / "intercept_mission_referee_node.cpp"
REFEREE_LIFECYCLE = SOURCE / "intercept_mission_referee_lifecycle.cpp"
REFEREE_SUPPORT = SOURCE / "intercept_referee_support.cpp"
ASSIGNMENT_COORDINATOR = SOURCE / "target_assignment_coordinator_node.cpp"
LAUNCH = PACKAGE / "launch" / "multi_vehicle.launch.py"
MISSION_LAUNCH = PACKAGE / "launch" / "multi_vehicle_mission_launch.py"


def read_execution_sources() -> str:
    return "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            EXECUTION,
            EXECUTION_ASSEMBLER,
            EXECUTION_PUBLICATION,
            EXECUTION_HOLDS,
            EXECUTION_RETENTION,
            EXECUTION_RETENTION_SERVICE,
        )
    )


class PlannerReadinessContractTest(unittest.TestCase):

    def test_execution_horizon_omits_retired_route_classification(self) -> None:
        text = HORIZON_MESSAGE.read_text(encoding="utf-8")
        point = HORIZON_POINT_MESSAGE.read_text(encoding="utf-8")
        feedback = CONTROL_FEEDBACK_MESSAGE.read_text(encoding="utf-8")
        self.assertNotIn("ROUTE_PURPOSE_", text)
        self.assertNotIn("uint8 route_purpose", text)
        self.assertIn("geometry_msgs/Point route_target", text)
        self.assertIn("uint64 producer_instance_id", text)
        self.assertIn("uint64 target_offboard_instance_id", text)
        self.assertIn("int64 control_interval_ns", text)
        self.assertIn("int64 time_from_start_ns", point)
        self.assertIn("float32 yaw_acceleration_radps2", point)
        self.assertIn("uint64 producer_instance_id", feedback)
        self.assertIn("uint64 horizon_producer_instance_id", feedback)
        self.assertIn("uint64 horizon_sequence", feedback)

    def test_static_esdf_is_not_expired_by_lidar_time(self) -> None:
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")

        self.assertRegex(
            planning_tick,
            r"if \(world\)\s*\{\s*"
            r"esdf_age_ms\s*=\s*config_\.world\.use_static_map\s*\?\s*0\.0",
        )

    def test_observed_esdf_refresh_preserves_an_active_route(self) -> None:
        observed_esdf = OBSERVED_ESDF.read_text(encoding="utf-8")
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        planning_cycle = PLANNING_COORDINATOR.read_text(encoding="utf-8")
        extension = STATIC_EXTENSION.read_text(encoding="utf-8")

        self.assertIn(
            "const bool initial_route_search_required = resident_route_generation == 0U",
            observed_esdf,
        )
        self.assertIn("execution_supervisor_.plan()", observed_esdf)
        self.assertIn('"active_route_preserved"', observed_esdf)
        self.assertNotIn("dropped_route_planning_worlds_", observed_esdf)
        self.assertIn("request_route_extension", planning_tick)
        self.assertIn(
            "request.use_static_map || request.observed_3d_world", planning_cycle
        )
        self.assertIn("observed_world", extension)
        self.assertIn('"observed_resident_esdf"', extension)
        self.assertNotIn("Lattice3DRoutePurpose", extension)
        self.assertIn(
            "route_progress_tracker_ != nullptr && "
            "!request.direct_tracking_interception",
            planning_cycle,
        )

    def test_missing_executable_route_holds_without_a_clearance_gate(self) -> None:
        planning_cycle = PLANNING_COORDINATOR.read_text(encoding="utf-8")
        planning_cycle_test = PLANNING_COORDINATOR_TEST.read_text(encoding="utf-8")
        execution = read_execution_sources()
        route_execution = ROUTE_EXECUTION.read_text(encoding="utf-8")
        offboard = OFFBOARD.read_text(encoding="utf-8") + OFFBOARD_NAMES.read_text(
            encoding="utf-8"
        )
        planner = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (PLANNER, PLANNER_CONFIG_LOADER)
        )
        horizon_message = HORIZON_MESSAGE.read_text(encoding="utf-8")
        speed_policy = SPEED_POLICY.read_text(encoding="utf-8")
        finite_horizon_header = FINITE_HORIZON_HEADER.read_text(encoding="utf-8")
        stopping_capability = STOPPING_CAPABILITY.read_text(encoding="utf-8")
        finite_horizon = FINITE_HORIZON.read_text(encoding="utf-8")
        finite_execution_path = FINITE_EXECUTION_PATH.read_text(encoding="utf-8")
        horizon_contract = HORIZON_CONTRACT_ROS.read_text(encoding="utf-8")
        execution_hold_service = EXECUTION_HOLD_SERVICE.read_text(encoding="utf-8")
        execution_hold_test = EXECUTION_HOLD_TEST.read_text(encoding="utf-8")

        self.assertIn("kNoExecutableRouteHold", planning_cycle)
        self.assertIn(
            "const Point3& hold = output.route.execution.hold_position",
            planning_cycle,
        )
        self.assertIn(
            "NoActiveRouteProducesOneTypedStationaryControllerCycle",
            planning_cycle_test,
        )
        self.assertIn(".hold_position =", route_execution)
        self.assertIn("active_usable = true", route_execution)
        self.assertIn("result.route_usable = true", route_execution)
        self.assertIn("local_stop_is_terminal", planning_cycle)
        self.assertIn("ProductionMppiExecutionReason::kNoExecutableRoute", execution)
        self.assertIn(
            "ProductionMppiExecutionReason::kNoExecutableHorizon", execution
        )
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_ROUTE=4", horizon_message)
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_HORIZON=1", horizon_message)
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_ROUTE", offboard)
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_HORIZON", offboard)
        self.assertIn("execution_supervisor_.prepareHold", execution)
        self.assertNotIn("transferToExecutionHold3D", execution)
        self.assertIn("transferToExecutionHold3D", execution_hold_service)
        self.assertIn(
            "RefreshesAResidentInputAndUsesNoChangeOnlyForTheExactOwnerEvidence",
            execution_hold_test,
        )
        self.assertIn("publishExecutionRevocation", execution)
        self.assertIn("stationaryHold() != nullptr", execution)
        self.assertIn("action=hold_no_executable_path", execution)
        self.assertIn("publishPositionHold", execution)
        self.assertRegex(
            planner,
            r"execution\.stationary_hold_validity_ns\s*=\s*durationNanoseconds\(",
        )
        self.assertIn(
            "committed_valid_until_ns = committed_finite->valid_until_ns", execution
        )
        self.assertNotIn("terminal_offset_ns", execution)
        self.assertNotIn("ProductionMppiExecutionMode::kBraking", execution)
        self.assertNotIn("EXECUTION_MODE_BRAKING", horizon_message)
        self.assertNotIn("dynamicStopRequested", offboard)
        self.assertIn("publishUnavailablePathHoldSetpoint", offboard)
        self.assertIn("plannedFinitePathCompleted", offboard)
        self.assertIn("buildMppiPathTrajectorySetpoint", offboard)
        self.assertIn("buildValidatedFiniteExecutionPath", execution)
        self.assertIn("buildFiniteMotionHorizon3D", finite_execution_path)
        self.assertIn("finiteMotionHorizonHasTerminalRestState3D", finite_horizon)
        self.assertNotIn(
            "finite_path_arrival_maximum_horizontal_deceleration_mps2", planner
        )
        self.assertRegex(
            planner,
            r"makeFiniteHorizonConfig\(\s*"
            r"control\.speed_policy\.stopping_capability\)",
        )
        self.assertIn("StoppingCapability stopping_capability", speed_policy)
        self.assertIn("StoppingCapability stopping_capability", finite_horizon_header)
        self.assertIn(
            "maximum_commanded_horizontal_deceleration_mps2", stopping_capability
        )
        self.assertIn(
            "guaranteed_horizontal_deceleration_mps2", stopping_capability
        )
        self.assertIn("guaranteed_vertical_deceleration_mps2", stopping_capability)
        self.assertIn('"maximum_horizontal_acceleration_mps2"', planner)
        self.assertIn(
            '"guaranteed_vertical_stopping_deceleration_mps2"', planner
        )
        self.assertIn(
            "altitude_envelope.guaranteed_vertical_deceleration_mps2", planner
        )
        self.assertIn("config_.execution.finite_horizon", execution)
        self.assertIn("guaranteed_horizontal_deceleration_mps2", finite_horizon)
        self.assertIn("validateFiniteExecutionTrajectoryContinuation", execution)
        self.assertIn("validateFiniteExecutionPathContinuation", execution)
        self.assertIn("actual_state_validation", execution)
        self.assertIn("rebuildFiniteExecutionPathContinuation", execution)
        self.assertIn("recertified=true", execution)
        self.assertIn("prepareRouteRetention", execution)
        self.assertIn("prepareDirectRetention", execution)
        self.assertIn("execution_supervisor_.prepareRetention", execution)
        self.assertIn("FiniteExecutionPathTerminalBoundary", execution)
        self.assertNotIn("original_valid_until_ns", execution)
        self.assertIn("assessExecutionHorizonPayload", offboard)
        self.assertIn("kMissingTerminalRestState", horizon_contract)
        self.assertNotIn("route_free_", planning_cycle)
        self.assertNotIn("temporary_frontier_continuation_ready", planning_cycle)
        self.assertNotIn("route_endpoint_terminal_speed_mps", speed_policy)
        self.assertNotRegex(
            planning_cycle,
            r"clearance[^\n]*kNoExecutableRouteHold|"
            r"kNoExecutableRouteHold[^\n]*clearance",
        )

    def test_cpu_and_cuda_enforce_dynamic_altitude_recoverability(self) -> None:
        reference = MPPI_REFERENCE.read_text(encoding="utf-8")
        kernels = MPPI_KERNELS.read_text(encoding="utf-8")

        self.assertIn("altitudeEnvelopeDynamicallyRecoverable", reference)
        self.assertIn("altitudeEnvelopeDynamicallyRecoverable", kernels)

    def test_mission_acknowledgement_topics_match_single_and_multi_vehicle_owners(
        self,
    ) -> None:
        planner_interfaces = PLANNER_INTERFACES.read_text(encoding="utf-8")
        planner_config = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (PLANNER_CONFIG_HEADER, PLANNER_CONFIG_LOADER)
        )
        mission_monitor = MISSION_MONITOR.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")
        multi_vehicle_launch = LAUNCH.read_text(encoding="utf-8")
        mission_launch = MISSION_LAUNCH.read_text(encoding="utf-8")

        default_topic = "/drone_city_nav/mission_waypoint_acknowledgement"
        self.assertIn(default_topic, planner_config)
        self.assertIn(default_topic, mission_monitor)
        self.assertEqual(
            config.count(
                f"mission_waypoint_acknowledgement_topic: {default_topic}"
            ),
            2,
        )
        self.assertIn(
            '"mission_waypoint_acknowledgement_topic": (', multi_vehicle_launch
        )
        self.assertIn(
            'f"{prefix}/mission_waypoint_acknowledgement"', multi_vehicle_launch
        )
        self.assertNotIn("mission_waypoint_acknowledgement_topic", mission_launch)

    def test_planners_publish_latched_world_readiness(self) -> None:
        planner = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (PLANNER, PLANNER_INTERFACES, PLANNER_CONFIG_LOADER)
        )
        launch = LAUNCH.read_text(encoding="utf-8")

        self.assertIn('"world_readiness_topic"', planner)
        self.assertIn("reliable().transient_local()", planner)
        self.assertIn('f"{prefix}/mppi/world_ready"', launch)
        self.assertIn(
            '"navigation_readiness_topic": f"{prefix}/navigation_ready"',
            launch,
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

        self.assertIn("missionReady(const std::int64_t now_ns) const", referee)
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
