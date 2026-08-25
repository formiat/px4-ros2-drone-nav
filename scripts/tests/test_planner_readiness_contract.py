#!/usr/bin/env python3
"""Static contracts for planner-world and intercept-start readiness."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
SOURCE = PACKAGE / "src"
CONFIG = PACKAGE / "config" / "urban_mvp.yaml"
INPUTS = SOURCE / "production_mppi_node_inputs.cpp"
INPUTS_3D = SOURCE / "production_mppi_node_inputs_3d.cpp"
CONTROL_FEEDBACK = SOURCE / "production_mppi_node_control_feedback.cpp"
ESDF = SOURCE / "production_mppi_node_esdf.cpp"
OBSERVED_ESDF = SOURCE / "production_mppi_node_observed_esdf.cpp"
OBSERVED_EVIDENCE = SOURCE / "production_mppi_node_observed_evidence.cpp"
PLANNER = SOURCE / "production_mppi_node.cpp"
PLANNER_INTERFACES = SOURCE / "production_mppi_node_interfaces.cpp"
PLANNING_TICK = SOURCE / "production_mppi_node_planning_tick.cpp"
PLANNING_TICK_REARM = SOURCE / "production_mppi_node_planning_tick_rearm.cpp"
PLANNING_TICK_FINALIZE = SOURCE / "production_mppi_node_planning_tick_finalize.cpp"
STATIC_EXTENSION = SOURCE / "production_mppi_node_static_extension.cpp"
EXECUTION = SOURCE / "production_mppi_node_execution.cpp"
EXECUTION_PUBLICATION = SOURCE / "production_mppi_node_execution_publication.cpp"
EXECUTION_RETENTION = SOURCE / "production_mppi_node_execution_retention.cpp"
ROUTE_ACTIVATION = SOURCE / "production_mppi_route_activation.cpp"
ROUTE_EXECUTION = SOURCE / "production_mppi_route_execution.cpp"
OFFBOARD = SOURCE / "mppi_offboard_node.cpp"
COOPERATIVE_AGENT = SOURCE / "cooperative_traffic_agent_node.cpp"
COOPERATIVE_REFEREE = SOURCE / "cooperative_traffic_referee_node.cpp"
COOPERATIVE_REFEREE_LIFECYCLE = (
    SOURCE / "cooperative_traffic_referee_lifecycle.cpp"
)
MISSION_MONITOR = SOURCE / "mission_monitor_node.cpp"
PLANNER_MISSION = SOURCE / "production_mppi_node_mission.cpp"
MISSION_CAPTURE_GATE = SOURCE / "mission_waypoint_capture_gate.cpp"
MISSION_ACK_ADMISSION = SOURCE / "mission_waypoint_acknowledgement_admission.cpp"
MISSION_ACK_MESSAGE = PACKAGE / "msg" / "MissionWaypointAcknowledgement.msg"
INTERCEPT_DIAGNOSTICS_MUX = SOURCE / "intercept_diagnostics_mux_node.cpp"
OBSTACLE_MEMORY = SOURCE / "obstacle_memory_node.cpp"
HORIZON_MESSAGE = PACKAGE / "msg" / "MppiTrajectoryHorizon.msg"
HORIZON_POINT_MESSAGE = PACKAGE / "msg" / "MppiHorizonPoint.msg"
CONTROL_FEEDBACK_MESSAGE = PACKAGE / "msg" / "MppiControlFeedback.msg"
EXECUTION_EVIDENCE_HEADER = (
    PACKAGE / "include" / "drone_city_nav" / "execution_evidence_3d.hpp"
)
EXECUTION_SNAPSHOT_HEADER = (
    PACKAGE / "include" / "drone_city_nav" / "execution_route_snapshot_3d.hpp"
)
EXECUTION_SNAPSHOT_HOLD = SOURCE / "execution_route_snapshot_3d_hold.cpp"
MISSION_CAPTURE_TEST = PACKAGE / "tests" / "mission_waypoint_capture_gate_test.cpp"
HORIZON_ADMISSION = SOURCE / "execution_horizon_admission.cpp"
HORIZON_TIMING = SOURCE / "execution_horizon_timing.cpp"
HORIZON_CONTRACT_ROS = SOURCE / "execution_horizon_contract_ros.cpp"
HORIZON_CONTRACT_ROS_HEADER = (
    PACKAGE / "include" / "drone_city_nav" / "execution_horizon_contract_ros.hpp"
)
HORIZON_WITNESS = SOURCE / "execution_horizon_witness.cpp"
APPLIED_CONTROL_ADMISSION = SOURCE / "applied_control_admission.cpp"
OFFBOARD_SESSION_ADMISSION = SOURCE / "offboard_session_admission.cpp"
PENDING_CERTIFIED_ROUTE = SOURCE / "pending_certified_route_3d.cpp"
MISSION_WAYPOINT_SEQUENCE = SOURCE / "mission_waypoint_sequence.cpp"
SPEED_POLICY = PACKAGE / "include" / "drone_city_nav" / "mppi_speed_policy.hpp"
FINITE_HORIZON_HEADER = (
    PACKAGE / "include" / "drone_city_nav" / "mppi" / "mppi_finite_horizon.hpp"
)
STOPPING_CAPABILITY = (
    PACKAGE / "include" / "drone_city_nav" / "stopping_capability.hpp"
)
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


def read_execution_sources() -> str:
    return "\n".join(
        path.read_text(encoding="utf-8")
        for path in (EXECUTION, EXECUTION_PUBLICATION, EXECUTION_RETENTION)
    )


class PlannerReadinessContractTest(unittest.TestCase):
    def test_execution_horizon_carries_typed_route_purpose(self) -> None:
        text = HORIZON_MESSAGE.read_text(encoding="utf-8")
        point = HORIZON_POINT_MESSAGE.read_text(encoding="utf-8")
        feedback = CONTROL_FEEDBACK_MESSAGE.read_text(encoding="utf-8")
        self.assertIn("uint8 ROUTE_PURPOSE_MISSION_TRANSIT=0", text)
        self.assertIn("uint8 ROUTE_PURPOSE_LAUNCH_DEPARTURE=1", text)
        self.assertIn("uint8 ROUTE_PURPOSE_OBSERVATION_FRONTIER=2", text)
        self.assertIn("uint8 ROUTE_PURPOSE_TOPOLOGICAL_BACKTRACK=3", text)
        self.assertIn("uint8 route_purpose", text)
        self.assertIn("geometry_msgs/Point route_target", text)
        self.assertIn("uint64 producer_instance_id", text)
        self.assertIn("uint64 target_offboard_instance_id", text)
        self.assertIn("int64 control_interval_ns", text)
        self.assertIn("int64 time_from_start_ns", point)
        self.assertIn("float32 yaw_acceleration_radps2", point)
        self.assertIn("uint64 producer_instance_id", feedback)
        self.assertIn("uint64 horizon_producer_instance_id", feedback)
        self.assertIn("uint64 horizon_sequence", feedback)

    def test_static_esdf_bootstrap_keeps_lidar_as_execution_evidence(self) -> None:
        inputs = INPUTS.read_text(encoding="utf-8")
        esdf = ESDF.read_text(encoding="utf-8")
        lidar_callback = inputs.split(
            "void ProductionMppiNode::onLatestLidarObstacleScan", maxsplit=1
        )[1].split(
            "void ProductionMppiNode::publishRadarTrackModeCommand", maxsplit=1
        )[0]

        self.assertNotIn("if (use_static_map_) {\n    return;\n  }", lidar_callback)
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

    def test_observed_esdf_refresh_preserves_an_active_route(self) -> None:
        observed_esdf = OBSERVED_ESDF.read_text(encoding="utf-8")
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        extension = STATIC_EXTENSION.read_text(encoding="utf-8")

        self.assertIn(
            "prepared.global_guide_generation == 0U", observed_esdf
        )
        self.assertIn('"active_route_preserved"', observed_esdf)
        self.assertIn("initial_route_search_already_pending", observed_esdf)
        self.assertNotIn("dropped_guide_worlds_", observed_esdf)
        self.assertIn("use_static_map_ || observed_3d_world", planning_tick)
        self.assertIn("observed_world", extension)
        self.assertIn('"observed_resident_esdf"', extension)
        self.assertIn(
            "Lattice3DRoutePurpose::kLaunchDeparture", extension
        )
        self.assertRegex(
            planning_tick,
            r"guide_progress_tracker_\s*&&\s*!direct_tracking_interception\s*&&\s*"
            r"route_purpose\s*!=\s*Lattice3DRoutePurpose::kLaunchDeparture\s*&&\s*"
            r"route_purpose\s*!=\s*"
            r"Lattice3DRoutePurpose::kObservationFrontier",
        )

    def test_missing_executable_route_holds_without_a_clearance_gate(self) -> None:
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        execution = read_execution_sources()
        route_execution = ROUTE_EXECUTION.read_text(encoding="utf-8")
        offboard = OFFBOARD.read_text(encoding="utf-8")
        planner = PLANNER.read_text(encoding="utf-8")
        horizon_message = HORIZON_MESSAGE.read_text(encoding="utf-8")
        speed_policy = SPEED_POLICY.read_text(encoding="utf-8")
        finite_horizon_header = FINITE_HORIZON_HEADER.read_text(encoding="utf-8")
        stopping_capability = STOPPING_CAPABILITY.read_text(encoding="utf-8")
        finite_horizon = FINITE_HORIZON.read_text(encoding="utf-8")
        finite_execution_path = FINITE_EXECUTION_PATH.read_text(encoding="utf-8")
        horizon_contract = HORIZON_CONTRACT_ROS.read_text(encoding="utf-8")

        self.assertIn("kNoExecutableRouteHold", planning_tick)
        self.assertIn("route_hold_position = route_execution.hold_position", planning_tick)
        self.assertIn(".hold_position =", route_execution)
        self.assertIn("active_usable = true", route_execution)
        self.assertIn("result.route_usable = true", route_execution)
        self.assertIn("temporary_frontier_is_terminal", planning_tick)
        self.assertIn("ProductionMppiExecutionReason::kNoExecutableRoute", execution)
        self.assertIn(
            "ProductionMppiExecutionReason::kNoExecutableHorizon", execution
        )
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_ROUTE=4", horizon_message)
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_HORIZON=1", horizon_message)
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_ROUTE", offboard)
        self.assertIn("EXECUTION_REASON_NO_EXECUTABLE_HORIZON", offboard)
        self.assertIn("transferToExecutionHold3D", execution)
        self.assertIn("publishExecutionRevocation", execution)
        self.assertIn("stationary_hold.has_value()", execution)
        self.assertIn("action=hold_no_executable_path", execution)
        self.assertIn("publishPositionHold", execution)
        self.assertRegex(
            planner,
            r"stationary_hold_validity_ns_\s*=\s*durationNanoseconds\(",
        )
        hold_publication = execution.split(
            "ProductionMppiNode::publishPositionHold", maxsplit=1
        )[1].split("ProductionMppiNode::publishNoExecutablePathHold", maxsplit=1)[0]
        hold_overflow_guard = hold_publication.index(
            "std::numeric_limits<std::int64_t>::max() / 2"
        )
        hold_interval_double = hold_publication.index(
            "2 * cycle.finite_path_control_interval_ns"
        )
        hold_canonical_end = hold_publication.index(
            "canonicalHorizonEndTime(cycle.now_ns,"
        )
        self.assertLess(hold_overflow_guard, hold_interval_double)
        self.assertLess(hold_interval_double, hold_canonical_end)
        self.assertRegex(
            execution,
            r"canonicalHorizonEndTime\(\s*now_ns,\s*\*terminal_offset_ns\s*\)",
        )
        self.assertNotIn("ProductionMppiExecutionMode::kBraking", execution)
        self.assertNotIn("EXECUTION_MODE_BRAKING", horizon_message)
        self.assertNotIn("dynamicStopRequested", offboard)
        self.assertIn("publishUnavailablePathHoldSetpoint", offboard)
        self.assertIn("plannedFinitePathCompleted", offboard)
        self.assertIn("buildMppiPathTrajectorySetpoint", offboard)
        self.assertIn("buildValidatedFiniteExecutionPath", execution)
        self.assertIn("buildFiniteHorizon", finite_execution_path)
        self.assertIn("finiteHorizonHasTerminalRestState", finite_horizon)
        self.assertNotIn(
            "finite_path_arrival_maximum_horizontal_deceleration_mps2", planner
        )
        self.assertRegex(
            planner,
            r"makeFiniteHorizonConfig\(\s*"
            r"speed_policy_config_\.stopping_capability\)",
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
        self.assertIn("finite_horizon_config_", execution)
        self.assertIn("guaranteed_horizontal_deceleration_mps2", finite_horizon)
        self.assertIn("validateFiniteExecutionTrajectoryContinuation", execution)
        self.assertIn("validateFiniteExecutionPathContinuation", execution)
        self.assertIn("actual_state_validation", execution)
        self.assertIn("rebased_from_actual=true", execution)
        self.assertIn("retainSnapshotFinitePath", execution)
        self.assertIn("retainDirectFinitePath", execution)
        self.assertIn("FiniteExecutionPathTerminalBoundary", execution)
        self.assertIn("original_valid_until_ns", execution)
        self.assertIn("assessExecutionHorizonPayload", offboard)
        self.assertIn("kMissingTerminalRestState", horizon_contract)
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
        callback = inputs.split(
            "void ProductionMppiNode::onLatestLidarObstacleScan", maxsplit=1
        )[1].split("ProductionMppiNode::navigationObjective", maxsplit=1)[0]
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        execution = read_execution_sources()
        finite_execution_path = FINITE_EXECUTION_PATH.read_text(encoding="utf-8")

        self.assertIn("publishLatestLidarObstacleScan", obstacle_memory)
        self.assertIn("onLatestLidarObstacleScan", inputs)
        self.assertIn(
            "VersionedLatestLidarEvidence3D::capture(std::move(capture))", inputs
        )
        claim = callback.index("claimLatestLidarEvidenceIdentity3D")
        for payload_gate in (
            "const LidarProjectionBodyFrame frame",
            "const bool valid_counts",
            "for (const geometry_msgs::msg::Point32& point",
            "VersionedLatestLidarEvidence3D::capture",
        ):
            self.assertLess(claim, callback.index(payload_gate))
        self.assertIn("latestLidarRawWireFingerprint", inputs)
        self.assertIn("admitClaimedLatestLidarEvidence3D", inputs)
        self.assertIn("latest_lidar_evidence_.load", planning_tick)
        self.assertNotIn("captureLatestLidarEvidence", planning_tick)
        self.assertNotIn("latest_lidar_obstacle_scan_.load", planning_tick)
        self.assertIn("latest_lidar_evidence", execution)
        self.assertIn("latest_lidar_obstacle_maximum_age_ms_", execution)
        self.assertIn("lidar_validation_now_ns", execution)
        self.assertIn("latestLidarEvidenceFreshness", execution)
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

    def test_stage2_revocation_and_fallback_never_forge_hold_feedback(self) -> None:
        offboard = OFFBOARD.read_text(encoding="utf-8")
        stationary_active = offboard.split(
            "stationaryPositionHoldActive", maxsplit=1
        )[1].split("currentSpeedMps", maxsplit=1)[0]
        self.assertIn("horizonFresh()", stationary_active)

        stationary = offboard.split(
            "[[nodiscard]] bool publishStationaryPositionHoldSetpoint()", maxsplit=1
        )[1].split("publishLaunchDepartureSetpoint", maxsplit=1)[0]
        setpoint = stationary.index("setpoint_pub_->publish")
        exact_feedback = stationary.index("publishAppliedControlFeedback")
        self.assertLess(setpoint, exact_feedback)
        self.assertIn("EXECUTION_MODE_POSITION_HOLD", stationary)

        unavailable = offboard.split(
            "void publishUnavailableControlFeedback", maxsplit=1
        )[1].split("mapAltitudeM", maxsplit=1)[0]
        self.assertIn("publishOffboardSessionHeartbeat();", unavailable)
        self.assertNotIn("publishAppliedControlFeedback", unavailable)

        horizon_callback = offboard.split("void onHorizon", maxsplit=1)[1].split(
            "[[nodiscard]] bool horizonFresh", maxsplit=1
        )[0]
        revoked = horizon_callback.index("EXECUTION_MODE_REVOKED")
        clear = horizon_callback.index("horizon_.reset()", revoked)
        executable_install = horizon_callback.index("horizon_ = horizon")
        self.assertLess(revoked, clear)
        self.assertLess(clear, executable_install)

    def test_stage2_prestart_receipt_unblocks_start_without_control_authority(
        self,
    ) -> None:
        offboard = OFFBOARD.read_text(encoding="utf-8")
        cooperative_referee = COOPERATIVE_REFEREE.read_text(encoding="utf-8")
        intercept_referee = REFEREE.read_text(encoding="utf-8")
        cooperative_agent = COOPERATIVE_AGENT.read_text(encoding="utf-8")
        diagnostics_mux = INTERCEPT_DIAGNOSTICS_MUX.read_text(encoding="utf-8")
        launch = LAUNCH.read_text(encoding="utf-8")
        mission_launch = MISSION_LAUNCH.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")

        control_tick = offboard.split("void controlTick()", maxsplit=1)[1].split(
            "void publishTakeoffSetpoint()", maxsplit=1
        )[0]
        self.assertNotIn("publishOffboardSessionHeartbeat();", control_tick)
        takeoff = control_tick.index("publishTakeoffSetpoint();")
        waiting_for_start = control_tick.index(
            "takeoff_ready && require_mission_start_signal_ && !mission_started_"
        )
        receipt = control_tick.index("publishPrestartPlannedHorizonReceipt();")
        heartbeat_gate = control_tick.index("if (!exact_horizon_feedback_published)")
        self.assertLess(takeoff, waiting_for_start)
        self.assertLess(waiting_for_start, receipt)
        self.assertLess(receipt, heartbeat_gate)
        self.assertIn(
            "exact_horizon_feedback_published = "
            "publishPrestartPlannedHorizonReceipt();",
            control_tick,
        )

        prestart_receipt = offboard.split(
            "[[nodiscard]] bool publishPrestartPlannedHorizonReceipt()", maxsplit=1
        )[1].split(
            "[[nodiscard]] bool publishStationaryPositionHoldSetpoint()", maxsplit=1
        )[0]
        self.assertIn("if (!plannedFinitePathFresh())", prestart_receipt)
        self.assertIn("publishAppliedControlFeedback", prestart_receipt)
        self.assertIn("Point2{}, 0.0, 0.0, 0.0, false", prestart_receipt)
        self.assertIn("EXECUTION_MODE_PLANNED", prestart_receipt)
        self.assertNotIn("publishOffboardSessionHeartbeat", prestart_receipt)

        for consumer in (cooperative_referee, intercept_referee):
            self.assertIn("executionHorizonReceiptFreshAt", consumer)
            self.assertIn("executionHorizonWitnessFreshAt", consumer)
            self.assertIn("mission_started_ ? strict_witness : receipt", consumer)
        self.assertIn("executionHorizonWitnessFreshAt", cooperative_agent)
        self.assertIn("executionHorizonReceiptFreshAt", cooperative_agent)
        self.assertIn(
            "require_mission_start_signal_ && !mission_started_",
            cooperative_agent,
        )
        self.assertIn("!strict_witness && !prestart_receipt", cooperative_agent)
        self.assertIn("mission_started_ = mission_started_ || start->data", cooperative_agent)
        self.assertIn("executionHorizonWitnessFreshAt", diagnostics_mux)
        self.assertNotIn("executionHorizonReceiptFreshAt", diagnostics_mux)

        self.assertIn('"require_mission_start_signal": True', launch)
        cooperative_agents = mission_launch.split(
            "def make_cooperative_mission_nodes", maxsplit=1
        )[1]
        self.assertIn('"require_mission_start_signal": True', cooperative_agents)
        self.assertIn('"mission_start_topic": f"{prefix}/mission_start"', cooperative_agents)
        self.assertIn("require_mission_start_signal: false", config)

    def test_stage2_horizon_identity_timing_and_feedback_are_exact(self) -> None:
        execution = read_execution_sources()
        offboard = OFFBOARD.read_text(encoding="utf-8")
        inputs = INPUTS.read_text(encoding="utf-8")
        control_feedback = CONTROL_FEEDBACK.read_text(encoding="utf-8")
        observed_esdf = OBSERVED_ESDF.read_text(encoding="utf-8")
        observed_evidence = OBSERVED_EVIDENCE.read_text(encoding="utf-8")
        evidence = EXECUTION_EVIDENCE_HEADER.read_text(encoding="utf-8")
        horizon_admission = HORIZON_ADMISSION.read_text(encoding="utf-8")
        horizon_timing = HORIZON_TIMING.read_text(encoding="utf-8")
        horizon_contract = HORIZON_CONTRACT_ROS.read_text(encoding="utf-8")
        horizon_witness = HORIZON_WITNESS.read_text(encoding="utf-8")
        control_admission = APPLIED_CONTROL_ADMISSION.read_text(encoding="utf-8")
        session_admission = OFFBOARD_SESSION_ADMISSION.read_text(encoding="utf-8")

        execution_publication = EXECUTION_PUBLICATION.read_text(encoding="utf-8")
        append_points = execution_publication.split(
            "bool appendFiniteExecutionPoints(", maxsplit=1
        )[1].split("} // namespace", maxsplit=1)[0]
        self.assertIn(
            "index == 0U ? previous_applied_control : controls[index - 1U]",
            append_points,
        )
        self.assertIn("point.time_from_start_ns", append_points)
        self.assertIn("point.yaw_acceleration_radps2 = point_control.yaw_accel", append_points)
        self.assertNotIn("controls[std::min(index", append_points)

        horizon_callback = offboard.split("void onHorizon", maxsplit=1)[1].split(
            "[[nodiscard]] bool horizonFresh", maxsplit=1
        )[0]
        target_check = horizon_callback.index("horizon.target_offboard_instance_id")
        identity_candidate = horizon_callback.index(
            "const ExecutionHorizonAdmissionCandidate candidate"
        )
        payload_validation = horizon_callback.index(
            "const ExecutionHorizonPayloadStatus payload_status"
        )
        payload_admissibility = horizon_callback.index(
            "const bool payload_admissible"
        )
        admission_call = horizon_callback.index("admitExecutionHorizonIdentity")
        admission_install = horizon_callback.index("horizon_admission_ =")
        revocation = horizon_callback.index("if (admission.revoke)")
        self.assertLess(target_check, identity_candidate)
        self.assertLess(identity_candidate, payload_validation)
        self.assertLess(payload_validation, payload_admissibility)
        self.assertLess(payload_admissibility, admission_call)
        self.assertLess(admission_call, admission_install)
        self.assertLess(admission_install, revocation)
        self.assertIn("assessExecutionHorizonPayload", horizon_callback)
        self.assertIn("executionHorizonContentFingerprint", horizon_callback)
        self.assertIn("payload_admissible", horizon_callback)
        self.assertIn("admission.conflict", horizon_callback)
        self.assertIn("executionHorizonBracketAt", offboard)
        self.assertIn("first.yaw_acceleration_radps2", offboard)
        self.assertIn("second.yaw_acceleration_radps2", offboard)
        self.assertIn("publishOffboardSessionHeartbeat();", offboard)
        self.assertIn("horizon_producer_instance_id = 0U", offboard)

        self.assertIn("producer_instance_id", horizon_admission)
        self.assertIn("producerRetired", horizon_admission)
        self.assertIn("appendRetiredProducer", horizon_admission)
        self.assertIn("current_content_fingerprint", horizon_admission)
        self.assertIn("current_identity_tombstoned", horizon_admission)
        self.assertIn("executionHorizonTerminalOffsetNs", horizon_timing)
        self.assertIn("remainder_ns == 0", horizon_timing)
        self.assertIn("validExactTiming", horizon_contract)
        self.assertIn("validRevocationTiming", horizon_contract)
        self.assertIn("executionHorizonContentFingerprint", horizon_contract)
        self.assertIn("kInconsistentRevocation", horizon_contract)
        self.assertIn("time_from_start_ns != expected_ns", horizon_contract)
        self.assertIn("kInconsistentStationaryHold", horizon_contract)
        self.assertIn("assessExecutionControlFeedback", horizon_contract)
        self.assertIn("admitAppliedControlEvidence", control_feedback)
        self.assertIn("if (admission.session_transitioned)", control_feedback)
        self.assertIn("if (!admission.replay)", control_feedback)
        self.assertIn(
            "admitExecutionHorizonFeedbackPayload(", control_admission
        )
        self.assertIn(
            "result.install_control = owner.matches(candidate)", control_admission
        )
        self.assertIn(
            "result.revoke_control = !result.install_control", control_admission
        )
        self.assertIn("if (admission.state_advanced)", control_feedback)
        self.assertIn("producerRetired", session_admission)
        self.assertNotIn("pop_front", session_admission)

        self.assertIn(
            "previous_control_source_producer_instance_id", evidence
        )
        self.assertIn(
            "previousControlSourceProducerInstanceId", execution
        )
        self.assertIn("authoritativeBodyAxisForExecution", observed_evidence)
        self.assertIn("free_space_seed.has_value()", observed_evidence)
        self.assertNotIn(
            "value_or(FootprintBodyAxis{})", observed_evidence
        )

    def test_stage2_every_horizon_consumer_validates_before_authority_handoff(
        self,
    ) -> None:
        cooperative_agent = COOPERATIVE_AGENT.read_text(encoding="utf-8")
        cooperative_referee = COOPERATIVE_REFEREE.read_text(encoding="utf-8")
        cooperative_referee_lifecycle = COOPERATIVE_REFEREE_LIFECYCLE.read_text(
            encoding="utf-8"
        )
        mission_monitor = MISSION_MONITOR.read_text(encoding="utf-8")
        intercept_referee = REFEREE.read_text(encoding="utf-8")
        diagnostics_mux = INTERCEPT_DIAGNOSTICS_MUX.read_text(encoding="utf-8")

        callbacks = {
            "cooperative_agent": cooperative_agent.split(
                "void onExecutionHorizon", maxsplit=1
            )[1].split("[[nodiscard]] bool inputFresh", maxsplit=1)[0],
            "cooperative_referee": cooperative_referee.split(
                "void CooperativeTrafficRefereeNode::onExecutionHorizon",
                maxsplit=1,
            )[1].split(
                "void CooperativeTrafficRefereeNode::configureGroundTruthBoundary",
                maxsplit=1,
            )[0],
            "intercept_interceptor": intercept_referee.split(
                "void InterceptMissionRefereeNode::onInterceptorExecutionHorizon",
                maxsplit=1,
            )[1].split(
                "void InterceptMissionRefereeNode::onTargetExecutionHorizon",
                maxsplit=1,
            )[0],
            "intercept_target": intercept_referee.split(
                "void InterceptMissionRefereeNode::onTargetExecutionHorizon",
                maxsplit=1,
            )[1].split(
                "void InterceptMissionRefereeNode::configureGroundTruthBoundary",
                maxsplit=1,
            )[0],
            "diagnostics_mux": diagnostics_mux.split(
                "void onExecutionHorizon", maxsplit=1
            )[1].split("void expireExecutionHorizons", maxsplit=1)[0],
        }
        for name, callback in callbacks.items():
            with self.subTest(consumer=name):
                identity_candidate = callback.index(
                    "const ExecutionHorizonAdmissionCandidate candidate"
                )
                payload = callback.index("assessExecutionHorizonPayload")
                payload_admissibility = callback.index(
                    "const bool payload_admissible"
                )
                admission = callback.index("admitExecutionHorizonIdentity")
                state_advance = callback.index("admission.next_state")
                revocation = callback.index("if (admission.revoke)")
                self.assertLess(identity_candidate, payload)
                self.assertLess(payload, payload_admissibility)
                self.assertLess(payload_admissibility, admission)
                self.assertLess(admission, state_advance)
                self.assertLess(state_advance, revocation)

        executable_installs = {
            "cooperative_agent": "execution_horizon_ = horizon;",
            "cooperative_referee": "vehicle.hold_horizon = HoldHorizon{",
            "intercept_interceptor": "interceptor.accepted_horizon = AcceptedHorizon{",
            "intercept_target": "target.accepted_horizon = AcceptedHorizon{",
            "diagnostics_mux": "diagnostics.execution_horizon = horizon;",
        }
        for name, callback in callbacks.items():
            with self.subTest(explicit_revocation=name):
                revoked = callback.index("EXECUTION_MODE_REVOKED")
                revoked_return = callback.index("return;", revoked)
                executable_install = callback.index(executable_installs[name])
                self.assertLess(revoked, revoked_return)
                self.assertLess(revoked_return, executable_install)

        self.assertNotIn("void onExecutionHorizon", mission_monitor)
        self.assertNotIn("MppiTrajectoryHorizon", mission_monitor)

        self.assertIn("activeAt(now_ns)", cooperative_referee_lifecycle)
        self.assertIn(
            "activeAt(now_ns)", REFEREE_LIFECYCLE.read_text(encoding="utf-8")
        )
        self.assertIn("expireExecutionHorizons", diagnostics_mux)
        self.assertIn("horizonStrictlyWitnessed", diagnostics_mux)

    def test_stage2_execution_feedback_requires_live_exact_witness(self) -> None:
        inputs = INPUTS.read_text(encoding="utf-8")
        control_feedback = CONTROL_FEEDBACK.read_text(encoding="utf-8")
        execution = read_execution_sources()
        witness = HORIZON_WITNESS.read_text(encoding="utf-8")
        contract = HORIZON_CONTRACT_ROS.read_text(encoding="utf-8")
        contract_header = HORIZON_CONTRACT_ROS_HEADER.read_text(encoding="utf-8")
        mission = MISSION_MONITOR.read_text(encoding="utf-8")
        planner_mission = PLANNER_MISSION.read_text(encoding="utf-8")
        capture_gate = MISSION_CAPTURE_GATE.read_text(encoding="utf-8")
        ack_admission = MISSION_ACK_ADMISSION.read_text(encoding="utf-8")
        ack_message = MISSION_ACK_MESSAGE.read_text(encoding="utf-8")
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        cooperative_agent = COOPERATIVE_AGENT.read_text(encoding="utf-8")
        cooperative_referee = COOPERATIVE_REFEREE.read_text(encoding="utf-8")
        intercept_referee = REFEREE.read_text(encoding="utf-8")
        diagnostics_mux = INTERCEPT_DIAGNOSTICS_MUX.read_text(encoding="utf-8")
        multi_vehicle_launch = LAUNCH.read_text(encoding="utf-8")

        self.assertIn("feedbackAuthorityCoherent", witness)
        self.assertIn("executionHorizonReceiptFreshAt", witness)
        self.assertIn("executionHorizonWitnessFreshAt", witness)
        self.assertIn("result.witness_revoked = true", witness)
        self.assertIn("result.next_state.latest_horizon_feedback = {}", witness)
        self.assertIn("latest_session_receive_stamp_ns", witness)
        self.assertIn("assessExecutionControlFeedback", contract)
        self.assertIn("canonicalTime(feedback.header.stamp)", contract)
        self.assertIn("admissionCandidateValid", contract_header)
        self.assertIn("admitExecutionHorizonFeedbackPayload", witness)
        self.assertIn("return admitFeedback(state, candidate, payload_valid)", witness)
        self.assertIn("controlWireFingerprint", contract)
        feedback_assessment = contract.split(
            "assessExecutionControlFeedback", maxsplit=1
        )[1]
        self.assertLess(
            feedback_assessment.index(
                "executionHorizonTimeNanoseconds(feedback.header.stamp)"
            ),
            feedback_assessment.index("expected_frame_id.empty()"),
        )

        applied_callback = control_feedback.split(
            "void ProductionMppiNode::onAppliedControl", maxsplit=1
        )[1].split(
            "} // namespace drone_city_nav", maxsplit=1
        )[0]
        self.assertIn("assessExecutionControlFeedback", applied_callback)
        self.assertIn("if (!assessment.valid())", applied_callback)
        self.assertIn("revokeMalformedExecutionHorizonFeedback", applied_callback)
        self.assertIn("control_payload_valid", applied_callback)
        self.assertIn("admitAppliedControlEvidence", applied_callback)
        self.assertIn("if (!admission.replay)", applied_callback)
        self.assertIn("feedback.source_stamp_ns >", applied_callback)
        self.assertIn(
            "offboard_session_admission_.latest_source_stamp_ns",
            applied_callback,
        )
        authority_check = control_feedback.split(
            "bool appliedControlAuthoritativeForExecution", maxsplit=1
        )[1].split(
            "std::optional<FootprintBodyAxis>", maxsplit=1
        )[0]
        self.assertIn("source_age_ms", authority_check)
        self.assertIn("receive_age_ms", authority_check)

        vehicle_status_callback = inputs.split(
            "void ProductionMppiNode::onVehicleStatus", maxsplit=1
        )[1].split(
            "void ProductionMppiNode::onVehicleLandDetected", maxsplit=1
        )[0]
        timestamp_admission = vehicle_status_callback.index(
            "admitPx4TimestampEpoch("
        )
        timestamp_state_advance = vehicle_status_callback.index(
            "vehicle_status_timestamp_admission_ = admission.next_state"
        )
        payload_acceptance = vehicle_status_callback.index(
            "px4TimestampEpochAdmissionAccepted(admission.status)"
        )
        self.assertLess(timestamp_admission, timestamp_state_advance)
        self.assertLess(timestamp_state_advance, payload_acceptance)
        self.assertIn("std::chrono::steady_clock::now()", vehicle_status_callback)
        self.assertIn(
            ".receive_timestamp_ns = monotonic_receive_stamp_ns",
            vehicle_status_callback,
        )
        self.assertIn("vehicle_status_epoch_probation_", vehicle_status_callback)
        self.assertIn(
            "vehicle_status_timestamp_admission_.pending_confirmation_count != 0U",
            vehicle_status_callback,
        )
        self.assertIn("same_identity_conflict", vehicle_status_callback)
        conflict = vehicle_status_callback.split(
            "const bool same_identity_conflict", maxsplit=1
        )[1].split("if (same_identity_conflict)", maxsplit=1)[0]
        self.assertIn("vehicle_status_.valid", conflict)
        self.assertIn("vehicle_status_.valid = false", vehicle_status_callback)
        self.assertIn(
            "vehicle_status_.revision == std::numeric_limits<std::uint64_t>::max()",
            vehicle_status_callback,
        )
        self.assertIn("vehicle_status_revision_exhausted_", vehicle_status_callback)
        self.assertIn("requestExecutionRevocation", vehicle_status_callback)
        self.assertNotIn("publishExecutionRevocation", vehicle_status_callback)
        self.assertIn("vehicle_status_epoch_stable", planner_mission)
        self.assertIn("vehicle_status_epoch_probation_", planning_tick)
        self.assertIn("vehicle_status_revision_exhausted_", planning_tick)

        target_selection = execution.split(
            "std::uint64_t target_offboard_instance_id", maxsplit=1
        )[1].split(
            "if (target_offboard_instance_id == 0U)", maxsplit=1
        )[0]
        owner_commit = EXECUTION_PUBLICATION.read_text(encoding="utf-8").split(
            "ProductionMppiNode::commitAndPublishExecutionHorizon", maxsplit=1
        )[1].split("ProductionMppiNode::publishLegacyExecutionHorizon", maxsplit=1)[0]
        for section in (target_selection, owner_commit):
            self.assertIn(
                "offboard_session_admission_.latest_source_stamp_ns", section
            )
            self.assertIn("offboard_session_receive_stamp_ns_", section)

        self.assertIn("MissionWaypointCaptureGate", planner_mission)
        self.assertIn("navigation.state.vz", planner_mission)
        self.assertIn("vehicle_status.armed", planner_mission)
        self.assertIn("EXECUTION_REASON_GOAL_CAPTURE", planner_mission)
        self.assertIn("feedback_horizon_sequence", planner_mission)
        self.assertIn("acknowledgeGoalCapture", planner_mission)
        self.assertIn("mission_waypoint_acknowledgement_pub_->publish", planner_mission)
        continuity_loss = planner_mission.split(
            "if (capture.continuity_broken)", maxsplit=1
        )[1].split("if (!capture.ready", maxsplit=1)[0]
        self.assertIn("requestExecutionRevocation", continuity_loss)
        self.assertNotIn("execution_horizon_owner_ = {}", continuity_loss)
        epoch_overflow_guard = planner_mission.index(
            "objective->mission_epoch == std::numeric_limits<std::uint64_t>::max()"
        )
        sequence_advance = planner_mission.index("acknowledgeGoalCapture()")
        self.assertLess(epoch_overflow_guard, sequence_advance)
        self.assertIn("total_speed_mps", capture_gate)
        self.assertIn("maximum_vehicle_status_age_ns_", capture_gate)
        self.assertIn("distance3D(observation.stationary_hold_position", capture_gate)
        gate = planning_tick.index("updateMissionWaypoint(")
        previous_control = planning_tick.index(
            "!execution_input_preparation.previous_control_available"
        )
        self.assertLess(gate, previous_control)
        self.assertIn("matching_goal_capture_attempt", planning_tick)
        self.assertIn("if (mission_goal_capture_attempt_invalidated_)", planning_tick)
        self.assertIn("Freeze one committed lease/identity", planning_tick)
        self.assertIn("transient_local", mission)
        self.assertIn("admitMissionWaypointAcknowledgement", mission)
        self.assertNotIn("onExecutionHorizon", mission)
        self.assertNotIn("onControlFeedback", mission)
        self.assertIn("state_advanced", ack_admission)
        self.assertIn("producerRetired", ack_admission)
        self.assertIn("aggregate_replay", ack_admission)
        self.assertIn("uint64 acknowledgement_sequence", ack_message)
        self.assertIn("uint64 horizon_producer_instance_id", ack_message)
        self.assertIn("geometry_msgs/Point stationary_hold_position", ack_message)

        self.assertIn("executionHorizonWitnessFreshAt", cooperative_agent)
        self.assertIn("assessExecutionControlFeedback", cooperative_agent)
        for consumer in (
            cooperative_agent,
            cooperative_referee,
            diagnostics_mux,
        ):
            self.assertIn("if (!assessment.valid())", consumer)
            self.assertIn("revokeMalformedExecutionHorizonFeedback", consumer)
            self.assertIn("admitExecutionHorizonFeedbackPayload", consumer)
        self.assertEqual(intercept_referee.count("if (!assessment.valid())"), 2)
        self.assertEqual(
            intercept_referee.count("revokeMalformedExecutionHorizonFeedback"), 2
        )
        self.assertEqual(
            intercept_referee.count("admitExecutionHorizonFeedbackPayload"), 2
        )
        self.assertIn(
            '"mission_waypoint_acknowledgement_topic": (', multi_vehicle_launch
        )
        self.assertIn(
            'f"{prefix}/mission_waypoint_acknowledgement"', multi_vehicle_launch
        )
        for consumer in (
            cooperative_referee,
            intercept_referee,
            diagnostics_mux,
        ):
            self.assertIn("assessExecutionControlFeedback", consumer)
            self.assertIn("executionHorizonWitnessFreshAt", consumer)
            self.assertIn("admission.witness_revoked", consumer)

    def test_mission_acknowledgement_topics_match_single_and_multi_vehicle_owners(
        self,
    ) -> None:
        planner_interfaces = PLANNER_INTERFACES.read_text(encoding="utf-8")
        mission_monitor = MISSION_MONITOR.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")
        multi_vehicle_launch = LAUNCH.read_text(encoding="utf-8")
        mission_launch = MISSION_LAUNCH.read_text(encoding="utf-8")

        default_topic = "/drone_city_nav/mission_waypoint_acknowledgement"
        self.assertIn(default_topic, planner_interfaces)
        self.assertIn(default_topic, mission_monitor)
        self.assertEqual(
            config.count(
                f"mission_waypoint_acknowledgement_topic: {default_topic}"
            ),
            2,
        )
        planner_parameters = multi_vehicle_launch.split(
            "planner_params =", maxsplit=1
        )[1].split("offboard_params =", maxsplit=1)[0]
        self.assertIn(
            '"mission_waypoint_acknowledgement_topic": (', planner_parameters
        )
        self.assertIn(
            'f"{prefix}/mission_waypoint_acknowledgement"', planner_parameters
        )
        self.assertNotIn("mission_waypoint_acknowledgement_topic", mission_launch)

    def test_goal_capture_rearms_only_from_revoked_stationary_evidence(
        self,
    ) -> None:
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        planning_tick_rearm = PLANNING_TICK_REARM.read_text(encoding="utf-8")
        publication = EXECUTION_PUBLICATION.read_text(encoding="utf-8")
        evidence = EXECUTION_EVIDENCE_HEADER.read_text(encoding="utf-8")
        snapshot_header = EXECUTION_SNAPSHOT_HEADER.read_text(encoding="utf-8")
        snapshot_hold = EXECUTION_SNAPSHOT_HOLD.read_text(encoding="utf-8")
        gate = MISSION_CAPTURE_GATE.read_text(encoding="utf-8")
        gate_test = MISSION_CAPTURE_TEST.read_text(encoding="utf-8")

        revoke_barrier = planning_tick.index("handleRequestedExecutionRevocation")
        rearm_gate = planning_tick.index(
            "stationaryCaptureRearmEligibleForPlanningTick"
        )
        previous_control_gate = planning_tick.index(
            "!execution_input_preparation.previous_control_available"
        )
        self.assertLess(revoke_barrier, rearm_gate)
        self.assertLess(rearm_gate, previous_control_gate)
        self.assertIn("executionSnapshotRevokedEmpty", planning_tick_rearm)
        self.assertIn(
            "observedWorldCurrentForStationaryRearm", planning_tick_rearm
        )
        self.assertIn("VersionedStaticWorld3D::captureOwned", planning_tick_rearm)
        self.assertIn(
            "assessLatestLidarEvidenceFreshness3D", planning_tick_rearm
        )
        self.assertIn("offboard_session->valid()", planning_tick_rearm)
        self.assertIn("vehicle_status_epoch_stable", planning_tick_rearm)
        self.assertIn(
            "kStationaryExecutionHoldPositionToleranceM", planning_tick_rearm
        )
        self.assertIn(
            "kStationaryExecutionHoldSpeedToleranceMps", planning_tick_rearm
        )
        self.assertIn(
            "kStationaryExecutionHoldYawRateToleranceRadps", planning_tick_rearm
        )
        self.assertEqual(
            planning_tick_rearm.count(
                "ExecutionInputPurpose3D::kStationaryCaptureRearm"
            ),
            1,
        )
        self.assertEqual(
            planning_tick_rearm.count(
                "ExecutionPreviousControlEvidenceSource3D::kAssumedZero"
            ),
            1,
        )
        self.assertIn(
            "else if (stationary_capture_rearm)", planning_tick_rearm
        )

        self.assertIn("enum class ExecutionInputPurpose3D", evidence)
        self.assertIn("stationaryCaptureStateAuthoritative", evidence)
        self.assertIn("armStationaryCaptureHold3D", snapshot_header)
        self.assertIn("current.phase != ExecutionRoutePhase3D::kRevoked", snapshot_hold)
        self.assertIn("stationaryHoldPointSafe(certification, true)", snapshot_hold)
        self.assertIn("armStationaryCaptureHold3D", publication)
        self.assertIn("StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm", publication)
        self.assertIn("!execution_horizon_owner_.valid", publication)
        self.assertIn("!applied_control_.valid", publication)

        self.assertIn("StationaryRearmRequiresEveryAuthorityAndEvidenceGate", gate_test)
        self.assertIn("StationaryRearmRequiresFreshExactStoppedStateAtGoal", gate_test)
        for required_gate in (
            "execution_input_state_authoritative",
            "position_velocity_authoritative",
            "yaw_rate_authoritative",
            "vehicle_status_epoch_stable",
            "offboard_session_valid",
            "applied_control_empty",
            "horizon_owner_empty",
            "execution_snapshot_revoked_empty",
            "validation_policy_current",
            "world_evidence_current",
            "lidar_evidence_current",
        ):
            self.assertIn(f"!observation.{required_gate}", gate)

    def test_planners_publish_latched_world_readiness(self) -> None:
        planner = "\n".join(
            path.read_text(encoding="utf-8")
            for path in (PLANNER, PLANNER_INTERFACES)
        )
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
