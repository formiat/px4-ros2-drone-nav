#!/usr/bin/env python3
"""Static contracts for Stage-2 execution publication and revocation."""

from __future__ import annotations

import unittest
from pathlib import Path


REPOSITORY = Path(__file__).resolve().parents[2]
PACKAGE = REPOSITORY / "drone_city_nav"
SOURCE = PACKAGE / "src"
INCLUDE = PACKAGE / "include" / "drone_city_nav"
CONFIG = PACKAGE / "config" / "urban_mvp.yaml"
INPUTS = SOURCE / "production_mppi_node_inputs.cpp"
RAW_INPUT = SOURCE / "production_mppi_node_raw_input.cpp"
PLANNING_TICK = SOURCE / "production_mppi_node_planning_tick.cpp"
PLANNER_NODE = SOURCE / "production_mppi_node.cpp"
PLANNER_HEADER = SOURCE / "production_mppi_node.hpp"
PLANNER_MISSION = SOURCE / "production_mppi_node_mission.cpp"
EXECUTION = SOURCE / "production_mppi_node_execution.cpp"
EXECUTION_ASSEMBLER = SOURCE / "execution_horizon_assembler_3d.cpp"
EXECUTION_PUBLICATION = SOURCE / "production_mppi_node_execution_publication.cpp"
EXECUTION_HOLDS = SOURCE / "production_mppi_node_execution_holds.cpp"
EXECUTION_RETENTION = SOURCE / "production_mppi_node_execution_retention.cpp"
EXECUTION_RETENTION_TEST = (
    PACKAGE / "tests" / "execution_supervisor_retention_3d_test.cpp"
)
EXECUTION_HOLD_SERVICE = SOURCE / "execution_supervisor_3d_hold.cpp"
EXECUTION_HOLD_TEST = PACKAGE / "tests" / "execution_supervisor_hold_3d_test.cpp"
EXECUTION_HORIZON_SERVICE = SOURCE / "execution_supervisor_3d_horizon.cpp"
EXECUTION_HORIZON_TEST = (
    PACKAGE / "tests" / "execution_supervisor_horizon_3d_test.cpp"
)
OPTIONAL_CONSTRAINTS = SOURCE / "production_mppi_node_optional_constraints.cpp"
ROUTE_ACTIVATION = SOURCE / "route_activation_coordinator_3d.cpp"
ROUTE_ACTIVATION_PREPARATION = SOURCE / "route_activation_preparation_3d.cpp"
ROUTE_EXECUTION = SOURCE / "production_mppi_route_execution.cpp"
CONTROL_FEEDBACK = SOURCE / "production_mppi_node_control_feedback.cpp"
ROUTE_WORLD_TEST = PACKAGE / "tests" / "production_mppi_route_world_test.cpp"
OFFBOARD = SOURCE / "mppi_offboard_node.cpp"
COOPERATIVE_AGENT = SOURCE / "cooperative_traffic_agent_node.cpp"
COOPERATIVE_REFEREE = SOURCE / "cooperative_traffic_referee_node.cpp"
INTERCEPT_REFEREE = SOURCE / "intercept_mission_referee_node.cpp"
INTERCEPT_DIAGNOSTICS_MUX = SOURCE / "intercept_diagnostics_mux_node.cpp"
PRODUCER_INSTANCE = SOURCE / "producer_instance_id.cpp"
PRODUCER_INSTANCE_TEST = PACKAGE / "tests" / "producer_instance_id_test.cpp"
LATEST_LIDAR = SOURCE / "latest_lidar_obstacle_scan.cpp"
HORIZON_MESSAGE = PACKAGE / "msg" / "MppiTrajectoryHorizon.msg"
HORIZON_ADMISSION = SOURCE / "execution_horizon_admission.cpp"
HORIZON_CONTRACT_ROS = SOURCE / "execution_horizon_contract_ros.cpp"
HORIZON_WITNESS = SOURCE / "execution_horizon_witness.cpp"
HORIZON_WITNESS_HEADER = INCLUDE / "execution_horizon_witness.hpp"
CAPTURE_GATE_HEADER = INCLUDE / "mission_waypoint_capture_gate.hpp"
CAPTURE_GATE_TEST = PACKAGE / "tests" / "mission_waypoint_capture_gate_test.cpp"
ACK_ADMISSION = SOURCE / "mission_waypoint_acknowledgement_admission.cpp"
MISSION_MONITOR = SOURCE / "mission_monitor_node.cpp"


def read_execution_sources() -> str:
    return "\n".join(
        path.read_text(encoding="utf-8")
        for path in (
            EXECUTION,
            EXECUTION_ASSEMBLER,
            EXECUTION_PUBLICATION,
            EXECUTION_HOLDS,
            EXECUTION_RETENTION,
        )
    )


class Stage2ExecutionTransportContractTest(unittest.TestCase):
    def test_nonphysical_execution_revocation_is_optional_and_disabled(self) -> None:
        config = CONFIG.read_text(encoding="utf-8")
        optional_constraints = OPTIONAL_CONSTRAINTS.read_text(encoding="utf-8")
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        planner_node = PLANNER_NODE.read_text(encoding="utf-8")
        holds = EXECUTION_HOLDS.read_text(encoding="utf-8")

        self.assertIn("execution_nonphysical_revocation_enabled: false", config)
        self.assertIn("navigation_health_terminal_failure_enabled: false", config)
        self.assertIn(
            '"navigation_health_terminal_failure_enabled", false', planner_node
        )
        self.assertIn(
            'declare_parameter<bool>("execution_nonphysical_revocation_enabled", false)',
            optional_constraints,
        )
        request = holds.split(
            "ProductionMppiNode::requestExecutionRevocation", maxsplit=1
        )[1]
        self.assertIn(
            "!optional_constraints_.nonphysical_execution_revocation_enabled",
            request,
        )
        handler = holds.split(
            "ProductionMppiNode::handleRequestedExecutionRevocation", maxsplit=1
        )[1].split(
            "ProductionMppiNode::publishFailClosedExecutionRevocation", maxsplit=1
        )[0]
        policy_gate = handler.index(
            "!optional_constraints_.nonphysical_execution_revocation_enabled"
        )
        drain = handler.index(
            "handled_execution_revocation_request_ = requested_revocation"
        )
        resume = handler.index("return false", drain)
        self.assertLess(policy_gate, drain)
        self.assertLess(drain, resume)

        revoke = holds.split(
            "ProductionMppiNode::publishExecutionRevocation", maxsplit=1
        )[1].split(
            "ProductionMppiNode::handleRequestedExecutionRevocation", maxsplit=1
        )[0]
        self.assertIn("executionRevocationAllowed(", revoke)
        no_path = holds.split(
            "ProductionMppiNode::publishNoExecutablePathHold", maxsplit=1
        )[1].split(
            "ProductionMppiNode::publishExecutionRevocation", maxsplit=1
        )[0]
        self.assertIn("RouteLifecycleEventKind3D::kRawInvalidated", no_path)
        self.assertIn("physical_route_invalidation", no_path)
        self.assertIn(
            "navigation_health.terminal &&\n"
            "      optional_constraints_.nonphysical_execution_revocation_enabled",
            planning_tick,
        )
        self.assertIn('"replace_expired_owner"', planning_tick)

    def test_offboard_disarm_and_timestamp_rewind_fail_closed(self) -> None:
        offboard = OFFBOARD.read_text(encoding="utf-8")
        status_callback = offboard.split("void onVehicleStatus", maxsplit=1)[1].split(
            "void onLocalPosition", maxsplit=1
        )[0]
        self.assertIn("!armed && (was_armed || horizon_.has_value())", status_callback)
        disarm = status_callback.index("execution_horizon_rearm_required_ = true")
        clear = status_callback.index("horizon_.reset()", disarm)
        tombstone = status_callback.index("tombstoneExecutionHorizonIdentity", clear)
        self.assertLess(disarm, clear)
        self.assertLess(clear, tombstone)

        horizon_callback = offboard.split("void onHorizon", maxsplit=1)[1].split(
            "[[nodiscard]] bool horizonFresh", maxsplit=1
        )[0]
        candidate = horizon_callback.index(
            "const ExecutionHorizonAdmissionCandidate candidate"
        )
        payload = horizon_callback.index("assessExecutionHorizonPayload")
        rearm = horizon_callback.index("execution_horizon_rearm_required_")
        payload_admissible = horizon_callback.index("const bool payload_admissible")
        identity = horizon_callback.index("admitExecutionHorizonIdentity")
        revoke = horizon_callback.index("if (admission.revoke)")
        installable = horizon_callback.index("!admission.payload_installable")
        executable_install = horizon_callback.index("horizon_ = horizon")
        self.assertLess(candidate, payload)
        self.assertLess(payload, rearm)
        self.assertLess(rearm, payload_admissible)
        self.assertLess(payload_admissible, identity)
        self.assertLess(identity, revoke)
        self.assertLess(revoke, installable)
        self.assertLess(installable, executable_install)
        self.assertIn("payload_admissible", horizon_callback)

    def test_timestamp_rewind_revokes_without_installing_in_every_consumer(
        self,
    ) -> None:
        admission = HORIZON_ADMISSION.read_text(encoding="utf-8")
        same_producer_advance = admission.split(
            "const bool timestamps_advance", maxsplit=1
        )[1].split(
            "if (state.producerRetired", maxsplit=1
        )[0]
        self.assertIn(
            "candidate.source_stamp_ns > state.latest_source_stamp_ns",
            same_producer_advance,
        )
        self.assertIn(
            "candidate.valid_from_ns > state.latest_valid_from_ns",
            same_producer_advance,
        )
        self.assertIn("result.accept_identity = true", same_producer_advance)
        self.assertIn("result.state_advanced = true", same_producer_advance)
        self.assertIn("result.payload_installable = true", same_producer_advance)
        self.assertIn("result.revoke = true", same_producer_advance)
        self.assertIn(
            "result.next_state.current_identity_tombstoned = false",
            same_producer_advance,
        )
        self.assertIn("makeProspectiveClaim", admission)
        self.assertIn("prospective_identity_capacity_exhausted", admission)
        self.assertLess(
            admission.index("makeProspectiveClaim(candidate)"),
            admission.index("const bool handoff_timestamps_advance"),
        )
        prospective_handoff = admission.split(
            "const bool handoff_timestamps_advance", maxsplit=1
        )[1].split("claimCurrentIdentity", maxsplit=1)[0]
        self.assertIn("if (!payload_admissible)", prospective_handoff)
        self.assertIn("result.invalid = true", prospective_handoff)

        consumers = {
            "offboard": OFFBOARD.read_text(encoding="utf-8"),
            "cooperative_agent": COOPERATIVE_AGENT.read_text(encoding="utf-8"),
            "cooperative_referee": COOPERATIVE_REFEREE.read_text(encoding="utf-8"),
            "diagnostics_mux": INTERCEPT_DIAGNOSTICS_MUX.read_text(encoding="utf-8"),
            "intercept_referee": INTERCEPT_REFEREE.read_text(encoding="utf-8"),
        }
        for name, consumer in consumers.items():
            with self.subTest(consumer=name):
                self.assertIn("!admission.payload_installable", consumer)
                self.assertIn("if (admission.state_advanced)", consumer)
        self.assertEqual(
            consumers["intercept_referee"].count(
                "if (!admission.payload_installable)"
            ),
            2,
        )

    def test_malformed_feedback_revokes_matching_witness_immediately(self) -> None:
        contract = HORIZON_CONTRACT_ROS.read_text(encoding="utf-8")
        witness = HORIZON_WITNESS.read_text(encoding="utf-8")
        witness_header = HORIZON_WITNESS_HEADER.read_text(encoding="utf-8")
        feedback_assessment = contract.split(
            "assessExecutionControlFeedback", maxsplit=1
        )[1]
        source = feedback_assessment.index(
            "executionHorizonTimeNanoseconds(feedback.header.stamp)"
        )
        wire = feedback_assessment.index("controlWireFingerprint(feedback)")
        frame_gate = feedback_assessment.index("expected_frame_id.empty()")
        self.assertLess(
            feedback_assessment.index("ExecutionHorizonFeedbackCandidate"),
            frame_gate,
        )
        self.assertLess(source, frame_gate)
        self.assertLess(wire, frame_gate)
        self.assertIn("claimCurrentSource", witness)
        self.assertIn("claimProspectiveSource", witness)
        self.assertIn("wire_fingerprint", witness_header)
        self.assertIn(
            "prospective_feedback_source_capacity_exhausted", witness_header
        )

        consumers = {
            "planner": CONTROL_FEEDBACK.read_text(encoding="utf-8"),
            "cooperative_agent": COOPERATIVE_AGENT.read_text(encoding="utf-8"),
            "cooperative_referee": COOPERATIVE_REFEREE.read_text(encoding="utf-8"),
            "diagnostics_mux": INTERCEPT_DIAGNOSTICS_MUX.read_text(encoding="utf-8"),
            "intercept_referee": INTERCEPT_REFEREE.read_text(encoding="utf-8"),
        }
        for name, consumer in consumers.items():
            with self.subTest(consumer=name):
                self.assertIn("revokeMalformedExecutionHorizonFeedback", consumer)
                self.assertIn("if (!assessment.valid())", consumer)
                self.assertIn("state_advanced", consumer)
        self.assertEqual(
            consumers["intercept_referee"].count(
                "revokeMalformedExecutionHorizonFeedback"
            ),
            2,
        )

    def test_raw_invalidation_recertifies_before_retirement(self) -> None:
        retention_adapter = EXECUTION_RETENTION.read_text(encoding="utf-8")
        retention_test = EXECUTION_RETENTION_TEST.read_text(encoding="utf-8")
        route_execution = ROUTE_EXECUTION.read_text(encoding="utf-8")
        self.assertIn("execution_supervisor_.prepareRetention", retention_adapter)
        self.assertNotIn(
            "certifyRawInvalidatedFiniteExecution3DDetailed", retention_adapter
        )
        self.assertNotIn("retireCertifiedRoute3D", retention_adapter)
        self.assertNotIn(
            "rebuildFiniteExecutionPathContinuation", retention_adapter
        )
        self.assertIn(
            "RawInvalidationPreparesOnlyAnExactOwnerEmergencyBrakeTail",
            retention_test,
        )
        self.assertIn("ExecutionRoutePhase3D::kBraking", retention_test)
        self.assertIn("FiniteExecutionKind3D::kEmergencyBrakeTail", retention_test)
        evidence_derivation = route_execution.split(
            "deriveLatestObservedRouteEvidence", maxsplit=1
        )[1].split("observedRouteEvidenceIsCurrent", maxsplit=1)[0]
        self.assertIn("deriveRouteEvidence", evidence_derivation)
        self.assertNotIn("rawWorldExecutionOwnerExact", evidence_derivation)
        self.assertIn("lifecycle_observed_raw_world", retention_adapter)

    def test_route_consumers_preserve_source_ownership(self) -> None:
        observed_consumers = {
            "activation": (
                ROUTE_ACTIVATION.read_text(encoding="utf-8")
                + ROUTE_ACTIVATION_PREPARATION.read_text(encoding="utf-8")
            ),
            "execution": read_execution_sources(),
            "route_execution": ROUTE_EXECUTION.read_text(encoding="utf-8"),
        }
        for name, source in observed_consumers.items():
            with self.subTest(observed_consumer=name):
                self.assertIn("deriveRouteEvidence(", source)
                self.assertNotIn("VersionedObservedRawWorld3D::capture(", source)
                self.assertNotIn("VersionedObservedRawWorld3D::captureOwned(", source)

        static_consumers = {
            "activation": observed_consumers["activation"],
            "execution": observed_consumers["execution"],
        }
        for name, source in static_consumers.items():
            with self.subTest(static_consumer=name):
                self.assertIn("VersionedStaticWorld3D::captureOwned(", source)
                self.assertNotIn("VersionedStaticWorld3D::capture(", source)

    def test_vehicle_status_budget_covers_px4_two_hertz_cadence(self) -> None:
        planner_node = PLANNER_NODE.read_text(encoding="utf-8")
        planner_header = PLANNER_HEADER.read_text(encoding="utf-8")
        capture_header = CAPTURE_GATE_HEADER.read_text(encoding="utf-8")
        config = CONFIG.read_text(encoding="utf-8")
        capture_test = CAPTURE_GATE_TEST.read_text(encoding="utf-8")

        self.assertIn(
            'declare_parameter<double>("maximum_vehicle_status_age_ms", 1000.0)',
            planner_node,
        )
        self.assertIn("maximum_vehicle_status_age_ms_{1000.0}", planner_header)
        self.assertEqual(
            capture_header.count("maximum_vehicle_status_age_s{1.0}"), 2
        )
        self.assertIn("maximum_vehicle_status_age_ms: 1000.0", config)
        self.assertNotIn("maximum_vehicle_status_age_ms: 200.0", config)
        self.assertIn("maximum_control_feedback_age_ms: 200.0", config)
        self.assertIn(
            "AcceptsNominalTwoHertzVehicleStatusAndRejectsAfterBudget",
            capture_test,
        )
        self.assertIn(
            "nominal.vehicle_status_receive_stamp_ns = 1'500'000'000",
            capture_test,
        )

    def test_waypoint_epoch_advance_queues_old_owner_revocation(self) -> None:
        mission = PLANNER_MISSION.read_text(encoding="utf-8")
        transaction = mission.split(
            "Goal capture is an irreversible mission transition", maxsplit=1
        )[1].split("if (!update.waypoint_completed)", maxsplit=1)[0]
        advance = mission.split(
            "mission_goal_ = mission_waypoint_sequence_->activeGoal();", maxsplit=1
        )[1].split("if (topological_navigation_3d_)", maxsplit=1)[0]
        self.assertIn(
            "const std::scoped_lock lock{input_mutex_, objective_replan_mutex_}",
            transaction,
        )
        store = advance.index("navigation_objective_.store")
        clear_applied = advance.index("invalidateAppliedControlWitnessLocked();")
        request_revoke = advance.index("requestExecutionRevocation(")
        self.assertLess(store, clear_applied)
        self.assertLess(clear_applied, request_revoke)
        self.assertIn(
            "ProductionMppiExecutionReason::kNoExecutableHorizon",
            advance,
        )
        self.assertNotIn("execution_horizon_owner_ = {};", advance)

    def test_waypoint_acknowledgement_rechecks_exact_evidence_at_commit_time(
        self,
    ) -> None:
        mission = PLANNER_MISSION.read_text(encoding="utf-8")
        transaction = mission.split(
            "Goal capture is an irreversible mission transition", maxsplit=1
        )[1].split(
            "if (!update.waypoint_completed)", maxsplit=1
        )[0]
        lock = transaction.index(
            "const std::scoped_lock lock{input_mutex_, objective_replan_mutex_}"
        )
        commit_time = transaction.index(
            "const std::int64_t commit_now_ns = get_clock()->now().nanoseconds()"
        )
        acknowledgement = transaction.index("acknowledgeGoalCapture()")
        self.assertLess(lock, commit_time)
        self.assertLess(commit_time, acknowledgement)
        for exact_guard in (
            "navigation_objective_.load(std::memory_order_acquire) == objective",
            "navigation_.revision == navigation.revision",
            "vehicle_status_.revision == vehicle_status.revision",
            "execution_supervisor_.authority() == execution_authority",
            "requested_execution_revocation_.load(std::memory_order_acquire)",
            "commit_now_ns >= execution_horizon_owner.valid_from_ns",
            "commit_now_ns < execution_horizon_owner.valid_until_ns",
            "commit_now_ns - navigation_.receive_stamp_ns",
            "commit_now_ns - applied_control.source_stamp_ns",
            "commit_now_ns - applied_control.receive_stamp_ns",
        ):
            self.assertIn(exact_guard, transaction)
        self.assertIn(".stamp_ns = commit_now_ns", transaction)
        self.assertIn("acknowledgement_stamp_ns = commit_now_ns", transaction)

    def test_waypoint_capture_tracks_hidden_feedback_discontinuities(self) -> None:
        feedback = CONTROL_FEEDBACK.read_text(encoding="utf-8")
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        mission = PLANNER_MISSION.read_text(encoding="utf-8")
        capture_test = CAPTURE_GATE_TEST.read_text(encoding="utf-8")

        invalidation = feedback.split(
            "ProductionMppiNode::invalidateAppliedControlWitnessLocked", maxsplit=1
        )[1].split("ProductionMppiNode::onAppliedControl", maxsplit=1)[0]
        self.assertIn("expected->control().valid", invalidation)
        self.assertIn("clearAppliedControlIfSame(expected)", invalidation)
        self.assertIn("recordAppliedControlDiscontinuityLocked()", invalidation)
        self.assertIn("applied_control_discontinuity_generation_", invalidation)
        self.assertIn(
            "applied_control_discontinuity_generation_exhausted_", invalidation
        )
        callback = feedback.split(
            "ProductionMppiNode::onAppliedControl", maxsplit=1
        )[1]
        self.assertGreaterEqual(
            callback.count("invalidateAppliedControlWitnessLocked()"), 5
        )

        self.assertRegex(
            planning_tick,
            r"applied_control_discontinuity_generation\s*=\s*"
            r"applied_control_discontinuity_generation_",
        )
        self.assertRegex(
            mission,
            r"\.feedback_continuity_generation\s*=\s*"
            r"applied_control_discontinuity_generation",
        )
        self.assertRegex(
            mission,
            r"applied_control_discontinuity_generation_\s*==\s*"
            r"applied_control_discontinuity_generation",
        )
        self.assertIn(
            "HiddenFeedbackFallbackGenerationRestartsTheFullHold", capture_test
        )

    def test_waypoint_acknowledgement_claims_wire_identity_before_progress(
        self,
    ) -> None:
        admission = ACK_ADMISSION.read_text(encoding="utf-8")
        monitor = MISSION_MONITOR.read_text(encoding="utf-8")
        reducer = admission.split(
            "admitMissionWaypointAcknowledgement(", maxsplit=1
        )[1]
        claimable = reducer.index("candidate.identityClaimable()")
        structural = reducer.index("candidate.identityValid()")
        self.assertLess(claimable, structural)
        for invariant in (
            "last_accepted",
            "prospective_identity_claims",
            "current_source_stamp_high_water_ns",
            "current_receive_stamp_high_water_ns",
            "prospective_identity_capacity_exhausted",
            "sameAcknowledgement",
        ):
            self.assertIn(invariant, admission)

        fingerprint = monitor.split(
            "acknowledgementContentFingerprint", maxsplit=1
        )[1].split("acknowledgementRejectionReason", maxsplit=1)[0]
        for wire_field in (
            "header.stamp.sec",
            "header.stamp.nanosec",
            "header.frame_id",
            "producer_instance_id",
            "acknowledgement_sequence",
            "mission_epoch",
            "completed_waypoint_index",
            "completed_waypoint_count",
            "waypoint_count",
            "active_waypoint_index",
            "mission_completed",
            "completed_goal",
            "horizon_producer_instance_id",
            "horizon_sequence",
            "offboard_producer_instance_id",
            "horizon_valid_from",
            "horizon_valid_until",
            "witness_stamp",
            "route_target",
            "stationary_hold_position",
        ):
            self.assertIn(wire_field, fingerprint)

    def test_producer_identity_contains_process_entropy(self) -> None:
        producer = PRODUCER_INSTANCE.read_text(encoding="utf-8")
        producer_test = PRODUCER_INSTANCE_TEST.read_text(encoding="utf-8")
        latest_lidar = LATEST_LIDAR.read_text(encoding="utf-8")
        self.assertIn("::getpid()", producer)
        identity_mix = producer.split(
            "const std::uint64_t identity", maxsplit=1
        )[1].split("return identity", maxsplit=1)[0]
        self.assertIn("mixEntropy(process_id)", identity_mix)
        self.assertIn("mixEntropy(domain)", identity_mix)
        self.assertIn("invocation", identity_mix)
        self.assertIn(
            "SeparatesForkedProcessesWithInheritedClockEpoch", producer_test
        )
        self.assertIn("::fork()", producer_test)
        self.assertIn(
            "EXPECT_NE(parent_identities[index], child_identities[index])",
            producer_test,
        )
        lidar_identity = latest_lidar.split(
            "createLatestLidarObstacleProducerInstanceId", maxsplit=1
        )[1].split("} // namespace drone_city_nav", maxsplit=1)[0]
        self.assertIn(
            "createProducerInstanceId(kLatestLidarProducerDomain)", lidar_identity
        )
        self.assertNotIn("system_clock", lidar_identity)
        self.assertNotIn("invocation_sequence", lidar_identity)

    def test_intercept_horizon_transport_is_reliable(self) -> None:
        referee = INTERCEPT_REFEREE.read_text(encoding="utf-8")
        self.assertEqual(
            referee.count(
                "runtime.horizon_sub = "
                "create_subscription<msg::MppiTrajectoryHorizon>("
            ),
            2,
        )
        self.assertEqual(
            referee.count("topics.execution_horizon[index], control_feedback_qos"),
            2,
        )
        self.assertNotIn(
            "topics.execution_horizon[index], rclcpp::QoS{10}.best_effort()",
            referee,
        )

    def test_every_execution_owner_requires_current_stable_armed_status(self) -> None:
        inputs = INPUTS.read_text(encoding="utf-8")
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        publication = EXECUTION_PUBLICATION.read_text(encoding="utf-8")
        feedback = CONTROL_FEEDBACK.read_text(encoding="utf-8")
        route_world_test = ROUTE_WORLD_TEST.read_text(encoding="utf-8")

        authority = feedback.split(
            "bool vehicleStatusAuthoritativeForExecution", maxsplit=1
        )[1].split("bool appliedControlAuthoritativeForExecution", maxsplit=1)[0]
        for requirement in (
            "status.valid",
            "status.armed",
            "timestamp_epoch_stable",
            "status.source_timestamp_us == 0U",
            "status.receive_stamp_ns <= 0",
            "now_ns < status.receive_stamp_ns",
            "maximum_age_ms",
        ):
            self.assertIn(requirement, authority)

        callback = inputs.split(
            "void ProductionMppiNode::onVehicleStatus", maxsplit=1
        )[1].split("void ProductionMppiNode::onVehicleLandDetected", maxsplit=1)[0]
        probation = callback.split(
            "const bool timestamp_probation_opened", maxsplit=1
        )[1].split("if (!px4TimestampEpochAdmissionAccepted", maxsplit=1)[0]
        self.assertIn("pending_confirmation_count == 0U", probation)
        self.assertIn("admission.next_state.pending_confirmation_count != 0U", probation)
        self.assertEqual(probation.count("if (timestamp_probation_opened)"), 1)
        self.assertIn("invalidate_vehicle_status();", probation)
        self.assertIn("invalidateAppliedControlWitnessLocked();", probation)
        self.assertIn("requestExecutionRevocation", probation)
        self.assertNotIn("execution_horizon_owner_ = {};", callback)
        self.assertNotIn("publishExecutionRevocation", callback)
        conflict_identity = callback.split(
            "const bool same_identity_conflict", maxsplit=1
        )[1].split("if (same_identity_conflict)", maxsplit=1)[0]
        self.assertIn("vehicle_status_.valid", conflict_identity)
        self.assertIn("const bool disarm_transition", callback)
        self.assertIn("admission.epoch_reset || disarm_transition", callback)

        early_gate = planning_tick.index("vehicleStatusAuthoritativeForExecution(")
        waypoint_gate = planning_tick.index("updateMissionWaypoint(")
        self.assertLess(early_gate, waypoint_gate)
        early_gate_block = planning_tick[early_gate:waypoint_gate]
        self.assertIn("if (execution_horizon_owner.valid)", early_gate_block)
        self.assertIn("handleRequestedExecutionRevocation(now_ns)", early_gate_block)

        owner_commit = publication.split(
            "ProductionMppiNode::commitAndPublishExecutionHorizon", maxsplit=1
        )[1].split("ProductionMppiNode::commitExecutionSnapshotHorizon", maxsplit=1)[0]
        horizon_service = EXECUTION_HORIZON_SERVICE.read_text(encoding="utf-8")
        commit_lock = owner_commit.index("input_lock{input_mutex_}")
        status_state = owner_commit.index("const bool vehicle_status_epoch_stable")
        commit_status = owner_commit.index("vehicleStatusAuthoritativeForExecution(")
        snapshot_commit = owner_commit.index("execution_supervisor_.commitHorizon")
        dds_publish = owner_commit.index(
            "execution_horizon_pub_->publish(publication_horizon)"
        )
        self.assertLess(commit_lock, commit_status)
        self.assertLess(commit_status, snapshot_commit)
        self.assertLess(snapshot_commit, dds_publish)
        status_block = owner_commit[status_state:snapshot_commit]
        self.assertIn("!vehicle_status_epoch_probation_", status_block)
        self.assertIn("!vehicle_status_revision_exhausted_", status_block)
        self.assertIn(".vehicle_status_authoritative =", owner_commit)
        self.assertIn(
            "if (!request.runtime.vehicle_status_authoritative)", horizon_service
        )
        self.assertIn("resident_owner.valid", horizon_service)

        self.assertIn(
            "VehicleStatusAuthorityRequiresFreshStableArmedObservation",
            route_world_test,
        )

    def test_execution_evidence_publications_are_linearized(self) -> None:
        inputs = INPUTS.read_text(encoding="utf-8")
        activation = ROUTE_ACTIVATION.read_text(encoding="utf-8")
        execution = read_execution_sources()
        route_execution = ROUTE_EXECUTION.read_text(encoding="utf-8")

        lidar_producer = inputs.split(
            "void ProductionMppiNode::onLatestLidarObstacleScan", maxsplit=1
        )[1].split(
            "void ProductionMppiNode::publishRadarTrackModeCommand", maxsplit=1
        )[0]
        lidar_commit = lidar_producer.split(
            "LatestLidarEvidenceUpdateStatus3D update_status", maxsplit=1
        )[1]
        lidar_lock = lidar_commit.index(
            "const std::scoped_lock lock{latest_lidar_evidence_commit_mutex_};"
        )
        lidar_store = lidar_commit.index(
            "latest_lidar_evidence_.store(evidence, std::memory_order_release);"
        )
        self.assertLess(lidar_lock, lidar_store)
        authority_boundary = lidar_commit.index(
            "if (producer_handoff || acquisition_epoch_reset ||"
        )
        revocation = lidar_commit.index("requestExecutionRevocation", authority_boundary)
        lock_end = lidar_commit.index("\n  }\n  if (producer_handoff)", revocation)
        update_handling = lidar_commit.index("if (update_status ==", lock_end)
        self.assertLess(lidar_store, authority_boundary)
        self.assertLess(authority_boundary, revocation)
        self.assertLess(revocation, lock_end)
        self.assertLess(lock_end, update_handling)

        execution_publication = (
            EXECUTION_PUBLICATION.read_text(encoding="utf-8")
            + EXECUTION_HOLDS.read_text(encoding="utf-8")
        )
        execution_hold_service = EXECUTION_HOLD_SERVICE.read_text(encoding="utf-8")
        execution_hold_test = EXECUTION_HOLD_TEST.read_text(encoding="utf-8")
        execution_horizon_service = EXECUTION_HORIZON_SERVICE.read_text(
            encoding="utf-8"
        )
        execution_horizon_test = EXECUTION_HORIZON_TEST.read_text(encoding="utf-8")
        owner_commit = execution_publication.split(
            "ProductionMppiNode::commitAndPublishExecutionHorizon", maxsplit=1
        )[1].split("ProductionMppiNode::commitExecutionSnapshotHorizon", maxsplit=1)[0]
        owner_callback = owner_commit.index("candidate.owner = owner")
        supervisor_commit = owner_commit.index("execution_supervisor_.commitHorizon")
        owner_dds = owner_commit.index(
            "execution_horizon_pub_->publish(publication_horizon);"
        )
        self.assertLess(owner_callback, supervisor_commit)
        self.assertLess(supervisor_commit, owner_dds)
        self.assertNotIn("applied_control_", owner_commit)
        self.assertNotIn("execution_horizon_owner_", owner_commit)
        self.assertIn("execution_horizon_producer_instance_id_", owner_commit)
        self.assertIn("owner.target_offboard_instance_id", owner_commit)
        self.assertIn("assessOffboardSessionPublicationCurrentness", owner_commit)
        self.assertIn("cycle.evidence.offboard_session", owner_commit)

        snapshot_commit = execution_publication.split(
            "ProductionMppiNode::commitExecutionSnapshotHorizon", maxsplit=1
        )[1].split("ProductionMppiNode::publishPositionHold", maxsplit=1)[0]
        snapshot_lock = snapshot_commit.index(
            "const std::scoped_lock evidence_lock{execution_evidence_commit_mutex_,"
        )
        self.assertIn("latest_lidar_evidence_commit_mutex_", snapshot_commit)
        snapshot_owner_commit = snapshot_commit.index(
            "commitAndPublishExecutionHorizon("
        )
        self.assertLess(snapshot_lock, snapshot_owner_commit)
        self.assertNotIn("evidence_lock.unlock", snapshot_commit)
        self.assertNotIn("assessExecutionPublicationCurrentness3D", snapshot_commit)

        hold_commit = execution_publication.split(
            "ProductionMppiNode::publishPositionHold", maxsplit=1
        )[1].split("ProductionMppiNode::publishNoExecutablePathHold", maxsplit=1)[0]
        hold_evidence_lock = hold_commit.index(
            "const std::scoped_lock evidence_lock{execution_evidence_commit_mutex_,"
        )
        hold_lidar_capture = hold_commit.index(
            "latest_lidar_evidence_.load(std::memory_order_acquire)"
        )
        hold_preparation = hold_commit.index("execution_supervisor_.prepareHold(")
        hold_horizon = hold_commit.index("makeExecutionHorizon(")
        hold_owner_commit = hold_commit.index(
            "commitAndPublishExecutionHorizon(cycle, horizon, std::move(candidate))"
        )
        self.assertLess(hold_evidence_lock, hold_lidar_capture)
        self.assertLess(hold_lidar_capture, hold_preparation)
        self.assertLess(hold_preparation, hold_horizon)
        self.assertLess(hold_horizon, hold_owner_commit)
        self.assertIn(
            "ExecutionHorizonCommitKind3D::kTransition",
            hold_commit,
        )
        self.assertIn(
            "candidate.expected_authority = prepared.expected_authority", hold_commit
        )
        self.assertIn("ExecutionHoldIntent3D::kRefreshResident", execution)
        self.assertIn("ExecutionHoldIntent3D::kExplicitTransfer", execution)
        self.assertIn(
            "ExecutionHoldIntent3D::kExplicitTransferWithStationaryCaptureRearm",
            execution,
        )
        self.assertNotIn("transferToExecutionHold3D", hold_commit)
        self.assertNotIn("armStationaryCaptureHold3D", hold_commit)
        self.assertNotIn("StationaryExecutionHoldCertification3D", hold_commit)
        self.assertIn("transferToExecutionHold3D", execution_hold_service)
        self.assertIn("armStationaryCaptureHold3D", execution_hold_service)
        self.assertIn("manager_.authority()", execution_hold_service)
        self.assertIn("latestLidarCurrent", execution_hold_service)
        self.assertIn("stationaryCaptureWorldCurrent", execution_hold_service)
        self.assertIn(
            "StationaryCaptureRearmIsAnExplicitRevokedOwnerTransaction",
            execution_hold_test,
        )
        self.assertIn(
            "PreparedHoldCannotCommitAcrossAnAuthorityRevision",
            execution_hold_test,
        )
        self.assertIn(
            "StationaryExecutionHoldOrigin3D::kStationaryCaptureRearm",
            execution_horizon_service,
        )
        self.assertIn("!resident_owner.valid", execution_horizon_service)
        self.assertIn("!resident_control.valid", execution_horizon_service)
        self.assertIn("owner.producer_instance_id ==", execution_horizon_service)
        self.assertIn(
            "StationaryCaptureRearmIsAnExplicitRevokedOwnerTransaction",
            execution_hold_test,
        )
        self.assertIn(
            "NavigationAndControlWitnessMustOwnTheExactExecutionInput",
            execution_horizon_test,
        )
        self.assertNotIn(
            "execution_horizon_pub_->publish(publication_horizon);", hold_commit
        )

        pending_cleanup = route_execution.split(
            "pendingRoutePermanentlyObsolete", maxsplit=1
        )[1].split("} // namespace", maxsplit=1)[0]
        self.assertIn("snapshot.execution_owner_epoch", pending_cleanup)
        self.assertIn("pending.base_execution_owner_epoch", pending_cleanup)
        self.assertIn("pendingCertifiedRouteEligible3D", pending_cleanup)
        self.assertNotIn("base_hold_snapshot_version", pending_cleanup)

        route_progress_preparation = route_execution.split(
            "const ExecutionRouteTransitionResult3D advanced =", maxsplit=1
        )[1].split(
            "const std::shared_ptr<const ExecutionPlan3D>& route_state",
            maxsplit=1,
        )[0]
        route_advance = route_progress_preparation.index("advanceCertifiedRoute3D")
        route_preparation = route_progress_preparation.index(
            "result.progress_preparation ="
        )
        certification_snapshot = route_progress_preparation.index(
            "result.certification_snapshot = advanced.next"
        )
        self.assertLess(route_advance, route_preparation)
        self.assertLess(route_preparation, certification_snapshot)
        self.assertNotIn("execution_supervisor_.commitLease", route_progress_preparation)

        self.assertIn("publication_plan->publishable()", execution_horizon_service)
        self.assertIn("candidate.progress_preparation == nullptr", execution_horizon_service)

        pending_refresh = route_execution.split(
            "refreshPendingRoute", maxsplit=1
        )[1].split("RouteProgressProjection3D", maxsplit=1)[0]
        self.assertIn("recertifyExecutionRoute3D", pending_refresh)
        self.assertNotIn(
            "certifyExecutionRoute3D(ExecutionRouteActivation3D", pending_refresh
        )

    def test_revocation_is_fail_closed_current_and_reissued(self) -> None:
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        publication = (
            EXECUTION_PUBLICATION.read_text(encoding="utf-8")
            + EXECUTION_HOLDS.read_text(encoding="utf-8")
        )

        self.assertIn(
            "EXECUTION_MODE_REVOKED=2",
            HORIZON_MESSAGE.read_text(encoding="utf-8"),
        )
        self.assertGreaterEqual(
            planning_tick.count("publishFailClosedExecutionRevocation("), 8
        )
        request_barrier = planning_tick.index(
            "handleRequestedExecutionRevocation(now_ns)"
        )
        request_handler = publication.split(
            "ProductionMppiNode::handleRequestedExecutionRevocation", maxsplit=1
        )[1].split(
            "ProductionMppiNode::publishFailClosedExecutionRevocation", maxsplit=1
        )[0]
        request_load = request_handler.index(
            "requested_execution_revocation_.load(std::memory_order_acquire)"
        )
        mission_freeze = planning_tick.index("matching_goal_capture_attempt")
        self.assertLess(request_barrier, mission_freeze)
        self.assertGreaterEqual(request_load, 0)
        self.assertIn("if (revocation.published)", request_handler)
        self.assertIn("revocation_already_satisfied", request_handler)
        self.assertIn("snapshot_has_executable_authority", request_handler)
        self.assertIn("!authority->owner().valid", request_handler)
        self.assertIn("handled_execution_revocation_request_", request_handler)

        owner_commit = publication.split(
            "ProductionMppiNode::commitAndPublishExecutionHorizon", maxsplit=1
        )[1].split("ProductionMppiNode::commitExecutionSnapshotHorizon", maxsplit=1)[0]
        request_currentness = owner_commit.index(
            "requested_execution_revocation_.load(std::memory_order_acquire)"
        )
        snapshot_cas = owner_commit.index("execution_supervisor_.commitHorizon")
        dds_publish = owner_commit.index(
            "execution_horizon_pub_->publish(publication_horizon)"
        )
        self.assertLess(request_currentness, snapshot_cas)
        self.assertLess(request_currentness, dds_publish)
        snapshot_owner_commit = owner_commit.index(
            "execution_supervisor_.commitHorizon"
        )
        executable_owner_install = owner_commit.index(
            "execution_horizon_pub_->publish(publication_horizon)"
        )
        self.assertLess(snapshot_owner_commit, executable_owner_install)

        no_path = publication.split(
            "ProductionMppiNode::publishNoExecutablePathHold", maxsplit=1
        )[1].split("ProductionMppiNode::publishExecutionRevocation", maxsplit=1)[0]
        self.assertLess(
            no_path.index("retainActiveFinitePath"),
            no_path.index("publishExecutionRevocation"),
        )
        self.assertNotIn(
            ".value_or(publication)", EXECUTION.read_text(encoding="utf-8")
        )
        self.assertIn(
            "publishNoExecutablePathHold(cycle, candidate.failure_reason)",
            EXECUTION.read_text(encoding="utf-8"),
        )

        revoke = publication.split(
            "ProductionMppiNode::publishExecutionRevocation", maxsplit=1
        )[1].split("ProductionMppiNode::requestExecutionRevocation", maxsplit=1)[0]
        self.assertIn("ExecutionRouteTransitionStatus3D::kNoChange", revoke)
        session_gate = revoke.index("if (!current_session)")
        revoke_cas = revoke.index("execution_supervisor_.commitDetachedTransition")
        sequence_commit = revoke.index(
            "execution_horizon_sequence_ = revocation.sequence"
        )
        revoke_publish = revoke.index("execution_horizon_pub_->publish(revocation)")
        self.assertLess(session_gate, revoke_cas)
        self.assertLess(revoke_cas, sequence_commit)
        self.assertLess(sequence_commit, revoke_publish)
        self.assertIn("execution_supervisor_.clearLeaseIfSame", revoke)
        self.assertNotIn("execution_horizon_owner_", revoke)
        self.assertNotIn("applied_control_", revoke)
        self.assertIn(
            "offboard_session_admission_.current_producer_instance_id", revoke
        )
        self.assertIn(
            "revocation.sequence = execution_horizon_sequence_ + 1U", revoke
        )
        self.assertIn("assessExecutionHorizonPayload", revoke)
        self.assertNotIn("latest_lidar", revoke)
        self.assertNotIn("execution_input", revoke)

        request = publication.split(
            "ProductionMppiNode::requestExecutionRevocation", maxsplit=1
        )[1].split("ProductionMppiNode::publishExplicitHold", maxsplit=1)[0]
        self.assertIn("compare_exchange_weak", request)
        self.assertIn("std::memory_order_release", request)


if __name__ == "__main__":
    unittest.main()
