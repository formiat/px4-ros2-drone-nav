#!/usr/bin/env python3

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "drone_city_nav"
SOURCE = PACKAGE / "src"
NAVIGATION_INPUT = SOURCE / "production_mppi_node_navigation_input.cpp"
INPUTS = SOURCE / "production_mppi_node_inputs.cpp"
RAW_INPUT = SOURCE / "production_mppi_node_raw_input.cpp"
PLANNING_TICK = SOURCE / "production_mppi_node_planning_tick.cpp"
EXECUTION_PUBLICATION = SOURCE / "production_mppi_node_execution_publication.cpp"
NODE_HEADER = SOURCE / "production_mppi_node.hpp"
RAW_2D = SOURCE / "raw_obstacle_delta.cpp"
RAW_3D = SOURCE / "raw_obstacle_3d_ros.cpp"
PRODUCER_ADMISSION = SOURCE / "producer_epoch_admission.cpp"
CMAKE = PACKAGE / "CMakeLists.txt"


class ExecutionInputContractTest(unittest.TestCase):
    def test_navigation_identity_and_frame_resets_fail_closed(self) -> None:
        navigation_input = NAVIGATION_INPUT.read_text(encoding="utf-8")
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        node_header = NODE_HEADER.read_text(encoding="utf-8")
        callback = navigation_input.split(
            "void ProductionMppiNode::onLocalPosition", maxsplit=1
        )[1]
        rejected_navigation = callback.split(
            "if (!navigationAngularUpdateAccepted", maxsplit=1
        )[1].split("navigation.source_timestamp_us", maxsplit=1)[0]

        self.assertIn(
            ".receive_timestamp_ns = monotonic_receive_stamp_ns", callback
        )
        self.assertIn("std::chrono::steady_clock::now()", callback)
        self.assertIn(
            ".source_payload_fingerprint = source_payload_fingerprint", callback
        )
        self.assertIn("navigationPayloadFingerprint(message)", callback)
        for field in (
            "message.x",
            "message.y",
            "message.z",
            "message.delta_xy[0]",
            "message.delta_xy[1]",
            "message.delta_z",
            "message.vx",
            "message.vy",
            "message.vz",
            "message.delta_vxy[0]",
            "message.delta_vxy[1]",
            "message.delta_vz",
            "message.ax",
            "message.ay",
            "message.az",
            "message.heading",
            "message.delta_heading",
            "message.xy_reset_counter",
            "message.z_reset_counter",
            "message.vxy_reset_counter",
            "message.vz_reset_counter",
            "message.heading_reset_counter",
            "message.xy_valid",
            "message.z_valid",
            "message.v_xy_valid",
            "message.v_z_valid",
            "message.heading_good_for_control",
        ):
            self.assertIn(field, navigation_input)

        duplicate_return = callback.split(
            "kIdempotentSourceIdentityDuplicate", maxsplit=1
        )[1].split(
            "if (angular_derivative.source_identity_conflicted)", maxsplit=1
        )[0]
        self.assertIn("return;", duplicate_return)
        identity_conflict = callback.split(
            "if (angular_derivative.source_identity_conflicted)", maxsplit=1
        )[1].split("if (!navigationAngularUpdateAccepted", maxsplit=1)[0]
        self.assertIn("navigation_.valid = false", identity_conflict)
        self.assertIn("applied_control_ = {}", identity_conflict)
        self.assertIn("requestExecutionRevocation", identity_conflict)
        self.assertNotIn("execution_horizon_owner_ = {}", identity_conflict)

        self.assertIn("return;", rejected_navigation)
        self.assertNotIn("navigation_ =", rejected_navigation)
        self.assertIn("kRejectedTimestampEpochResetPending", rejected_navigation)
        self.assertIn("kRejectedTimestampReacquisitionPending", rejected_navigation)
        self.assertIn(
            "const bool navigation_was_valid = navigation_.valid",
            rejected_navigation,
        )
        self.assertIn("navigation_.valid = false", rejected_navigation)
        self.assertIn("applied_control_ = {}", rejected_navigation)
        self.assertIn("if (navigation_was_valid)", rejected_navigation)
        self.assertIn("requestExecutionRevocation", rejected_navigation)
        self.assertNotIn("execution_horizon_owner_ = {}", rejected_navigation)
        self.assertIn("assessNavigationLocalStateReset", callback)
        for counter in (
            "xy_reset_counter",
            "z_reset_counter",
            "vxy_reset_counter",
            "vz_reset_counter",
            "heading_reset_counter",
        ):
            self.assertIn(counter, callback)
        self.assertIn("state_reset.frame_compensation_required", callback)
        self.assertIn("navigation_frame_reset_unresolved_ = true", callback)
        self.assertIn("execution_lineage_discontinuity", callback)
        self.assertIn("applied_control_ = {}", callback)
        self.assertIn("requestExecutionRevocation", callback)
        self.assertNotIn("publishExecutionRevocation", callback)
        self.assertNotIn("execution_horizon_owner_ = {}", callback)
        self.assertIn("!navigation_frame_reset_unresolved_", callback)
        self.assertIn("navigation_frame_reset_unresolved_", node_header)
        self.assertIn("!navigation_frame_reset_unresolved_", planning_tick)

        revision_exhaustion = callback.index(
            "navigation_.revision == std::numeric_limits<std::uint64_t>::max()"
        )
        first_revision_advance = callback.index("navigation_.revision + 1U")
        self.assertLess(revision_exhaustion, first_revision_advance)
        exhausted_latch = callback.split(
            "if (navigation_revision_exhausted_)", maxsplit=1
        )[1].split(
            "if (navigation_.revision ==", maxsplit=1
        )[0]
        self.assertIn("return;", exhausted_latch)
        self.assertNotIn("requestExecutionRevocation", exhausted_latch)
        self.assertIn("navigation_revision_exhausted_ = true", callback)
        revision_exhaustion_block = callback.split(
            "if (navigation_.revision ==", maxsplit=1
        )[1].split("if (!authoritative_state_contract)", maxsplit=1)[0]
        self.assertIn("navigation_.valid = false", revision_exhaustion_block)
        self.assertIn("!navigation_revision_exhausted_", planning_tick)

    def test_lidar_epoch_boundaries_use_deferred_revocation_after_admission(
        self,
    ) -> None:
        inputs = INPUTS.read_text(encoding="utf-8")
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        callback = inputs.split(
            "void ProductionMppiNode::onLatestLidarObstacleScan", maxsplit=1
        )[1].split(
            "void ProductionMppiNode::publishRadarTrackModeCommand", maxsplit=1
        )[0]
        admission_state = callback.index(
            "latest_lidar_evidence_admission_state_ = admission.next_state"
        )
        evidence_install = callback.index("if (admission.install_candidate)")
        revocation_request = callback.index(
            "if (producer_handoff || acquisition_epoch_reset ||"
        )
        self.assertLess(admission_state, evidence_install)
        self.assertLess(evidence_install, revocation_request)
        evidence_commit = callback.split(
            "const std::scoped_lock lock{latest_lidar_evidence_commit_mutex_};",
            maxsplit=1,
        )[1].split("\n  }\n  if (producer_handoff)", maxsplit=1)[0]
        self.assertIn("requestExecutionRevocation", evidence_commit)
        self.assertIn(
            "latest_lidar_evidence_identity_conflicted_.store(", evidence_commit
        )
        self.assertIn("admission.current_identity_conflict", evidence_commit)
        self.assertNotIn("publishExecutionRevocation", callback)
        self.assertIn(
            "latest_lidar_evidence_identity_conflicted_.load(", planning_tick
        )
        self.assertIn("? nullptr", planning_tick)

    def test_raw_world_epoch_and_wire_identity_are_linearized(self) -> None:
        raw_input = RAW_INPUT.read_text(encoding="utf-8")
        raw_2d = RAW_2D.read_text(encoding="utf-8")
        raw_3d = RAW_3D.read_text(encoding="utf-8")
        planning_tick = PLANNING_TICK.read_text(encoding="utf-8")
        publication = EXECUTION_PUBLICATION.read_text(encoding="utf-8")
        cmake = CMAKE.read_text(encoding="utf-8")

        self.assertIn("src/production_mppi_node_raw_input.cpp", cmake)
        self.assertNotIn("publishExecutionRevocation", raw_input)
        for callback in (
            "onRawObstacleSnapshot",
            "onRawObstacleDelta",
            "onRawObstacleSnapshot3D",
            "onRawObstacleDelta3D",
        ):
            body = raw_input.split(
                f"void ProductionMppiNode::{callback}", maxsplit=1
            )[1].split("\n}\n", maxsplit=1)[0]
            self.assertIn("execution_evidence_commit_mutex_", body)
            self.assertIn("raw_world_identity_conflicted_", body)
            self.assertIn("invalidateAppliedControlWitnessLocked", body)
            self.assertIn("requestExecutionRevocation", body)

        status = raw_input.split(
            "void ProductionMppiNode::onMemoryStatus", maxsplit=1
        )[1]
        status_admission = status.index("latest_observation_tracker_.observe(")
        status_sync = status.index("synchronizeProducerEpoch(")
        status_revoke = status.index("requestExecutionRevocation(")
        self.assertLess(status_admission, status_sync)
        self.assertLess(status_sync, status_revoke)
        self.assertIn("pending_raw_world_update_", status)
        self.assertIn("statusAnnouncesRawUpdate(message)", status)
        pending_join = status.split("if (!installed_through_status)", maxsplit=1)[
            1
        ].split("} else {", maxsplit=1)[0]
        self.assertNotIn("clear_current_raw", pending_join)
        self.assertNotIn("requestExecutionRevocation", pending_join)

        for source, snapshot_fingerprint, delta_fingerprint in (
            (
                raw_2d,
                "rawObstacleSnapshotWireFingerprint(snapshot)",
                "rawObstacleDeltaWireFingerprint(delta)",
            ),
            (
                raw_3d,
                "rawObstacleSnapshot3DWireFingerprint(snapshot)",
                "rawObstacleDelta3DWireFingerprint(delta)",
            ),
        ):
            self.assertIn(snapshot_fingerprint, source)
            self.assertIn(delta_fingerprint, source)
            self.assertIn("pending.full_snapshot", source)
            self.assertIn("prospective_identity_capacity_exhausted_", source)
            self.assertIn("prospectiveSequenceHighWater", source)
            self.assertIn("const ProspectiveIdentity selected_pending", source)
            self.assertIn(
                ".evidence_observation = selected_pending.observation", source
            )

        planning_raw_gate = planning_tick.split(
            "double observation_age_ms", maxsplit=1
        )[1].split("const bool observation_fresh", maxsplit=1)[0]
        self.assertIn("committedRawWorldAgeMs", planning_raw_gate)
        self.assertIn("latest_raw_world_3d", planning_raw_gate)
        self.assertIn("!raw_world_identity_conflicted", planning_raw_gate)
        self.assertNotIn("latest_observation.ageMs", planning_raw_gate)

        commit = publication.split(
            "ProductionMppiNode::commitAndPublishExecutionHorizon", maxsplit=1
        )[1].split("ProductionMppiNode::publishLegacyExecutionHorizon", maxsplit=1)[0]
        input_lock = commit.index("input_lock{input_mutex_}")
        raw_currentness = commit.index("committed_world_current")
        snapshot_commit = commit.index("switch (commit.kind)")
        self.assertLess(input_lock, raw_currentness)
        self.assertLess(raw_currentness, snapshot_commit)
        self.assertIn("committedRawWorldAgeMs", commit)
        self.assertIn("cycle.esdf.producer_instance_id", commit)

    def test_producer_claims_raise_exact_once_sequence_high_water(self) -> None:
        admission = PRODUCER_ADMISSION.read_text(encoding="utf-8")
        self.assertGreaterEqual(admission.count("claimed_sequence_high_water"), 2)
        self.assertGreaterEqual(admission.count("first_receive_stamp_ns"), 2)
        self.assertIn("sameRejectedEvidenceIdentity", admission)
        self.assertIn("full_snapshot == full_snapshot", admission)

    def test_vehicle_status_conflict_revocation_is_one_shot_after_exhaustion(
        self,
    ) -> None:
        inputs = INPUTS.read_text(encoding="utf-8")
        callback = inputs.split(
            "void ProductionMppiNode::onVehicleStatus", maxsplit=1
        )[1].split("void ProductionMppiNode::publishRadarTrackModeCommand", maxsplit=1)[0]
        predicate = callback.split("const bool same_identity_conflict", maxsplit=1)[1]
        predicate = predicate.split(";", maxsplit=1)[0]
        self.assertIn("!vehicle_status_revision_exhausted_", predicate)


if __name__ == "__main__":
    unittest.main()
