#!/usr/bin/env python3

import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / "drone_city_nav"
SOURCE = PACKAGE / "src"
NAVIGATION_INPUT = SOURCE / "production_mppi_node_navigation_input.cpp"
INPUTS = SOURCE / "production_mppi_node_inputs.cpp"
PLANNING_TICK = SOURCE / "production_mppi_node_planning_tick.cpp"
NODE_HEADER = SOURCE / "production_mppi_node.hpp"
PRODUCER_ADMISSION = SOURCE / "producer_epoch_admission.cpp"


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
        self.assertIn("invalidateAppliedControlWitnessLocked()", identity_conflict)
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
        self.assertIn("invalidateAppliedControlWitnessLocked()", rejected_navigation)
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
        self.assertIn("invalidateAppliedControlWitnessLocked()", callback)
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
