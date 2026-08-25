#include "drone_city_nav/execution_horizon_contract_ros.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr std::int64_t kSecondNs{1'000'000'000};
constexpr std::int64_t kIntervalNs{50'000'000};

void setTime(builtin_interfaces::msg::Time& time, const std::int64_t nanoseconds) {
  time.sec = static_cast<std::int32_t>(nanoseconds / kSecondNs);
  time.nanosec = static_cast<std::uint32_t>(nanoseconds % kSecondNs);
}

[[nodiscard]] msg::MppiTrajectoryHorizon validPlannedHorizon() {
  msg::MppiTrajectoryHorizon horizon;
  horizon.header.frame_id = "map";
  setTime(horizon.header.stamp, 10 * kSecondNs);
  horizon.producer_instance_id = 7U;
  horizon.target_offboard_instance_id = 9U;
  horizon.sequence = 11U;
  setTime(horizon.valid_from, 10 * kSecondNs);
  setTime(horizon.valid_until, 10 * kSecondNs + 2 * kIntervalNs);
  horizon.control_interval_ns = kIntervalNs;
  horizon.execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_PLANNED;
  horizon.execution_reason = msg::MppiTrajectoryHorizon::EXECUTION_REASON_NONE;
  horizon.route_purpose = msg::MppiTrajectoryHorizon::ROUTE_PURPOSE_MISSION_TRANSIT;
  horizon.route_target.x = 2.0;
  horizon.route_target.y = 3.0;
  horizon.route_target.z = 5.0;
  horizon.points.resize(3U);
  for (std::size_t index = 0U; index < horizon.points.size(); ++index) {
    msg::MppiHorizonPoint& point = horizon.points[index];
    point.time_from_start_ns = static_cast<std::int64_t>(index) * kIntervalNs;
    point.time_from_start_s =
        static_cast<float>(static_cast<double>(point.time_from_start_ns) / kSecondNs);
    point.position.x = static_cast<double>(index);
    point.position.z = 5.0;
  }
  return horizon;
}

[[nodiscard]] ExecutionHorizonPayloadValidationConfig config() {
  static const FlightEnvelopeConfig envelope{
      .minimum_target_z_m = 1.0,
      .maximum_target_z_m = 20.0,
  };
  return {
      .expected_frame_id = "map",
      .flight_envelope = &envelope,
  };
}

[[nodiscard]] msg::MppiTrajectoryHorizon validRevocation() {
  msg::MppiTrajectoryHorizon horizon;
  horizon.header.frame_id = "map";
  setTime(horizon.header.stamp, 10 * kSecondNs);
  horizon.producer_instance_id = 7U;
  horizon.target_offboard_instance_id = 9U;
  horizon.sequence = 12U;
  setTime(horizon.valid_from, 10 * kSecondNs);
  setTime(horizon.valid_until, 10 * kSecondNs);
  horizon.execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_REVOKED;
  horizon.execution_reason =
      msg::MppiTrajectoryHorizon::EXECUTION_REASON_NO_EXECUTABLE_HORIZON;
  return horizon;
}

[[nodiscard]] msg::MppiControlFeedback validPlannedFeedback() {
  msg::MppiControlFeedback feedback;
  feedback.header.frame_id = "map";
  setTime(feedback.header.stamp, 10 * kSecondNs);
  feedback.producer_instance_id = 13U;
  feedback.horizon_producer_instance_id = 7U;
  feedback.horizon_sequence = 11U;
  feedback.execution_mode = msg::MppiControlFeedback::EXECUTION_MODE_PLANNED;
  feedback.control_authoritative = true;
  feedback.acceleration.x = 0.5;
  return feedback;
}

TEST(ExecutionHorizonContractRosTest, AcceptsExactPlannedAndStationaryHoldPayloads) {
  const msg::MppiTrajectoryHorizon planned = validPlannedHorizon();
  EXPECT_EQ(assessExecutionHorizonPayload(planned, config()),
            ExecutionHorizonPayloadStatus::kValid);

  msg::MppiTrajectoryHorizon hold = planned;
  hold.execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD;
  hold.stationary_position_hold = true;
  hold.stationary_hold_position = hold.route_target;
  for (msg::MppiHorizonPoint& point : hold.points) {
    point.position = hold.stationary_hold_position;
  }
  EXPECT_EQ(assessExecutionHorizonPayload(hold, config()),
            ExecutionHorizonPayloadStatus::kValid);
}

TEST(ExecutionHorizonContractRosTest,
     HorizonFingerprintBindsExactWireContentIncludingTimestampAndPoints) {
  const msg::MppiTrajectoryHorizon horizon = validPlannedHorizon();
  const std::uint64_t fingerprint = executionHorizonContentFingerprint(horizon);
  EXPECT_NE(fingerprint, 0U);
  EXPECT_EQ(executionHorizonContentFingerprint(horizon), fingerprint);

  msg::MppiTrajectoryHorizon changed_stamp = horizon;
  ++changed_stamp.header.stamp.nanosec;
  EXPECT_NE(executionHorizonContentFingerprint(changed_stamp), fingerprint);

  msg::MppiTrajectoryHorizon changed_point = horizon;
  changed_point.points[1].position.y = 0.25;
  EXPECT_NE(executionHorizonContentFingerprint(changed_point), fingerprint);

  msg::MppiTrajectoryHorizon malformed = horizon;
  malformed.points[1].position.y = std::numeric_limits<double>::quiet_NaN();
  EXPECT_NE(executionHorizonContentFingerprint(malformed), fingerprint);
}

TEST(ExecutionHorizonContractRosTest,
     AcceptsCanonicalRevocationWithoutExecutableAuthority) {
  EXPECT_EQ(assessExecutionHorizonPayload(validRevocation(), config()),
            ExecutionHorizonPayloadStatus::kValid);
}

TEST(ExecutionHorizonContractRosTest,
     RejectsRevocationWithExecutablePayloadOrNonFailClosedReason) {
  msg::MppiTrajectoryHorizon horizon = validRevocation();
  horizon.points.resize(1U);
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInvalidTiming);

  horizon = validRevocation();
  horizon.stationary_position_hold = true;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInconsistentRevocation);

  horizon = validRevocation();
  horizon.execution_reason = msg::MppiTrajectoryHorizon::EXECUTION_REASON_NONE;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInconsistentRevocation);

  horizon = validRevocation();
  horizon.route_target.z = 5.0;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInconsistentRevocation);

  horizon = validRevocation();
  horizon.pose_revision = 1U;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInconsistentRevocation);

  horizon = validRevocation();
  horizon.route_purpose =
      msg::MppiTrajectoryHorizon::ROUTE_PURPOSE_OBSERVATION_FRONTIER;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInconsistentRevocation);
}

TEST(ExecutionHorizonContractRosTest, RejectsIdentityFrameAndTimingMismatch) {
  msg::MppiTrajectoryHorizon horizon = validPlannedHorizon();
  horizon.producer_instance_id = 0U;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInvalidIdentity);

  horizon = validPlannedHorizon();
  horizon.header.frame_id = "odom";
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInvalidFrame);

  horizon = validPlannedHorizon();
  ++horizon.points[1].time_from_start_ns;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInvalidTiming);

  horizon = validPlannedHorizon();
  horizon.points[1].time_from_start_s += 0.001F;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInvalidTiming);

  horizon = validPlannedHorizon();
  horizon.valid_until.nanosec = 1'000'000'000U;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInvalidTiming);
}

TEST(ExecutionHorizonContractRosTest, RejectsNonfiniteAndOutOfEnvelopePayloads) {
  msg::MppiTrajectoryHorizon horizon = validPlannedHorizon();
  horizon.route_target.x = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kNonFiniteMetadata);

  horizon = validPlannedHorizon();
  horizon.points[1].acceleration.y = std::numeric_limits<double>::infinity();
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kNonFinitePoint);

  horizon = validPlannedHorizon();
  horizon.points[1].position.z = 21.0;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kOutsideFlightEnvelope);

  horizon = validPlannedHorizon();
  horizon.route_target.z = 21.0;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kOutsideFlightEnvelope);
}

TEST(ExecutionHorizonContractRosTest, RejectsModeTerminalAndHoldInconsistency) {
  msg::MppiTrajectoryHorizon horizon = validPlannedHorizon();
  horizon.route_purpose = 255U;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInvalidEnum);

  horizon = validPlannedHorizon();
  horizon.execution_mode = 255U;
  horizon.stationary_position_hold = true;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInvalidEnum);

  horizon = validPlannedHorizon();
  horizon.stationary_position_hold = true;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInconsistentExecutionMode);

  horizon = validPlannedHorizon();
  horizon.execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInconsistentExecutionMode);

  horizon = validPlannedHorizon();
  horizon.stationary_hold_position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kValid);

  horizon = validPlannedHorizon();
  horizon.points.back().velocity.x = 1.0;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kMissingTerminalRestState);

  horizon = validPlannedHorizon();
  horizon.points.back().acceleration.z = 0.01;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kMissingTerminalRestState);

  horizon = validPlannedHorizon();
  horizon.execution_mode = msg::MppiTrajectoryHorizon::EXECUTION_MODE_POSITION_HOLD;
  horizon.stationary_position_hold = true;
  horizon.stationary_hold_position = horizon.route_target;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kInconsistentStationaryHold);

  for (msg::MppiHorizonPoint& point : horizon.points) {
    point.position = horizon.stationary_hold_position;
  }
  horizon.stationary_hold_position.z = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kNonFiniteMetadata);

  horizon.stationary_hold_position = horizon.route_target;
  horizon.stationary_hold_position.z = 21.0;
  EXPECT_EQ(assessExecutionHorizonPayload(horizon, config()),
            ExecutionHorizonPayloadStatus::kOutsideFlightEnvelope);
}

TEST(ExecutionHorizonContractRosTest,
     ConvertsValidHeartbeatAndAuthoritativeOrUnavailableHorizonFeedback) {
  const std::int64_t receive_stamp_ns = 10 * kSecondNs + 1;
  const ExecutionControlFeedbackAssessment planned =
      assessExecutionControlFeedback(validPlannedFeedback(), "map", receive_stamp_ns);
  ASSERT_TRUE(planned.valid());
  EXPECT_TRUE(planned.candidate.horizonFeedback());
  EXPECT_TRUE(planned.candidate.control_authoritative);
  EXPECT_NE(planned.candidate.content_fingerprint, 0U);
  EXPECT_NE(planned.candidate.wire_fingerprint, 0U);

  msg::MppiControlFeedback unavailable = validPlannedFeedback();
  unavailable.control_authoritative = false;
  const ExecutionControlFeedbackAssessment unavailable_assessment =
      assessExecutionControlFeedback(unavailable, "map", receive_stamp_ns);
  ASSERT_TRUE(unavailable_assessment.valid());
  EXPECT_TRUE(unavailable_assessment.candidate.horizonFeedback());
  EXPECT_FALSE(unavailable_assessment.candidate.control_authoritative);
  EXPECT_NE(unavailable_assessment.candidate.content_fingerprint,
            planned.candidate.content_fingerprint);

  msg::MppiControlFeedback heartbeat = validPlannedFeedback();
  heartbeat.horizon_producer_instance_id = 0U;
  heartbeat.horizon_sequence = 0U;
  heartbeat.execution_mode = msg::MppiControlFeedback::EXECUTION_MODE_POSITION_HOLD;
  heartbeat.control_authoritative = false;
  heartbeat.acceleration.x = 0.0;
  const ExecutionControlFeedbackAssessment heartbeat_assessment =
      assessExecutionControlFeedback(heartbeat, "map", receive_stamp_ns);
  ASSERT_TRUE(heartbeat_assessment.valid());
  EXPECT_TRUE(heartbeat_assessment.candidate.heartbeat());
  EXPECT_EQ(heartbeat_assessment.candidate.content_fingerprint, 0U);
  EXPECT_NE(heartbeat_assessment.candidate.wire_fingerprint, 0U);

  msg::MppiControlFeedback mutated_heartbeat = heartbeat;
  mutated_heartbeat.acceleration.x = 0.25;
  const ExecutionControlFeedbackAssessment heartbeat_mutation =
      assessExecutionControlFeedback(mutated_heartbeat, "map", receive_stamp_ns);
  ASSERT_TRUE(heartbeat_mutation.valid());
  EXPECT_EQ(heartbeat_mutation.candidate.content_fingerprint, 0U);
  EXPECT_NE(heartbeat_mutation.candidate.wire_fingerprint,
            heartbeat_assessment.candidate.wire_fingerprint);

  msg::MppiControlFeedback changed = validPlannedFeedback();
  changed.acceleration.x = 0.75;
  const ExecutionControlFeedbackAssessment changed_assessment =
      assessExecutionControlFeedback(changed, "map", receive_stamp_ns);
  ASSERT_TRUE(changed_assessment.valid());
  EXPECT_NE(changed_assessment.candidate.content_fingerprint,
            planned.candidate.content_fingerprint);
}

TEST(ExecutionHorizonContractRosTest,
     RejectsMalformedFeedbackTimingIdentityModeAndControl) {
  constexpr std::int64_t kReceiveStampNs{10 * kSecondNs + 1};
  msg::MppiControlFeedback feedback = validPlannedFeedback();
  feedback.header.frame_id = "odom";
  const ExecutionControlFeedbackAssessment invalid_frame =
      assessExecutionControlFeedback(feedback, "map", kReceiveStampNs);
  EXPECT_EQ(invalid_frame.status, ExecutionControlFeedbackStatus::kInvalidFrame);
  EXPECT_TRUE(invalid_frame.admissionCandidateValid());
  EXPECT_TRUE(invalid_frame.candidate.sourceIdentityClaimable());
  EXPECT_EQ(invalid_frame.candidate.offboard_producer_instance_id,
            feedback.producer_instance_id);
  EXPECT_EQ(invalid_frame.candidate.horizon_producer_instance_id,
            feedback.horizon_producer_instance_id);
  EXPECT_EQ(invalid_frame.candidate.horizon_sequence, feedback.horizon_sequence);
  const ExecutionControlFeedbackAssessment corrected_frame =
      assessExecutionControlFeedback(validPlannedFeedback(), "map", kReceiveStampNs);
  EXPECT_EQ(invalid_frame.candidate.source_stamp_ns,
            corrected_frame.candidate.source_stamp_ns);
  EXPECT_NE(invalid_frame.candidate.wire_fingerprint,
            corrected_frame.candidate.wire_fingerprint);

  feedback = validPlannedFeedback();
  feedback.header.stamp.sec = 9;
  feedback.header.stamp.nanosec = 1'000'000'000U;
  const ExecutionControlFeedbackAssessment invalid_timing =
      assessExecutionControlFeedback(feedback, "map", kReceiveStampNs);
  EXPECT_EQ(invalid_timing.status, ExecutionControlFeedbackStatus::kInvalidTiming);
  EXPECT_TRUE(invalid_timing.candidate.sourceIdentityClaimable());
  EXPECT_EQ(invalid_timing.candidate.horizon_sequence, feedback.horizon_sequence);
  EXPECT_EQ(invalid_timing.candidate.source_stamp_ns,
            corrected_frame.candidate.source_stamp_ns);
  EXPECT_NE(invalid_timing.candidate.wire_fingerprint,
            corrected_frame.candidate.wire_fingerprint);

  feedback = validPlannedFeedback();
  feedback.horizon_sequence = 0U;
  EXPECT_EQ(assessExecutionControlFeedback(feedback, "map", kReceiveStampNs).status,
            ExecutionControlFeedbackStatus::kInvalidIdentity);

  feedback = validPlannedFeedback();
  feedback.execution_mode = 255U;
  const ExecutionControlFeedbackAssessment invalid_enum =
      assessExecutionControlFeedback(feedback, "map", kReceiveStampNs);
  EXPECT_EQ(invalid_enum.status, ExecutionControlFeedbackStatus::kInvalidEnum);
  EXPECT_EQ(invalid_enum.candidate.horizon_sequence, feedback.horizon_sequence);

  feedback = validPlannedFeedback();
  feedback.execution_mode = msg::MppiControlFeedback::EXECUTION_MODE_POSITION_HOLD;
  const ExecutionControlFeedbackAssessment inconsistent_mode =
      assessExecutionControlFeedback(feedback, "map", kReceiveStampNs);
  EXPECT_EQ(inconsistent_mode.status,
            ExecutionControlFeedbackStatus::kInconsistentExecutionMode);
  EXPECT_EQ(inconsistent_mode.candidate.horizon_sequence, feedback.horizon_sequence);

  feedback = validPlannedFeedback();
  feedback.acceleration.z = std::numeric_limits<double>::quiet_NaN();
  const ExecutionControlFeedbackAssessment non_finite =
      assessExecutionControlFeedback(feedback, "map", kReceiveStampNs);
  EXPECT_EQ(non_finite.status, ExecutionControlFeedbackStatus::kNonFiniteControl);
  EXPECT_TRUE(non_finite.admissionCandidateValid());

  feedback = validPlannedFeedback();
  feedback.acceleration.x = std::numeric_limits<double>::max();
  const ExecutionControlFeedbackAssessment overflow =
      assessExecutionControlFeedback(feedback, "map", kReceiveStampNs);
  EXPECT_EQ(overflow.status, ExecutionControlFeedbackStatus::kNonFiniteControl);
  EXPECT_TRUE(overflow.admissionCandidateValid());
  EXPECT_NE(overflow.candidate.content_fingerprint, 0U);

  const ExecutionControlFeedbackAssessment baseline =
      assessExecutionControlFeedback(validPlannedFeedback(), "map", kReceiveStampNs);
  ASSERT_TRUE(baseline.valid());
  EXPECT_NE(overflow.candidate.content_fingerprint,
            baseline.candidate.content_fingerprint);
}

TEST(ExecutionHorizonContractRosTest,
     AcceptsStructurallyValidFeedbackWhenReceiptClockLagsSourceClock) {
  msg::MppiControlFeedback feedback = validPlannedFeedback();
  const ExecutionControlFeedbackAssessment assessment =
      assessExecutionControlFeedback(feedback, "map", 10 * kSecondNs - 1);

  EXPECT_TRUE(assessment.valid());
  EXPECT_EQ(assessment.candidate.source_stamp_ns, 10 * kSecondNs);
  EXPECT_EQ(assessment.candidate.receive_stamp_ns, 10 * kSecondNs - 1);
}

TEST(ExecutionHorizonContractRosTest,
     WrongFrameClaimsRawSourceAndOnlyHigherSourceCanRecover) {
  msg::MppiControlFeedback heartbeat = validPlannedFeedback();
  setTime(heartbeat.header.stamp, 8 * kSecondNs);
  heartbeat.horizon_producer_instance_id = 0U;
  heartbeat.horizon_sequence = 0U;
  heartbeat.execution_mode = msg::MppiControlFeedback::EXECUTION_MODE_POSITION_HOLD;
  heartbeat.control_authoritative = false;
  const ExecutionControlFeedbackAssessment heartbeat_assessment =
      assessExecutionControlFeedback(heartbeat, "map", 8 * kSecondNs + 1);
  ASSERT_TRUE(heartbeat_assessment.valid());
  const ExecutionHorizonWitnessAdmissionResult session =
      admitExecutionHorizonFeedback({}, heartbeat_assessment.candidate);
  ASSERT_TRUE(session.accept);

  msg::MppiControlFeedback wrong_frame = validPlannedFeedback();
  wrong_frame.header.frame_id = "odom";
  const std::int64_t malformed_receive_ns = 10 * kSecondNs + 1;
  const ExecutionControlFeedbackAssessment malformed_assessment =
      assessExecutionControlFeedback(wrong_frame, "map", malformed_receive_ns);
  ASSERT_EQ(malformed_assessment.status, ExecutionControlFeedbackStatus::kInvalidFrame);
  const ExecutionHorizonWitnessAdmissionResult claimed =
      revokeMalformedExecutionHorizonFeedback(
          session.next_state, malformed_assessment.candidate, malformed_receive_ns);
  ASSERT_FALSE(claimed.accept);
  ASSERT_TRUE(claimed.invalid);
  ASSERT_TRUE(claimed.state_advanced);
  EXPECT_TRUE(claimed.next_state.feedback_source_identity.tombstoned);
  EXPECT_EQ(claimed.next_state.offboard_session.latest_source_stamp_ns,
            heartbeat_assessment.candidate.source_stamp_ns);
  EXPECT_EQ(claimed.next_state.latest_session_receive_stamp_ns,
            heartbeat_assessment.candidate.receive_stamp_ns);

  const ExecutionControlFeedbackAssessment corrected = assessExecutionControlFeedback(
      validPlannedFeedback(), "map", malformed_receive_ns + 1);
  ASSERT_TRUE(corrected.valid());
  const ExecutionHorizonWitnessAdmissionResult mutation =
      admitExecutionHorizonFeedback(claimed.next_state, corrected.candidate);
  EXPECT_FALSE(mutation.accept);
  EXPECT_TRUE(mutation.conflict);
  EXPECT_FALSE(mutation.state_advanced);

  msg::MppiControlFeedback higher = validPlannedFeedback();
  setTime(higher.header.stamp, 11 * kSecondNs);
  const ExecutionControlFeedbackAssessment higher_assessment =
      assessExecutionControlFeedback(higher, "map", 11 * kSecondNs + 1);
  ASSERT_TRUE(higher_assessment.valid());
  const ExecutionHorizonWitnessAdmissionResult recovered =
      admitExecutionHorizonFeedback(mutation.next_state, higher_assessment.candidate);
  EXPECT_TRUE(recovered.accept);
  EXPECT_TRUE(recovered.witness_updated);
  EXPECT_FALSE(recovered.next_state.feedback_source_identity.tombstoned);
}

TEST(ExecutionHorizonContractRosTest,
     NonCanonicalRawTimeCannotCollideWithCorrectedCanonicalWire) {
  msg::MppiControlFeedback heartbeat = validPlannedFeedback();
  setTime(heartbeat.header.stamp, 8 * kSecondNs);
  heartbeat.horizon_producer_instance_id = 0U;
  heartbeat.horizon_sequence = 0U;
  heartbeat.execution_mode = msg::MppiControlFeedback::EXECUTION_MODE_POSITION_HOLD;
  heartbeat.control_authoritative = false;
  const ExecutionControlFeedbackAssessment heartbeat_assessment =
      assessExecutionControlFeedback(heartbeat, "map", 8 * kSecondNs + 1);
  const ExecutionHorizonWitnessAdmissionResult session =
      admitExecutionHorizonFeedback({}, heartbeat_assessment.candidate);
  ASSERT_TRUE(session.accept);

  msg::MppiControlFeedback noncanonical = validPlannedFeedback();
  noncanonical.header.stamp.sec = 9;
  noncanonical.header.stamp.nanosec = 1'000'000'000U;
  const ExecutionControlFeedbackAssessment malformed =
      assessExecutionControlFeedback(noncanonical, "map", 10 * kSecondNs + 1);
  ASSERT_EQ(malformed.status, ExecutionControlFeedbackStatus::kInvalidTiming);
  ASSERT_TRUE(malformed.candidate.sourceIdentityClaimable());
  const ExecutionHorizonWitnessAdmissionResult claimed =
      revokeMalformedExecutionHorizonFeedback(session.next_state, malformed.candidate,
                                              10 * kSecondNs + 1);
  ASSERT_TRUE(claimed.state_advanced);

  const ExecutionControlFeedbackAssessment canonical =
      assessExecutionControlFeedback(validPlannedFeedback(), "map", 10 * kSecondNs + 2);
  ASSERT_EQ(malformed.candidate.source_stamp_ns, canonical.candidate.source_stamp_ns);
  ASSERT_NE(malformed.candidate.wire_fingerprint, canonical.candidate.wire_fingerprint);
  const ExecutionHorizonWitnessAdmissionResult conflict =
      admitExecutionHorizonFeedback(claimed.next_state, canonical.candidate);
  EXPECT_FALSE(conflict.accept);
  EXPECT_TRUE(conflict.conflict);
  EXPECT_FALSE(conflict.state_advanced);

  msg::MppiControlFeedback higher = validPlannedFeedback();
  setTime(higher.header.stamp, 11 * kSecondNs);
  const ExecutionControlFeedbackAssessment higher_assessment =
      assessExecutionControlFeedback(higher, "map", 11 * kSecondNs + 1);
  const ExecutionHorizonWitnessAdmissionResult recovery =
      admitExecutionHorizonFeedback(conflict.next_state, higher_assessment.candidate);
  EXPECT_TRUE(recovery.accept);
  EXPECT_TRUE(recovery.witness_updated);
}

} // namespace
} // namespace drone_city_nav
