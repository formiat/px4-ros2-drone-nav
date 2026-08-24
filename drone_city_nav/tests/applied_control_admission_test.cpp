#include "drone_city_nav/applied_control_admission.hpp"

#include <gtest/gtest.h>

#include <cstdint>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kOffboardProducer{7U};
constexpr std::uint64_t kHorizonProducer{31U};

[[nodiscard]] std::uint64_t wireFingerprint(const std::int64_t source_stamp_ns,
                                            const std::uint64_t content_fingerprint) {
  const std::uint64_t fingerprint = static_cast<std::uint64_t>(source_stamp_ns) ^
                                    content_fingerprint ^ 0x4150504c57495245ULL;
  return fingerprint == 0U ? 1U : fingerprint;
}

[[nodiscard]] ExecutionHorizonFeedbackCandidate
heartbeat(const std::uint64_t producer = kOffboardProducer,
          const std::int64_t source_stamp_ns = 100,
          const std::int64_t receive_stamp_ns = 110) {
  return {
      .offboard_producer_instance_id = producer,
      .horizon_producer_instance_id = 0U,
      .horizon_sequence = 0U,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .content_fingerprint = 0U,
      .wire_fingerprint = wireFingerprint(source_stamp_ns, 0U),
      .execution_mode = ExecutionHorizonWitnessMode::kPositionHold,
      .control_authoritative = false,
  };
}

[[nodiscard]] ExecutionHorizonFeedbackCandidate feedback(
    const std::uint64_t horizon_sequence = 4U, const std::int64_t source_stamp_ns = 120,
    const std::int64_t receive_stamp_ns = 130,
    const std::uint64_t content_fingerprint = 41U, const bool authoritative = true,
    const ExecutionHorizonWitnessMode mode = ExecutionHorizonWitnessMode::kPlanned) {
  return {
      .offboard_producer_instance_id = kOffboardProducer,
      .horizon_producer_instance_id = kHorizonProducer,
      .horizon_sequence = horizon_sequence,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .content_fingerprint = content_fingerprint,
      .wire_fingerprint = wireFingerprint(source_stamp_ns, content_fingerprint),
      .execution_mode = mode,
      .control_authoritative = authoritative,
  };
}

[[nodiscard]] AppliedControlExecutionOwner owner() {
  return {
      .offboard_producer_instance_id = kOffboardProducer,
      .horizon_producer_instance_id = kHorizonProducer,
      .horizon_sequence = 4U,
      .execution_mode = ExecutionHorizonWitnessMode::kPlanned,
  };
}

[[nodiscard]] ExecutionHorizonWitnessState initialSession() {
  const AppliedControlAdmissionResult admitted =
      admitAppliedControlEvidence({}, heartbeat(), {}, true);
  EXPECT_TRUE(admitted.feedback.accept);
  EXPECT_TRUE(admitted.feedback.session_transitioned);
  EXPECT_TRUE(admitted.revoke_control);
  EXPECT_FALSE(admitted.install_control);
  return admitted.feedback.next_state;
}

[[nodiscard]] ExecutionHorizonWitnessState stateWithInstalledControl() {
  const AppliedControlAdmissionResult admitted =
      admitAppliedControlEvidence(initialSession(), feedback(), owner(), true);
  EXPECT_TRUE(admitted.feedback.accept);
  EXPECT_TRUE(admitted.feedback.witness_updated);
  EXPECT_TRUE(admitted.install_control);
  EXPECT_FALSE(admitted.revoke_control);
  return admitted.feedback.next_state;
}

TEST(AppliedControlAdmissionTest, ExactReplayDoesNotRefreshOrReinstallControl) {
  const ExecutionHorizonWitnessState state = stateWithInstalledControl();

  const AppliedControlAdmissionResult replay =
      admitAppliedControlEvidence(state, feedback(), owner(), true);

  EXPECT_FALSE(replay.feedback.accept);
  EXPECT_TRUE(replay.feedback.replay);
  EXPECT_FALSE(replay.install_control);
  EXPECT_FALSE(replay.revoke_control);
  EXPECT_EQ(replay.feedback.next_state.latest_session_receive_stamp_ns,
            state.latest_session_receive_stamp_ns);
}

TEST(AppliedControlAdmissionTest,
     SameStampAuthorityDowngradeRevokesAndNewerDowngradeInstallsUnavailableEvidence) {
  const ExecutionHorizonWitnessState state = stateWithInstalledControl();
  const AppliedControlAdmissionResult same_stamp_downgrade =
      admitAppliedControlEvidence(state, feedback(4U, 120, 140, 42U, false), owner(),
                                  true);
  EXPECT_FALSE(same_stamp_downgrade.feedback.accept);
  EXPECT_TRUE(same_stamp_downgrade.feedback.conflict);
  EXPECT_TRUE(same_stamp_downgrade.revoke_control);
  EXPECT_TRUE(same_stamp_downgrade.feedback.next_state.latest_horizon_feedback.empty());

  const AppliedControlAdmissionResult newer_downgrade = admitAppliedControlEvidence(
      state, feedback(4U, 140, 150, 43U, false), owner(), true);
  EXPECT_TRUE(newer_downgrade.feedback.accept);
  EXPECT_TRUE(newer_downgrade.install_control);
  EXPECT_FALSE(newer_downgrade.revoke_control);
  EXPECT_FALSE(newer_downgrade.feedback.next_state.latest_horizon_feedback
                   .control_authoritative);
}

TEST(AppliedControlAdmissionTest,
     SameStampDifferentHorizonOrContentRevokesInstalledControl) {
  const ExecutionHorizonWitnessState state = stateWithInstalledControl();

  const AppliedControlAdmissionResult horizon_conflict =
      admitAppliedControlEvidence(state, feedback(5U, 120, 140, 42U), owner(), true);
  EXPECT_FALSE(horizon_conflict.feedback.accept);
  EXPECT_TRUE(horizon_conflict.feedback.conflict);
  EXPECT_TRUE(horizon_conflict.revoke_control);

  const AppliedControlAdmissionResult content_conflict =
      admitAppliedControlEvidence(state, feedback(4U, 120, 125, 99U), owner(), false);
  EXPECT_FALSE(content_conflict.feedback.accept);
  EXPECT_TRUE(content_conflict.feedback.conflict);
  EXPECT_TRUE(content_conflict.revoke_control);
  EXPECT_EQ(
      content_conflict.feedback.next_state.horizon_feedback_receive_stamp_high_water_ns,
      state.horizon_feedback_receive_stamp_high_water_ns);
}

TEST(AppliedControlAdmissionTest,
     NewerInvalidPayloadLeavesTombstoneAndRevokesOldControl) {
  const ExecutionHorizonWitnessState state = stateWithInstalledControl();

  const AppliedControlAdmissionResult invalid_successor =
      admitAppliedControlEvidence(state, feedback(4U, 140, 150, 99U), owner(), false);

  EXPECT_FALSE(invalid_successor.feedback.accept);
  EXPECT_FALSE(invalid_successor.feedback.witness_updated);
  EXPECT_TRUE(invalid_successor.feedback.witness_revoked);
  EXPECT_FALSE(invalid_successor.install_control);
  EXPECT_TRUE(invalid_successor.revoke_control);
  EXPECT_TRUE(invalid_successor.feedback.next_state.latest_horizon_feedback.empty());
  EXPECT_EQ(
      invalid_successor.feedback.next_state.horizon_feedback_source_stamp_high_water_ns,
      140);

  const AppliedControlAdmissionResult replay =
      admitAppliedControlEvidence(invalid_successor.feedback.next_state,
                                  feedback(4U, 140, 160, 99U), owner(), true);
  EXPECT_FALSE(replay.feedback.accept);
  EXPECT_TRUE(replay.feedback.replay);
  EXPECT_FALSE(replay.feedback.conflict);
  EXPECT_TRUE(replay.revoke_control);
  EXPECT_EQ(replay.feedback.next_state.horizon_feedback_receive_stamp_high_water_ns,
            150);
  EXPECT_EQ(
      replay.feedback.next_state.feedback_source_identity.candidate.receive_stamp_ns,
      150);

  const AppliedControlAdmissionResult recovery = admitAppliedControlEvidence(
      replay.feedback.next_state, feedback(4U, 170, 180, 100U), owner(), true);
  EXPECT_TRUE(recovery.feedback.accept);
  EXPECT_TRUE(recovery.install_control);
  EXPECT_FALSE(recovery.revoke_control);
}

TEST(AppliedControlAdmissionTest, MalformedCandidateDoesNotMutateValidState) {
  const ExecutionHorizonWitnessState state = stateWithInstalledControl();

  const AppliedControlAdmissionResult malformed =
      admitAppliedControlEvidence(state, {}, owner(), false);

  EXPECT_FALSE(malformed.feedback.accept);
  EXPECT_TRUE(malformed.feedback.invalid);
  EXPECT_FALSE(malformed.install_control);
  EXPECT_FALSE(malformed.revoke_control);
  EXPECT_EQ(malformed.feedback.next_state.latest_horizon_feedback.source_stamp_ns,
            state.latest_horizon_feedback.source_stamp_ns);
  EXPECT_EQ(malformed.feedback.next_state.latest_horizon_feedback.content_fingerprint,
            state.latest_horizon_feedback.content_fingerprint);
}

TEST(AppliedControlAdmissionTest, AcceptedNewerNonOwningFeedbackRevokesOldControl) {
  const ExecutionHorizonWitnessState state = stateWithInstalledControl();

  const AppliedControlAdmissionResult non_owning =
      admitAppliedControlEvidence(state, feedback(5U, 140, 150, 42U), owner(), true);

  EXPECT_TRUE(non_owning.feedback.accept);
  EXPECT_TRUE(non_owning.feedback.witness_updated);
  EXPECT_FALSE(non_owning.install_control);
  EXPECT_TRUE(non_owning.revoke_control);
  EXPECT_EQ(non_owning.feedback.next_state.latest_horizon_feedback.horizon_sequence,
            5U);
}

TEST(AppliedControlAdmissionTest,
     NewerHeartbeatRevokesControlAndRequiresStrictlyNewerRecovery) {
  const ExecutionHorizonWitnessState state = stateWithInstalledControl();
  const AppliedControlAdmissionResult heartbeat_update = admitAppliedControlEvidence(
      state, heartbeat(kOffboardProducer, 140, 150), owner(), true);
  ASSERT_TRUE(heartbeat_update.feedback.accept);
  EXPECT_FALSE(heartbeat_update.install_control);
  EXPECT_TRUE(heartbeat_update.revoke_control);
  ASSERT_TRUE(heartbeat_update.feedback.next_state.latest_horizon_feedback.empty());

  const AppliedControlAdmissionResult conflict = admitAppliedControlEvidence(
      heartbeat_update.feedback.next_state, feedback(4U, 120, 160, 99U), owner(), true);
  EXPECT_FALSE(conflict.feedback.accept);
  EXPECT_TRUE(conflict.feedback.stale);
  EXPECT_FALSE(conflict.revoke_control);
  EXPECT_TRUE(conflict.feedback.next_state.latest_horizon_feedback.empty());

  const AppliedControlAdmissionResult replay = admitAppliedControlEvidence(
      conflict.feedback.next_state, feedback(4U, 120, 170, 41U), owner(), true);
  EXPECT_FALSE(replay.feedback.accept);
  EXPECT_TRUE(replay.feedback.stale);
  EXPECT_FALSE(replay.feedback.conflict);
  EXPECT_FALSE(replay.revoke_control);
  EXPECT_TRUE(replay.feedback.next_state.latest_horizon_feedback.empty());

  const AppliedControlAdmissionResult recovery = admitAppliedControlEvidence(
      replay.feedback.next_state, feedback(4U, 180, 190, 43U), owner(), true);
  EXPECT_TRUE(recovery.feedback.accept);
  EXPECT_TRUE(recovery.install_control);
  EXPECT_FALSE(recovery.revoke_control);
  EXPECT_TRUE(recovery.feedback.next_state.latest_horizon_feedback.valid());
}

TEST(AppliedControlAdmissionTest, SessionTransitionRevokesAndClearsPriorTombstone) {
  const ExecutionHorizonWitnessState state = stateWithInstalledControl();
  const AppliedControlAdmissionResult conflict =
      admitAppliedControlEvidence(state, feedback(5U, 120, 140, 42U), owner(), true);
  ASSERT_TRUE(conflict.feedback.conflict);
  ASSERT_TRUE(conflict.feedback.next_state.latest_horizon_feedback.empty());
  ASSERT_NE(conflict.feedback.next_state.horizon_feedback_source_stamp_high_water_ns,
            0);

  const AppliedControlAdmissionResult transition = admitAppliedControlEvidence(
      conflict.feedback.next_state, heartbeat(8U, 150, 160), owner(), true);

  EXPECT_TRUE(transition.feedback.accept);
  EXPECT_TRUE(transition.feedback.session_transitioned);
  EXPECT_TRUE(transition.revoke_control);
  EXPECT_TRUE(transition.feedback.next_state.latest_horizon_feedback.empty());
  EXPECT_EQ(transition.feedback.next_state.horizon_feedback_source_stamp_high_water_ns,
            150);
  EXPECT_EQ(transition.feedback.next_state.horizon_feedback_receive_stamp_high_water_ns,
            160);
}

TEST(AppliedControlAdmissionTest, SameStampHeartbeatConflictsAndRevokesControl) {
  const ExecutionHorizonWitnessState state = stateWithInstalledControl();
  const std::int64_t source_stamp = state.latest_horizon_feedback.source_stamp_ns;

  const AppliedControlAdmissionResult fallback = admitAppliedControlEvidence(
      state,
      heartbeat(kOffboardProducer, source_stamp, state.latest_session_receive_stamp_ns),
      owner(), true);

  EXPECT_FALSE(fallback.feedback.accept);
  EXPECT_TRUE(fallback.feedback.conflict);
  EXPECT_TRUE(fallback.feedback.witness_revoked);
  EXPECT_TRUE(fallback.revoke_control);
  EXPECT_FALSE(fallback.install_control);
  EXPECT_TRUE(fallback.feedback.next_state.latest_horizon_feedback.empty());
}

} // namespace
} // namespace drone_city_nav
