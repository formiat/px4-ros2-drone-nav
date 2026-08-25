#include "drone_city_nav/execution_horizon_witness.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kOffboardProducer{7U};
constexpr std::uint64_t kHorizonProducer{31U};
constexpr std::int64_t kHeartbeatSourceNs{1'000'000'000LL};
constexpr std::int64_t kHeartbeatReceiveNs{1'010'000'000LL};
constexpr std::int64_t kMaximumAgeNs{1'000'000'000LL};

[[nodiscard]] std::uint64_t wireFingerprint(const std::uint64_t producer,
                                            const std::int64_t source_stamp_ns,
                                            const std::uint64_t content_fingerprint) {
  const std::uint64_t fingerprint = producer ^
                                    static_cast<std::uint64_t>(source_stamp_ns) ^
                                    content_fingerprint ^ 0x57495245534f5552ULL;
  return fingerprint == 0U ? 1U : fingerprint;
}

[[nodiscard]] ExecutionHorizonFeedbackCandidate
heartbeat(const std::uint64_t producer = kOffboardProducer,
          const std::int64_t source_stamp_ns = kHeartbeatSourceNs,
          const std::int64_t receive_stamp_ns = kHeartbeatReceiveNs) {
  return {
      .offboard_producer_instance_id = producer,
      .horizon_producer_instance_id = 0U,
      .horizon_sequence = 0U,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .wire_fingerprint = wireFingerprint(producer, source_stamp_ns, 0U),
      .execution_mode = ExecutionHorizonWitnessMode::kPositionHold,
      .control_authoritative = false,
  };
}

[[nodiscard]] ExecutionHorizonFeedbackCandidate
feedback(const std::uint64_t offboard_producer = kOffboardProducer,
         const std::uint64_t horizon_producer = kHorizonProducer,
         const std::uint64_t horizon_sequence = 4U,
         const std::int64_t source_stamp_ns = 1'020'000'000LL,
         const std::int64_t receive_stamp_ns = 1'030'000'000LL,
         const ExecutionHorizonWitnessMode mode = ExecutionHorizonWitnessMode::kPlanned,
         const std::uint64_t content_fingerprint = 101U) {
  return {
      .offboard_producer_instance_id = offboard_producer,
      .horizon_producer_instance_id = horizon_producer,
      .horizon_sequence = horizon_sequence,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .content_fingerprint = content_fingerprint,
      .wire_fingerprint =
          wireFingerprint(offboard_producer, source_stamp_ns, content_fingerprint),
      .execution_mode = mode,
      .control_authoritative = mode == ExecutionHorizonWitnessMode::kPlanned,
  };
}

[[nodiscard]] ExecutionHorizonWitnessRequirement requirement(
    const std::uint64_t target_offboard = kOffboardProducer,
    const std::uint64_t horizon_producer = kHorizonProducer,
    const std::uint64_t horizon_sequence = 4U,
    const ExecutionHorizonWitnessMode mode = ExecutionHorizonWitnessMode::kPlanned) {
  return {
      .target_offboard_instance_id = target_offboard,
      .horizon_producer_instance_id = horizon_producer,
      .horizon_sequence = horizon_sequence,
      .valid_from_ns = 1'015'000'000LL,
      .valid_until_ns = 2'500'000'000LL,
      .execution_mode = mode,
  };
}

[[nodiscard]] ExecutionHorizonWitnessState initialSession() {
  const ExecutionHorizonWitnessAdmissionResult admitted =
      admitExecutionHorizonFeedback({}, heartbeat());
  EXPECT_TRUE(admitted.accept);
  EXPECT_TRUE(admitted.heartbeat);
  EXPECT_TRUE(admitted.session_transitioned);
  EXPECT_TRUE(admitted.state_advanced);
  EXPECT_TRUE(admitted.next_state.valid());
  return admitted.next_state;
}

[[nodiscard]] ExecutionHorizonWitnessState stateWithWitness() {
  const ExecutionHorizonWitnessAdmissionResult admitted =
      admitExecutionHorizonFeedback(initialSession(), feedback());
  EXPECT_TRUE(admitted.accept);
  EXPECT_TRUE(admitted.witness_updated);
  EXPECT_TRUE(admitted.state_advanced);
  EXPECT_TRUE(admitted.next_state.valid());
  return admitted.next_state;
}

TEST(ExecutionHorizonWitnessTest, AdmitsStartupHeartbeatAndExactHorizonWitness) {
  const ExecutionHorizonWitnessState session = initialSession();
  EXPECT_EQ(session.offboard_session.current_producer_instance_id, kOffboardProducer);
  EXPECT_EQ(session.latest_session_receive_stamp_ns, kHeartbeatReceiveNs);
  EXPECT_FALSE(session.feedback_source_identity.tombstoned);
  EXPECT_EQ(session.feedback_source_identity.candidate.receive_stamp_ns,
            kHeartbeatReceiveNs);
  EXPECT_TRUE(session.latest_horizon_feedback.empty());
  EXPECT_TRUE(offboardSessionFreshAt(session, 1'100'000'000LL, kMaximumAgeNs));

  const ExecutionHorizonWitnessAdmissionResult admitted =
      admitExecutionHorizonFeedback(session, feedback());
  ASSERT_TRUE(admitted.accept);
  EXPECT_FALSE(admitted.heartbeat);
  EXPECT_TRUE(admitted.witness_updated);
  EXPECT_FALSE(admitted.session_transitioned);
  EXPECT_TRUE(executionHorizonWitnessFreshAt(admitted.next_state, requirement(),
                                             1'100'000'000LL, kMaximumAgeNs));
}

TEST(ExecutionHorizonWitnessTest,
     PrestartReceiptThenAuthoritativeExecutionThenFallbackRevocation) {
  const ExecutionHorizonWitnessState session = initialSession();
  ExecutionHorizonFeedbackCandidate prestart_receipt =
      feedback(kOffboardProducer, kHorizonProducer, 4U, 1'020'000'000LL,
               1'030'000'000LL, ExecutionHorizonWitnessMode::kPlanned, 201U);
  prestart_receipt.control_authoritative = false;
  ExecutionHorizonFeedbackCandidate authoritative_execution = prestart_receipt;
  authoritative_execution.source_stamp_ns = 1'040'000'000LL;
  authoritative_execution.receive_stamp_ns = 1'050'000'000LL;
  authoritative_execution.content_fingerprint = 202U;
  authoritative_execution.control_authoritative = true;

  const ExecutionHorizonWitnessAdmissionResult receipt =
      admitExecutionHorizonFeedback(session, prestart_receipt);
  ASSERT_TRUE(receipt.accept);
  EXPECT_TRUE(executionHorizonReceiptFreshAt(receipt.next_state, requirement(),
                                             1'060'000'000LL, kMaximumAgeNs));
  EXPECT_FALSE(executionHorizonWitnessFreshAt(receipt.next_state, requirement(),
                                              1'060'000'000LL, kMaximumAgeNs));

  const ExecutionHorizonWitnessAdmissionResult executing =
      admitExecutionHorizonFeedback(receipt.next_state, authoritative_execution);
  ASSERT_TRUE(executing.accept);
  EXPECT_TRUE(executionHorizonReceiptFreshAt(executing.next_state, requirement(),
                                             1'060'000'000LL, kMaximumAgeNs));
  EXPECT_TRUE(executionHorizonWitnessFreshAt(executing.next_state, requirement(),
                                             1'060'000'000LL, kMaximumAgeNs));

  const ExecutionHorizonWitnessAdmissionResult fallback = admitExecutionHorizonFeedback(
      executing.next_state,
      heartbeat(kOffboardProducer, 1'070'000'000LL, 1'080'000'000LL));
  ASSERT_TRUE(fallback.accept);
  EXPECT_TRUE(fallback.witness_revoked);
  EXPECT_FALSE(executionHorizonReceiptFreshAt(fallback.next_state, requirement(),
                                              1'090'000'000LL, kMaximumAgeNs));
  EXPECT_FALSE(executionHorizonWitnessFreshAt(fallback.next_state, requirement(),
                                              1'090'000'000LL, kMaximumAgeNs));
}

TEST(ExecutionHorizonWitnessTest,
     NewHeartbeatRevokesWitnessAndOnlyLaterExactFeedbackRecovers) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();
  const ExecutionHorizonWitnessAdmissionResult heartbeat_update =
      admitExecutionHorizonFeedback(
          witnessed, heartbeat(kOffboardProducer, 1'040'000'000LL, 1'050'000'000LL));
  ASSERT_TRUE(heartbeat_update.accept);
  EXPECT_FALSE(heartbeat_update.session_transitioned);
  EXPECT_TRUE(heartbeat_update.witness_revoked);
  EXPECT_TRUE(heartbeat_update.next_state.latest_horizon_feedback.empty());
  EXPECT_EQ(heartbeat_update.next_state.horizon_feedback_source_stamp_high_water_ns,
            1'040'000'000LL);
  EXPECT_EQ(heartbeat_update.next_state.horizon_feedback_receive_stamp_high_water_ns,
            1'050'000'000LL);

  const ExecutionHorizonFeedbackCandidate next_feedback =
      feedback(kOffboardProducer, kHorizonProducer, 5U, 1'060'000'000LL,
               1'070'000'000LL, ExecutionHorizonWitnessMode::kPositionHold);
  const ExecutionHorizonWitnessAdmissionResult next =
      admitExecutionHorizonFeedback(heartbeat_update.next_state, next_feedback);
  ASSERT_TRUE(next.accept);
  EXPECT_TRUE(next.witness_updated);
  EXPECT_EQ(next.next_state.latest_horizon_feedback.horizon_sequence, 5U);
  EXPECT_EQ(next.next_state.latest_horizon_feedback.execution_mode,
            ExecutionHorizonWitnessMode::kPositionHold);
  EXPECT_FALSE(next.next_state.latest_horizon_feedback.control_authoritative);
  EXPECT_TRUE(executionHorizonReceiptFreshAt(
      next.next_state,
      requirement(kOffboardProducer, kHorizonProducer, 5U,
                  ExecutionHorizonWitnessMode::kPositionHold),
      1'100'000'000LL, kMaximumAgeNs));
  EXPECT_TRUE(executionHorizonWitnessFreshAt(
      next.next_state,
      requirement(kOffboardProducer, kHorizonProducer, 5U,
                  ExecutionHorizonWitnessMode::kPositionHold),
      1'100'000'000LL, kMaximumAgeNs));
}

TEST(ExecutionHorizonWitnessTest,
     MalformedCurrentIdentityImmediatelyTombstonesExactWitness) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();
  const ExecutionHorizonFeedbackCandidate malformed_identity{
      .offboard_producer_instance_id = kOffboardProducer,
      .horizon_producer_instance_id = kHorizonProducer,
      .horizon_sequence = 4U,
  };

  const ExecutionHorizonWitnessAdmissionResult malformed =
      revokeMalformedExecutionHorizonFeedback(witnessed, malformed_identity,
                                              1'040'000'000LL);
  ASSERT_TRUE(malformed.witness_revoked);
  EXPECT_TRUE(malformed.conflict);
  EXPECT_TRUE(malformed.invalid);
  EXPECT_TRUE(malformed.next_state.latest_horizon_feedback.empty());
  EXPECT_EQ(malformed.next_state.horizon_feedback_source_stamp_high_water_ns,
            witnessed.horizon_feedback_source_stamp_high_water_ns);
  EXPECT_EQ(malformed.next_state.horizon_feedback_receive_stamp_high_water_ns,
            witnessed.horizon_feedback_receive_stamp_high_water_ns);
  EXPECT_TRUE(malformed.state_advanced);
  EXPECT_TRUE(malformed.next_state.feedback_source_identity.tombstoned);

  const ExecutionHorizonWitnessAdmissionResult delayed_exact =
      admitExecutionHorizonFeedback(malformed.next_state, feedback());
  EXPECT_FALSE(delayed_exact.accept);
  EXPECT_TRUE(delayed_exact.replay);
  EXPECT_FALSE(delayed_exact.conflict);
  EXPECT_TRUE(delayed_exact.witness_revoked);
  EXPECT_FALSE(delayed_exact.state_advanced);

  ExecutionHorizonFeedbackCandidate different_horizon = malformed_identity;
  ++different_horizon.horizon_sequence;
  const ExecutionHorizonWitnessAdmissionResult unrelated =
      revokeMalformedExecutionHorizonFeedback(witnessed, different_horizon,
                                              1'040'000'000LL);
  EXPECT_FALSE(unrelated.witness_revoked);
}

TEST(ExecutionHorizonWitnessTest,
     MalformedCurrentSessionHeartbeatImmediatelyRevokesWitness) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();
  const ExecutionHorizonFeedbackCandidate malformed_heartbeat{
      .offboard_producer_instance_id = kOffboardProducer,
  };

  const ExecutionHorizonWitnessAdmissionResult malformed =
      revokeMalformedExecutionHorizonFeedback(witnessed, malformed_heartbeat,
                                              1'040'000'000LL);

  EXPECT_TRUE(malformed.witness_revoked);
  EXPECT_TRUE(malformed.next_state.latest_horizon_feedback.empty());
}

TEST(ExecutionHorizonWitnessTest,
     IdentifiableMalformedSourceIsClaimedBeforeModeAndContentValidation) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();
  ExecutionHorizonFeedbackCandidate malformed =
      feedback(kOffboardProducer, kHorizonProducer, 4U, 1'040'000'000LL,
               1'050'000'000LL, ExecutionHorizonWitnessMode::kPlanned, 0U);
  malformed.execution_mode = static_cast<ExecutionHorizonWitnessMode>(255U);

  const ExecutionHorizonWitnessAdmissionResult claimed =
      revokeMalformedExecutionHorizonFeedback(witnessed, malformed,
                                              malformed.receive_stamp_ns);
  EXPECT_FALSE(claimed.accept);
  EXPECT_TRUE(claimed.invalid);
  EXPECT_TRUE(claimed.state_advanced);
  EXPECT_TRUE(claimed.witness_revoked);
  EXPECT_TRUE(claimed.next_state.feedback_source_identity.tombstoned);
  EXPECT_EQ(claimed.next_state.feedback_source_identity.candidate.execution_mode,
            malformed.execution_mode);
  EXPECT_EQ(claimed.next_state.feedback_source_identity.candidate.content_fingerprint,
            0U);
  EXPECT_EQ(claimed.next_state.offboard_session.latest_source_stamp_ns,
            witnessed.offboard_session.latest_source_stamp_ns);
  EXPECT_EQ(claimed.next_state.latest_session_receive_stamp_ns,
            witnessed.latest_session_receive_stamp_ns);
  EXPECT_TRUE(claimed.next_state.valid());

  const ExecutionHorizonWitnessAdmissionResult replay =
      revokeMalformedExecutionHorizonFeedback(claimed.next_state, malformed,
                                              1'090'000'000LL);
  EXPECT_TRUE(replay.replay);
  EXPECT_TRUE(replay.invalid);
  EXPECT_FALSE(replay.state_advanced);
  EXPECT_FALSE(replay.accept);
  EXPECT_EQ(replay.next_state.feedback_source_identity.candidate.receive_stamp_ns,
            1'050'000'000LL);
  EXPECT_EQ(replay.next_state.horizon_feedback_receive_stamp_high_water_ns,
            1'050'000'000LL);
  EXPECT_EQ(replay.next_state.latest_session_receive_stamp_ns,
            witnessed.latest_session_receive_stamp_ns);

  const ExecutionHorizonWitnessAdmissionResult mutation = admitExecutionHorizonFeedback(
      claimed.next_state,
      feedback(kOffboardProducer, kHorizonProducer, 4U, 1'040'000'000LL,
               1'060'000'000LL, ExecutionHorizonWitnessMode::kPlanned, 103U));
  EXPECT_TRUE(mutation.conflict);
  EXPECT_FALSE(mutation.state_advanced);
  EXPECT_FALSE(mutation.accept);
  EXPECT_EQ(mutation.next_state.feedback_source_identity.candidate.content_fingerprint,
            0U);

  const ExecutionHorizonWitnessAdmissionResult recovered =
      admitExecutionHorizonFeedback(
          claimed.next_state,
          feedback(kOffboardProducer, kHorizonProducer, 4U, 1'060'000'000LL,
                   1'070'000'000LL, ExecutionHorizonWitnessMode::kPlanned, 104U));
  EXPECT_TRUE(recovered.accept);
  EXPECT_TRUE(recovered.state_advanced);
  EXPECT_TRUE(recovered.witness_updated);
  EXPECT_FALSE(recovered.next_state.feedback_source_identity.tombstoned);
  EXPECT_TRUE(recovered.next_state.valid());
}

TEST(ExecutionHorizonWitnessTest, HeartbeatTombstoneRejectsDelayedExactFeedback) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();
  const ExecutionHorizonWitnessAdmissionResult heartbeat_update =
      admitExecutionHorizonFeedback(
          witnessed, heartbeat(kOffboardProducer, 1'040'000'000LL, 1'050'000'000LL));
  ASSERT_TRUE(heartbeat_update.accept);
  ASSERT_TRUE(heartbeat_update.next_state.latest_horizon_feedback.empty());

  ExecutionHorizonFeedbackCandidate delayed = feedback();
  delayed.receive_stamp_ns = 1'060'000'000LL;
  const ExecutionHorizonWitnessAdmissionResult rejected =
      admitExecutionHorizonFeedback(heartbeat_update.next_state, delayed);
  EXPECT_FALSE(rejected.accept);
  EXPECT_TRUE(rejected.stale);
  EXPECT_FALSE(rejected.witness_revoked);
  EXPECT_TRUE(rejected.next_state.latest_horizon_feedback.empty());
  EXPECT_FALSE(executionHorizonWitnessFreshAt(rejected.next_state, requirement(),
                                              1'100'000'000LL, kMaximumAgeNs));

  const ExecutionHorizonWitnessAdmissionResult recovery = admitExecutionHorizonFeedback(
      rejected.next_state, feedback(kOffboardProducer, kHorizonProducer, 4U,
                                    1'060'000'000LL, 1'080'000'000LL));
  EXPECT_TRUE(recovery.accept);
  EXPECT_TRUE(recovery.witness_updated);
  EXPECT_TRUE(recovery.next_state.latest_horizon_feedback.valid());
}

TEST(ExecutionHorizonWitnessTest,
     SameStampHeartbeatConflictsWithExactWitnessAndTombstonesThatStamp) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();
  const std::int64_t witness_stamp = witnessed.latest_horizon_feedback.source_stamp_ns;
  const ExecutionHorizonWitnessAdmissionResult heartbeat_update =
      admitExecutionHorizonFeedback(
          witnessed, heartbeat(kOffboardProducer, witness_stamp,
                               witnessed.latest_session_receive_stamp_ns));

  ASSERT_FALSE(heartbeat_update.accept);
  EXPECT_TRUE(heartbeat_update.conflict);
  EXPECT_TRUE(heartbeat_update.witness_revoked);
  EXPECT_TRUE(heartbeat_update.state_advanced);
  EXPECT_TRUE(heartbeat_update.next_state.latest_horizon_feedback.empty());
  EXPECT_EQ(heartbeat_update.next_state.horizon_feedback_source_stamp_high_water_ns,
            witness_stamp);

  ExecutionHorizonFeedbackCandidate delayed_exact = feedback();
  delayed_exact.receive_stamp_ns += 1;
  const ExecutionHorizonWitnessAdmissionResult rejected =
      admitExecutionHorizonFeedback(heartbeat_update.next_state, delayed_exact);
  EXPECT_FALSE(rejected.accept);
  EXPECT_TRUE(rejected.replay);
  EXPECT_FALSE(rejected.conflict);
  EXPECT_FALSE(rejected.state_advanced);
  EXPECT_TRUE(rejected.next_state.latest_horizon_feedback.empty());
}

TEST(ExecutionHorizonWitnessTest,
     SameSourceHeartbeatWireMutationTombstonesAcceptedLivenessIdentity) {
  const ExecutionHorizonWitnessState session = initialSession();
  ExecutionHorizonFeedbackCandidate mutated = heartbeat();
  mutated.receive_stamp_ns += 10'000'000LL;
  ++mutated.wire_fingerprint;

  const ExecutionHorizonWitnessAdmissionResult conflict =
      admitExecutionHorizonFeedback(session, mutated);
  EXPECT_FALSE(conflict.accept);
  EXPECT_TRUE(conflict.conflict);
  EXPECT_TRUE(conflict.state_advanced);
  EXPECT_TRUE(conflict.next_state.feedback_source_identity.tombstoned);
  EXPECT_EQ(conflict.next_state.latest_session_receive_stamp_ns,
            session.latest_session_receive_stamp_ns);

  ExecutionHorizonFeedbackCandidate exact = heartbeat();
  exact.receive_stamp_ns += 20'000'000LL;
  const ExecutionHorizonWitnessAdmissionResult replay =
      admitExecutionHorizonFeedback(conflict.next_state, exact);
  EXPECT_FALSE(replay.accept);
  EXPECT_TRUE(replay.replay);
  EXPECT_FALSE(replay.state_advanced);
  EXPECT_EQ(replay.next_state.feedback_source_identity.candidate.receive_stamp_ns,
            kHeartbeatReceiveNs);
  EXPECT_EQ(replay.next_state.latest_session_receive_stamp_ns,
            session.latest_session_receive_stamp_ns);
}

TEST(ExecutionHorizonWitnessTest,
     MalformedFirstSourceClaimsProspectAndRequiresHigherHeartbeatForStartup) {
  ExecutionHorizonFeedbackCandidate malformed =
      feedback(18U, kHorizonProducer, 4U, 1'000'000'000LL, 1'010'000'000LL);
  const ExecutionHorizonWitnessAdmissionResult claimed =
      revokeMalformedExecutionHorizonFeedback({}, malformed,
                                              malformed.receive_stamp_ns);
  EXPECT_FALSE(claimed.accept);
  EXPECT_TRUE(claimed.invalid);
  EXPECT_TRUE(claimed.state_advanced);
  EXPECT_EQ(claimed.next_state.offboard_session.current_producer_instance_id, 0U);
  ASSERT_EQ(claimed.next_state.prospective_feedback_source_count, 1U);
  EXPECT_TRUE(claimed.next_state.valid());

  const ExecutionHorizonWitnessAdmissionResult mutation = admitExecutionHorizonFeedback(
      claimed.next_state, heartbeat(18U, malformed.source_stamp_ns,
                                    malformed.receive_stamp_ns + 10'000'000LL));
  EXPECT_FALSE(mutation.accept);
  EXPECT_TRUE(mutation.conflict);
  EXPECT_FALSE(mutation.state_advanced);
  EXPECT_EQ(mutation.next_state.offboard_session.current_producer_instance_id, 0U);

  const ExecutionHorizonWitnessAdmissionResult recovery = admitExecutionHorizonFeedback(
      mutation.next_state, heartbeat(18U, malformed.source_stamp_ns + 20'000'000LL,
                                     malformed.receive_stamp_ns + 20'000'000LL));
  EXPECT_TRUE(recovery.accept);
  EXPECT_TRUE(recovery.session_transitioned);
  EXPECT_EQ(recovery.next_state.offboard_session.current_producer_instance_id, 18U);
  EXPECT_EQ(recovery.next_state.prospective_feedback_source_count, 0U);
  EXPECT_TRUE(recovery.next_state.valid());
}

TEST(ExecutionHorizonWitnessTest,
     MalformedProspectiveSourceCannotMutateIntoSameStampHandoff) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();
  ExecutionHorizonFeedbackCandidate malformed =
      feedback(18U, kHorizonProducer, 4U, 1'100'000'000LL, 1'110'000'000LL);
  const ExecutionHorizonWitnessAdmissionResult claimed =
      revokeMalformedExecutionHorizonFeedback(witnessed, malformed,
                                              malformed.receive_stamp_ns);
  EXPECT_FALSE(claimed.accept);
  EXPECT_TRUE(claimed.invalid);
  EXPECT_TRUE(claimed.non_current_session);
  EXPECT_TRUE(claimed.state_advanced);
  EXPECT_EQ(claimed.next_state.offboard_session.current_producer_instance_id,
            kOffboardProducer);
  EXPECT_EQ(claimed.next_state.latest_session_receive_stamp_ns,
            witnessed.latest_session_receive_stamp_ns);
  ASSERT_EQ(claimed.next_state.prospective_feedback_source_count, 1U);

  const ExecutionHorizonWitnessAdmissionResult replay =
      revokeMalformedExecutionHorizonFeedback(
          claimed.next_state, malformed, malformed.receive_stamp_ns + 50'000'000LL);
  EXPECT_TRUE(replay.replay);
  EXPECT_FALSE(replay.state_advanced);
  EXPECT_EQ(replay.next_state.prospective_feedback_source_identities[0]
                .candidate.receive_stamp_ns,
            malformed.receive_stamp_ns);

  const ExecutionHorizonWitnessAdmissionResult mutation = admitExecutionHorizonFeedback(
      replay.next_state, heartbeat(18U, malformed.source_stamp_ns,
                                   malformed.receive_stamp_ns + 60'000'000LL));
  EXPECT_FALSE(mutation.accept);
  EXPECT_TRUE(mutation.conflict);
  EXPECT_FALSE(mutation.state_advanced);
  EXPECT_EQ(mutation.next_state.offboard_session.current_producer_instance_id,
            kOffboardProducer);

  const ExecutionHorizonWitnessAdmissionResult recovery = admitExecutionHorizonFeedback(
      mutation.next_state, heartbeat(18U, malformed.source_stamp_ns + 70'000'000LL,
                                     malformed.receive_stamp_ns + 70'000'000LL));
  EXPECT_TRUE(recovery.accept);
  EXPECT_TRUE(recovery.session_transitioned);
  EXPECT_EQ(recovery.next_state.offboard_session.current_producer_instance_id, 18U);
  EXPECT_TRUE(recovery.next_state.offboard_session.producerRetired(kOffboardProducer));
  EXPECT_EQ(recovery.next_state.prospective_feedback_source_count, 0U);
}

TEST(ExecutionHorizonWitnessTest,
     ProspectiveCapacityLatchStaysClosedAfterExistingClaimFreesSlot) {
  ExecutionHorizonWitnessState state = stateWithWitness();
  for (std::size_t index = 0U; index < kExecutionHorizonFeedbackProspectiveCapacity;
       ++index) {
    const std::uint64_t producer = 100U + static_cast<std::uint64_t>(index);
    const std::int64_t source =
        1'100'000'000LL + static_cast<std::int64_t>(index) * 20'000'000LL;
    const ExecutionHorizonFeedbackCandidate malformed =
        feedback(producer, kHorizonProducer, 4U, source, source + 10'000'000LL);
    const ExecutionHorizonWitnessAdmissionResult claimed =
        revokeMalformedExecutionHorizonFeedback(state, malformed,
                                                malformed.receive_stamp_ns);
    ASSERT_TRUE(claimed.state_advanced);
    ASSERT_FALSE(claimed.accept);
    state = claimed.next_state;
  }
  ASSERT_EQ(state.prospective_feedback_source_count,
            kExecutionHorizonFeedbackProspectiveCapacity);

  const ExecutionHorizonFeedbackCandidate overflow_candidate =
      feedback(200U, kHorizonProducer, 4U, 1'500'000'000LL, 1'510'000'000LL);
  const ExecutionHorizonWitnessAdmissionResult overflow =
      revokeMalformedExecutionHorizonFeedback(state, overflow_candidate,
                                              overflow_candidate.receive_stamp_ns);
  EXPECT_FALSE(overflow.accept);
  EXPECT_TRUE(overflow.invalid);
  EXPECT_TRUE(overflow.prospective_capacity_exhausted);
  EXPECT_TRUE(overflow.state_advanced);
  EXPECT_TRUE(overflow.next_state.prospective_feedback_source_capacity_exhausted);

  const ExecutionHorizonFeedbackProspectiveSourceIdentity first_claim =
      overflow.next_state.prospective_feedback_source_identities[0];
  const ExecutionHorizonWitnessAdmissionResult handoff = admitExecutionHorizonFeedback(
      overflow.next_state,
      heartbeat(first_claim.candidate.offboard_producer_instance_id,
                first_claim.candidate.source_stamp_ns + 500'000'000LL,
                first_claim.candidate.receive_stamp_ns + 500'000'000LL));
  ASSERT_TRUE(handoff.accept);
  ASSERT_TRUE(handoff.session_transitioned);
  ASSERT_EQ(handoff.next_state.prospective_feedback_source_count,
            kExecutionHorizonFeedbackProspectiveCapacity - 1U);

  const ExecutionHorizonWitnessAdmissionResult still_closed =
      revokeMalformedExecutionHorizonFeedback(handoff.next_state, overflow_candidate,
                                              overflow_candidate.receive_stamp_ns +
                                                  1'000'000'000LL);
  EXPECT_FALSE(still_closed.accept);
  EXPECT_TRUE(still_closed.invalid);
  EXPECT_TRUE(still_closed.prospective_capacity_exhausted);
  EXPECT_FALSE(still_closed.state_advanced);
  EXPECT_EQ(still_closed.next_state.prospective_feedback_source_count,
            kExecutionHorizonFeedbackProspectiveCapacity - 1U);
}

TEST(ExecutionHorizonWitnessTest,
     SameSourceConflictRevokesRegardlessOfReceiveOrdering) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();
  ASSERT_EQ(witnessed.horizon_feedback_receive_stamp_high_water_ns, 1'030'000'000LL);

  ExecutionHorizonFeedbackCandidate delayed_conflict = feedback();
  ++delayed_conflict.content_fingerprint;
  delayed_conflict.receive_stamp_ns = 1'025'000'000LL;
  const ExecutionHorizonWitnessAdmissionResult conflict =
      admitExecutionHorizonFeedback(witnessed, delayed_conflict);

  EXPECT_FALSE(conflict.accept);
  EXPECT_TRUE(conflict.conflict);
  EXPECT_TRUE(conflict.witness_revoked);
  EXPECT_TRUE(conflict.next_state.latest_horizon_feedback.empty());
  EXPECT_EQ(conflict.next_state.horizon_feedback_receive_stamp_high_water_ns,
            witnessed.horizon_feedback_receive_stamp_high_water_ns);
}

TEST(ExecutionHorizonWitnessTest,
     InvalidPayloadClaimsTombstoneAndRequiresStrictlyNewerRecovery) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();
  const ExecutionHorizonWitnessAdmissionResult invalid_successor =
      admitExecutionHorizonFeedbackPayload(
          witnessed,
          feedback(kOffboardProducer, kHorizonProducer, 4U, 1'040'000'000LL,
                   1'050'000'000LL, ExecutionHorizonWitnessMode::kPlanned, 102U),
          false);
  EXPECT_FALSE(invalid_successor.accept);
  EXPECT_FALSE(invalid_successor.witness_updated);
  EXPECT_TRUE(invalid_successor.witness_revoked);
  EXPECT_TRUE(invalid_successor.state_advanced);
  EXPECT_TRUE(invalid_successor.next_state.latest_horizon_feedback.empty());
  EXPECT_TRUE(invalid_successor.next_state.feedback_source_identity.tombstoned);
  EXPECT_TRUE(invalid_successor.next_state.valid());

  const ExecutionHorizonWitnessAdmissionResult replay =
      admitExecutionHorizonFeedbackPayload(
          invalid_successor.next_state,
          feedback(kOffboardProducer, kHorizonProducer, 4U, 1'040'000'000LL,
                   1'060'000'000LL, ExecutionHorizonWitnessMode::kPlanned, 102U),
          true);
  EXPECT_FALSE(replay.accept);
  EXPECT_TRUE(replay.replay);
  EXPECT_FALSE(replay.conflict);
  EXPECT_TRUE(replay.witness_revoked);
  EXPECT_FALSE(replay.state_advanced);
  EXPECT_EQ(replay.next_state.feedback_source_identity.candidate.receive_stamp_ns,
            1'050'000'000LL);
  EXPECT_EQ(replay.next_state.horizon_feedback_receive_stamp_high_water_ns,
            1'050'000'000LL);

  const ExecutionHorizonWitnessAdmissionResult recovery =
      admitExecutionHorizonFeedbackPayload(
          replay.next_state,
          feedback(kOffboardProducer, kHorizonProducer, 4U, 1'070'000'000LL,
                   1'080'000'000LL, ExecutionHorizonWitnessMode::kPlanned, 103U),
          true);
  EXPECT_TRUE(recovery.accept);
  EXPECT_TRUE(recovery.witness_updated);
  EXPECT_TRUE(recovery.next_state.latest_horizon_feedback.valid());
}

TEST(ExecutionHorizonWitnessTest,
     InvalidHeartbeatRevokesWitnessWhileMalformedCandidateDoesNotMutate) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();
  const ExecutionHorizonWitnessAdmissionResult invalid_heartbeat =
      admitExecutionHorizonFeedbackPayload(
          witnessed, heartbeat(kOffboardProducer, 1'040'000'000LL, 1'050'000'000LL),
          false);
  EXPECT_FALSE(invalid_heartbeat.accept);
  EXPECT_TRUE(invalid_heartbeat.heartbeat);
  EXPECT_TRUE(invalid_heartbeat.witness_revoked);
  EXPECT_TRUE(invalid_heartbeat.next_state.latest_horizon_feedback.empty());
  EXPECT_TRUE(invalid_heartbeat.next_state.valid());

  const ExecutionHorizonWitnessAdmissionResult malformed =
      admitExecutionHorizonFeedbackPayload(witnessed, {}, false);
  EXPECT_FALSE(malformed.accept);
  EXPECT_TRUE(malformed.invalid);
  EXPECT_FALSE(malformed.witness_revoked);
  EXPECT_EQ(malformed.next_state.latest_horizon_feedback.content_fingerprint,
            witnessed.latest_horizon_feedback.content_fingerprint);
}

TEST(ExecutionHorizonWitnessTest,
     SessionTransitionClearsWitnessAndRejectsRetiredFeedback) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();
  const ExecutionHorizonWitnessAdmissionResult transition =
      admitExecutionHorizonFeedback(witnessed,
                                    heartbeat(8U, 1'100'000'000LL, 1'110'000'000LL));
  ASSERT_TRUE(transition.accept);
  EXPECT_TRUE(transition.session_transitioned);
  EXPECT_TRUE(transition.next_state.latest_horizon_feedback.empty());
  EXPECT_EQ(transition.next_state.offboard_session.current_producer_instance_id, 8U);
  EXPECT_EQ(transition.next_state.horizon_feedback_source_stamp_high_water_ns,
            1'100'000'000LL);

  const ExecutionHorizonWitnessAdmissionResult retired = admitExecutionHorizonFeedback(
      transition.next_state, feedback(kOffboardProducer, kHorizonProducer, 4U,
                                      1'120'000'000LL, 1'130'000'000LL));
  EXPECT_FALSE(retired.accept);
  EXPECT_TRUE(retired.non_current_session);
  EXPECT_TRUE(retired.stale);
  EXPECT_TRUE(retired.next_state.latest_horizon_feedback.empty());
}

TEST(ExecutionHorizonWitnessTest, RetirementCapacityFailsClosedWithoutEviction) {
  ExecutionHorizonWitnessState state = initialSession();
  for (std::size_t transition = 0U; transition < kRetiredOffboardSessionCapacity;
       ++transition) {
    const std::uint64_t producer =
        static_cast<std::uint64_t>(transition) + kOffboardProducer + 1U;
    const std::int64_t source =
        kHeartbeatSourceNs + static_cast<std::int64_t>(transition + 2U) * 10'000'000LL;
    const ExecutionHorizonWitnessAdmissionResult handoff =
        admitExecutionHorizonFeedback(state, heartbeat(producer, source, source));
    ASSERT_TRUE(handoff.accept);
    ASSERT_TRUE(handoff.session_transitioned);
    state = handoff.next_state;
  }
  ASSERT_EQ(state.offboard_session.retired_producer_count,
            kRetiredOffboardSessionCapacity);

  const ExecutionHorizonWitnessAdmissionResult overflow = admitExecutionHorizonFeedback(
      state, heartbeat(99U, 2'000'000'000LL, 2'000'000'000LL));
  EXPECT_FALSE(overflow.accept);
  EXPECT_TRUE(overflow.invalid);
  EXPECT_EQ(overflow.next_state.offboard_session.current_producer_instance_id,
            state.offboard_session.current_producer_instance_id);

  const ExecutionHorizonWitnessAdmissionResult retired_replay =
      admitExecutionHorizonFeedback(
          state, heartbeat(kOffboardProducer, 2'100'000'000LL, 2'100'000'000LL));
  EXPECT_FALSE(retired_replay.accept);
  EXPECT_TRUE(retired_replay.stale);
}

TEST(ExecutionHorizonWitnessTest, RejectsMalformedFeedbackContracts) {
  const ExecutionHorizonWitnessState session = initialSession();
  ExecutionHorizonFeedbackCandidate candidate = heartbeat();

  candidate.offboard_producer_instance_id = 0U;
  EXPECT_TRUE(admitExecutionHorizonFeedback(session, candidate).invalid);

  candidate = heartbeat();
  candidate.horizon_producer_instance_id = kHorizonProducer;
  EXPECT_TRUE(admitExecutionHorizonFeedback(session, candidate).invalid);

  candidate = heartbeat();
  candidate.horizon_sequence = 4U;
  EXPECT_TRUE(admitExecutionHorizonFeedback(session, candidate).invalid);

  candidate = feedback();
  candidate.execution_mode = static_cast<ExecutionHorizonWitnessMode>(255U);
  EXPECT_TRUE(admitExecutionHorizonFeedback(session, candidate).invalid);

  candidate = feedback();
  candidate.execution_mode = ExecutionHorizonWitnessMode::kPositionHold;
  candidate.control_authoritative = true;
  EXPECT_TRUE(admitExecutionHorizonFeedback(session, candidate).invalid);

  candidate = feedback();
  candidate.source_stamp_ns = 0;
  EXPECT_TRUE(admitExecutionHorizonFeedback(session, candidate).invalid);

  candidate = feedback();
  candidate.receive_stamp_ns = candidate.source_stamp_ns - 1;
  EXPECT_FALSE(admitExecutionHorizonFeedback(session, candidate).invalid);

  candidate = feedback();
  candidate.content_fingerprint = 0U;
  EXPECT_TRUE(admitExecutionHorizonFeedback(session, candidate).invalid);

  candidate = heartbeat();
  candidate.content_fingerprint = 1U;
  EXPECT_TRUE(admitExecutionHorizonFeedback(session, candidate).invalid);
}

TEST(ExecutionHorizonWitnessTest, RejectsStaleReplayAndConflictingFeedback) {
  const ExecutionHorizonWitnessState witnessed = stateWithWitness();

  const ExecutionHorizonWitnessAdmissionResult heartbeat_replay =
      admitExecutionHorizonFeedback(
          witnessed, heartbeat(kOffboardProducer, 1'020'000'000LL, 1'040'000'000LL));
  EXPECT_FALSE(heartbeat_replay.accept);
  EXPECT_TRUE(heartbeat_replay.conflict);
  EXPECT_TRUE(heartbeat_replay.witness_revoked);
  EXPECT_TRUE(heartbeat_replay.state_advanced);
  EXPECT_TRUE(heartbeat_replay.next_state.latest_horizon_feedback.empty());
  EXPECT_EQ(heartbeat_replay.next_state.horizon_feedback_source_stamp_high_water_ns,
            1'020'000'000LL);

  const ExecutionHorizonWitnessAdmissionResult witness_replay =
      admitExecutionHorizonFeedback(witnessed, feedback());
  EXPECT_FALSE(witness_replay.accept);
  EXPECT_TRUE(witness_replay.replay);

  ExecutionHorizonFeedbackCandidate conflicting = feedback();
  ++conflicting.horizon_sequence;
  const ExecutionHorizonWitnessAdmissionResult conflict =
      admitExecutionHorizonFeedback(witnessed, conflicting);
  EXPECT_FALSE(conflict.accept);
  EXPECT_TRUE(conflict.conflict);
  EXPECT_TRUE(conflict.witness_revoked);
  EXPECT_TRUE(conflict.next_state.latest_horizon_feedback.empty());
  EXPECT_FALSE(executionHorizonWitnessFreshAt(conflict.next_state, requirement(),
                                              1'100'000'000LL, kMaximumAgeNs));
  const ExecutionHorizonWitnessAdmissionResult replay_after_conflict =
      admitExecutionHorizonFeedback(conflict.next_state, feedback());
  EXPECT_FALSE(replay_after_conflict.accept);
  EXPECT_TRUE(replay_after_conflict.replay);
  EXPECT_FALSE(replay_after_conflict.conflict);
  EXPECT_TRUE(replay_after_conflict.next_state.latest_horizon_feedback.empty());
  const ExecutionHorizonWitnessAdmissionResult recovery = admitExecutionHorizonFeedback(
      conflict.next_state, feedback(kOffboardProducer, kHorizonProducer, 6U,
                                    1'040'000'000LL, 1'050'000'000LL));
  EXPECT_TRUE(recovery.accept);
  EXPECT_TRUE(recovery.next_state.latest_horizon_feedback.valid());

  conflicting = feedback();
  conflicting.control_authoritative = false;
  const ExecutionHorizonWitnessAdmissionResult authority_conflict =
      admitExecutionHorizonFeedback(witnessed, conflicting);
  EXPECT_FALSE(authority_conflict.accept);
  EXPECT_TRUE(authority_conflict.conflict);
  EXPECT_TRUE(authority_conflict.witness_revoked);
  EXPECT_TRUE(authority_conflict.next_state.latest_horizon_feedback.empty());

  conflicting = feedback();
  ++conflicting.content_fingerprint;
  const ExecutionHorizonWitnessAdmissionResult content_conflict =
      admitExecutionHorizonFeedback(witnessed, conflicting);
  EXPECT_FALSE(content_conflict.accept);
  EXPECT_TRUE(content_conflict.conflict);
  EXPECT_TRUE(content_conflict.witness_revoked);
  EXPECT_TRUE(content_conflict.next_state.latest_horizon_feedback.empty());

  const ExecutionHorizonWitnessAdmissionResult stale_sequence =
      admitExecutionHorizonFeedback(witnessed,
                                    feedback(kOffboardProducer, kHorizonProducer, 3U,
                                             1'040'000'000LL, 1'050'000'000LL));
  EXPECT_FALSE(stale_sequence.accept);
  EXPECT_TRUE(stale_sequence.stale);
  EXPECT_TRUE(stale_sequence.state_advanced);
  EXPECT_TRUE(stale_sequence.witness_revoked);
  EXPECT_EQ(
      stale_sequence.next_state.feedback_source_identity.candidate.horizon_sequence,
      3U);
  EXPECT_TRUE(stale_sequence.next_state.feedback_source_identity.tombstoned);

  const ExecutionHorizonWitnessAdmissionResult stale_heartbeat =
      admitExecutionHorizonFeedback(witnessed,
                                    heartbeat(kOffboardProducer, kHeartbeatSourceNs - 1,
                                              kHeartbeatReceiveNs + 1));
  EXPECT_FALSE(stale_heartbeat.accept);
  EXPECT_TRUE(stale_heartbeat.stale);
}

TEST(ExecutionHorizonWitnessTest,
     ReceiptAndExecutionWitnessDistinguishUnavailablePlannedFeedback) {
  ExecutionHorizonWitnessState state = initialSession();
  ExecutionHorizonFeedbackCandidate unavailable = feedback();
  unavailable.control_authoritative = false;
  const ExecutionHorizonWitnessAdmissionResult unavailable_result =
      admitExecutionHorizonFeedback(state, unavailable);
  ASSERT_TRUE(unavailable_result.accept);
  state = unavailable_result.next_state;
  EXPECT_EQ(state.offboard_session.latest_source_stamp_ns, unavailable.source_stamp_ns);
  EXPECT_EQ(state.latest_session_receive_stamp_ns, unavailable.receive_stamp_ns);

  const std::int64_t now_ns = 1'100'000'000LL;
  EXPECT_TRUE(
      executionHorizonReceiptFreshAt(state, requirement(), now_ns, kMaximumAgeNs));
  EXPECT_FALSE(
      executionHorizonWitnessFreshAt(state, requirement(), now_ns, kMaximumAgeNs));

  const ExecutionHorizonWitnessAdmissionResult authoritative_result =
      admitExecutionHorizonFeedback(state,
                                    feedback(kOffboardProducer, kHorizonProducer, 4U,
                                             1'040'000'000LL, 1'050'000'000LL));
  ASSERT_TRUE(authoritative_result.accept);
  state = authoritative_result.next_state;
  EXPECT_TRUE(
      executionHorizonReceiptFreshAt(state, requirement(), now_ns, kMaximumAgeNs));
  EXPECT_TRUE(
      executionHorizonWitnessFreshAt(state, requirement(), now_ns, kMaximumAgeNs));

  unavailable.source_stamp_ns = 1'060'000'000LL;
  unavailable.receive_stamp_ns = 1'070'000'000LL;
  const ExecutionHorizonWitnessAdmissionResult unavailable_again =
      admitExecutionHorizonFeedback(state, unavailable);
  ASSERT_TRUE(unavailable_again.accept);
  EXPECT_TRUE(executionHorizonReceiptFreshAt(unavailable_again.next_state,
                                             requirement(), now_ns, kMaximumAgeNs));
  EXPECT_FALSE(executionHorizonWitnessFreshAt(unavailable_again.next_state,
                                              requirement(), now_ns, kMaximumAgeNs));
}

TEST(ExecutionHorizonWitnessTest, FreshnessRequiresExactTupleModeAndWindow) {
  const ExecutionHorizonWitnessState state = stateWithWitness();
  const std::int64_t now_ns = 1'100'000'000LL;
  EXPECT_TRUE(
      executionHorizonWitnessFreshAt(state, requirement(), now_ns, kMaximumAgeNs));

  ExecutionHorizonWitnessRequirement check = requirement();
  ++check.target_offboard_instance_id;
  EXPECT_FALSE(executionHorizonWitnessFreshAt(state, check, now_ns, kMaximumAgeNs));
  check = requirement();
  ++check.horizon_producer_instance_id;
  EXPECT_FALSE(executionHorizonWitnessFreshAt(state, check, now_ns, kMaximumAgeNs));
  check = requirement();
  ++check.horizon_sequence;
  EXPECT_FALSE(executionHorizonWitnessFreshAt(state, check, now_ns, kMaximumAgeNs));
  check = requirement();
  check.execution_mode = ExecutionHorizonWitnessMode::kPositionHold;
  EXPECT_FALSE(executionHorizonWitnessFreshAt(state, check, now_ns, kMaximumAgeNs));
  check = requirement();
  check.valid_from_ns = now_ns + 1;
  EXPECT_FALSE(executionHorizonWitnessFreshAt(state, check, now_ns, kMaximumAgeNs));
  check = requirement();
  check.valid_until_ns = now_ns;
  EXPECT_FALSE(executionHorizonWitnessFreshAt(state, check, now_ns, kMaximumAgeNs));
}

TEST(ExecutionHorizonWitnessTest, FreshnessRejectsStaleAndFutureEvidence) {
  const ExecutionHorizonWitnessState state = stateWithWitness();
  EXPECT_FALSE(offboardSessionFreshAt(state, 2'100'000'001LL, kMaximumAgeNs));
  EXPECT_FALSE(executionHorizonWitnessFreshAt(state, requirement(), 2'100'000'001LL,
                                              kMaximumAgeNs));
  EXPECT_TRUE(offboardSessionFreshAt(state, kHeartbeatReceiveNs - 1, kMaximumAgeNs));
  EXPECT_TRUE(executionHorizonWitnessFreshAt(state, requirement(), 1'025'000'000LL,
                                             kMaximumAgeNs));
  EXPECT_FALSE(offboardSessionFreshAt(state, 1'100'000'000LL, 0));

  ExecutionHorizonWitnessState future_witness = state;
  future_witness.latest_horizon_feedback.source_stamp_ns = 1'200'000'000LL;
  future_witness.latest_horizon_feedback.receive_stamp_ns = 1'210'000'000LL;
  EXPECT_FALSE(executionHorizonWitnessFreshAt(future_witness, requirement(),
                                              1'100'000'000LL, kMaximumAgeNs));
}

} // namespace
} // namespace drone_city_nav
