#include "drone_city_nav/producer_epoch_admission.hpp"

#include <gtest/gtest.h>

#include <cstdint>
#include <limits>

namespace drone_city_nav {
namespace {

constexpr ProducerEpochAdmissionConfig kConfig{
    .maximum_observation_age_ns = 100,
    .maximum_confirmation_interval_ns = 50,
};

[[nodiscard]] ProducerEpochObservation
observation(const std::uint64_t producer, const std::uint64_t sequence,
            const std::int64_t source_stamp_ns, const std::int64_t receive_stamp_ns,
            const std::uint64_t fingerprint) noexcept {
  return ProducerEpochObservation{
      .producer_instance_id = producer,
      .sequence = sequence,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .content_fingerprint = fingerprint,
  };
}

[[nodiscard]] ProducerEpochAdmissionResult
admit(const ProducerEpochAdmissionState& state,
      const ProducerEpochObservation& candidate, const std::int64_t now_ns) noexcept {
  return admitProducerEpoch(kConfig, state, candidate, now_ns);
}

TEST(ProducerEpochAdmissionTest, ExactReplayDoesNotRejuvenateHighWater) {
  const ProducerEpochAdmissionResult initial =
      admit({}, observation(7U, 10U, 900, 910, 101U), 920);
  ASSERT_EQ(initial.status, ProducerEpochAdmissionStatus::kAcceptedInitial);

  const ProducerEpochAdmissionResult replay =
      admit(initial.next_state, observation(7U, 10U, 900, 990, 101U), 1'000);

  EXPECT_EQ(replay.status, ProducerEpochAdmissionStatus::kIdempotentReplay);
  EXPECT_FALSE(replay.install_observation);
  EXPECT_EQ(replay.next_state.current_receive_stamp_ns, 910);
  EXPECT_EQ(replay.next_state.current_source_stamp_ns, 900);
}

TEST(ProducerEpochAdmissionTest,
     SameIdentityConflictQuarantinesUntilStrictlyNewerObservation) {
  const ProducerEpochAdmissionResult initial =
      admit({}, observation(7U, 10U, 900, 910, 101U), 920);
  const ProducerEpochAdmissionResult conflict =
      admit(initial.next_state, observation(7U, 10U, 900, 930, 202U), 1'100);
  ASSERT_EQ(conflict.status, ProducerEpochAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_TRUE(conflict.current_identity_conflict);
  EXPECT_FALSE(producerEpochAuthority(conflict.next_state).valid());

  const ProducerEpochAdmissionResult repeated =
      admit(conflict.next_state, observation(7U, 10U, 900, 940, 101U), 1'100);
  EXPECT_EQ(repeated.status, ProducerEpochAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_FALSE(repeated.current_identity_conflict);

  const ProducerEpochAdmissionResult recovered =
      admit(repeated.next_state, observation(7U, 11U, 1'050, 1'060, 303U), 1'070);
  ASSERT_EQ(recovered.status, ProducerEpochAdmissionStatus::kAcceptedNewer);
  EXPECT_TRUE(producerEpochAuthority(recovered.next_state).valid());
  EXPECT_FALSE(recovered.next_state.current_identity_conflicted);
}

TEST(ProducerEpochAdmissionTest, RejectedHigherIdentityCannotReturnWithMutatedContent) {
  const ProducerEpochAdmissionResult initial =
      admit({}, observation(7U, 10U, 900, 910, 101U), 920);
  const ProducerEpochAdmissionResult stale =
      admit(initial.next_state, observation(7U, 20U, 800, 940, 202U), 950);
  ASSERT_EQ(stale.status, ProducerEpochAdmissionStatus::kRejectedStaleCandidate);
  EXPECT_EQ(stale.next_state.current_sequence, 10U);
  ASSERT_EQ(stale.next_state.rejected_identity_count, 1U);
  EXPECT_EQ(stale.next_state.rejected_identities[0].first_receive_stamp_ns, 940);

  const ProducerEpochAdmissionResult exact_replay =
      admit(stale.next_state, observation(7U, 20U, 800, 949, 202U), 950);
  EXPECT_EQ(exact_replay.status, ProducerEpochAdmissionStatus::kRejectedStaleCandidate);
  EXPECT_EQ(exact_replay.next_state.rejected_identities[0].first_receive_stamp_ns, 940);

  const ProducerEpochAdmissionResult conflict =
      admit(exact_replay.next_state, observation(7U, 20U, 800, 945, 999U), 950);
  ASSERT_EQ(conflict.status, ProducerEpochAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_TRUE(conflict.current_identity_conflict);
  EXPECT_FALSE(producerEpochAuthority(conflict.next_state).valid());

  const ProducerEpochAdmissionResult recovered =
      admit(conflict.next_state, observation(7U, 21U, 930, 940, 303U), 950);
  EXPECT_EQ(recovered.status, ProducerEpochAdmissionStatus::kAcceptedNewer);
  EXPECT_EQ(recovered.next_state.current_sequence, 21U);
  EXPECT_TRUE(producerEpochAuthority(recovered.next_state).valid());
}

TEST(ProducerEpochAdmissionTest,
     MalformedTemporalClaimCannotResurrectWithMutatedIdentity) {
  const ProducerEpochAdmissionResult malformed =
      admit({}, observation(7U, 1U, 0, 910, 101U), 920);
  ASSERT_EQ(malformed.status, ProducerEpochAdmissionStatus::kRejectedInvalid);
  ASSERT_EQ(malformed.next_state.rejected_identity_count, 1U);

  const ProducerEpochAdmissionResult exact_replay =
      admit(malformed.next_state, observation(7U, 1U, 0, 920, 101U), 930);
  EXPECT_EQ(exact_replay.status, ProducerEpochAdmissionStatus::kRejectedInvalid);
  EXPECT_EQ(exact_replay.next_state.rejected_identities[0].first_receive_stamp_ns, 910);

  const ProducerEpochAdmissionResult conflict =
      admit(exact_replay.next_state, observation(7U, 1U, 900, 920, 202U), 930);
  EXPECT_EQ(conflict.status, ProducerEpochAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_FALSE(conflict.install_observation);
  EXPECT_FALSE(producerEpochAuthority(conflict.next_state).valid());
}

TEST(ProducerEpochAdmissionTest,
     RejectedHigherIdentityPermanentlyRaisesSequenceHighWater) {
  const ProducerEpochAdmissionResult malformed =
      admit({}, observation(7U, 10U, 0, 910, 101U), 920);
  ASSERT_EQ(malformed.status, ProducerEpochAdmissionStatus::kRejectedInvalid);

  const ProducerEpochAdmissionResult lower =
      admit(malformed.next_state, observation(7U, 9U, 900, 920, 202U), 930);
  ASSERT_EQ(lower.status, ProducerEpochAdmissionStatus::kRejectedRegression);
  EXPECT_FALSE(lower.install_observation);
  EXPECT_EQ(lower.next_state.rejected_identity_count, 2U);

  const ProducerEpochAdmissionResult recovered =
      admit(lower.next_state, observation(7U, 11U, 920, 930, 303U), 940);
  EXPECT_EQ(recovered.status, ProducerEpochAdmissionStatus::kAcceptedInitial);
  EXPECT_TRUE(recovered.install_observation);
  EXPECT_EQ(recovered.next_state.current_sequence, 11U);
}

TEST(ProducerEpochAdmissionTest,
     DifferentProducerNeedsStaleCurrentAndTwoOrderedFreshSamples) {
  const ProducerEpochAdmissionResult initial =
      admit({}, observation(7U, 10U, 900, 910, 101U), 920);
  const ProducerEpochAdmissionResult fresh_rejection =
      admit(initial.next_state, observation(8U, 1U, 930, 940, 201U), 950);
  EXPECT_EQ(fresh_rejection.status,
            ProducerEpochAdmissionStatus::kRejectedCurrentProducerFresh);

  const ProducerEpochAdmissionResult pending =
      admit(initial.next_state, observation(8U, 1U, 1'020, 1'030, 201U), 1'040);
  ASSERT_EQ(pending.status, ProducerEpochAdmissionStatus::kPendingProducerHandoff);
  EXPECT_EQ(pending.next_state.pending_receive_stamp_ns, 1'030);

  const ProducerEpochAdmissionResult replay =
      admit(pending.next_state, observation(8U, 1U, 1'020, 1'045, 201U), 1'050);
  EXPECT_EQ(replay.status, ProducerEpochAdmissionStatus::kPendingProducerHandoff);
  EXPECT_EQ(replay.next_state.pending_receive_stamp_ns, 1'030);

  const ProducerEpochAdmissionResult unrelated =
      admit(replay.next_state, observation(9U, 1U, 1'030, 1'040, 301U), 1'050);
  EXPECT_EQ(unrelated.status,
            ProducerEpochAdmissionStatus::kRejectedPendingProducerMismatch);
  EXPECT_EQ(unrelated.next_state.pending_producer_instance_id, 8U);

  const ProducerEpochAdmissionResult handoff =
      admit(unrelated.next_state, observation(8U, 2U, 1'040, 1'050, 202U), 1'060);
  ASSERT_EQ(handoff.status, ProducerEpochAdmissionStatus::kAcceptedProducerHandoff);
  EXPECT_TRUE(handoff.producer_handoff);
  EXPECT_EQ(handoff.next_state.current_producer_instance_id, 8U);
  EXPECT_EQ(handoff.next_state.authority_generation, 2U);
  ASSERT_EQ(handoff.next_state.retired_producer_count, 1U);
  EXPECT_EQ(handoff.next_state.retired_producer_instance_ids[0], 7U);
}

TEST(ProducerEpochAdmissionTest,
     PendingIdentityConflictRestartsRatherThanConfirmingAmbiguousSample) {
  const ProducerEpochAdmissionResult initial =
      admit({}, observation(7U, 10U, 900, 910, 101U), 920);
  const ProducerEpochAdmissionResult pending =
      admit(initial.next_state, observation(8U, 1U, 1'020, 1'030, 201U), 1'040);
  const ProducerEpochAdmissionResult conflict =
      admit(pending.next_state, observation(8U, 1U, 1'020, 1'035, 999U), 1'040);
  ASSERT_EQ(conflict.status, ProducerEpochAdmissionStatus::kRejectedIdentityConflict);

  const ProducerEpochAdmissionResult restarted =
      admit(conflict.next_state, observation(8U, 2U, 1'040, 1'050, 202U), 1'060);
  EXPECT_EQ(restarted.status, ProducerEpochAdmissionStatus::kPendingProducerHandoff);
  EXPECT_EQ(restarted.next_state.pending_sequence, 2U);

  const ProducerEpochAdmissionResult handoff =
      admit(restarted.next_state, observation(8U, 3U, 1'060, 1'070, 203U), 1'080);
  EXPECT_EQ(handoff.status, ProducerEpochAdmissionStatus::kAcceptedProducerHandoff);
}

TEST(ProducerEpochAdmissionTest,
     RejectedPendingIdentityCannotConfirmWithMutatedContent) {
  const ProducerEpochAdmissionResult initial =
      admit({}, observation(7U, 10U, 900, 910, 101U), 920);
  const ProducerEpochAdmissionResult pending =
      admit(initial.next_state, observation(8U, 1U, 1'020, 1'030, 201U), 1'040);
  ASSERT_EQ(pending.status, ProducerEpochAdmissionStatus::kPendingProducerHandoff);
  const ProducerEpochAdmissionResult stale =
      admit(pending.next_state, observation(8U, 2U, 900, 1'040, 202U), 1'050);
  ASSERT_EQ(stale.status, ProducerEpochAdmissionStatus::kRejectedStaleCandidate);

  const ProducerEpochAdmissionResult conflict =
      admit(stale.next_state, observation(8U, 2U, 900, 1'045, 999U), 1'050);
  ASSERT_EQ(conflict.status, ProducerEpochAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_TRUE(conflict.next_state.pending_identity_conflicted);

  const ProducerEpochAdmissionResult restarted =
      admit(conflict.next_state, observation(8U, 3U, 1'050, 1'060, 203U), 1'070);
  EXPECT_EQ(restarted.status, ProducerEpochAdmissionStatus::kPendingProducerHandoff);
  const ProducerEpochAdmissionResult handoff =
      admit(restarted.next_state, observation(8U, 4U, 1'070, 1'080, 204U), 1'090);
  EXPECT_EQ(handoff.status, ProducerEpochAdmissionStatus::kAcceptedProducerHandoff);
}

TEST(ProducerEpochAdmissionTest, PendingLowerIdentityIsPermanentlyClaimed) {
  const ProducerEpochAdmissionResult initial =
      admit({}, observation(7U, 10U, 900, 910, 101U), 920);
  const ProducerEpochAdmissionResult pending =
      admit(initial.next_state, observation(8U, 2U, 1'020, 1'030, 202U), 1'040);
  ASSERT_EQ(pending.status, ProducerEpochAdmissionStatus::kPendingProducerHandoff);
  const ProducerEpochAdmissionResult lower =
      admit(pending.next_state, observation(8U, 1U, 1'010, 1'040, 201U), 1'050);
  ASSERT_EQ(lower.status, ProducerEpochAdmissionStatus::kRejectedRegression);
  ASSERT_EQ(lower.next_state.rejected_identity_count, 1U);

  const ProducerEpochAdmissionResult exact_replay =
      admit(lower.next_state, observation(8U, 1U, 1'010, 1'100, 201U), 1'110);
  EXPECT_EQ(exact_replay.status, ProducerEpochAdmissionStatus::kRejectedRegression);
  EXPECT_EQ(exact_replay.next_state.rejected_identities[0].first_receive_stamp_ns,
            1'040);
}

TEST(ProducerEpochAdmissionTest,
     ExpiredSingleCandidateCannotBlockAnotherProducerForever) {
  const ProducerEpochAdmissionResult initial =
      admit({}, observation(7U, 10U, 900, 910, 101U), 920);
  const ProducerEpochAdmissionResult abandoned =
      admit(initial.next_state, observation(8U, 1U, 1'020, 1'030, 201U), 1'040);
  ASSERT_EQ(abandoned.status, ProducerEpochAdmissionStatus::kPendingProducerHandoff);

  const ProducerEpochAdmissionResult replacement =
      admit(abandoned.next_state, observation(9U, 1U, 1'090, 1'100, 301U), 1'110);
  ASSERT_EQ(replacement.status, ProducerEpochAdmissionStatus::kPendingProducerHandoff);
  EXPECT_EQ(replacement.next_state.pending_producer_instance_id, 9U);
  ASSERT_EQ(replacement.next_state.rejected_identity_count, 1U);
  EXPECT_EQ(replacement.next_state.rejected_identities[0].producer_instance_id, 8U);
  EXPECT_EQ(replacement.next_state.rejected_identities[0].first_receive_stamp_ns,
            1'030);

  const ProducerEpochAdmissionResult abandoned_replay =
      admit(replacement.next_state, observation(8U, 1U, 1'020, 1'110, 201U), 1'120);
  EXPECT_EQ(abandoned_replay.status, ProducerEpochAdmissionStatus::kRejectedRegression);
  EXPECT_EQ(abandoned_replay.next_state.pending_producer_instance_id, 9U);
  EXPECT_EQ(abandoned_replay.next_state.rejected_identities[0].first_receive_stamp_ns,
            1'030);

  const ProducerEpochAdmissionResult handoff = admit(
      abandoned_replay.next_state, observation(9U, 2U, 1'130, 1'140, 302U), 1'150);
  EXPECT_EQ(handoff.status, ProducerEpochAdmissionStatus::kAcceptedProducerHandoff);
  EXPECT_EQ(handoff.next_state.current_producer_instance_id, 9U);
}

TEST(ProducerEpochAdmissionTest,
     RejectedForeignIdentityCannotOpenProbationOnExactRedelivery) {
  const ProducerEpochAdmissionResult initial =
      admit({}, observation(7U, 10U, 900, 910, 101U), 920);
  const ProducerEpochAdmissionResult rejected =
      admit(initial.next_state, observation(8U, 1U, 930, 940, 201U), 950);
  ASSERT_EQ(rejected.status,
            ProducerEpochAdmissionStatus::kRejectedCurrentProducerFresh);

  const ProducerEpochAdmissionResult exact_later =
      admit(rejected.next_state, observation(8U, 1U, 930, 1'030, 201U), 1'040);
  EXPECT_EQ(exact_later.status,
            ProducerEpochAdmissionStatus::kRejectedCurrentProducerFresh);
  EXPECT_EQ(exact_later.next_state.pending_producer_instance_id, 0U);

  const ProducerEpochAdmissionResult next_identity =
      admit(exact_later.next_state, observation(8U, 2U, 1'020, 1'030, 202U), 1'040);
  EXPECT_EQ(next_identity.status,
            ProducerEpochAdmissionStatus::kPendingProducerHandoff);
}

TEST(ProducerEpochAdmissionTest, RetiredProducerCannotReturnAfterLongGap) {
  ProducerEpochAdmissionState state =
      admit({}, observation(7U, 10U, 900, 910, 101U), 920).next_state;
  state = admit(state, observation(8U, 1U, 1'020, 1'030, 201U), 1'040).next_state;
  state = admit(state, observation(8U, 2U, 1'040, 1'050, 202U), 1'060).next_state;

  const ProducerEpochAdmissionResult replay =
      admit(state, observation(7U, 99U, 9'900, 9'910, 909U), 9'920);

  EXPECT_EQ(replay.status, ProducerEpochAdmissionStatus::kRejectedRetiredProducer);
  EXPECT_EQ(replay.next_state.current_producer_instance_id, 8U);
}

TEST(ProducerEpochAdmissionTest, BoundedRetiredCapacityFailsClosed) {
  ProducerEpochAdmissionState state =
      admit({}, observation(1U, 1U, 900, 910, 101U), 920).next_state;
  std::int64_t now_ns{1'100};
  for (std::uint64_t producer = 2U; producer <= kProducerEpochRetiredCapacity + 1U;
       ++producer) {
    const ProducerEpochAdmissionResult pending = admit(
        state, observation(producer, 1U, now_ns - 20, now_ns - 10, 100U + producer),
        now_ns);
    ASSERT_EQ(pending.status, ProducerEpochAdmissionStatus::kPendingProducerHandoff);
    const ProducerEpochAdmissionResult handoff = admit(
        pending.next_state,
        observation(producer, 2U, now_ns, now_ns + 10, 200U + producer), now_ns + 20);
    ASSERT_EQ(handoff.status, ProducerEpochAdmissionStatus::kAcceptedProducerHandoff);
    state = handoff.next_state;
    now_ns += 200;
  }
  ASSERT_EQ(state.retired_producer_count, kProducerEpochRetiredCapacity);

  const ProducerEpochAdmissionResult overflow =
      admit(state, observation(99U, 1U, now_ns - 20, now_ns - 10, 999U), now_ns);
  EXPECT_EQ(overflow.status, ProducerEpochAdmissionStatus::kRejectedHandoffCapacity);
  EXPECT_EQ(overflow.next_state.current_producer_instance_id,
            state.current_producer_instance_id);
}

TEST(ProducerEpochAdmissionTest, StaleFutureAndClockRewindFailClosed) {
  EXPECT_EQ(admit({}, observation(7U, 1U, 800, 810, 101U), 920).status,
            ProducerEpochAdmissionStatus::kRejectedStaleCandidate);
  EXPECT_EQ(admit({}, observation(7U, 1U, 930, 910, 101U), 920).status,
            ProducerEpochAdmissionStatus::kRejectedStaleCandidate);

  const ProducerEpochAdmissionState state =
      admit({}, observation(7U, 1U, 900, 910, 101U), 920).next_state;
  EXPECT_EQ(admit(state, observation(8U, 1U, 800, 810, 201U), 820).status,
            ProducerEpochAdmissionStatus::kRejectedClockDiscontinuity);
}

TEST(ProducerEpochAdmissionTest, RejectedIdentityCapacityLatchesFailClosed) {
  ProducerEpochAdmissionState state;
  for (std::uint64_t sequence = 1U; sequence <= kProducerRejectedIdentityCapacity;
       ++sequence) {
    const ProducerEpochAdmissionResult rejected =
        admit(state, observation(7U, sequence, 0, 900, 100U + sequence), 920);
    ASSERT_EQ(rejected.status, ProducerEpochAdmissionStatus::kRejectedInvalid);
    state = rejected.next_state;
  }
  const ProducerEpochAdmissionResult overflow =
      admit(state,
            observation(7U, kProducerRejectedIdentityCapacity + 1U, 0, 900, 999U), 920);
  ASSERT_EQ(overflow.status, ProducerEpochAdmissionStatus::kRejectedIdentityCapacity);
  EXPECT_TRUE(overflow.next_state.rejected_identity_capacity_exhausted);

  const ProducerEpochAdmissionResult fresh = admit(
      overflow.next_state,
      observation(7U, kProducerRejectedIdentityCapacity + 2U, 900, 910, 1'000U), 920);
  EXPECT_EQ(fresh.status, ProducerEpochAdmissionStatus::kRejectedIdentityCapacity);
  EXPECT_FALSE(fresh.install_observation);
}

TEST(ProducerEvidenceAdmissionTest,
     RequiresFullSnapshotForAuthorityAndThenAdmitsOrderedEvidence) {
  const ProducerEpochAuthority authority{.producer_instance_id = 7U, .generation = 1U};
  const ProducerEvidenceAdmissionResult delta = admitProducerEvidence(
      kConfig, {}, authority, observation(7U, 10U, 900, 910, 101U), false, 920);
  EXPECT_EQ(delta.status,
            ProducerEvidenceAdmissionStatus::kRejectedFullSnapshotRequired);

  const ProducerEvidenceAdmissionResult initial = admitProducerEvidence(
      kConfig, {}, authority, observation(7U, 10U, 900, 910, 101U), true, 920);
  ASSERT_EQ(initial.status, ProducerEvidenceAdmissionStatus::kAcceptedInitial);
  const ProducerEvidenceAdmissionResult newer =
      admitProducerEvidence(kConfig, initial.next_state, authority,
                            observation(7U, 11U, 930, 940, 102U), false, 950);
  EXPECT_EQ(newer.status, ProducerEvidenceAdmissionStatus::kAcceptedNewer);
}

TEST(ProducerEvidenceAdmissionTest,
     ReplayDoesNotRejuvenateAndConflictTombstonesUntilNewerIdentity) {
  const ProducerEpochAuthority authority{.producer_instance_id = 7U, .generation = 1U};
  const ProducerEvidenceAdmissionResult initial = admitProducerEvidence(
      kConfig, {}, authority, observation(7U, 10U, 900, 910, 101U), true, 920);
  const ProducerEvidenceAdmissionResult replay =
      admitProducerEvidence(kConfig, initial.next_state, authority,
                            observation(7U, 10U, 900, 990, 101U), true, 1'000);
  EXPECT_EQ(replay.status, ProducerEvidenceAdmissionStatus::kIdempotentReplay);
  EXPECT_EQ(replay.next_state.receive_stamp_ns, 910);

  const ProducerEvidenceAdmissionResult conflict =
      admitProducerEvidence(kConfig, replay.next_state, authority,
                            observation(7U, 10U, 900, 1'010, 202U), false, 1'100);
  ASSERT_EQ(conflict.status,
            ProducerEvidenceAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_TRUE(conflict.current_identity_conflict);
  EXPECT_TRUE(conflict.next_state.current_identity_conflicted);

  const ProducerEvidenceAdmissionResult recovered =
      admitProducerEvidence(kConfig, conflict.next_state, authority,
                            observation(7U, 11U, 1'050, 1'060, 303U), false, 1'070);
  EXPECT_EQ(recovered.status, ProducerEvidenceAdmissionStatus::kAcceptedNewer);
  EXPECT_FALSE(recovered.next_state.current_identity_conflicted);
}

TEST(ProducerEvidenceAdmissionTest,
     RejectedHigherIdentityCannotReturnWithMutatedContent) {
  const ProducerEpochAuthority authority{.producer_instance_id = 7U, .generation = 1U};
  const ProducerEvidenceAdmissionResult initial = admitProducerEvidence(
      kConfig, {}, authority, observation(7U, 10U, 900, 910, 101U), true, 920);
  const ProducerEvidenceAdmissionResult stale =
      admitProducerEvidence(kConfig, initial.next_state, authority,
                            observation(7U, 20U, 800, 940, 202U), false, 950);
  ASSERT_EQ(stale.status, ProducerEvidenceAdmissionStatus::kRejectedStaleCandidate);
  ASSERT_EQ(stale.next_state.rejected_identity_count, 1U);
  EXPECT_EQ(stale.next_state.rejected_identities[0].first_receive_stamp_ns, 940);

  const ProducerEvidenceAdmissionResult exact_replay =
      admitProducerEvidence(kConfig, stale.next_state, authority,
                            observation(7U, 20U, 800, 949, 202U), false, 950);
  EXPECT_EQ(exact_replay.status,
            ProducerEvidenceAdmissionStatus::kRejectedStaleCandidate);
  EXPECT_EQ(exact_replay.next_state.rejected_identities[0].first_receive_stamp_ns, 940);

  const ProducerEvidenceAdmissionResult conflict =
      admitProducerEvidence(kConfig, exact_replay.next_state, authority,
                            observation(7U, 20U, 800, 945, 999U), false, 950);
  ASSERT_EQ(conflict.status,
            ProducerEvidenceAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_TRUE(conflict.current_identity_conflict);

  const ProducerEvidenceAdmissionResult recovered =
      admitProducerEvidence(kConfig, conflict.next_state, authority,
                            observation(7U, 21U, 930, 940, 303U), false, 950);
  EXPECT_EQ(recovered.status, ProducerEvidenceAdmissionStatus::kAcceptedNewer);
  EXPECT_FALSE(recovered.next_state.current_identity_conflicted);
}

TEST(ProducerEvidenceAdmissionTest,
     MalformedTemporalClaimCannotResurrectWithMutatedIdentity) {
  const ProducerEpochAuthority authority{.producer_instance_id = 7U, .generation = 1U};
  const ProducerEvidenceAdmissionResult malformed = admitProducerEvidence(
      kConfig, {}, authority, observation(7U, 10U, 0, 910, 101U), true, 920);
  ASSERT_EQ(malformed.status, ProducerEvidenceAdmissionStatus::kRejectedInvalid);
  ASSERT_EQ(malformed.next_state.rejected_identity_count, 1U);

  const ProducerEvidenceAdmissionResult conflict =
      admitProducerEvidence(kConfig, malformed.next_state, authority,
                            observation(7U, 10U, 900, 920, 202U), true, 930);
  EXPECT_EQ(conflict.status,
            ProducerEvidenceAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_FALSE(conflict.install_evidence);
}

TEST(ProducerEvidenceAdmissionTest,
     RejectedHigherIdentityPermanentlyRaisesSequenceHighWater) {
  const ProducerEpochAuthority authority{.producer_instance_id = 7U, .generation = 1U};
  const ProducerEvidenceAdmissionResult malformed = admitProducerEvidence(
      kConfig, {}, authority, observation(7U, 10U, 0, 910, 101U), true, 920);
  ASSERT_EQ(malformed.status, ProducerEvidenceAdmissionStatus::kRejectedInvalid);

  const ProducerEvidenceAdmissionResult lower =
      admitProducerEvidence(kConfig, malformed.next_state, authority,
                            observation(7U, 9U, 900, 920, 202U), true, 930);
  ASSERT_EQ(lower.status, ProducerEvidenceAdmissionStatus::kRejectedRegression);
  EXPECT_FALSE(lower.install_evidence);
  EXPECT_EQ(lower.next_state.rejected_identity_count, 2U);

  const ProducerEvidenceAdmissionResult recovered =
      admitProducerEvidence(kConfig, lower.next_state, authority,
                            observation(7U, 11U, 920, 930, 303U), true, 940);
  EXPECT_EQ(recovered.status, ProducerEvidenceAdmissionStatus::kAcceptedInitial);
  EXPECT_TRUE(recovered.install_evidence);
  EXPECT_EQ(recovered.next_state.sequence, 11U);
}

TEST(ProducerEvidenceAdmissionTest,
     RejectedDeltaCannotReappearAsSameIdentityFullSnapshot) {
  const ProducerEpochAuthority first_authority{.producer_instance_id = 7U,
                                               .generation = 1U};
  const ProducerEvidenceAdmissionState first =
      admitProducerEvidence(kConfig, {}, first_authority,
                            observation(7U, 10U, 900, 910, 101U), true, 920)
          .next_state;
  const ProducerEpochAuthority second_authority{.producer_instance_id = 8U,
                                                .generation = 2U};
  const ProducerEvidenceAdmissionResult delta =
      admitProducerEvidence(kConfig, first, second_authority,
                            observation(8U, 1U, 1'020, 1'030, 201U), false, 1'040);
  ASSERT_EQ(delta.status,
            ProducerEvidenceAdmissionStatus::kRejectedFullSnapshotRequired);

  const ProducerEvidenceAdmissionResult changed_kind =
      admitProducerEvidence(kConfig, delta.next_state, second_authority,
                            observation(8U, 1U, 1'020, 1'040, 201U), true, 1'050);
  EXPECT_EQ(changed_kind.status,
            ProducerEvidenceAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_FALSE(changed_kind.install_evidence);

  const ProducerEvidenceAdmissionResult recovered =
      admitProducerEvidence(kConfig, changed_kind.next_state, second_authority,
                            observation(8U, 2U, 1'040, 1'050, 202U), true, 1'060);
  EXPECT_EQ(recovered.status,
            ProducerEvidenceAdmissionStatus::kAcceptedAuthorityHandoff);
}

TEST(ProducerEvidenceAdmissionTest,
     NewAuthorityNeedsFullSnapshotAndRejectsRetiredEpochReplay) {
  const ProducerEpochAuthority first_authority{.producer_instance_id = 7U,
                                               .generation = 1U};
  const ProducerEvidenceAdmissionState first =
      admitProducerEvidence(kConfig, {}, first_authority,
                            observation(7U, 10U, 900, 910, 101U), true, 920)
          .next_state;
  const ProducerEpochAuthority second_authority{.producer_instance_id = 8U,
                                                .generation = 2U};
  EXPECT_EQ(admitProducerEvidence(kConfig, first, second_authority,
                                  observation(8U, 1U, 1'020, 1'030, 201U), false, 1'040)
                .status,
            ProducerEvidenceAdmissionStatus::kRejectedFullSnapshotRequired);
  const ProducerEvidenceAdmissionResult handoff =
      admitProducerEvidence(kConfig, first, second_authority,
                            observation(8U, 1U, 1'020, 1'030, 201U), true, 1'040);
  ASSERT_EQ(handoff.status, ProducerEvidenceAdmissionStatus::kAcceptedAuthorityHandoff);
  EXPECT_TRUE(handoff.authority_handoff);

  EXPECT_EQ(admitProducerEvidence(kConfig, handoff.next_state, first_authority,
                                  observation(7U, 99U, 1'050, 1'060, 999U), true, 1'070)
                .status,
            ProducerEvidenceAdmissionStatus::kRejectedAuthorityMismatch);
}

// The urban flight r229: the deltas based on a full snapshot outran it on
// the wire. Each was rejected for lack of its base and remembered, the
// snapshot then read as a regression against their revisions, and once eight
// had been remembered every later snapshot was refused for capacity; the raw
// world stood still for the rest of the flight. A full snapshot from the
// authority is judged on its own order and content and releases the identities
// it supersedes.
TEST(ProducerEvidenceAdmissionTest,
     AFullSnapshotOvertakenByTheDeltasBasedOnItIsNotARegression) {
  const ProducerEpochAuthority authority{.producer_instance_id = 7U, .generation = 1U};
  ProducerEvidenceAdmissionState state =
      admitProducerEvidence(kConfig, {}, authority,
                            observation(7U, 10U, 900, 910, 101U), true, 920)
          .next_state;
  for (std::uint64_t sequence = 20U; sequence < 20U + kProducerRejectedIdentityCapacity;
       ++sequence) {
    const ProducerEvidenceAdmissionResult orphan = admitProducerEvidence(
        kConfig, state, authority, observation(7U, sequence, 900, 912, 200U + sequence),
        false, 920, false);
    ASSERT_EQ(orphan.status, ProducerEvidenceAdmissionStatus::kRejectedInvalid);
    state = orphan.next_state;
  }
  const ProducerEvidenceAdmissionResult overflow =
      admitProducerEvidence(kConfig, state, authority,
                            observation(7U, 40U, 900, 913, 300U), false, 920, false);
  ASSERT_EQ(overflow.status,
            ProducerEvidenceAdmissionStatus::kRejectedIdentityCapacity);
  state = overflow.next_state;
  ASSERT_TRUE(state.rejected_identity_capacity_exhausted);

  const ProducerEvidenceAdmissionResult snapshot = admitProducerEvidence(
      kConfig, state, authority, observation(7U, 15U, 930, 940, 400U), true, 950);
  ASSERT_EQ(snapshot.status, ProducerEvidenceAdmissionStatus::kAcceptedNewer);
  EXPECT_TRUE(snapshot.install_evidence);
  EXPECT_FALSE(snapshot.next_state.rejected_identity_capacity_exhausted);
  EXPECT_EQ(snapshot.next_state.sequence, 15U);

  const ProducerEvidenceAdmissionResult delta =
      admitProducerEvidence(kConfig, snapshot.next_state, authority,
                            observation(7U, 41U, 960, 970, 500U), false, 980);
  EXPECT_EQ(delta.status, ProducerEvidenceAdmissionStatus::kAcceptedNewer);
  // An older snapshot than the installed evidence is still a regression.
  const ProducerEvidenceAdmissionResult older =
      admitProducerEvidence(kConfig, delta.next_state, authority,
                            observation(7U, 12U, 990, 1'000, 600U), true, 1'010);
  EXPECT_EQ(older.status, ProducerEvidenceAdmissionStatus::kRejectedRegression);
}

TEST(ProducerEpochAdmissionTest, GenerationExhaustionFailsClosed) {
  ProducerEpochAdmissionState state =
      admit({}, observation(7U, 10U, 900, 910, 101U), 920).next_state;
  state.authority_generation = std::numeric_limits<std::uint64_t>::max();
  const ProducerEpochAdmissionResult result =
      admit(state, observation(8U, 1U, 1'020, 1'030, 201U), 1'040);
  EXPECT_EQ(result.status, ProducerEpochAdmissionStatus::kRejectedGenerationExhausted);
}

} // namespace
} // namespace drone_city_nav
