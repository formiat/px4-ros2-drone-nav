#include "drone_city_nav/execution_horizon_admission.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>

namespace drone_city_nav {
namespace {

[[nodiscard]] ExecutionHorizonAdmissionCandidate
candidate(const std::uint64_t producer_instance_id, const std::uint64_t sequence,
          const std::int64_t source_stamp_ns, const std::int64_t valid_from_ns,
          const std::uint64_t content_fingerprint = 0U) {
  return {
      .producer_instance_id = producer_instance_id,
      .sequence = sequence,
      .source_stamp_ns = source_stamp_ns,
      .valid_from_ns = valid_from_ns,
      .content_fingerprint =
          content_fingerprint != 0U
              ? content_fingerprint
              : producer_instance_id ^ (sequence << 8U) ^
                    (static_cast<std::uint64_t>(source_stamp_ns) << 16U) ^
                    (static_cast<std::uint64_t>(valid_from_ns) << 32U) ^ 1U,
  };
}

[[nodiscard]] ExecutionHorizonAdmissionResult
admitCandidate(const ExecutionHorizonAdmissionState& state,
               const ExecutionHorizonAdmissionCandidate& candidate,
               const bool payload_admissible = true) {
  return admitExecutionHorizonIdentity(state, candidate, payload_admissible);
}

void expectSameState(const ExecutionHorizonAdmissionState& actual,
                     const ExecutionHorizonAdmissionState& expected) {
  EXPECT_EQ(actual.current_producer_instance_id, expected.current_producer_instance_id);
  EXPECT_EQ(actual.current_sequence, expected.current_sequence);
  EXPECT_EQ(actual.current_source_stamp_ns, expected.current_source_stamp_ns);
  EXPECT_EQ(actual.current_valid_from_ns, expected.current_valid_from_ns);
  EXPECT_EQ(actual.latest_source_stamp_ns, expected.latest_source_stamp_ns);
  EXPECT_EQ(actual.latest_valid_from_ns, expected.latest_valid_from_ns);
  EXPECT_EQ(actual.current_content_fingerprint, expected.current_content_fingerprint);
  EXPECT_EQ(actual.retired_producer_count, expected.retired_producer_count);
  EXPECT_EQ(actual.retired_producer_instance_ids,
            expected.retired_producer_instance_ids);
  EXPECT_EQ(actual.prospective_identity_count, expected.prospective_identity_count);
  for (std::size_t index = 0U; index < actual.prospective_identity_claims.size();
       ++index) {
    const ExecutionHorizonProspectiveIdentityClaim& actual_claim =
        actual.prospective_identity_claims[index];
    const ExecutionHorizonProspectiveIdentityClaim& expected_claim =
        expected.prospective_identity_claims[index];
    EXPECT_EQ(actual_claim.candidate.producer_instance_id,
              expected_claim.candidate.producer_instance_id);
    EXPECT_EQ(actual_claim.candidate.sequence, expected_claim.candidate.sequence);
    EXPECT_EQ(actual_claim.candidate.source_stamp_ns,
              expected_claim.candidate.source_stamp_ns);
    EXPECT_EQ(actual_claim.candidate.valid_from_ns,
              expected_claim.candidate.valid_from_ns);
    EXPECT_EQ(actual_claim.candidate.content_fingerprint,
              expected_claim.candidate.content_fingerprint);
    EXPECT_EQ(actual_claim.source_stamp_high_water_ns,
              expected_claim.source_stamp_high_water_ns);
    EXPECT_EQ(actual_claim.valid_from_high_water_ns,
              expected_claim.valid_from_high_water_ns);
    EXPECT_EQ(actual_claim.tombstoned, expected_claim.tombstoned);
  }
  EXPECT_EQ(actual.current_identity_tombstoned, expected.current_identity_tombstoned);
  EXPECT_EQ(actual.prospective_identity_capacity_exhausted,
            expected.prospective_identity_capacity_exhausted);
}

[[nodiscard]] ExecutionHorizonAdmissionState
admittedInitialState(const ExecutionHorizonAdmissionCandidate& initial) {
  const ExecutionHorizonAdmissionResult result = admitCandidate({}, initial);
  EXPECT_TRUE(result.accept_identity);
  EXPECT_TRUE(result.payload_installable);
  EXPECT_FALSE(result.revoke);
  EXPECT_FALSE(result.stale);
  EXPECT_FALSE(result.invalid);
  return result.next_state;
}

TEST(ExecutionHorizonAdmissionTest, AdmitsInitialIdentityWithoutRevocation) {
  const ExecutionHorizonAdmissionCandidate initial = candidate(17U, 4U, 100, 200);

  const ExecutionHorizonAdmissionResult result = admitCandidate({}, initial);

  EXPECT_TRUE(result.accept_identity);
  EXPECT_TRUE(result.payload_installable);
  EXPECT_FALSE(result.revoke);
  EXPECT_FALSE(result.stale);
  EXPECT_FALSE(result.invalid);
  EXPECT_EQ(result.next_state.current_producer_instance_id, 17U);
  EXPECT_EQ(result.next_state.current_sequence, 4U);
  EXPECT_EQ(result.next_state.current_source_stamp_ns, 100);
  EXPECT_EQ(result.next_state.current_valid_from_ns, 200);
  EXPECT_EQ(result.next_state.latest_source_stamp_ns, 100);
  EXPECT_EQ(result.next_state.latest_valid_from_ns, 200);
  EXPECT_EQ(result.next_state.current_content_fingerprint, initial.content_fingerprint);
  EXPECT_EQ(result.next_state.retired_producer_count, 0U);
  EXPECT_TRUE(result.next_state.valid());
}

TEST(ExecutionHorizonAdmissionTest, UnclaimableIdentityCannotRevokeOrMutateState) {
  const ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(17U, 4U, 100, 200));
  const ExecutionHorizonAdmissionCandidate zero_producer = candidate(0U, 5U, 110, 210);
  const ExecutionHorizonAdmissionCandidate zero_sequence = candidate(17U, 0U, 110, 210);
  for (const ExecutionHorizonAdmissionCandidate invalid_candidate :
       {zero_producer, zero_sequence}) {
    const ExecutionHorizonAdmissionResult result =
        admitCandidate(state, invalid_candidate);
    EXPECT_FALSE(result.accept_identity);
    EXPECT_FALSE(result.revoke);
    EXPECT_FALSE(result.stale);
    EXPECT_TRUE(result.invalid);
    expectSameState(result.next_state, state);
  }
}

TEST(ExecutionHorizonAdmissionTest,
     InitialMalformedIdentityIsClaimedAndOnlyHigherSequenceRecovers) {
  ExecutionHorizonAdmissionCandidate malformed = candidate(17U, 4U, 100, 200);
  malformed.content_fingerprint = 0U;

  const ExecutionHorizonAdmissionResult claimed = admitCandidate({}, malformed);

  EXPECT_TRUE(claimed.accept_identity);
  EXPECT_TRUE(claimed.state_advanced);
  EXPECT_FALSE(claimed.payload_installable);
  EXPECT_TRUE(claimed.invalid);
  EXPECT_TRUE(claimed.next_state.current_identity_tombstoned);
  EXPECT_TRUE(claimed.next_state.valid());

  const ExecutionHorizonAdmissionResult exact =
      admitCandidate(claimed.next_state, malformed);
  EXPECT_TRUE(exact.replay);
  EXPECT_TRUE(exact.revoke);
  EXPECT_FALSE(exact.state_advanced);
  EXPECT_FALSE(exact.payload_installable);
  expectSameState(exact.next_state, claimed.next_state);

  ExecutionHorizonAdmissionCandidate mutation = malformed;
  mutation.content_fingerprint = 91U;
  const ExecutionHorizonAdmissionResult conflict =
      admitCandidate(claimed.next_state, mutation);
  EXPECT_TRUE(conflict.conflict);
  EXPECT_TRUE(conflict.revoke);
  EXPECT_FALSE(conflict.state_advanced);
  expectSameState(conflict.next_state, claimed.next_state);

  const ExecutionHorizonAdmissionResult recovered =
      admitCandidate(claimed.next_state, candidate(17U, 5U, 101, 201));
  EXPECT_TRUE(recovered.accept_identity);
  EXPECT_TRUE(recovered.payload_installable);
  EXPECT_TRUE(recovered.state_advanced);
  EXPECT_FALSE(recovered.next_state.current_identity_tombstoned);
}

TEST(ExecutionHorizonAdmissionTest,
     InitialPayloadRejectionClaimsIdentityAndRequiresHigherSequence) {
  const ExecutionHorizonAdmissionCandidate rejected = candidate(17U, 4U, 100, 200);

  const ExecutionHorizonAdmissionResult claimed = admitCandidate({}, rejected, false);

  EXPECT_TRUE(claimed.accept_identity);
  EXPECT_TRUE(claimed.state_advanced);
  EXPECT_FALSE(claimed.payload_installable);
  EXPECT_TRUE(claimed.invalid);
  EXPECT_TRUE(claimed.next_state.current_identity_tombstoned);

  const ExecutionHorizonAdmissionResult exact =
      admitCandidate(claimed.next_state, rejected, true);
  EXPECT_TRUE(exact.replay);
  EXPECT_TRUE(exact.revoke);
  EXPECT_FALSE(exact.payload_installable);
  expectSameState(exact.next_state, claimed.next_state);

  const ExecutionHorizonAdmissionResult recovered =
      admitCandidate(claimed.next_state, candidate(17U, 5U, 101, 201), true);
  EXPECT_TRUE(recovered.payload_installable);
  EXPECT_FALSE(recovered.next_state.current_identity_tombstoned);
}

TEST(ExecutionHorizonAdmissionTest,
     HigherSequenceWithRegressingTimestampsRevokesWithoutPayloadInstall) {
  const ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(17U, 4U, 100, 200));
  const ExecutionHorizonAdmissionCandidate newer_identity_with_older_stamps =
      candidate(17U, 5U, 90, 190);

  const ExecutionHorizonAdmissionResult result =
      admitCandidate(state, newer_identity_with_older_stamps);

  EXPECT_TRUE(result.accept_identity);
  EXPECT_TRUE(result.state_advanced);
  EXPECT_FALSE(result.payload_installable);
  EXPECT_TRUE(result.revoke);
  EXPECT_TRUE(result.stale);
  EXPECT_FALSE(result.invalid);
  EXPECT_EQ(result.next_state.current_producer_instance_id, 17U);
  EXPECT_EQ(result.next_state.current_sequence, 5U);
  EXPECT_EQ(result.next_state.current_source_stamp_ns, 90);
  EXPECT_EQ(result.next_state.current_valid_from_ns, 190);
  EXPECT_EQ(result.next_state.latest_source_stamp_ns, 100);
  EXPECT_EQ(result.next_state.latest_valid_from_ns, 200);
  EXPECT_EQ(result.next_state.current_content_fingerprint,
            newer_identity_with_older_stamps.content_fingerprint);
  EXPECT_TRUE(result.next_state.current_identity_tombstoned);
  EXPECT_EQ(result.next_state.retired_producer_count, 0U);

  const ExecutionHorizonAdmissionResult replay =
      admitCandidate(result.next_state, newer_identity_with_older_stamps);
  EXPECT_TRUE(replay.replay);
  EXPECT_FALSE(replay.conflict);
  EXPECT_TRUE(replay.revoke);
  EXPECT_FALSE(replay.state_advanced);
}

TEST(ExecutionHorizonAdmissionTest, MalformedNewerIdentityStillRevokesAndIsTombstoned) {
  const ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(17U, 4U, 100, 200));
  ExecutionHorizonAdmissionCandidate malformed = candidate(17U, 5U, 0, 0);
  ASSERT_FALSE(malformed.valid());

  const ExecutionHorizonAdmissionResult admitted = admitCandidate(state, malformed);

  EXPECT_TRUE(admitted.accept_identity);
  EXPECT_TRUE(admitted.state_advanced);
  EXPECT_FALSE(admitted.payload_installable);
  EXPECT_TRUE(admitted.revoke);
  EXPECT_TRUE(admitted.invalid);
  EXPECT_EQ(admitted.next_state.current_sequence, 5U);
  EXPECT_EQ(admitted.next_state.current_source_stamp_ns, 0);
  EXPECT_EQ(admitted.next_state.current_valid_from_ns, 0);
  EXPECT_EQ(admitted.next_state.latest_source_stamp_ns, 100);
  EXPECT_EQ(admitted.next_state.latest_valid_from_ns, 200);
  EXPECT_EQ(admitted.next_state.current_content_fingerprint,
            malformed.content_fingerprint);
  EXPECT_TRUE(admitted.next_state.current_identity_tombstoned);
  EXPECT_TRUE(admitted.next_state.valid());
}

TEST(ExecutionHorizonAdmissionTest,
     RejectedCurrentProducerPayloadRevokesAndRequiresHigherSequence) {
  const ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(17U, 4U, 100, 200));
  const ExecutionHorizonAdmissionCandidate rejected = candidate(17U, 5U, 101, 201);

  const ExecutionHorizonAdmissionResult result = admitCandidate(state, rejected, false);

  EXPECT_TRUE(result.accept_identity);
  EXPECT_TRUE(result.state_advanced);
  EXPECT_TRUE(result.revoke);
  EXPECT_TRUE(result.invalid);
  EXPECT_FALSE(result.payload_installable);
  EXPECT_EQ(result.next_state.current_sequence, 5U);
  EXPECT_TRUE(result.next_state.current_identity_tombstoned);

  const ExecutionHorizonAdmissionResult replay =
      admitCandidate(result.next_state, rejected, true);
  EXPECT_TRUE(replay.replay);
  EXPECT_TRUE(replay.revoke);
  EXPECT_FALSE(replay.payload_installable);
}

TEST(ExecutionHorizonAdmissionTest,
     HigherSequenceWithAdvancingTimestampsIsPayloadInstallable) {
  const ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(17U, 4U, 100, 200));

  const ExecutionHorizonAdmissionResult result =
      admitCandidate(state, candidate(17U, 5U, 101, 201));

  EXPECT_TRUE(result.accept_identity);
  EXPECT_TRUE(result.state_advanced);
  EXPECT_TRUE(result.payload_installable);
  EXPECT_TRUE(result.revoke);
  EXPECT_FALSE(result.next_state.current_identity_tombstoned);
  EXPECT_EQ(result.next_state.latest_source_stamp_ns, 101);
  EXPECT_EQ(result.next_state.latest_valid_from_ns, 201);
}

TEST(ExecutionHorizonAdmissionTest,
     MalformedSameIdentityMutationRevokesInstalledPayload) {
  const ExecutionHorizonAdmissionCandidate initial = candidate(17U, 4U, 100, 200);
  const ExecutionHorizonAdmissionState state = admittedInitialState(initial);
  ExecutionHorizonAdmissionCandidate malformed = initial;
  malformed.source_stamp_ns = 0;
  malformed.valid_from_ns = 0;
  ++malformed.content_fingerprint;
  ASSERT_FALSE(malformed.valid());

  const ExecutionHorizonAdmissionResult conflict = admitCandidate(state, malformed);

  EXPECT_FALSE(conflict.accept_identity);
  EXPECT_TRUE(conflict.revoke);
  EXPECT_TRUE(conflict.conflict);
  EXPECT_TRUE(conflict.state_advanced);
  EXPECT_FALSE(conflict.invalid);
  EXPECT_TRUE(conflict.next_state.current_identity_tombstoned);
}

TEST(ExecutionHorizonAdmissionTest,
     ExactReplayIsIdempotentAndSameIdentityMutationIsTombstoned) {
  const ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(17U, 4U, 100, 200));

  const ExecutionHorizonAdmissionResult replay =
      admitCandidate(state, candidate(17U, 4U, 100, 200));
  EXPECT_FALSE(replay.accept_identity);
  EXPECT_FALSE(replay.revoke);
  EXPECT_TRUE(replay.replay);
  EXPECT_FALSE(replay.conflict);
  expectSameState(replay.next_state, state);

  const ExecutionHorizonAdmissionResult conflict =
      admitCandidate(state, candidate(17U, 4U, 101, 200));
  EXPECT_FALSE(conflict.accept_identity);
  EXPECT_TRUE(conflict.revoke);
  EXPECT_FALSE(conflict.replay);
  EXPECT_TRUE(conflict.conflict);
  EXPECT_TRUE(conflict.next_state.current_identity_tombstoned);
  EXPECT_TRUE(conflict.next_state.valid());

  const ExecutionHorizonAdmissionResult old_exact_after_conflict =
      admitCandidate(conflict.next_state, candidate(17U, 4U, 100, 200));
  EXPECT_FALSE(old_exact_after_conflict.accept_identity);
  EXPECT_TRUE(old_exact_after_conflict.revoke);
  EXPECT_TRUE(old_exact_after_conflict.replay);
  EXPECT_FALSE(old_exact_after_conflict.conflict);
  EXPECT_FALSE(old_exact_after_conflict.state_advanced);

  const ExecutionHorizonAdmissionResult lower =
      admitCandidate(state, candidate(17U, 3U, 300, 400));
  EXPECT_FALSE(lower.accept_identity);
  EXPECT_FALSE(lower.revoke);
  EXPECT_TRUE(lower.stale);
  EXPECT_FALSE(lower.invalid);
  expectSameState(lower.next_state, state);
}

TEST(ExecutionHorizonAdmissionTest, InvalidPayloadCanTombstoneExactAdmittedIdentity) {
  const ExecutionHorizonAdmissionCandidate initial = candidate(17U, 4U, 100, 200);
  ExecutionHorizonAdmissionState state = admittedInitialState(initial);

  EXPECT_TRUE(tombstoneExecutionHorizonIdentity(state, initial));
  EXPECT_TRUE(state.current_identity_tombstoned);
  EXPECT_TRUE(state.valid());

  ExecutionHorizonAdmissionCandidate changed = initial;
  ++changed.content_fingerprint;
  EXPECT_FALSE(tombstoneExecutionHorizonIdentity(state, changed));
}

TEST(ExecutionHorizonAdmissionTest,
     DisarmTombstoneRequiresStrictlyNewerIdentityAfterRearm) {
  const ExecutionHorizonAdmissionCandidate installed = candidate(17U, 4U, 100, 200);
  ExecutionHorizonAdmissionState state = admittedInitialState(installed);
  ASSERT_TRUE(tombstoneExecutionHorizonIdentity(state, installed));

  const ExecutionHorizonAdmissionResult delayed_replay =
      admitCandidate(state, installed);
  EXPECT_FALSE(delayed_replay.accept_identity);
  EXPECT_TRUE(delayed_replay.revoke);
  EXPECT_TRUE(delayed_replay.replay);
  EXPECT_FALSE(delayed_replay.conflict);
  EXPECT_TRUE(delayed_replay.next_state.current_identity_tombstoned);

  const ExecutionHorizonAdmissionResult newer =
      admitCandidate(delayed_replay.next_state, candidate(17U, 5U, 101, 201));
  EXPECT_TRUE(newer.accept_identity);
  EXPECT_TRUE(newer.payload_installable);
  EXPECT_TRUE(newer.revoke);
  EXPECT_FALSE(newer.conflict);
  EXPECT_FALSE(newer.next_state.current_identity_tombstoned);
}

TEST(ExecutionHorizonAdmissionTest,
     StaleProspectiveIdentityCannotMutateIntoAnEligibleHandoff) {
  const ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(17U, 4U, 100, 200));
  const ExecutionHorizonAdmissionCandidate stale_candidate =
      candidate(23U, 1U, 100, 201);

  const ExecutionHorizonAdmissionResult claimed =
      admitCandidate(state, stale_candidate);
  EXPECT_FALSE(claimed.accept_identity);
  EXPECT_FALSE(claimed.revoke);
  EXPECT_TRUE(claimed.stale);
  EXPECT_TRUE(claimed.state_advanced);
  EXPECT_EQ(claimed.next_state.current_producer_instance_id, 17U);
  ASSERT_EQ(claimed.next_state.prospective_identity_count, 1U);
  EXPECT_EQ(claimed.next_state.prospective_identity_claims.front()
                .candidate.content_fingerprint,
            stale_candidate.content_fingerprint);

  const ExecutionHorizonAdmissionResult exact =
      admitCandidate(claimed.next_state, stale_candidate);
  EXPECT_TRUE(exact.replay);
  EXPECT_FALSE(exact.state_advanced);
  expectSameState(exact.next_state, claimed.next_state);

  const ExecutionHorizonAdmissionResult mutation =
      admitCandidate(claimed.next_state, candidate(23U, 1U, 101, 202));
  EXPECT_TRUE(mutation.conflict);
  EXPECT_FALSE(mutation.accept_identity);
  EXPECT_FALSE(mutation.state_advanced);
  expectSameState(mutation.next_state, claimed.next_state);

  const ExecutionHorizonAdmissionResult accepted =
      admitCandidate(claimed.next_state, candidate(23U, 2U, 101, 202));
  EXPECT_TRUE(accepted.accept_identity);
  EXPECT_TRUE(accepted.payload_installable);
  EXPECT_TRUE(accepted.revoke);
  EXPECT_FALSE(accepted.stale);
  EXPECT_FALSE(accepted.invalid);
  EXPECT_EQ(accepted.next_state.current_producer_instance_id, 23U);
  EXPECT_EQ(accepted.next_state.current_sequence, 2U);
  EXPECT_EQ(accepted.next_state.latest_source_stamp_ns, 101);
  EXPECT_EQ(accepted.next_state.latest_valid_from_ns, 202);
  EXPECT_EQ(accepted.next_state.prospective_identity_count, 0U);
  ASSERT_EQ(accepted.next_state.retired_producer_count, 1U);
  EXPECT_EQ(accepted.next_state.retired_producer_instance_ids.front(), 17U);
}

TEST(ExecutionHorizonAdmissionTest,
     MalformedProspectiveIdentityRequiresHigherSequenceToRecover) {
  const ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(17U, 4U, 100, 200));
  ExecutionHorizonAdmissionCandidate malformed = candidate(23U, 1U, 101, 201);
  malformed.content_fingerprint = 0U;

  const ExecutionHorizonAdmissionResult claimed = admitCandidate(state, malformed);
  EXPECT_TRUE(claimed.invalid);
  EXPECT_TRUE(claimed.state_advanced);
  EXPECT_FALSE(claimed.accept_identity);
  EXPECT_FALSE(claimed.revoke);
  EXPECT_EQ(claimed.next_state.current_producer_instance_id, 17U);
  ASSERT_EQ(claimed.next_state.prospective_identity_count, 1U);
  EXPECT_EQ(claimed.next_state.prospective_identity_claims.front()
                .candidate.content_fingerprint,
            0U);

  const ExecutionHorizonAdmissionResult mutation =
      admitCandidate(claimed.next_state, candidate(23U, 1U, 101, 201));
  EXPECT_TRUE(mutation.conflict);
  EXPECT_FALSE(mutation.state_advanced);
  EXPECT_FALSE(mutation.accept_identity);
  EXPECT_EQ(mutation.next_state.prospective_identity_claims.front()
                .candidate.content_fingerprint,
            0U);

  const ExecutionHorizonAdmissionResult recovered =
      admitCandidate(claimed.next_state, candidate(23U, 2U, 102, 202));
  EXPECT_TRUE(recovered.accept_identity);
  EXPECT_TRUE(recovered.payload_installable);
  EXPECT_TRUE(recovered.revoke);
  EXPECT_EQ(recovered.next_state.current_producer_instance_id, 23U);
  EXPECT_EQ(recovered.next_state.current_sequence, 2U);
}

TEST(ExecutionHorizonAdmissionTest,
     RejectedProspectivePayloadCannotRetireTheActiveProducer) {
  const ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(17U, 4U, 100, 200));
  const ExecutionHorizonAdmissionCandidate rejected = candidate(23U, 1U, 101, 201);

  const ExecutionHorizonAdmissionResult claimed =
      admitCandidate(state, rejected, false);

  EXPECT_FALSE(claimed.accept_identity);
  EXPECT_FALSE(claimed.payload_installable);
  EXPECT_FALSE(claimed.revoke);
  EXPECT_TRUE(claimed.invalid);
  EXPECT_TRUE(claimed.state_advanced);
  EXPECT_EQ(claimed.next_state.current_producer_instance_id, 17U);
  EXPECT_EQ(claimed.next_state.retired_producer_count, 0U);
  ASSERT_EQ(claimed.next_state.prospective_identity_count, 1U);

  const ExecutionHorizonAdmissionResult exact =
      admitCandidate(claimed.next_state, rejected, true);
  EXPECT_TRUE(exact.replay);
  EXPECT_FALSE(exact.revoke);
  EXPECT_FALSE(exact.payload_installable);
  expectSameState(exact.next_state, claimed.next_state);

  ExecutionHorizonAdmissionCandidate mutation = rejected;
  ++mutation.content_fingerprint;
  const ExecutionHorizonAdmissionResult conflict =
      admitCandidate(claimed.next_state, mutation, true);
  EXPECT_TRUE(conflict.conflict);
  EXPECT_FALSE(conflict.revoke);
  expectSameState(conflict.next_state, claimed.next_state);

  const ExecutionHorizonAdmissionResult recovered =
      admitCandidate(claimed.next_state, candidate(23U, 2U, 102, 202), true);
  EXPECT_TRUE(recovered.accept_identity);
  EXPECT_TRUE(recovered.payload_installable);
  EXPECT_TRUE(recovered.revoke);
  EXPECT_EQ(recovered.next_state.current_producer_instance_id, 23U);
  ASSERT_EQ(recovered.next_state.retired_producer_count, 1U);
  EXPECT_EQ(recovered.next_state.retired_producer_instance_ids.front(), 17U);
}

TEST(ExecutionHorizonAdmissionTest, RetiredProducerCannotReturn) {
  ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(17U, 4U, 100, 200));
  state = admitCandidate(state, candidate(23U, 1U, 101, 201)).next_state;

  const ExecutionHorizonAdmissionResult result =
      admitCandidate(state, candidate(17U, 99U, 1'000, 2'000));

  EXPECT_FALSE(result.accept_identity);
  EXPECT_FALSE(result.revoke);
  EXPECT_TRUE(result.stale);
  EXPECT_FALSE(result.invalid);
  expectSameState(result.next_state, state);
}

TEST(ExecutionHorizonAdmissionTest, RetiredProducerStorageFailsClosedAtItsBound) {
  ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(1U, 1U, 100, 200));
  for (std::size_t transition = 0U;
       transition < kExecutionHorizonRetiredProducerCapacity; ++transition) {
    const std::uint64_t next_producer = static_cast<std::uint64_t>(transition) + 2U;
    const std::int64_t next_source = 101 + static_cast<std::int64_t>(transition);
    const std::int64_t next_valid_from = 201 + static_cast<std::int64_t>(transition);
    const ExecutionHorizonAdmissionResult accepted = admitCandidate(
        state, candidate(next_producer, 1U, next_source, next_valid_from));
    ASSERT_TRUE(accepted.accept_identity);
    ASSERT_TRUE(accepted.revoke);
    ASSERT_FALSE(accepted.stale);
    ASSERT_FALSE(accepted.invalid);
    state = accepted.next_state;
  }
  ASSERT_EQ(state.retired_producer_count, kExecutionHorizonRetiredProducerCapacity);

  const ExecutionHorizonAdmissionResult overflow =
      admitCandidate(state, candidate(99U, 1U, 1'000, 2'000));

  EXPECT_FALSE(overflow.accept_identity);
  EXPECT_FALSE(overflow.revoke);
  EXPECT_FALSE(overflow.stale);
  EXPECT_FALSE(overflow.invalid);
  EXPECT_TRUE(overflow.retired_capacity_exhausted);
  EXPECT_TRUE(overflow.state_advanced);
  EXPECT_EQ(overflow.next_state.current_producer_instance_id,
            state.current_producer_instance_id);
  EXPECT_EQ(overflow.next_state.prospective_identity_count, 1U);
  EXPECT_TRUE(overflow.next_state.valid());

  const ExecutionHorizonAdmissionResult retired_return =
      admitCandidate(overflow.next_state, candidate(1U, 99U, 2'000, 3'000));
  EXPECT_FALSE(retired_return.accept_identity);
  EXPECT_FALSE(retired_return.revoke);
  EXPECT_TRUE(retired_return.stale);
  EXPECT_FALSE(retired_return.invalid);
  expectSameState(retired_return.next_state, overflow.next_state);
}

TEST(ExecutionHorizonAdmissionTest,
     ProspectiveProducerStorageLatchesCapacityExhaustion) {
  ExecutionHorizonAdmissionState state =
      admittedInitialState(candidate(1U, 1U, 100, 200));
  for (std::size_t index = 0U; index < kExecutionHorizonProspectiveProducerCapacity;
       ++index) {
    const std::uint64_t producer_instance_id = 10U + static_cast<std::uint64_t>(index);
    const ExecutionHorizonAdmissionResult claimed =
        admitCandidate(state, candidate(producer_instance_id, 1U, 100, 200));
    ASSERT_FALSE(claimed.accept_identity);
    ASSERT_TRUE(claimed.stale);
    ASSERT_TRUE(claimed.state_advanced);
    ASSERT_EQ(claimed.next_state.current_producer_instance_id, 1U);
    state = claimed.next_state;
  }
  ASSERT_EQ(state.prospective_identity_count,
            kExecutionHorizonProspectiveProducerCapacity);

  const ExecutionHorizonAdmissionCandidate overflow_candidate =
      candidate(99U, 1U, 101, 201);
  const ExecutionHorizonAdmissionResult overflow =
      admitCandidate(state, overflow_candidate);
  EXPECT_FALSE(overflow.accept_identity);
  EXPECT_FALSE(overflow.revoke);
  EXPECT_TRUE(overflow.invalid);
  EXPECT_TRUE(overflow.prospective_capacity_exhausted);
  EXPECT_TRUE(overflow.state_advanced);
  EXPECT_TRUE(overflow.next_state.prospective_identity_capacity_exhausted);
  EXPECT_EQ(overflow.next_state.current_producer_instance_id, 1U);
  EXPECT_TRUE(overflow.next_state.valid());

  const ExecutionHorizonAdmissionResult repeated_overflow =
      admitCandidate(overflow.next_state, overflow_candidate);
  EXPECT_TRUE(repeated_overflow.invalid);
  EXPECT_TRUE(repeated_overflow.prospective_capacity_exhausted);
  EXPECT_FALSE(repeated_overflow.state_advanced);
  expectSameState(repeated_overflow.next_state, overflow.next_state);

  const ExecutionHorizonAdmissionResult existing_claim_handoff =
      admitCandidate(overflow.next_state, candidate(10U, 2U, 101, 201));
  ASSERT_TRUE(existing_claim_handoff.accept_identity);
  ASSERT_TRUE(existing_claim_handoff.payload_installable);
  ASSERT_EQ(existing_claim_handoff.next_state.prospective_identity_count,
            kExecutionHorizonProspectiveProducerCapacity - 1U);
  ASSERT_TRUE(
      existing_claim_handoff.next_state.prospective_identity_capacity_exhausted);

  const ExecutionHorizonAdmissionResult still_closed =
      admitCandidate(existing_claim_handoff.next_state, candidate(100U, 1U, 102, 202));
  EXPECT_TRUE(still_closed.invalid);
  EXPECT_TRUE(still_closed.prospective_capacity_exhausted);
  EXPECT_FALSE(still_closed.state_advanced);
  EXPECT_EQ(still_closed.next_state.current_producer_instance_id, 10U);
  EXPECT_EQ(still_closed.next_state.prospective_identity_count,
            kExecutionHorizonProspectiveProducerCapacity - 1U);
}

TEST(ExecutionHorizonAdmissionTest, MalformedStateIsInvalidAndCannotRevoke) {
  ExecutionHorizonAdmissionState malformed =
      admittedInitialState(candidate(17U, 4U, 100, 200));
  malformed.current_sequence = 0U;

  const ExecutionHorizonAdmissionResult result =
      admitCandidate(malformed, candidate(17U, 5U, 110, 210));

  EXPECT_FALSE(result.accept_identity);
  EXPECT_FALSE(result.revoke);
  EXPECT_FALSE(result.stale);
  EXPECT_TRUE(result.invalid);
  expectSameState(result.next_state, malformed);
}

TEST(ExecutionHorizonAdmissionTest, EmptyStateWithHiddenRetiredIdentityIsInvalid) {
  ExecutionHorizonAdmissionState malformed;
  malformed.retired_producer_instance_ids.front() = 99U;

  const ExecutionHorizonAdmissionResult result =
      admitCandidate(malformed, candidate(17U, 1U, 100, 200));

  EXPECT_FALSE(result.accept_identity);
  EXPECT_FALSE(result.revoke);
  EXPECT_FALSE(result.stale);
  EXPECT_TRUE(result.invalid);
  expectSameState(result.next_state, malformed);
}

} // namespace
} // namespace drone_city_nav
