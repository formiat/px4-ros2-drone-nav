#include "drone_city_nav/mission_waypoint_acknowledgement_admission.hpp"

#include <gtest/gtest.h>

namespace drone_city_nav {
namespace {

[[nodiscard]] MissionWaypointAcknowledgementCandidate
acknowledgement(const std::uint64_t producer = 11U, const std::uint64_t sequence = 1U,
                const std::uint32_t completed_count = 1U,
                const std::uint64_t mission_epoch = 0U,
                const std::int64_t source_stamp_ns = 4'000'000'000LL,
                const std::int64_t receive_stamp_ns = 4'010'000'000LL) {
  const bool completed = completed_count == 3U;
  MissionWaypointAcknowledgementCandidate candidate{
      .producer_instance_id = producer,
      .acknowledgement_sequence = sequence,
      .content_fingerprint = (producer << 32U) ^ (sequence << 16U) ^ completed_count ^
                             mission_epoch ^
                             static_cast<std::uint64_t>(source_stamp_ns) ^ 1U,
      .mission_epoch = mission_epoch,
      .horizon_producer_instance_id = producer,
      .horizon_sequence = sequence + 100U,
      .offboard_producer_instance_id = 31U,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .horizon_valid_from_ns = source_stamp_ns - 3'000'000'000LL,
      .horizon_valid_until_ns = source_stamp_ns + 1'000'000'000LL,
      .witness_stamp_ns = source_stamp_ns - 10'000'000LL,
      .completed_waypoint_index = completed_count - 1U,
      .completed_waypoint_count = completed_count,
      .waypoint_count = 3U,
      .active_waypoint_index = completed ? 2U : completed_count,
      .completed_goal = Point3{10.0 * completed_count, 20.0, 18.0},
      .route_target = Point3{10.0 * completed_count, 20.0, 18.0},
      .stationary_hold_position = Point3{10.0 * completed_count, 20.0, 18.0},
      .mission_completed = completed,
      .payload_valid = true,
  };
  if (candidate.content_fingerprint == 0U) {
    candidate.content_fingerprint = 1U;
  }
  return candidate;
}

[[nodiscard]] MissionWaypointAcknowledgementAdmissionState initialState() {
  const MissionWaypointAcknowledgementAdmissionResult admitted =
      admitMissionWaypointAcknowledgement({}, acknowledgement());
  EXPECT_TRUE(admitted.accept);
  EXPECT_TRUE(admitted.state_advanced);
  return admitted.next_state;
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     ReplaysExactlyOnceAndRejectsReorderedSequence) {
  const MissionWaypointAcknowledgementCandidate second =
      acknowledgement(11U, 3U, 2U, 1U, 5'000'000'000LL, 5'010'000'000LL);
  const MissionWaypointAcknowledgementAdmissionState progressed =
      admitMissionWaypointAcknowledgement(initialState(), second).next_state;
  EXPECT_TRUE(admitMissionWaypointAcknowledgement(progressed, second).replay);
  EXPECT_TRUE(admitMissionWaypointAcknowledgement(progressed, acknowledgement()).stale);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     ExactWireRedeliveryDoesNotConflictOrRejuvenateLocalReceipt) {
  const MissionWaypointAcknowledgementAdmissionState state = initialState();
  MissionWaypointAcknowledgementCandidate redelivery = acknowledgement();
  redelivery.receive_stamp_ns = redelivery.horizon_valid_until_ns + 5'000'000'000LL;

  const MissionWaypointAcknowledgementAdmissionResult replay =
      admitMissionWaypointAcknowledgement(state, redelivery);
  EXPECT_TRUE(replay.replay);
  EXPECT_FALSE(replay.accept);
  EXPECT_FALSE(replay.conflict);
  EXPECT_FALSE(replay.state_advanced);
  EXPECT_EQ(replay.next_state.current.receive_stamp_ns, state.current.receive_stamp_ns);
  EXPECT_TRUE(replay.next_state.current.payload_valid);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     AggregateProgressRecoversAcrossDroppedSequence) {
  const MissionWaypointAcknowledgementAdmissionState state = initialState();
  const MissionWaypointAcknowledgementCandidate after_drop =
      acknowledgement(11U, 3U, 2U, 1U, 5'000'000'000LL, 5'010'000'000LL);
  const MissionWaypointAcknowledgementAdmissionResult admitted =
      admitMissionWaypointAcknowledgement(state, after_drop);
  EXPECT_TRUE(admitted.accept);
  EXPECT_EQ(admitted.next_state.current.completed_waypoint_count, 2U);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     ProducerHandoffAcceptsExternallyPreservedStatusAndRetiresOldProducer) {
  const MissionWaypointAcknowledgementAdmissionState progressed =
      admitMissionWaypointAcknowledgement(
          initialState(),
          acknowledgement(11U, 2U, 2U, 1U, 5'000'000'000LL, 5'010'000'000LL))
          .next_state;
  const MissionWaypointAcknowledgementCandidate restarted =
      acknowledgement(22U, 1U, 2U, 1U, 6'000'000'000LL, 6'010'000'000LL);
  const MissionWaypointAcknowledgementAdmissionResult transition =
      admitMissionWaypointAcknowledgement(progressed, restarted);
  EXPECT_FALSE(transition.accept);
  EXPECT_TRUE(transition.aggregate_replay);
  EXPECT_TRUE(transition.producer_transitioned);
  EXPECT_TRUE(transition.next_state.producerRetired(11U));
  const MissionWaypointAcknowledgementAdmissionResult resumed_progress =
      admitMissionWaypointAcknowledgement(
          transition.next_state,
          acknowledgement(22U, 2U, 3U, 2U, 7'000'000'000LL, 7'010'000'000LL));
  EXPECT_TRUE(resumed_progress.accept);
  EXPECT_TRUE(admitMissionWaypointAcknowledgement(
                  resumed_progress.next_state,
                  acknowledgement(11U, 3U, 3U, 2U, 7'000'000'000LL, 7'010'000'000LL))
                  .stale);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     ProducerHandoffRejectsResetCountOrObjectiveEpoch) {
  const MissionWaypointAcknowledgementAdmissionState progressed =
      admitMissionWaypointAcknowledgement(
          initialState(),
          acknowledgement(11U, 2U, 2U, 1U, 5'000'000'000LL, 5'010'000'000LL))
          .next_state;
  EXPECT_TRUE(admitMissionWaypointAcknowledgement(
                  progressed,
                  acknowledgement(22U, 1U, 1U, 0U, 6'000'000'000LL, 6'010'000'000LL))
                  .stale);
  EXPECT_TRUE(admitMissionWaypointAcknowledgement(
                  progressed,
                  acknowledgement(22U, 1U, 3U, 1U, 6'000'000'000LL, 6'010'000'000LL))
                  .stale);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     SameIdentityWithDifferentGeometryConflictsAndTombstonesReplay) {
  const MissionWaypointAcknowledgementAdmissionState state = initialState();
  MissionWaypointAcknowledgementCandidate conflict = acknowledgement();
  conflict.route_target.z += 1.0;
  const MissionWaypointAcknowledgementAdmissionResult rejected =
      admitMissionWaypointAcknowledgement(state, conflict);
  ASSERT_TRUE(rejected.conflict);
  ASSERT_TRUE(rejected.state_advanced);
  EXPECT_TRUE(rejected.next_state.current_identity_tombstoned);
  EXPECT_TRUE(rejected.next_state.current.payload_valid);
  EXPECT_TRUE(
      admitMissionWaypointAcknowledgement(rejected.next_state, acknowledgement())
          .conflict);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     NewerMalformedGeometryAdvancesHighWaterAndCannotBeResurrected) {
  const MissionWaypointAcknowledgementAdmissionState state = initialState();
  MissionWaypointAcknowledgementCandidate malformed =
      acknowledgement(11U, 2U, 2U, 1U, 5'000'000'000LL, 5'010'000'000LL);
  malformed.payload_valid = false;
  const MissionWaypointAcknowledgementAdmissionResult rejected =
      admitMissionWaypointAcknowledgement(state, malformed);
  ASSERT_TRUE(rejected.invalid);
  ASSERT_TRUE(rejected.state_advanced);
  malformed.payload_valid = true;
  EXPECT_TRUE(
      admitMissionWaypointAcknowledgement(rejected.next_state, malformed).conflict);
}

TEST(MissionWaypointAcknowledgementAdmissionTest, RejectsMalformedStoredState) {
  MissionWaypointAcknowledgementAdmissionState malformed;
  malformed.current.producer_instance_id = 9U;
  EXPECT_FALSE(malformed.valid());
  EXPECT_TRUE(
      admitMissionWaypointAcknowledgement(malformed, acknowledgement()).invalid);

  malformed = {};
  malformed.current.payload_valid = true;
  EXPECT_FALSE(malformed.valid());
  EXPECT_TRUE(
      admitMissionWaypointAcknowledgement(malformed, acknowledgement()).invalid);

  malformed = initialState();
  malformed.retired_producer_count = 1U;
  EXPECT_FALSE(malformed.valid());
  EXPECT_TRUE(
      admitMissionWaypointAcknowledgement(malformed, acknowledgement()).invalid);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     SameProducerRequiresCountAndMissionEpochAdvance) {
  const MissionWaypointAcknowledgementAdmissionState state = initialState();
  EXPECT_TRUE(
      admitMissionWaypointAcknowledgement(
          state, acknowledgement(11U, 2U, 1U, 1U, 5'000'000'000LL, 5'010'000'000LL))
          .stale);
  EXPECT_TRUE(
      admitMissionWaypointAcknowledgement(
          state, acknowledgement(11U, 2U, 2U, 0U, 5'000'000'000LL, 5'010'000'000LL))
          .stale);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     RejectedSameProducerIdentityCannotBeMutatedAndHigherSequenceRecovers) {
  const MissionWaypointAcknowledgementAdmissionState state = initialState();
  const MissionWaypointAcknowledgementCandidate regressed =
      acknowledgement(11U, 2U, 1U, 0U, 5'000'000'000LL, 5'010'000'000LL);
  const MissionWaypointAcknowledgementAdmissionResult rejected =
      admitMissionWaypointAcknowledgement(state, regressed);
  ASSERT_TRUE(rejected.stale);
  ASSERT_TRUE(rejected.state_advanced);
  EXPECT_EQ(rejected.next_state.current.acknowledgement_sequence, 2U);
  EXPECT_EQ(rejected.next_state.last_accepted.completed_waypoint_count, 1U);

  const MissionWaypointAcknowledgementCandidate mutated =
      acknowledgement(11U, 2U, 2U, 1U, 5'000'000'000LL, 5'010'000'000LL);
  const MissionWaypointAcknowledgementAdmissionResult conflict =
      admitMissionWaypointAcknowledgement(rejected.next_state, mutated);
  EXPECT_TRUE(conflict.conflict);
  EXPECT_TRUE(conflict.next_state.current_identity_tombstoned);

  const MissionWaypointAcknowledgementAdmissionResult recovered =
      admitMissionWaypointAcknowledgement(
          conflict.next_state,
          acknowledgement(11U, 3U, 2U, 1U, 6'000'000'000LL, 6'010'000'000LL));
  EXPECT_TRUE(recovered.accept);
  EXPECT_EQ(recovered.next_state.last_accepted.completed_waypoint_count, 2U);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     StructurallyInvalidIdentityIsClaimedBeforePayloadAndRequiresHigherSequence) {
  MissionWaypointAcknowledgementCandidate malformed = acknowledgement();
  malformed.waypoint_count = 0U;
  const MissionWaypointAcknowledgementAdmissionResult rejected =
      admitMissionWaypointAcknowledgement({}, malformed);
  ASSERT_TRUE(rejected.invalid);
  ASSERT_TRUE(rejected.state_advanced);
  ASSERT_TRUE(rejected.next_state.valid());

  const MissionWaypointAcknowledgementAdmissionResult conflict =
      admitMissionWaypointAcknowledgement(rejected.next_state, acknowledgement());
  EXPECT_TRUE(conflict.conflict);
  const MissionWaypointAcknowledgementAdmissionResult recovered =
      admitMissionWaypointAcknowledgement(
          conflict.next_state,
          acknowledgement(11U, 2U, 1U, 0U, 5'000'000'000LL, 5'010'000'000LL));
  EXPECT_TRUE(recovered.accept);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     RejectedProspectiveIdentityCannotBeMutatedAndHigherSequenceRecovers) {
  const MissionWaypointAcknowledgementAdmissionState state = initialState();
  const MissionWaypointAcknowledgementCandidate stale_handoff =
      acknowledgement(22U, 1U, 1U, 0U, 4'000'000'000LL, 4'020'000'000LL);
  const MissionWaypointAcknowledgementAdmissionResult rejected =
      admitMissionWaypointAcknowledgement(state, stale_handoff);
  ASSERT_TRUE(rejected.stale);
  ASSERT_TRUE(rejected.state_advanced);
  ASSERT_EQ(rejected.next_state.prospective_identity_count, 1U);
  EXPECT_EQ(rejected.next_state.current.producer_instance_id, 11U);

  const MissionWaypointAcknowledgementCandidate mutated =
      acknowledgement(22U, 1U, 2U, 1U, 5'000'000'000LL, 5'010'000'000LL);
  const MissionWaypointAcknowledgementAdmissionResult conflict =
      admitMissionWaypointAcknowledgement(rejected.next_state, mutated);
  EXPECT_TRUE(conflict.conflict);

  const MissionWaypointAcknowledgementAdmissionResult recovered =
      admitMissionWaypointAcknowledgement(
          conflict.next_state,
          acknowledgement(22U, 2U, 2U, 1U, 5'000'000'000LL, 5'010'000'000LL));
  EXPECT_TRUE(recovered.accept);
  EXPECT_TRUE(recovered.producer_transitioned);
  EXPECT_TRUE(recovered.next_state.producerRetired(11U));
  EXPECT_EQ(recovered.next_state.prospective_identity_count, 0U);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     InvalidProspectivePayloadCannotRetireTheActiveProducer) {
  const MissionWaypointAcknowledgementAdmissionState state = initialState();
  MissionWaypointAcknowledgementCandidate malformed =
      acknowledgement(22U, 1U, 1U, 0U, 5'000'000'000LL, 5'010'000'000LL);
  malformed.payload_valid = false;

  const MissionWaypointAcknowledgementAdmissionResult rejected =
      admitMissionWaypointAcknowledgement(state, malformed);
  ASSERT_TRUE(rejected.invalid);
  ASSERT_TRUE(rejected.state_advanced);
  EXPECT_FALSE(rejected.producer_transitioned);
  EXPECT_EQ(rejected.next_state.current.producer_instance_id, 11U);
  EXPECT_FALSE(rejected.next_state.producerRetired(11U));
  ASSERT_EQ(rejected.next_state.prospective_identity_count, 1U);
  EXPECT_EQ(
      rejected.next_state.prospective_identity_claims[0U].candidate.receive_stamp_ns,
      malformed.receive_stamp_ns);

  MissionWaypointAcknowledgementCandidate redelivery = malformed;
  redelivery.receive_stamp_ns = 9'000'000'000LL;
  const MissionWaypointAcknowledgementAdmissionResult replay =
      admitMissionWaypointAcknowledgement(rejected.next_state, redelivery);
  EXPECT_TRUE(replay.replay);
  EXPECT_FALSE(replay.state_advanced);
  EXPECT_EQ(
      replay.next_state.prospective_identity_claims[0U].candidate.receive_stamp_ns,
      malformed.receive_stamp_ns);

  MissionWaypointAcknowledgementCandidate corrected_same_identity = malformed;
  corrected_same_identity.payload_valid = true;
  const MissionWaypointAcknowledgementAdmissionResult conflict =
      admitMissionWaypointAcknowledgement(rejected.next_state, corrected_same_identity);
  EXPECT_TRUE(conflict.conflict);
  EXPECT_FALSE(conflict.producer_transitioned);
  EXPECT_EQ(conflict.next_state.current.producer_instance_id, 11U);

  const MissionWaypointAcknowledgementAdmissionResult recovered =
      admitMissionWaypointAcknowledgement(
          conflict.next_state,
          acknowledgement(22U, 2U, 2U, 1U, 6'000'000'000LL, 6'010'000'000LL));
  EXPECT_TRUE(recovered.accept);
  EXPECT_TRUE(recovered.producer_transitioned);
  EXPECT_TRUE(recovered.next_state.producerRetired(11U));
  EXPECT_EQ(recovered.next_state.current.producer_instance_id, 22U);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     ProspectiveCapacityLatchStaysClosedAfterExistingClaimFreesSlot) {
  MissionWaypointAcknowledgementAdmissionState state = initialState();
  for (std::size_t index = 0U; index < kProspectiveMissionWaypointProducerCapacity;
       ++index) {
    const std::uint64_t producer = 20U + static_cast<std::uint64_t>(index);
    const MissionWaypointAcknowledgementAdmissionResult claimed =
        admitMissionWaypointAcknowledgement(
            state,
            acknowledgement(producer, 1U, 1U, 0U, 4'000'000'000LL, 4'020'000'000LL));
    ASSERT_TRUE(claimed.stale);
    ASSERT_TRUE(claimed.state_advanced);
    state = claimed.next_state;
  }
  ASSERT_EQ(state.prospective_identity_count,
            kProspectiveMissionWaypointProducerCapacity);

  const MissionWaypointAcknowledgementAdmissionResult overflow =
      admitMissionWaypointAcknowledgement(
          state, acknowledgement(99U, 1U, 1U, 0U, 4'000'000'000LL, 4'020'000'000LL));
  EXPECT_TRUE(overflow.prospective_capacity_exhausted);
  EXPECT_TRUE(overflow.invalid);
  EXPECT_TRUE(overflow.state_advanced);
  EXPECT_EQ(overflow.next_state.current.producer_instance_id, 11U);
  EXPECT_EQ(overflow.next_state.prospective_identity_count,
            kProspectiveMissionWaypointProducerCapacity);
  EXPECT_TRUE(overflow.next_state.prospective_identity_capacity_exhausted);

  const MissionWaypointAcknowledgementAdmissionResult handoff =
      admitMissionWaypointAcknowledgement(
          overflow.next_state,
          acknowledgement(20U, 2U, 1U, 0U, 5'000'000'000LL, 5'010'000'000LL));
  ASSERT_TRUE(handoff.producer_transitioned);
  ASSERT_TRUE(handoff.aggregate_replay);
  ASSERT_FALSE(handoff.accept);
  ASSERT_EQ(handoff.next_state.current.producer_instance_id, 20U);
  ASSERT_TRUE(handoff.next_state.producerRetired(11U));
  ASSERT_EQ(handoff.next_state.prospective_identity_count,
            kProspectiveMissionWaypointProducerCapacity - 1U);
  ASSERT_TRUE(handoff.next_state.prospective_identity_capacity_exhausted);

  const MissionWaypointAcknowledgementAdmissionResult permanently_closed =
      admitMissionWaypointAcknowledgement(
          handoff.next_state,
          acknowledgement(100U, 1U, 1U, 0U, 6'000'000'000LL, 6'020'000'000LL));
  EXPECT_TRUE(permanently_closed.prospective_capacity_exhausted);
  EXPECT_FALSE(permanently_closed.state_advanced);
  EXPECT_EQ(permanently_closed.next_state.current.producer_instance_id, 20U);
  EXPECT_EQ(permanently_closed.next_state.prospective_identity_count,
            kProspectiveMissionWaypointProducerCapacity - 1U);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     ProspectiveReplayDoesNotRefreshReceiptAndHigherSequenceMustAdvanceIt) {
  const MissionWaypointAcknowledgementAdmissionState state = initialState();
  MissionWaypointAcknowledgementCandidate first =
      acknowledgement(22U, 1U, 1U, 0U, 5'000'000'000LL, 5'010'000'000LL);
  first.waypoint_count = 4U;
  const MissionWaypointAcknowledgementAdmissionResult claimed =
      admitMissionWaypointAcknowledgement(state, first);
  ASSERT_TRUE(claimed.stale);
  ASSERT_TRUE(claimed.state_advanced);

  MissionWaypointAcknowledgementCandidate redelivery =
      claimed.next_state.prospective_identity_claims[0U].candidate;
  redelivery.receive_stamp_ns = 9'000'000'000LL;
  const MissionWaypointAcknowledgementAdmissionResult replay =
      admitMissionWaypointAcknowledgement(claimed.next_state, redelivery);
  EXPECT_TRUE(replay.replay);
  EXPECT_FALSE(replay.state_advanced);
  EXPECT_EQ(
      replay.next_state.prospective_identity_claims[0U].candidate.receive_stamp_ns,
      claimed.next_state.prospective_identity_claims[0U].candidate.receive_stamp_ns);

  MissionWaypointAcknowledgementCandidate regressed_successor =
      acknowledgement(22U, 2U, 2U, 1U, 4'500'000'000LL, 5'500'000'000LL);
  const MissionWaypointAcknowledgementAdmissionResult rejected_successor =
      admitMissionWaypointAcknowledgement(claimed.next_state, regressed_successor);
  EXPECT_TRUE(rejected_successor.stale);
  EXPECT_TRUE(rejected_successor.state_advanced);
  EXPECT_EQ(rejected_successor.next_state.prospective_identity_claims[0U]
                .source_stamp_high_water_ns,
            5'000'000'000LL);

  const MissionWaypointAcknowledgementAdmissionResult recovered =
      admitMissionWaypointAcknowledgement(
          rejected_successor.next_state,
          acknowledgement(22U, 3U, 2U, 1U, 6'000'000'000LL, 6'010'000'000LL));
  EXPECT_TRUE(recovered.accept);
}

TEST(MissionWaypointAcknowledgementAdmissionTest,
     RejectsAcknowledgementProducedOutsideWitnessedLease) {
  MissionWaypointAcknowledgementCandidate after_expiry = acknowledgement();
  after_expiry.source_stamp_ns = after_expiry.horizon_valid_until_ns;
  after_expiry.receive_stamp_ns = after_expiry.source_stamp_ns;
  EXPECT_TRUE(admitMissionWaypointAcknowledgement({}, after_expiry).invalid);

  MissionWaypointAcknowledgementCandidate before_lease = acknowledgement();
  before_lease.source_stamp_ns = before_lease.horizon_valid_from_ns - 1;
  before_lease.receive_stamp_ns = before_lease.horizon_valid_from_ns;
  EXPECT_TRUE(admitMissionWaypointAcknowledgement({}, before_lease).invalid);
}

} // namespace
} // namespace drone_city_nav
