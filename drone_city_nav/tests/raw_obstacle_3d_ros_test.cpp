#include "drone_city_nav/raw_obstacle_3d_ros.hpp"

#include <gtest/gtest.h>

#include <array>

namespace drone_city_nav {
namespace {

constexpr GridBounds3D kBounds{-10.0, -20.0, 0.0, 0.5, 64, 80, 40};
constexpr ProducerEpochAdmissionConfig kAdmissionConfig{
    .maximum_observation_age_ns = 10'000'000'000LL,
    .maximum_confirmation_interval_ns = 2'000'000'000LL,
};

[[nodiscard]] std_msgs::msg::Header headerAt(const std::int64_t stamp_ns) {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  return header;
}

[[nodiscard]] ProducerEpochAdmissionState
authorizedEpoch(const std::uint64_t producer,
                const std::int64_t stamp_ns = 900'000'000LL) {
  const ProducerEpochAdmissionResult admission = admitProducerEpoch(
      kAdmissionConfig, {},
      ProducerEpochObservation{.producer_instance_id = producer,
                               .sequence = 1U,
                               .source_stamp_ns = stamp_ns,
                               .receive_stamp_ns = stamp_ns + 10'000'000LL,
                               .content_fingerprint = 101U},
      stamp_ns + 20'000'000LL);
  EXPECT_TRUE(admission.install_observation);
  return admission.next_state;
}

[[nodiscard]] RawObstacleGridUpdate3D
applySnapshot(RawObstacleDeltaAccumulator3D& accumulator,
              const msg::RawObstacleSnapshot3D& snapshot,
              const ProducerEpochAdmissionState& epoch,
              const std::int64_t receive_stamp_ns) {
  return accumulator.apply(snapshot, epoch, receive_stamp_ns,
                           receive_stamp_ns + 10'000'000LL, kAdmissionConfig);
}

[[nodiscard]] RawObstacleGridUpdate3D applyDelta(
    RawObstacleDeltaAccumulator3D& accumulator, const msg::RawObstacleDelta3D& delta,
    const ProducerEpochAdmissionState& epoch, const std::int64_t receive_stamp_ns) {
  return accumulator.apply(delta, epoch, receive_stamp_ns,
                           receive_stamp_ns + 10'000'000LL, kAdmissionConfig);
}

TEST(RawObstacle3DRos, SnapshotAndCumulativeDeltaRoundTripTriStateChunks) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  static_cast<void>(producer.setState({20, 3, 4}, ObservedVoxelState::kOccupied));
  const std_msgs::msg::Header header = headerAt(1'000'000'000LL);
  const msg::RawObstacleSnapshot3D snapshot =
      makeRawObstacleSnapshot3D(producer, header, 42U, 1U);

  RawObstacleDeltaAccumulator3D accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(42U);
  const RawObstacleGridUpdate3D initial =
      applySnapshot(accumulator, snapshot, epoch, 1'010'000'000LL);
  ASSERT_TRUE(initial.accepted());
  EXPECT_TRUE(initial.full_reset);
  EXPECT_TRUE(initial.dirty_chunks.empty());
  ASSERT_NE(initial.state.occupancy, nullptr);
  EXPECT_TRUE(initial.state.occupancy->isKnownFree({2, 3, 4}));
  EXPECT_TRUE(initial.state.occupancy->isOccupied({20, 3, 4}));
  EXPECT_FALSE(initial.state.occupancy->isKnown({3, 3, 4}));

  static_cast<void>(producer.setState({20, 3, 4}, ObservedVoxelState::kFree));
  static_cast<void>(producer.setState({35, 3, 4}, ObservedVoxelState::kOccupied));
  const std::array dirty{ObservedOccupancyGrid3D::chunkIndex({20, 3, 4}),
                         ObservedOccupancyGrid3D::chunkIndex({35, 3, 4})};
  const msg::RawObstacleDelta3D delta =
      makeRawObstacleDelta3D(producer, headerAt(2'000'000'000LL), 42U, 1U, 2U, dirty);
  const RawObstacleGridUpdate3D updated =
      applyDelta(accumulator, delta, epoch, 2'010'000'000LL);

  ASSERT_TRUE(updated.accepted());
  EXPECT_FALSE(updated.full_reset);
  ASSERT_EQ(updated.dirty_chunks.size(), 2U);
  EXPECT_EQ(updated.dirty_chunks[0], ObservedOccupancyGrid3D::chunkIndex({20, 3, 4}));
  EXPECT_EQ(updated.dirty_chunks[1], ObservedOccupancyGrid3D::chunkIndex({35, 3, 4}));
  ASSERT_NE(updated.state.occupancy, nullptr);
  EXPECT_TRUE(updated.state.occupancy->isKnownFree({20, 3, 4}));
  EXPECT_TRUE(updated.state.occupancy->isOccupied({35, 3, 4}));
  EXPECT_EQ(updated.state.base_snapshot_revision, 1U);
  EXPECT_EQ(updated.state.obstacle_snapshot_revision, 2U);

  const msg::RawObstacleDelta3D repeated_delta =
      makeRawObstacleDelta3D(producer, headerAt(3'000'000'000LL), 42U, 1U, 3U, dirty);
  const RawObstacleGridUpdate3D repeated =
      applyDelta(accumulator, repeated_delta, epoch, 3'010'000'000LL);
  ASSERT_TRUE(repeated.accepted());
  EXPECT_TRUE(repeated.dirty_chunks.empty());
}

TEST(RawObstacle3DRos, RejectsDeltaFromDifferentBase) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  const std_msgs::msg::Header header = headerAt(1'000'000'000LL);
  RawObstacleDeltaAccumulator3D accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(42U);
  ASSERT_TRUE(applySnapshot(accumulator,
                            makeRawObstacleSnapshot3D(producer, header, 42U, 5U), epoch,
                            1'010'000'000LL)
                  .accepted());
  const std::array dirty{ObservedOccupancyGrid3D::chunkIndex({2, 3, 4})};

  const RawObstacleGridUpdate3D update = applyDelta(
      accumulator,
      makeRawObstacleDelta3D(producer, headerAt(2'000'000'000LL), 42U, 4U, 6U, dirty),
      epoch, 2'010'000'000LL);

  EXPECT_EQ(update.status, RawObstacleGridUpdateStatus3D::kBaseUnavailable);
}

TEST(RawObstacle3DRos, RejectsDuplicateChunkInDelta) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  const std_msgs::msg::Header header = headerAt(1'000'000'000LL);
  RawObstacleDeltaAccumulator3D accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(42U);
  ASSERT_TRUE(applySnapshot(accumulator,
                            makeRawObstacleSnapshot3D(producer, header, 42U, 1U), epoch,
                            1'010'000'000LL)
                  .accepted());
  const OccupancyChunkIndex3D chunk = ObservedOccupancyGrid3D::chunkIndex({2, 3, 4});
  const std::array dirty{chunk, chunk};

  const RawObstacleGridUpdate3D update = applyDelta(
      accumulator,
      makeRawObstacleDelta3D(producer, headerAt(2'000'000'000LL), 42U, 1U, 2U, dirty),
      epoch, 2'010'000'000LL);

  EXPECT_EQ(update.status, RawObstacleGridUpdateStatus3D::kInvalidMessage);
}

TEST(RawObstacle3DRos, RejectsOccupiedBitsOutsideObservedMask) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kOccupied));
  RawObstacleDeltaAccumulator3D accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(42U);
  ASSERT_TRUE(applySnapshot(accumulator,
                            makeRawObstacleSnapshot3D(
                                producer, headerAt(1'000'000'000LL), 42U, 1U),
                            epoch, 1'010'000'000LL)
                  .accepted());
  const std::array dirty{ObservedOccupancyGrid3D::chunkIndex({2, 3, 4})};
  msg::RawObstacleDelta3D contradictory =
      makeRawObstacleDelta3D(producer, headerAt(2'000'000'000LL), 42U, 1U, 2U, dirty);
  ASSERT_FALSE(contradictory.chunks.empty());
  contradictory.chunks.front().observed_words.fill(0U);

  const RawObstacleGridUpdate3D update =
      applyDelta(accumulator, contradictory, epoch, 2'010'000'000LL);

  EXPECT_EQ(update.status, RawObstacleGridUpdateStatus3D::kInvalidMessage);
  EXPECT_EQ(accumulator.state().obstacle_snapshot_revision, 1U);
}

TEST(RawObstacle3DRos, NewerSnapshotUsesDirtyChunksWhenGeometryIsStable) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  static_cast<void>(producer.setState({20, 3, 4}, ObservedVoxelState::kOccupied));
  const std_msgs::msg::Header header = headerAt(1'000'000'000LL);
  RawObstacleDeltaAccumulator3D accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(42U);

  ASSERT_TRUE(applySnapshot(accumulator,
                            makeRawObstacleSnapshot3D(producer, header, 42U, 1U), epoch,
                            1'010'000'000LL)
                  .accepted());

  const RawObstacleGridUpdate3D unchanged = applySnapshot(
      accumulator,
      makeRawObstacleSnapshot3D(producer, headerAt(2'000'000'000LL), 42U, 2U), epoch,
      2'010'000'000LL);
  ASSERT_TRUE(unchanged.accepted());
  EXPECT_FALSE(unchanged.full_reset);
  EXPECT_TRUE(unchanged.dirty_chunks.empty());

  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kOccupied));
  static_cast<void>(producer.setState({35, 3, 4}, ObservedVoxelState::kFree));
  const RawObstacleGridUpdate3D changed = applySnapshot(
      accumulator,
      makeRawObstacleSnapshot3D(producer, headerAt(3'000'000'000LL), 42U, 3U), epoch,
      3'010'000'000LL);

  ASSERT_TRUE(changed.accepted());
  EXPECT_FALSE(changed.full_reset);
  const std::array expected{ObservedOccupancyGrid3D::chunkIndex({2, 3, 4}),
                            ObservedOccupancyGrid3D::chunkIndex({35, 3, 4})};
  ASSERT_EQ(changed.dirty_chunks.size(), expected.size());
  EXPECT_EQ(changed.dirty_chunks[0], expected[0]);
  EXPECT_EQ(changed.dirty_chunks[1], expected[1]);
  ASSERT_NE(changed.state.occupancy, nullptr);
  EXPECT_TRUE(changed.state.occupancy->isOccupied({2, 3, 4}));
  EXPECT_TRUE(changed.state.occupancy->isKnownFree({35, 3, 4}));
}

TEST(RawObstacle3DRos, SameRevisionDifferentCanonicalWorldQuarantinesUntilNewerWorld) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  RawObstacleDeltaAccumulator3D accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(42U);
  const msg::RawObstacleSnapshot3D initial =
      makeRawObstacleSnapshot3D(producer, headerAt(1'000'000'000LL), 42U, 1U);
  ASSERT_TRUE(applySnapshot(accumulator, initial, epoch, 1'010'000'000LL).accepted());

  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kOccupied));
  const msg::RawObstacleSnapshot3D conflict =
      makeRawObstacleSnapshot3D(producer, headerAt(1'000'000'000LL), 42U, 1U);
  const RawObstacleGridUpdate3D conflicted =
      applySnapshot(accumulator, conflict, epoch, 1'020'000'000LL);
  ASSERT_EQ(conflicted.status, RawObstacleGridUpdateStatus3D::kIdentityConflict);
  EXPECT_TRUE(conflicted.current_identity_conflict);
  EXPECT_TRUE(accumulator.evidenceAdmissionState().current_identity_conflicted);

  const RawObstacleGridUpdate3D recovered = applySnapshot(
      accumulator,
      makeRawObstacleSnapshot3D(producer, headerAt(2'000'000'000LL), 42U, 2U), epoch,
      2'010'000'000LL);
  ASSERT_TRUE(recovered.accepted());
  EXPECT_FALSE(accumulator.evidenceAdmissionState().current_identity_conflicted);
  EXPECT_TRUE(recovered.state.occupancy->isOccupied({2, 3, 4}));
}

TEST(RawObstacle3DRos,
     MalformedClaimCannotBeRewrittenAndStrictlyNewerSnapshotRecovers) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  RawObstacleDeltaAccumulator3D accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(42U);
  const msg::RawObstacleSnapshot3D initial =
      makeRawObstacleSnapshot3D(producer, headerAt(1'000'000'000LL), 42U, 1U);
  ASSERT_TRUE(applySnapshot(accumulator, initial, epoch, 1'010'000'000LL).accepted());

  msg::RawObstacleSnapshot3D malformed = initial;
  malformed.obstacle_snapshot_revision = 2U;
  malformed.header = headerAt(2'000'000'000LL);
  ASSERT_FALSE(malformed.chunks.empty());
  malformed.chunks.push_back(malformed.chunks.front());
  const RawObstacleGridUpdate3D rejected =
      applySnapshot(accumulator, malformed, epoch, 2'010'000'000LL);
  ASSERT_EQ(rejected.status, RawObstacleGridUpdateStatus3D::kInvalidMessage);
  ASSERT_EQ(accumulator.evidenceAdmissionState().rejected_identity_count, 1U);
  EXPECT_EQ(accumulator.evidenceAdmissionState()
                .rejected_identities[0]
                .first_receive_stamp_ns,
            2'010'000'000LL);

  EXPECT_EQ(applySnapshot(accumulator, malformed, epoch, 2'500'000'000LL).status,
            RawObstacleGridUpdateStatus3D::kInvalidMessage);
  EXPECT_EQ(accumulator.evidenceAdmissionState()
                .rejected_identities[0]
                .first_receive_stamp_ns,
            2'010'000'000LL);

  msg::RawObstacleSnapshot3D rewritten = malformed;
  rewritten.chunks.pop_back();
  const RawObstacleGridUpdate3D conflict =
      applySnapshot(accumulator, rewritten, epoch, 2'510'000'000LL);
  ASSERT_EQ(conflict.status, RawObstacleGridUpdateStatus3D::kIdentityConflict);
  EXPECT_TRUE(conflict.current_identity_conflict);

  msg::RawObstacleSnapshot3D recovered = rewritten;
  recovered.obstacle_snapshot_revision = 3U;
  recovered.header = headerAt(3'000'000'000LL);
  const RawObstacleGridUpdate3D newer =
      applySnapshot(accumulator, recovered, epoch, 3'010'000'000LL);
  EXPECT_TRUE(newer.accepted());
  EXPECT_FALSE(accumulator.evidenceAdmissionState().current_identity_conflicted);
}

TEST(RawObstacle3DRos,
     SameRevisionDeltaConflictQuarantinesResidentWorldUntilNewerDelta) {
  ObservedOccupancyGrid3D producer{kBounds};
  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  RawObstacleDeltaAccumulator3D accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(42U);
  ASSERT_TRUE(applySnapshot(accumulator,
                            makeRawObstacleSnapshot3D(
                                producer, headerAt(1'000'000'000LL), 42U, 1U),
                            epoch, 1'010'000'000LL)
                  .accepted());
  const std::array dirty{ObservedOccupancyGrid3D::chunkIndex({2, 3, 4})};

  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kOccupied));
  const msg::RawObstacleDelta3D accepted_delta =
      makeRawObstacleDelta3D(producer, headerAt(2'000'000'000LL), 42U, 1U, 2U, dirty);
  ASSERT_TRUE(
      applyDelta(accumulator, accepted_delta, epoch, 2'010'000'000LL).accepted());

  static_cast<void>(producer.setState({2, 3, 4}, ObservedVoxelState::kFree));
  const msg::RawObstacleDelta3D conflicting_delta =
      makeRawObstacleDelta3D(producer, headerAt(2'000'000'000LL), 42U, 1U, 2U, dirty);
  const RawObstacleGridUpdate3D conflict =
      applyDelta(accumulator, conflicting_delta, epoch, 2'020'000'000LL);
  ASSERT_EQ(conflict.status, RawObstacleGridUpdateStatus3D::kIdentityConflict);
  EXPECT_TRUE(conflict.current_identity_conflict);
  EXPECT_EQ(applyDelta(accumulator, accepted_delta, epoch, 2'030'000'000LL).status,
            RawObstacleGridUpdateStatus3D::kIdentityConflict);

  const msg::RawObstacleDelta3D newer_delta =
      makeRawObstacleDelta3D(producer, headerAt(3'000'000'000LL), 42U, 1U, 3U, dirty);
  const RawObstacleGridUpdate3D recovered =
      applyDelta(accumulator, newer_delta, epoch, 3'010'000'000LL);
  ASSERT_TRUE(recovered.accepted());
  EXPECT_FALSE(accumulator.evidenceAdmissionState().current_identity_conflicted);
  EXPECT_TRUE(recovered.state.occupancy->isKnownFree({2, 3, 4}));
}

TEST(RawObstacle3DRos,
     CachedFullSnapshotActivatesAfterTwoStatusSamplesWithoutWaitingForNextFull) {
  ObservedOccupancyGrid3D first_grid{kBounds};
  static_cast<void>(first_grid.setState({2, 3, 4}, ObservedVoxelState::kFree));
  RawObstacleDeltaAccumulator3D accumulator;
  ProducerEpochAdmissionState epoch = authorizedEpoch(42U);
  ASSERT_TRUE(applySnapshot(accumulator,
                            makeRawObstacleSnapshot3D(
                                first_grid, headerAt(1'000'000'000LL), 42U, 1U),
                            epoch, 1'010'000'000LL)
                  .accepted());

  ObservedOccupancyGrid3D second_grid{kBounds};
  static_cast<void>(second_grid.setState({20, 3, 4}, ObservedVoxelState::kOccupied));
  const msg::RawObstacleSnapshot3D cached =
      makeRawObstacleSnapshot3D(second_grid, headerAt(11'900'000'000LL), 84U, 1U);
  EXPECT_EQ(applySnapshot(accumulator, cached, epoch, 11'910'000'000LL).status,
            RawObstacleGridUpdateStatus3D::kProducerEpochPending);

  const ProducerEpochAdmissionResult pending =
      admitProducerEpoch(kAdmissionConfig, epoch,
                         ProducerEpochObservation{.producer_instance_id = 84U,
                                                  .sequence = 1U,
                                                  .source_stamp_ns = 12'000'000'000LL,
                                                  .receive_stamp_ns = 12'010'000'000LL,
                                                  .content_fingerprint = 201U},
                         12'020'000'000LL);
  const ProducerEpochAdmissionResult handoff =
      admitProducerEpoch(kAdmissionConfig, pending.next_state,
                         ProducerEpochObservation{.producer_instance_id = 84U,
                                                  .sequence = 2U,
                                                  .source_stamp_ns = 12'100'000'000LL,
                                                  .receive_stamp_ns = 12'110'000'000LL,
                                                  .content_fingerprint = 202U},
                         12'120'000'000LL);
  ASSERT_EQ(handoff.status, ProducerEpochAdmissionStatus::kAcceptedProducerHandoff);
  epoch = handoff.next_state;

  const RawObstacleGridUpdate3D activated =
      accumulator.synchronizeProducerEpoch(epoch, 12'130'000'000LL, kAdmissionConfig);
  ASSERT_TRUE(activated.accepted());
  EXPECT_TRUE(activated.full_reset);
  EXPECT_TRUE(activated.authority_handoff);
  EXPECT_EQ(activated.state.producer_instance_id, 84U);
  EXPECT_TRUE(activated.state.occupancy->isOccupied({20, 3, 4}));
}

TEST(RawObstacle3DRos, ProspectiveDeltaCannotResurrectAsSameIdentityFullSnapshot) {
  ObservedOccupancyGrid3D first_grid{kBounds};
  static_cast<void>(first_grid.setState({2, 3, 4}, ObservedVoxelState::kFree));
  RawObstacleDeltaAccumulator3D accumulator;
  ProducerEpochAdmissionState epoch = authorizedEpoch(42U);
  ASSERT_TRUE(applySnapshot(accumulator,
                            makeRawObstacleSnapshot3D(
                                first_grid, headerAt(1'000'000'000LL), 42U, 1U),
                            epoch, 1'010'000'000LL)
                  .accepted());

  ObservedOccupancyGrid3D foreign_grid{kBounds};
  static_cast<void>(foreign_grid.setState({20, 3, 4}, ObservedVoxelState::kOccupied));
  const std::array dirty{ObservedOccupancyGrid3D::chunkIndex({20, 3, 4})};
  const msg::RawObstacleDelta3D foreign_delta = makeRawObstacleDelta3D(
      foreign_grid, headerAt(11'900'000'000LL), 84U, 1U, 2U, dirty);
  EXPECT_EQ(applyDelta(accumulator, foreign_delta, epoch, 11'910'000'000LL).status,
            RawObstacleGridUpdateStatus3D::kBaseUnavailable);
  EXPECT_EQ(applyDelta(accumulator, foreign_delta, epoch, 11'950'000'000LL).status,
            RawObstacleGridUpdateStatus3D::kBaseUnavailable);

  const msg::RawObstacleSnapshot3D changed_kind =
      makeRawObstacleSnapshot3D(foreign_grid, headerAt(11'900'000'000LL), 84U, 2U);
  EXPECT_NE(rawObstacleDelta3DWireFingerprint(foreign_delta),
            rawObstacleSnapshot3DWireFingerprint(changed_kind));
  EXPECT_EQ(applySnapshot(accumulator, changed_kind, epoch, 11'960'000'000LL).status,
            RawObstacleGridUpdateStatus3D::kIdentityConflict);

  const msg::RawObstacleSnapshot3D newer =
      makeRawObstacleSnapshot3D(foreign_grid, headerAt(12'000'000'000LL), 84U, 3U);
  constexpr std::int64_t kFirstRawReceipt{12'010'000'000LL};
  EXPECT_EQ(applySnapshot(accumulator, newer, epoch, kFirstRawReceipt).status,
            RawObstacleGridUpdateStatus3D::kProducerEpochPending);

  const ProducerEpochAdmissionResult pending =
      admitProducerEpoch(kAdmissionConfig, epoch,
                         ProducerEpochObservation{.producer_instance_id = 84U,
                                                  .sequence = 1U,
                                                  .source_stamp_ns = 12'020'000'000LL,
                                                  .receive_stamp_ns = 12'030'000'000LL,
                                                  .content_fingerprint = 201U},
                         12'040'000'000LL);
  const ProducerEpochAdmissionResult handoff =
      admitProducerEpoch(kAdmissionConfig, pending.next_state,
                         ProducerEpochObservation{.producer_instance_id = 84U,
                                                  .sequence = 2U,
                                                  .source_stamp_ns = 12'100'000'000LL,
                                                  .receive_stamp_ns = 12'110'000'000LL,
                                                  .content_fingerprint = 202U},
                         12'120'000'000LL);
  ASSERT_EQ(handoff.status, ProducerEpochAdmissionStatus::kAcceptedProducerHandoff);
  epoch = handoff.next_state;

  const RawObstacleGridUpdate3D activated =
      accumulator.synchronizeProducerEpoch(epoch, 12'130'000'000LL, kAdmissionConfig);
  ASSERT_TRUE(activated.accepted());
  EXPECT_EQ(activated.evidence_observation.producer_instance_id, 84U);
  EXPECT_EQ(activated.evidence_observation.sequence, 3U);
  EXPECT_EQ(activated.evidence_observation.receive_stamp_ns, kFirstRawReceipt);
}

TEST(RawObstacle3DRos, ProspectiveHigherDeltaBlocksLowerFullUntilStrictlyNewerFull) {
  ObservedOccupancyGrid3D initial_grid{kBounds};
  static_cast<void>(initial_grid.setState({2, 3, 4}, ObservedVoxelState::kFree));
  RawObstacleDeltaAccumulator3D accumulator;
  ProducerEpochAdmissionState epoch = authorizedEpoch(42U);
  ASSERT_TRUE(applySnapshot(accumulator,
                            makeRawObstacleSnapshot3D(
                                initial_grid, headerAt(1'000'000'000LL), 42U, 1U),
                            epoch, 1'010'000'000LL)
                  .accepted());

  ObservedOccupancyGrid3D foreign_grid{kBounds};
  static_cast<void>(foreign_grid.setState({20, 3, 4}, ObservedVoxelState::kOccupied));
  const std::array dirty{ObservedOccupancyGrid3D::chunkIndex({20, 3, 4})};
  const msg::RawObstacleDelta3D higher_delta = makeRawObstacleDelta3D(
      foreign_grid, headerAt(11'900'000'000LL), 84U, 1U, 3U, dirty);
  EXPECT_EQ(applyDelta(accumulator, higher_delta, epoch, 11'910'000'000LL).status,
            RawObstacleGridUpdateStatus3D::kBaseUnavailable);
  const msg::RawObstacleSnapshot3D lower_full =
      makeRawObstacleSnapshot3D(foreign_grid, headerAt(11'950'000'000LL), 84U, 2U);
  EXPECT_EQ(applySnapshot(accumulator, lower_full, epoch, 11'960'000'000LL).status,
            RawObstacleGridUpdateStatus3D::kStale);

  const ProducerEpochAdmissionResult pending =
      admitProducerEpoch(kAdmissionConfig, epoch,
                         ProducerEpochObservation{.producer_instance_id = 84U,
                                                  .sequence = 1U,
                                                  .source_stamp_ns = 12'000'000'000LL,
                                                  .receive_stamp_ns = 12'010'000'000LL,
                                                  .content_fingerprint = 201U},
                         12'020'000'000LL);
  const ProducerEpochAdmissionResult handoff =
      admitProducerEpoch(kAdmissionConfig, pending.next_state,
                         ProducerEpochObservation{.producer_instance_id = 84U,
                                                  .sequence = 2U,
                                                  .source_stamp_ns = 12'100'000'000LL,
                                                  .receive_stamp_ns = 12'110'000'000LL,
                                                  .content_fingerprint = 202U},
                         12'120'000'000LL);
  ASSERT_EQ(handoff.status, ProducerEpochAdmissionStatus::kAcceptedProducerHandoff);
  epoch = handoff.next_state;

  const RawObstacleGridUpdate3D blocked =
      accumulator.synchronizeProducerEpoch(epoch, 12'130'000'000LL, kAdmissionConfig);
  EXPECT_FALSE(blocked.accepted());
  EXPECT_EQ(blocked.state.producer_instance_id, 42U);

  const msg::RawObstacleSnapshot3D recovered =
      makeRawObstacleSnapshot3D(foreign_grid, headerAt(12'200'000'000LL), 84U, 4U);
  const RawObstacleGridUpdate3D newer =
      applySnapshot(accumulator, recovered, epoch, 12'210'000'000LL);
  EXPECT_TRUE(newer.accepted());
  EXPECT_EQ(newer.state.producer_instance_id, 84U);
  EXPECT_EQ(newer.state.obstacle_snapshot_revision, 4U);
}

TEST(RawObstacle3DRos, CachedNewerFullRecoversAfterSameProducerStatusConflict) {
  ObservedOccupancyGrid3D initial_grid{kBounds};
  static_cast<void>(initial_grid.setState({2, 3, 4}, ObservedVoxelState::kFree));
  RawObstacleDeltaAccumulator3D accumulator;
  ProducerEpochAdmissionState epoch = authorizedEpoch(42U);
  ASSERT_TRUE(applySnapshot(accumulator,
                            makeRawObstacleSnapshot3D(
                                initial_grid, headerAt(1'000'000'000LL), 42U, 1U),
                            epoch, 1'010'000'000LL)
                  .accepted());
  const ProducerEpochAdmissionResult conflict =
      admitProducerEpoch(kAdmissionConfig, epoch,
                         ProducerEpochObservation{.producer_instance_id = 42U,
                                                  .sequence = 1U,
                                                  .source_stamp_ns = 900'000'000LL,
                                                  .receive_stamp_ns = 1'100'000'000LL,
                                                  .content_fingerprint = 999U},
                         1'110'000'000LL);
  ASSERT_EQ(conflict.status, ProducerEpochAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_FALSE(producerEpochAuthority(conflict.next_state).valid());

  ObservedOccupancyGrid3D recovered_grid{kBounds};
  static_cast<void>(recovered_grid.setState({20, 3, 4}, ObservedVoxelState::kOccupied));
  const msg::RawObstacleSnapshot3D cached =
      makeRawObstacleSnapshot3D(recovered_grid, headerAt(2'000'000'000LL), 42U, 2U);
  EXPECT_EQ(
      applySnapshot(accumulator, cached, conflict.next_state, 2'010'000'000LL).status,
      RawObstacleGridUpdateStatus3D::kProducerEpochPending);

  const ProducerEpochAdmissionResult status_recovered =
      admitProducerEpoch(kAdmissionConfig, conflict.next_state,
                         ProducerEpochObservation{.producer_instance_id = 42U,
                                                  .sequence = 2U,
                                                  .source_stamp_ns = 2'100'000'000LL,
                                                  .receive_stamp_ns = 2'110'000'000LL,
                                                  .content_fingerprint = 202U},
                         2'120'000'000LL);
  ASSERT_EQ(status_recovered.status, ProducerEpochAdmissionStatus::kAcceptedNewer);

  const RawObstacleGridUpdate3D activated = accumulator.synchronizeProducerEpoch(
      status_recovered.next_state, 2'130'000'000LL, kAdmissionConfig);
  ASSERT_TRUE(activated.accepted());
  EXPECT_EQ(activated.state.obstacle_snapshot_revision, 2U);
  EXPECT_TRUE(activated.state.occupancy->isOccupied({20, 3, 4}));
}

} // namespace
} // namespace drone_city_nav
