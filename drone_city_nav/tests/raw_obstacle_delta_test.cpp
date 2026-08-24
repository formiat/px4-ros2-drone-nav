#include "drone_city_nav/raw_obstacle_delta.hpp"

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <utility>
#include <vector>

namespace drone_city_nav {
namespace {

constexpr ProducerEpochAdmissionConfig kAdmissionConfig{
    .maximum_observation_age_ns = 10'000'000'000LL,
    .maximum_confirmation_interval_ns = 2'000'000'000LL,
};

void setStamp(std_msgs::msg::Header& header, const std::int64_t stamp_ns) {
  header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
}

[[nodiscard]] ProducerEpochAdmissionState
authorizedEpoch(const std::uint64_t producer,
                const std::int64_t stamp_ns = 900'000'000LL) {
  const ProducerEpochAdmissionResult admission =
      admitProducerEpoch(kAdmissionConfig, {},
                         ProducerEpochObservation{
                             .producer_instance_id = producer,
                             .sequence = 1U,
                             .source_stamp_ns = stamp_ns,
                             .receive_stamp_ns = stamp_ns + 10'000'000LL,
                             .content_fingerprint = 101U,
                         },
                         stamp_ns + 20'000'000LL);
  EXPECT_TRUE(admission.install_observation);
  return admission.next_state;
}

[[nodiscard]] RawObstacleGridUpdate applySnapshot(
    RawObstacleDeltaAccumulator& accumulator, const msg::RawObstacleSnapshot& snapshot,
    const ProducerEpochAdmissionState& epoch, const std::int64_t receive_stamp_ns) {
  return accumulator.apply(snapshot, epoch, receive_stamp_ns,
                           receive_stamp_ns + 10'000'000LL, kAdmissionConfig);
}

[[nodiscard]] RawObstacleGridUpdate applyDelta(RawObstacleDeltaAccumulator& accumulator,
                                               const msg::RawObstacleDelta& delta,
                                               const ProducerEpochAdmissionState& epoch,
                                               const std::int64_t receive_stamp_ns) {
  return accumulator.apply(delta, epoch, receive_stamp_ns,
                           receive_stamp_ns + 10'000'000LL, kAdmissionConfig);
}

[[nodiscard]] msg::RawObstacleSnapshot makeSnapshot() {
  msg::RawObstacleSnapshot snapshot;
  snapshot.producer_instance_id = 7U;
  snapshot.obstacle_snapshot_revision = 10U;
  snapshot.risk_policy_fingerprint = 1234U;
  snapshot.risk_critical_distance_m = 1.0;
  snapshot.risk_preferred_distance_m = 6.0;
  snapshot.grid.header.frame_id = "map";
  setStamp(snapshot.grid.header, 1'000'000'000LL);
  snapshot.grid.info.resolution = 1.0F;
  snapshot.grid.info.width = 5U;
  snapshot.grid.info.height = 3U;
  snapshot.grid.info.origin.orientation.w = 1.0;
  snapshot.grid.data.assign(15U, static_cast<std::int8_t>(CellState::kUnknown));
  return snapshot;
}

[[nodiscard]] msg::RawObstacleDelta
requireDelta(std::optional<msg::RawObstacleDelta> delta) {
  if (!delta.has_value()) {
    ADD_FAILURE() << "expected a valid raw obstacle delta";
    return msg::RawObstacleDelta{};
  }
  return std::move(delta).value();
}

TEST(RawObstacleDelta, ReconstructsCumulativeAbsoluteChunks) {
  RawObstacleDeltaAccumulator accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(7U);
  const msg::RawObstacleSnapshot snapshot = makeSnapshot();
  const RawObstacleGridUpdate initial =
      applySnapshot(accumulator, snapshot, epoch, 1'010'000'000LL);
  ASSERT_TRUE(initial.accepted());
  ASSERT_TRUE(initial.state.occupancy);

  OccupancyGrid2D source = *initial.state.occupancy;
  source.setFree(GridIndex{1, 1});
  source.setOccupied(GridIndex{4, 2});
  const std::vector<std::size_t> changed_cells{6U, 14U};
  const std::vector<std::uint32_t> chunks =
      rawObstacleChunkIndices(source.bounds(), changed_cells, 2U);
  ASSERT_EQ(chunks, (std::vector<std::uint32_t>{0U, 5U}));

  std_msgs::msg::Header first_header = snapshot.grid.header;
  setStamp(first_header, 2'000'000'000LL);
  const msg::RawObstacleDelta first = requireDelta(makeRawObstacleDelta(
      source, first_header, 7U, 10U, 11U, 1234U, 1.0, 6.0, 2U, chunks));
  const RawObstacleGridUpdate first_update =
      applyDelta(accumulator, first, epoch, 2'010'000'000LL);
  ASSERT_TRUE(first_update.accepted());
  EXPECT_EQ(first_update.state.occupancy->state(GridIndex{1, 1}), CellState::kFree);
  EXPECT_EQ(first_update.state.occupancy->state(GridIndex{4, 2}), CellState::kOccupied);

  source.setOccupied(GridIndex{1, 1});
  const msg::RawObstacleDelta conflict = requireDelta(makeRawObstacleDelta(
      source, first_header, 7U, 10U, 11U, 1234U, 1.0, 6.0, 2U, chunks));
  const RawObstacleGridUpdate conflict_update =
      applyDelta(accumulator, conflict, epoch, 2'020'000'000LL);
  ASSERT_EQ(conflict_update.status, RawObstacleGridUpdateStatus::kIdentityConflict);
  EXPECT_TRUE(conflict_update.current_identity_conflict);
  EXPECT_EQ(applyDelta(accumulator, first, epoch, 2'030'000'000LL).status,
            RawObstacleGridUpdateStatus::kIdentityConflict);

  std_msgs::msg::Header second_header = snapshot.grid.header;
  setStamp(second_header, 3'000'000'000LL);
  const msg::RawObstacleDelta second = requireDelta(makeRawObstacleDelta(
      source, second_header, 7U, 10U, 12U, 1234U, 1.0, 6.0, 2U, chunks));
  const RawObstacleGridUpdate second_update =
      applyDelta(accumulator, second, epoch, 3'010'000'000LL);
  ASSERT_TRUE(second_update.accepted());
  EXPECT_EQ(second_update.state.obstacle_snapshot_revision, 12U);
  EXPECT_EQ(second_update.state.occupancy->state(GridIndex{1, 1}),
            CellState::kOccupied);
  EXPECT_EQ(second_update.state.occupancy->state(GridIndex{4, 2}),
            CellState::kOccupied);

  EXPECT_EQ(applyDelta(accumulator, first, epoch, 4'010'000'000LL).status,
            RawObstacleGridUpdateStatus::kStale);
}

TEST(RawObstacleDelta, RejectsMissingBaseAndInvalidGeometry) {
  RawObstacleDeltaAccumulator accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(7U);
  const msg::RawObstacleSnapshot snapshot = makeSnapshot();
  ASSERT_TRUE(applySnapshot(accumulator, snapshot, epoch, 1'010'000'000LL).accepted());
  OccupancyGrid2D source = *accumulator.state().occupancy;
  source.setOccupied(GridIndex{1, 1});
  const std::vector<std::uint32_t> chunks{0U};
  msg::RawObstacleDelta delta = requireDelta(makeRawObstacleDelta(
      source, snapshot.grid.header, 7U, 9U, 11U, 1234U, 1.0, 6.0, 2U, chunks));
  EXPECT_EQ(applyDelta(accumulator, delta, epoch, 2'010'000'000LL).status,
            RawObstacleGridUpdateStatus::kBaseUnavailable);

  delta = requireDelta(makeRawObstacleDelta(source, snapshot.grid.header, 7U, 10U, 11U,
                                            1234U, 1.0, 6.0, 2U, chunks));
  delta.obstacle_snapshot_revision = 12U;
  setStamp(delta.header, 2'000'000'000LL);
  delta.map_info.width = 6U;
  EXPECT_EQ(applyDelta(accumulator, delta, epoch, 2'010'000'000LL).status,
            RawObstacleGridUpdateStatus::kInvalidMessage);
}

TEST(RawObstacleDelta, RejectsUnsupportedRotatedOrElevatedMapGeometry) {
  const ProducerEpochAdmissionState epoch = authorizedEpoch(7U);

  msg::RawObstacleSnapshot rotated = makeSnapshot();
  rotated.grid.info.origin.orientation.z = 0.1;
  rotated.grid.info.origin.orientation.w = 0.99498743710662;
  RawObstacleDeltaAccumulator rotated_accumulator;
  EXPECT_EQ(applySnapshot(rotated_accumulator, rotated, epoch, 1'010'000'000LL).status,
            RawObstacleGridUpdateStatus::kInvalidMessage);
  EXPECT_FALSE(rotated_accumulator.state().occupancy);

  msg::RawObstacleSnapshot elevated = makeSnapshot();
  elevated.grid.info.origin.position.z = 1.0;
  RawObstacleDeltaAccumulator elevated_accumulator;
  EXPECT_EQ(
      applySnapshot(elevated_accumulator, elevated, epoch, 1'010'000'000LL).status,
      RawObstacleGridUpdateStatus::kInvalidMessage);
  EXPECT_FALSE(elevated_accumulator.state().occupancy);

  RawObstacleDeltaAccumulator accumulator;
  const msg::RawObstacleSnapshot snapshot = makeSnapshot();
  ASSERT_TRUE(applySnapshot(accumulator, snapshot, epoch, 1'010'000'000LL).accepted());
  OccupancyGrid2D source = *accumulator.state().occupancy;
  source.setOccupied(GridIndex{1, 1});
  const std::vector<std::uint32_t> chunks{0U};
  msg::RawObstacleDelta delta = requireDelta(makeRawObstacleDelta(
      source, snapshot.grid.header, 7U, 10U, 11U, 1234U, 1.0, 6.0, 2U, chunks));
  setStamp(delta.header, 2'000'000'000LL);
  delta.map_info.origin.orientation.z = 0.1;
  delta.map_info.origin.orientation.w = 0.99498743710662;
  EXPECT_EQ(applyDelta(accumulator, delta, epoch, 2'010'000'000LL).status,
            RawObstacleGridUpdateStatus::kInvalidMessage);
  EXPECT_EQ(accumulator.state().obstacle_snapshot_revision, 10U);
}

TEST(RawObstacleDelta, NewFullSnapshotChangesTheRequiredBase) {
  RawObstacleDeltaAccumulator accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(7U);
  msg::RawObstacleSnapshot first_snapshot = makeSnapshot();
  ASSERT_TRUE(
      applySnapshot(accumulator, first_snapshot, epoch, 1'010'000'000LL).accepted());
  OccupancyGrid2D source = *accumulator.state().occupancy;
  source.setOccupied(GridIndex{1, 1});
  const std::vector<std::uint32_t> chunks{0U};
  const msg::RawObstacleDelta old_delta = requireDelta(makeRawObstacleDelta(
      source, first_snapshot.grid.header, 7U, 10U, 11U, 1234U, 1.0, 6.0, 2U, chunks));

  msg::RawObstacleSnapshot resync = first_snapshot;
  resync.obstacle_snapshot_revision = 20U;
  setStamp(resync.grid.header, 2'000'000'000LL);
  ASSERT_TRUE(applySnapshot(accumulator, resync, epoch, 2'010'000'000LL).accepted());
  EXPECT_EQ(applyDelta(accumulator, old_delta, epoch, 3'010'000'000LL).status,
            RawObstacleGridUpdateStatus::kBaseUnavailable);
}

TEST(RawObstacleDelta, ExactReplayDoesNotRejuvenateRawEvidence) {
  RawObstacleDeltaAccumulator accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(7U);
  const msg::RawObstacleSnapshot snapshot = makeSnapshot();
  ASSERT_TRUE(applySnapshot(accumulator, snapshot, epoch, 1'010'000'000LL).accepted());

  const RawObstacleGridUpdate replay =
      applySnapshot(accumulator, snapshot, epoch, 2'010'000'000LL);

  EXPECT_EQ(replay.status, RawObstacleGridUpdateStatus::kIdempotentReplay);
  EXPECT_EQ(accumulator.evidenceAdmissionState().receive_stamp_ns, 1'010'000'000LL);
}

TEST(RawObstacleDelta,
     MalformedClaimCannotBeRewrittenAndStrictlyNewerSnapshotRecovers) {
  RawObstacleDeltaAccumulator accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(7U);
  const msg::RawObstacleSnapshot initial = makeSnapshot();
  ASSERT_TRUE(applySnapshot(accumulator, initial, epoch, 1'010'000'000LL).accepted());

  msg::RawObstacleSnapshot malformed = initial;
  malformed.obstacle_snapshot_revision = 11U;
  malformed.grid.info.width = 6U;
  setStamp(malformed.grid.header, 2'000'000'000LL);
  const RawObstacleGridUpdate rejected =
      applySnapshot(accumulator, malformed, epoch, 2'010'000'000LL);
  ASSERT_EQ(rejected.status, RawObstacleGridUpdateStatus::kInvalidMessage);
  ASSERT_EQ(accumulator.evidenceAdmissionState().rejected_identity_count, 1U);
  EXPECT_EQ(accumulator.evidenceAdmissionState()
                .rejected_identities[0]
                .first_receive_stamp_ns,
            2'010'000'000LL);

  const RawObstacleGridUpdate exact_replay =
      applySnapshot(accumulator, malformed, epoch, 2'500'000'000LL);
  EXPECT_EQ(exact_replay.status, RawObstacleGridUpdateStatus::kInvalidMessage);
  EXPECT_EQ(accumulator.evidenceAdmissionState()
                .rejected_identities[0]
                .first_receive_stamp_ns,
            2'010'000'000LL);

  msg::RawObstacleSnapshot rewritten = malformed;
  rewritten.grid.info.width = initial.grid.info.width;
  const RawObstacleGridUpdate conflict =
      applySnapshot(accumulator, rewritten, epoch, 2'510'000'000LL);
  ASSERT_EQ(conflict.status, RawObstacleGridUpdateStatus::kIdentityConflict);
  EXPECT_TRUE(conflict.current_identity_conflict);

  msg::RawObstacleSnapshot recovered = rewritten;
  recovered.obstacle_snapshot_revision = 12U;
  setStamp(recovered.grid.header, 3'000'000'000LL);
  const RawObstacleGridUpdate newer =
      applySnapshot(accumulator, recovered, epoch, 3'010'000'000LL);
  EXPECT_TRUE(newer.accepted());
  EXPECT_FALSE(accumulator.evidenceAdmissionState().current_identity_conflicted);
}

TEST(RawObstacleDelta, SameRevisionDifferentCanonicalWorldQuarantinesUntilNewerWorld) {
  RawObstacleDeltaAccumulator accumulator;
  const ProducerEpochAdmissionState epoch = authorizedEpoch(7U);
  const msg::RawObstacleSnapshot snapshot = makeSnapshot();
  ASSERT_TRUE(applySnapshot(accumulator, snapshot, epoch, 1'010'000'000LL).accepted());

  msg::RawObstacleSnapshot conflict = snapshot;
  conflict.grid.data[0] = static_cast<std::int8_t>(CellState::kOccupied);
  const RawObstacleGridUpdate conflicted =
      applySnapshot(accumulator, conflict, epoch, 1'020'000'000LL);
  ASSERT_EQ(conflicted.status, RawObstacleGridUpdateStatus::kIdentityConflict);
  EXPECT_TRUE(conflicted.current_identity_conflict);
  EXPECT_TRUE(accumulator.evidenceAdmissionState().current_identity_conflicted);

  const RawObstacleGridUpdate old_exact =
      applySnapshot(accumulator, snapshot, epoch, 1'030'000'000LL);
  EXPECT_EQ(old_exact.status, RawObstacleGridUpdateStatus::kIdentityConflict);
  EXPECT_FALSE(old_exact.current_identity_conflict);

  msg::RawObstacleSnapshot recovered = conflict;
  recovered.obstacle_snapshot_revision = 11U;
  setStamp(recovered.grid.header, 2'000'000'000LL);
  const RawObstacleGridUpdate newer =
      applySnapshot(accumulator, recovered, epoch, 2'010'000'000LL);
  ASSERT_TRUE(newer.accepted());
  EXPECT_FALSE(accumulator.evidenceAdmissionState().current_identity_conflicted);
  EXPECT_EQ(newer.state.occupancy->state(GridIndex{0, 0}), CellState::kOccupied);
}

TEST(RawObstacleDelta,
     CachedFullSnapshotNeedsConfirmedStatusAuthorityBeforeInstallation) {
  RawObstacleDeltaAccumulator accumulator;
  ProducerEpochAdmissionState epoch = authorizedEpoch(7U);
  const msg::RawObstacleSnapshot first = makeSnapshot();
  ASSERT_TRUE(applySnapshot(accumulator, first, epoch, 1'010'000'000LL).accepted());

  msg::RawObstacleSnapshot next = first;
  next.producer_instance_id = 8U;
  next.obstacle_snapshot_revision = 1U;
  next.grid.data[0] = static_cast<std::int8_t>(CellState::kOccupied);
  setStamp(next.grid.header, 11'900'000'000LL);
  EXPECT_EQ(applySnapshot(accumulator, next, epoch, 11'910'000'000LL).status,
            RawObstacleGridUpdateStatus::kProducerEpochPending);
  EXPECT_EQ(accumulator.state().producer_instance_id, 7U);

  const ProducerEpochAdmissionResult pending =
      admitProducerEpoch(kAdmissionConfig, epoch,
                         ProducerEpochObservation{.producer_instance_id = 8U,
                                                  .sequence = 1U,
                                                  .source_stamp_ns = 12'000'000'000LL,
                                                  .receive_stamp_ns = 12'010'000'000LL,
                                                  .content_fingerprint = 201U},
                         12'020'000'000LL);
  ASSERT_EQ(pending.status, ProducerEpochAdmissionStatus::kPendingProducerHandoff);
  const ProducerEpochAdmissionResult handoff =
      admitProducerEpoch(kAdmissionConfig, pending.next_state,
                         ProducerEpochObservation{.producer_instance_id = 8U,
                                                  .sequence = 2U,
                                                  .source_stamp_ns = 12'100'000'000LL,
                                                  .receive_stamp_ns = 12'110'000'000LL,
                                                  .content_fingerprint = 202U},
                         12'120'000'000LL);
  ASSERT_EQ(handoff.status, ProducerEpochAdmissionStatus::kAcceptedProducerHandoff);
  epoch = handoff.next_state;

  const RawObstacleGridUpdate activated =
      accumulator.synchronizeProducerEpoch(epoch, 12'130'000'000LL, kAdmissionConfig);
  ASSERT_TRUE(activated.accepted());
  EXPECT_TRUE(activated.authority_handoff);
  EXPECT_EQ(activated.state.producer_instance_id, 8U);
  EXPECT_EQ(activated.state.occupancy->state(GridIndex{0, 0}), CellState::kOccupied);

  msg::RawObstacleSnapshot retired = first;
  retired.obstacle_snapshot_revision = 99U;
  setStamp(retired.grid.header, 13'000'000'000LL);
  EXPECT_EQ(applySnapshot(accumulator, retired, epoch, 13'010'000'000LL).status,
            RawObstacleGridUpdateStatus::kStale);
  EXPECT_EQ(accumulator.state().producer_instance_id, 8U);
}

TEST(RawObstacleDelta, ProspectiveDeltaCannotResurrectAsSameIdentityFullSnapshot) {
  RawObstacleDeltaAccumulator accumulator;
  ProducerEpochAdmissionState epoch = authorizedEpoch(7U);
  const msg::RawObstacleSnapshot initial = makeSnapshot();
  ASSERT_TRUE(applySnapshot(accumulator, initial, epoch, 1'010'000'000LL).accepted());

  OccupancyGrid2D foreign_world = *accumulator.state().occupancy;
  foreign_world.setOccupied(GridIndex{1, 1});
  std_msgs::msg::Header foreign_header = initial.grid.header;
  setStamp(foreign_header, 11'900'000'000LL);
  const std::vector<std::uint32_t> chunks{0U};
  const msg::RawObstacleDelta foreign_delta = requireDelta(makeRawObstacleDelta(
      foreign_world, foreign_header, 8U, 1U, 2U, 1234U, 1.0, 6.0, 2U, chunks));
  EXPECT_EQ(applyDelta(accumulator, foreign_delta, epoch, 11'910'000'000LL).status,
            RawObstacleGridUpdateStatus::kBaseUnavailable);
  EXPECT_EQ(applyDelta(accumulator, foreign_delta, epoch, 11'950'000'000LL).status,
            RawObstacleGridUpdateStatus::kBaseUnavailable);

  msg::RawObstacleSnapshot changed_kind = initial;
  changed_kind.producer_instance_id = 8U;
  changed_kind.obstacle_snapshot_revision = 2U;
  changed_kind.grid.header = foreign_header;
  EXPECT_NE(rawObstacleDeltaWireFingerprint(foreign_delta),
            rawObstacleSnapshotWireFingerprint(changed_kind));
  EXPECT_EQ(applySnapshot(accumulator, changed_kind, epoch, 11'960'000'000LL).status,
            RawObstacleGridUpdateStatus::kIdentityConflict);

  msg::RawObstacleSnapshot newer = changed_kind;
  newer.obstacle_snapshot_revision = 3U;
  setStamp(newer.grid.header, 12'000'000'000LL);
  constexpr std::int64_t kFirstRawReceipt{12'010'000'000LL};
  EXPECT_EQ(applySnapshot(accumulator, newer, epoch, kFirstRawReceipt).status,
            RawObstacleGridUpdateStatus::kProducerEpochPending);

  const ProducerEpochAdmissionResult pending =
      admitProducerEpoch(kAdmissionConfig, epoch,
                         ProducerEpochObservation{.producer_instance_id = 8U,
                                                  .sequence = 2U,
                                                  .source_stamp_ns = 12'020'000'000LL,
                                                  .receive_stamp_ns = 12'030'000'000LL,
                                                  .content_fingerprint = 201U},
                         12'040'000'000LL);
  const ProducerEpochAdmissionResult handoff =
      admitProducerEpoch(kAdmissionConfig, pending.next_state,
                         ProducerEpochObservation{.producer_instance_id = 8U,
                                                  .sequence = 3U,
                                                  .source_stamp_ns = 12'100'000'000LL,
                                                  .receive_stamp_ns = 12'110'000'000LL,
                                                  .content_fingerprint = 202U},
                         12'120'000'000LL);
  ASSERT_EQ(handoff.status, ProducerEpochAdmissionStatus::kAcceptedProducerHandoff);
  epoch = handoff.next_state;

  const RawObstacleGridUpdate activated =
      accumulator.synchronizeProducerEpoch(epoch, 12'130'000'000LL, kAdmissionConfig);
  ASSERT_TRUE(activated.accepted());
  EXPECT_EQ(activated.evidence_observation.producer_instance_id, 8U);
  EXPECT_EQ(activated.evidence_observation.sequence, 3U);
  EXPECT_EQ(activated.evidence_observation.receive_stamp_ns, kFirstRawReceipt);
}

TEST(RawObstacleDelta, ProspectiveHigherDeltaBlocksLowerFullUntilStrictlyNewerFull) {
  RawObstacleDeltaAccumulator accumulator;
  ProducerEpochAdmissionState epoch = authorizedEpoch(7U);
  const msg::RawObstacleSnapshot initial = makeSnapshot();
  ASSERT_TRUE(applySnapshot(accumulator, initial, epoch, 1'010'000'000LL).accepted());

  OccupancyGrid2D foreign_world = *accumulator.state().occupancy;
  foreign_world.setOccupied(GridIndex{1, 1});
  const std::vector<std::uint32_t> chunks{0U};
  std_msgs::msg::Header higher_header = initial.grid.header;
  setStamp(higher_header, 11'900'000'000LL);
  const msg::RawObstacleDelta higher_delta = requireDelta(makeRawObstacleDelta(
      foreign_world, higher_header, 8U, 1U, 3U, 1234U, 1.0, 6.0, 2U, chunks));
  EXPECT_EQ(applyDelta(accumulator, higher_delta, epoch, 11'910'000'000LL).status,
            RawObstacleGridUpdateStatus::kBaseUnavailable);

  msg::RawObstacleSnapshot lower_full = initial;
  lower_full.producer_instance_id = 8U;
  lower_full.obstacle_snapshot_revision = 2U;
  setStamp(lower_full.grid.header, 11'950'000'000LL);
  EXPECT_EQ(applySnapshot(accumulator, lower_full, epoch, 11'960'000'000LL).status,
            RawObstacleGridUpdateStatus::kStale);

  const ProducerEpochAdmissionResult pending =
      admitProducerEpoch(kAdmissionConfig, epoch,
                         ProducerEpochObservation{.producer_instance_id = 8U,
                                                  .sequence = 1U,
                                                  .source_stamp_ns = 12'000'000'000LL,
                                                  .receive_stamp_ns = 12'010'000'000LL,
                                                  .content_fingerprint = 201U},
                         12'020'000'000LL);
  const ProducerEpochAdmissionResult handoff =
      admitProducerEpoch(kAdmissionConfig, pending.next_state,
                         ProducerEpochObservation{.producer_instance_id = 8U,
                                                  .sequence = 2U,
                                                  .source_stamp_ns = 12'100'000'000LL,
                                                  .receive_stamp_ns = 12'110'000'000LL,
                                                  .content_fingerprint = 202U},
                         12'120'000'000LL);
  ASSERT_EQ(handoff.status, ProducerEpochAdmissionStatus::kAcceptedProducerHandoff);
  epoch = handoff.next_state;

  const RawObstacleGridUpdate blocked =
      accumulator.synchronizeProducerEpoch(epoch, 12'130'000'000LL, kAdmissionConfig);
  EXPECT_FALSE(blocked.accepted());
  EXPECT_EQ(blocked.state.producer_instance_id, 7U);

  msg::RawObstacleSnapshot recovered = lower_full;
  recovered.obstacle_snapshot_revision = 4U;
  setStamp(recovered.grid.header, 12'200'000'000LL);
  const RawObstacleGridUpdate newer =
      applySnapshot(accumulator, recovered, epoch, 12'210'000'000LL);
  EXPECT_TRUE(newer.accepted());
  EXPECT_EQ(newer.state.producer_instance_id, 8U);
  EXPECT_EQ(newer.state.obstacle_snapshot_revision, 4U);
}

TEST(RawObstacleDelta, CachedNewerFullRecoversAfterSameProducerStatusConflict) {
  RawObstacleDeltaAccumulator accumulator;
  ProducerEpochAdmissionState epoch = authorizedEpoch(7U);
  const msg::RawObstacleSnapshot initial = makeSnapshot();
  ASSERT_TRUE(applySnapshot(accumulator, initial, epoch, 1'010'000'000LL).accepted());
  const ProducerEpochAdmissionResult conflict =
      admitProducerEpoch(kAdmissionConfig, epoch,
                         ProducerEpochObservation{.producer_instance_id = 7U,
                                                  .sequence = 1U,
                                                  .source_stamp_ns = 900'000'000LL,
                                                  .receive_stamp_ns = 1'100'000'000LL,
                                                  .content_fingerprint = 999U},
                         1'110'000'000LL);
  ASSERT_EQ(conflict.status, ProducerEpochAdmissionStatus::kRejectedIdentityConflict);
  EXPECT_FALSE(producerEpochAuthority(conflict.next_state).valid());

  msg::RawObstacleSnapshot cached = initial;
  cached.obstacle_snapshot_revision = 11U;
  cached.grid.data[0] = static_cast<std::int8_t>(CellState::kOccupied);
  setStamp(cached.grid.header, 2'000'000'000LL);
  EXPECT_EQ(
      applySnapshot(accumulator, cached, conflict.next_state, 2'010'000'000LL).status,
      RawObstacleGridUpdateStatus::kProducerEpochPending);

  const ProducerEpochAdmissionResult status_recovered =
      admitProducerEpoch(kAdmissionConfig, conflict.next_state,
                         ProducerEpochObservation{.producer_instance_id = 7U,
                                                  .sequence = 2U,
                                                  .source_stamp_ns = 2'100'000'000LL,
                                                  .receive_stamp_ns = 2'110'000'000LL,
                                                  .content_fingerprint = 202U},
                         2'120'000'000LL);
  ASSERT_EQ(status_recovered.status, ProducerEpochAdmissionStatus::kAcceptedNewer);

  const RawObstacleGridUpdate activated = accumulator.synchronizeProducerEpoch(
      status_recovered.next_state, 2'130'000'000LL, kAdmissionConfig);
  ASSERT_TRUE(activated.accepted());
  EXPECT_EQ(activated.state.obstacle_snapshot_revision, 11U);
  EXPECT_EQ(activated.state.occupancy->state(GridIndex{0, 0}), CellState::kOccupied);
}

} // namespace
} // namespace drone_city_nav
