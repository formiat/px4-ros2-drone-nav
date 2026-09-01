#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <optional>

#include "raw_world_ingress_ros_3d.hpp"

namespace drone_city_nav {
namespace {

constexpr GridBounds3D kRawBounds{-4.0, -4.0, -2.0, 1.0, 8, 8, 4};
const RawWorldIngressRosConfig3D kIngressConfig{
    .producer_epoch =
        ProducerEpochAdmissionConfig{
            .maximum_observation_age_ns = 100'000'000'000LL,
            .maximum_confirmation_interval_ns = 10'000'000'000LL,
        },
    .frame_id = "map",
};

[[nodiscard]] ObservedWorldRuntime3D observedRuntime() {
  return ObservedWorldRuntime3D{
      .builder_config = ObservedWorldBuilderConfig3D{},
      .request_provider = [](std::shared_ptr<const ProductionMppiRawWorld3D>)
          -> std::optional<ObservedWorldBuildRequest3D> { return std::nullopt; },
      .uploader =
          [](const WorldEsdfUploadRequest3D& request) {
            return WorldEsdfUploadResult3D{
                .accepted = true,
                .revision = request.revision,
            };
          },
      .evidence_handler = {},
      .update_handler = {},
  };
}

[[nodiscard]] std_msgs::msg::Header headerAt(const std::int64_t stamp_ns,
                                             const char* frame_id = "map") {
  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  return header;
}

[[nodiscard]] msg::ObstacleMemoryStatus
memoryStatus(const ObservedOccupancyGrid3D& grid, const std::uint64_t producer,
             const std::uint64_t sequence, const std::int64_t source_stamp_ns,
             const char* frame_id = "map") {
  msg::ObstacleMemoryStatus status;
  status.header = headerAt(source_stamp_ns, frame_id);
  status.producer_instance_id = producer;
  status.sequence = sequence;
  status.occupied_cell_count = grid.occupiedVoxelCount();
  status.raw_snapshot_published = true;
  status.full_snapshot_published = true;
  return status;
}

[[nodiscard]] RawWorldIngestionResult3D
ingestSnapshot(RawWorldIngressRos3D& ingress, const ObservedOccupancyGrid3D& grid,
               const std::uint64_t producer, const std::uint64_t sequence,
               const char* frame_id = "map") {
  const std::int64_t source_stamp_ns =
      static_cast<std::int64_t>(sequence) * 1'000'000'000LL;
  const MemoryStatusIngestionResult3D status = ingress.ingestMemoryStatus(
      memoryStatus(grid, producer, sequence, source_stamp_ns, frame_id),
      source_stamp_ns + 10'000'000LL);
  EXPECT_FALSE(status.execution_revocation_required);
  return ingress.ingestRawSnapshot(
      makeRawObstacleSnapshot3D(grid, headerAt(source_stamp_ns, frame_id), producer,
                                sequence),
      source_stamp_ns + 20'000'000LL);
}

[[nodiscard]] RawWorldCommitResult3D
commitSnapshot(RawWorldIngressRos3D& ingress, const RawObstacleGridUpdate3D& update) {
  return ingress.commitRawUpdate(
      update, 1.5, update.evidence_observation.source_stamp_ns + 30'000'000LL);
}

[[nodiscard]] RawWorldCommitResult3D
ingestAndCommit(RawWorldIngressRos3D& ingress, const ObservedOccupancyGrid3D& grid,
                const std::uint64_t producer, const std::uint64_t sequence) {
  const RawWorldIngestionResult3D ingestion =
      ingestSnapshot(ingress, grid, producer, sequence);
  EXPECT_TRUE(ingestion.update.accepted());
  EXPECT_FALSE(ingestion.execution_revocation_required);
  return commitSnapshot(ingress, ingestion.update);
}

TEST(RawWorldIngressRos3DTest,
     PublishesOnlyEvidenceAdmittedForTheCurrentProducerEpoch) {
  WorldPipeline3D pipeline{observedRuntime()};
  RawWorldIngressRos3D ingress{pipeline, kIngressConfig};
  pipeline.start();
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(grid.setState({1, 1, 1}, ObservedVoxelState::kFree));

  const RawWorldCommitResult3D committed = ingestAndCommit(ingress, grid, 42U, 1U);

  ASSERT_TRUE(committed.committed());
  ASSERT_NE(committed.world->authoritativeOwner(), nullptr);
  EXPECT_TRUE(committed.world->valid());
  EXPECT_EQ(committed.world, pipeline.latestRawWorld());
  const RawWorldIngressSnapshot3D snapshot = ingress.snapshot();
  EXPECT_EQ(snapshot.latest_observation.producer_instance_id, 42U);
  EXPECT_EQ(snapshot.latest_observation.sequence, 1U);
  EXPECT_EQ(snapshot.latest_raw_world, committed.world);
  EXPECT_FALSE(snapshot.raw_world_identity_conflicted);
  pipeline.stop();
}

TEST(RawWorldIngressRos3DTest,
     SupersededReconstructionCannotPublishAfterNewerEvidenceArrives) {
  WorldPipeline3D pipeline{observedRuntime()};
  RawWorldIngressRos3D ingress{pipeline, kIngressConfig};
  pipeline.start();
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(grid.setState({1, 1, 1}, ObservedVoxelState::kFree));
  const RawWorldIngestionResult3D first = ingestSnapshot(ingress, grid, 42U, 1U);
  ASSERT_TRUE(first.update.accepted());

  ASSERT_TRUE(grid.setState({2, 1, 1}, ObservedVoxelState::kOccupied));
  const RawWorldIngestionResult3D second = ingestSnapshot(ingress, grid, 42U, 2U);
  ASSERT_TRUE(second.update.accepted());

  EXPECT_EQ(commitSnapshot(ingress, first.update).status,
            RawWorldCommitStatus3D::kSupersededEvidence);
  EXPECT_EQ(pipeline.latestRawWorld(), nullptr);
  const RawWorldCommitResult3D committed = commitSnapshot(ingress, second.update);
  ASSERT_TRUE(committed.committed());
  EXPECT_EQ(committed.world->version().revision, 2U);
  pipeline.stop();
}

TEST(RawWorldIngressRos3DTest,
     IdentityConflictRevokesPublishedAuthorityUntilNewerEvidence) {
  WorldPipeline3D pipeline{observedRuntime()};
  RawWorldIngressRos3D ingress{pipeline, kIngressConfig};
  pipeline.start();
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(grid.setState({1, 1, 1}, ObservedVoxelState::kFree));
  ASSERT_TRUE(ingestAndCommit(ingress, grid, 42U, 1U).committed());

  ASSERT_TRUE(grid.setState({1, 1, 1}, ObservedVoxelState::kOccupied));
  const msg::RawObstacleSnapshot3D conflict =
      makeRawObstacleSnapshot3D(grid, headerAt(1'000'000'000LL), 42U, 1U);
  const RawWorldIngestionResult3D conflicted =
      ingress.ingestRawSnapshot(conflict, 1'040'000'000LL);

  EXPECT_EQ(conflicted.update.status, RawObstacleGridUpdateStatus3D::kIdentityConflict);
  EXPECT_TRUE(conflicted.execution_revocation_required);
  EXPECT_TRUE(ingress.snapshot().raw_world_identity_conflicted);
  EXPECT_EQ(pipeline.latestRawWorld(), nullptr);

  ASSERT_TRUE(ingestAndCommit(ingress, grid, 42U, 2U).committed());
  EXPECT_FALSE(ingress.snapshot().raw_world_identity_conflicted);
  ASSERT_NE(pipeline.latestRawWorld(), nullptr);
  EXPECT_EQ(pipeline.latestRawWorld()->version().revision, 2U);
  pipeline.stop();
}

TEST(RawWorldIngressRos3DTest, RejectsRosEvidenceFromAnotherFrame) {
  WorldPipeline3D pipeline{observedRuntime()};
  RawWorldIngressRos3D ingress{pipeline, kIngressConfig};
  pipeline.start();
  ObservedOccupancyGrid3D grid{kRawBounds};
  const RawWorldIngestionResult3D rejected =
      ingestSnapshot(ingress, grid, 42U, 1U, "odom");

  EXPECT_FALSE(rejected.update.accepted());
  EXPECT_EQ(commitSnapshot(ingress, rejected.update).status,
            RawWorldCommitStatus3D::kInvalidUpdate);
  EXPECT_EQ(pipeline.latestRawWorld(), nullptr);
  pipeline.stop();
}

} // namespace
} // namespace drone_city_nav
