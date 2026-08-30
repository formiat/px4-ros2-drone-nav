#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <thread>
#include <vector>

#include "production_mppi_route_world.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {
namespace {

using namespace std::chrono_literals;

constexpr GridBounds3D kRawBounds{-4.0, -4.0, -2.0, 1.0, 8, 8, 4};
constexpr ProducerEpochAdmissionConfig kAdmissionConfig{
    .maximum_observation_age_ns = 100'000'000'000LL,
    .maximum_confirmation_interval_ns = 10'000'000'000LL,
};

[[nodiscard]] std_msgs::msg::Header headerAt(const std::int64_t stamp_ns) {
  std_msgs::msg::Header header;
  header.frame_id = "map";
  header.stamp.sec = static_cast<std::int32_t>(stamp_ns / 1'000'000'000LL);
  header.stamp.nanosec = static_cast<std::uint32_t>(stamp_ns % 1'000'000'000LL);
  return header;
}

[[nodiscard]] ProducerEpochObservation
statusObservation(const std::uint64_t producer, const std::uint64_t sequence,
                  const std::int64_t source_stamp_ns,
                  const std::int64_t receive_stamp_ns) noexcept {
  return ProducerEpochObservation{
      .producer_instance_id = producer,
      .sequence = sequence,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = receive_stamp_ns,
      .content_fingerprint = 10'000U + sequence,
  };
}

[[nodiscard]] RawWorldCommitResult3D
ingestAndCommit(WorldPipeline3D& pipeline, const ObservedOccupancyGrid3D& grid,
                const std::uint64_t producer, const std::uint64_t sequence) {
  const std::int64_t source_stamp_ns =
      static_cast<std::int64_t>(sequence) * 1'000'000'000LL;
  const std::int64_t status_receive_stamp_ns = source_stamp_ns + 10'000'000LL;
  const MemoryStatusIngestionResult3D status = pipeline.ingestMemoryStatus(
      statusObservation(producer, sequence, source_stamp_ns, status_receive_stamp_ns),
      true, status_receive_stamp_ns, kAdmissionConfig, true);
  EXPECT_FALSE(status.execution_revocation_required);
  const msg::RawObstacleSnapshot3D snapshot =
      makeRawObstacleSnapshot3D(grid, headerAt(source_stamp_ns), producer, sequence);
  const std::int64_t raw_receive_stamp_ns = source_stamp_ns + 20'000'000LL;
  const RawWorldIngestionResult3D ingestion = pipeline.ingestRawSnapshot(
      snapshot, raw_receive_stamp_ns, kAdmissionConfig, true);
  EXPECT_TRUE(ingestion.update.accepted());
  EXPECT_FALSE(ingestion.execution_revocation_required);
  RawWorldCommitResult3D result = pipeline.commitRawUpdate(
      ingestion.update, 1.5, source_stamp_ns + 30'000'000LL, kAdmissionConfig);
  if (result.committed()) {
    EXPECT_NE(result.world->execution_owner, nullptr);
    EXPECT_TRUE(result.world->execution_owner->valid());
    EXPECT_EQ(result.world->execution_owner->version().producer_instance_id,
              result.world->version.producer_instance_id);
    EXPECT_EQ(result.world->execution_owner->version().base_snapshot_revision,
              result.world->version.base_snapshot_revision);
    EXPECT_EQ(result.world->execution_owner->version().revision,
              result.world->version.revision);
    EXPECT_EQ(&result.world->execution_owner->occupancy(),
              result.world->occupancy.get());
  }
  return result;
}

[[nodiscard]] WorldSnapshot3D coherentObservedWorldWithoutGeneration() {
  WorldSnapshot3D world;
  const GridBounds3D bounds{0.0, 0.0, 0.0, 1.0, 4, 4, 4};
  world.producer_instance_id = 7U;
  world.source_raw_revision = 451U;
  world.source_occupied_fingerprint = 88U;
  world.grid = mppi::EsdfGrid{.width = 4,
                              .height = 4,
                              .resolution_m = 1.0F,
                              .depth = 4,
                              .outside_is_unknown = true};
  world.observed_occupancy = std::make_shared<const ObservedOccupancyGrid3D>(bounds);
  const KnownObstacleDistance3DBuildResult distance =
      buildKnownObstacleDistance3D(*world.observed_occupancy, bounds, 7.0);
  world.revision = distance.field->sourceFingerprint();
  world.distances_m = distance.field->materializeDense();
  const RawMapVersion raw_version{
      .producer_instance_id = 7U,
      .base_snapshot_revision = 400U,
      .revision = 451U,
  };
  world.observed_raw_world_owner = VersionedObservedRawWorld3D::captureOwned(
      raw_version, world.observed_occupancy, std::nullopt, std::nullopt);
  world.raw_occupied_fingerprint =
      world.observed_raw_world_owner->occupiedContentFingerprint();
  world.planner_full_reset = true;
  world.observed_esdf_resource = ObservedEsdfResource3D{
      .local_occupancy = std::make_shared<const ObservedOccupancyGrid3D>(bounds),
      .known_obstacle_distance = distance.field,
      .classification_override_cells =
          std::make_shared<const std::vector<GridIndex3D>>(),
      .coverage =
          ObservedEsdfCoverage3D{
              .source_raw_version = raw_version,
              .raw_local_fingerprint = 88U,
              .esdf_fingerprint = world.revision,
              .total_voxels = 64U,
              .recomputed_voxels = 64U,
              .maximum_distance_m = 7.0,
              .mode = ObservedEsdf3DBuildMode::kFull,
          },
  };
  return world;
}

TEST(WorldPipeline3DTest, LatestWinsWorkerOwnsOverloadAndDirtyLineage) {
  std::mutex mutex;
  std::condition_variable condition;
  bool first_entered{false};
  bool release_first{false};
  std::vector<std::uint64_t> processed;
  WorldPipeline3D pipeline{false,
                           [&](const ProductionMppiRawWorld3D& world) {
                             std::unique_lock lock{mutex};
                             processed.push_back(world.version.revision);
                             if (world.version.revision == 1U) {
                               first_entered = true;
                               condition.notify_all();
                               condition.wait(lock,
                                              [&]() noexcept { return release_first; });
                             }
                             condition.notify_all();
                             return std::optional<WorldPipeline3D::TimePoint>{};
                           },
                           {}};
  pipeline.start();
  ObservedOccupancyGrid3D grid{kRawBounds};
  static_cast<void>(grid.setState({1, 1, 1}, ObservedVoxelState::kOccupied));

  ASSERT_TRUE(ingestAndCommit(pipeline, grid, 42U, 1U).committed());
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(condition.wait_for(lock, 1s, [&]() noexcept { return first_entered; }));
  }
  static_cast<void>(grid.setState({2, 1, 1}, ObservedVoxelState::kOccupied));
  ASSERT_TRUE(ingestAndCommit(pipeline, grid, 42U, 2U).committed());
  static_cast<void>(grid.setState({3, 1, 1}, ObservedVoxelState::kOccupied));
  const RawWorldCommitResult3D third = ingestAndCommit(pipeline, grid, 42U, 3U);
  ASSERT_TRUE(third.committed());
  EXPECT_TRUE(third.replaced_pending);
  {
    const std::scoped_lock lock{mutex};
    release_first = true;
  }
  condition.notify_all();
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(condition.wait_for(lock, 1s,
                                   [&]() noexcept { return processed.size() == 2U; }));
  }
  pipeline.stop();

  EXPECT_EQ(processed, (std::vector<std::uint64_t>{1U, 3U}));
  const WorldPipelineStatistics3D statistics = pipeline.statistics();
  EXPECT_EQ(statistics.raw_updates, 3U);
  EXPECT_EQ(statistics.dropped_raw_worlds, 1U);
  ASSERT_NE(pipeline.latestRawWorld(), nullptr);
  EXPECT_EQ(pipeline.latestRawWorld()->version.revision, 3U);
}

TEST(WorldPipeline3DTest, IdentityConflictClearsAuthorityUntilNewerEvidence) {
  WorldPipeline3D pipeline{false,
                           [](const ProductionMppiRawWorld3D&) {
                             return std::optional<WorldPipeline3D::TimePoint>{};
                           },
                           {}};
  pipeline.start();
  ObservedOccupancyGrid3D grid{kRawBounds};
  static_cast<void>(grid.setState({1, 1, 1}, ObservedVoxelState::kFree));
  ASSERT_TRUE(ingestAndCommit(pipeline, grid, 42U, 1U).committed());

  static_cast<void>(grid.setState({1, 1, 1}, ObservedVoxelState::kOccupied));
  const msg::RawObstacleSnapshot3D conflict =
      makeRawObstacleSnapshot3D(grid, headerAt(1'000'000'000LL), 42U, 1U);
  const RawWorldIngestionResult3D conflicted =
      pipeline.ingestRawSnapshot(conflict, 1'040'000'000LL, kAdmissionConfig, true);
  EXPECT_EQ(conflicted.update.status, RawObstacleGridUpdateStatus3D::kIdentityConflict);
  EXPECT_TRUE(conflicted.execution_revocation_required);
  EXPECT_TRUE(pipeline.inputSnapshot().raw_world_identity_conflicted);
  EXPECT_EQ(pipeline.latestRawWorld(), nullptr);

  ASSERT_TRUE(ingestAndCommit(pipeline, grid, 42U, 2U).committed());
  EXPECT_FALSE(pipeline.inputSnapshot().raw_world_identity_conflicted);
  ASSERT_NE(pipeline.latestRawWorld(), nullptr);
  EXPECT_EQ(pipeline.latestRawWorld()->version.revision, 2U);
  pipeline.stop();
}

TEST(WorldPipeline3DTest, PublicationLeaseLinearizesGenerationAndReaders) {
  WorldPipeline3D pipeline{false,
                           [](const ProductionMppiRawWorld3D&) {
                             return std::optional<WorldPipeline3D::TimePoint>{};
                           },
                           {}};
  WorldSnapshot3D mutable_world = coherentObservedWorldWithoutGeneration();
  {
    WorldPipeline3D::PublicationLease publication = pipeline.lockPublication();
    const std::optional<LocalWorldGeneration> generation = publication.issueGeneration(
        mutable_world.observed_esdf_resource.coverage.source_raw_version, 21U,
        mutable_world.revision, mutable_world.revision);
    ASSERT_TRUE(generation.has_value());
    mutable_world.local_world_generation = generation.value_or(LocalWorldGeneration{});
    ASSERT_TRUE(publication.publish(
        std::make_shared<const WorldSnapshot3D>(std::move(mutable_world)),
        ProductionWorldBuildTelemetry3D{.build_ms = 4.0}));
  }

  const WorldPipelineResidentSnapshot3D captured = pipeline.residentSnapshot();
  ASSERT_NE(captured.world, nullptr);
  EXPECT_TRUE(productionWorldGenerationCoherent(*captured.world));
  EXPECT_DOUBLE_EQ(captured.telemetry.build_ms, 4.0);

  std::optional<WorldPipeline3D::ResidentLease> reader{std::in_place,
                                                       pipeline.lockResident()};
  std::mutex mutex;
  std::condition_variable condition;
  bool publication_attempted{false};
  bool publication_acquired{false};
  std::jthread publisher{[&]() {
    {
      const std::scoped_lock lock{mutex};
      publication_attempted = true;
    }
    condition.notify_all();
    WorldPipeline3D::PublicationLease publication = pipeline.lockPublication();
    {
      const std::scoped_lock lock{mutex};
      publication_acquired = true;
    }
    condition.notify_all();
  }};
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(
        condition.wait_for(lock, 1s, [&]() noexcept { return publication_attempted; }));
    EXPECT_FALSE(condition.wait_for(lock, 50ms,
                                    [&]() noexcept { return publication_acquired; }));
  }
  reader.reset();
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(
        condition.wait_for(lock, 1s, [&]() noexcept { return publication_acquired; }));
  }
}

TEST(WorldPipeline3DTest, TransientRefreshRequiresTheExactResidentPublication) {
  WorldPipeline3D pipeline{false,
                           [](const ProductionMppiRawWorld3D&) {
                             return std::optional<WorldPipeline3D::TimePoint>{};
                           },
                           {}};
  WorldSnapshot3D mutable_world = coherentObservedWorldWithoutGeneration();
  {
    WorldPipeline3D::PublicationLease publication = pipeline.lockPublication();
    const std::optional<LocalWorldGeneration> generation = publication.issueGeneration(
        mutable_world.observed_esdf_resource.coverage.source_raw_version, 21U,
        mutable_world.revision, mutable_world.revision);
    ASSERT_TRUE(generation.has_value());
    mutable_world.local_world_generation = generation.value_or(LocalWorldGeneration{});
    ASSERT_TRUE(publication.publish(
        std::make_shared<const WorldSnapshot3D>(std::move(mutable_world)), {}));
  }
  const std::shared_ptr<const WorldSnapshot3D> original =
      pipeline.residentSnapshot().world;
  ASSERT_NE(original, nullptr);
  ASSERT_TRUE(pipeline.refreshTransientEvidence(
      original, original->observed_raw_world_owner, std::nullopt));
  const std::shared_ptr<const WorldSnapshot3D> refreshed =
      pipeline.residentSnapshot().world;
  ASSERT_NE(refreshed, nullptr);
  EXPECT_NE(refreshed, original);
  EXPECT_TRUE(
      refreshed->local_world_generation.sameSnapshot(original->local_world_generation));
  EXPECT_FALSE(pipeline.refreshTransientEvidence(
      original, original->observed_raw_world_owner, std::nullopt));
}

TEST(WorldPipeline3DTest, InvalidGenerationPublicationFailsClosedAndIsCounted) {
  WorldPipeline3D pipeline{false,
                           [](const ProductionMppiRawWorld3D&) {
                             return std::optional<WorldPipeline3D::TimePoint>{};
                           },
                           {}};
  WorldSnapshot3D mutable_world = coherentObservedWorldWithoutGeneration();
  {
    WorldPipeline3D::PublicationLease publication = pipeline.lockPublication();
    const std::optional<LocalWorldGeneration> generation = publication.issueGeneration(
        mutable_world.observed_esdf_resource.coverage.source_raw_version, 21U,
        mutable_world.revision, mutable_world.revision);
    ASSERT_TRUE(generation.has_value());
    mutable_world.local_world_generation = generation.value_or(LocalWorldGeneration{});
    ++mutable_world.source_raw_revision;
    EXPECT_FALSE(publication.publish(
        std::make_shared<const WorldSnapshot3D>(std::move(mutable_world)), {}));
  }
  EXPECT_EQ(pipeline.residentSnapshot().world, nullptr);
  EXPECT_EQ(pipeline.statistics().rejected_world_publications, 1U);
}

TEST(WorldPipeline3DTest, StaticWorkerCoalescesOrdinaryWorkAndRetainsForcedRefresh) {
  std::mutex mutex;
  std::condition_variable condition;
  std::size_t invocations{0U};
  bool release_first{false};
  WorldPipeline3D pipeline{true, {}, [&]() {
                             std::unique_lock lock{mutex};
                             ++invocations;
                             condition.notify_all();
                             if (invocations == 1U) {
                               condition.wait(lock,
                                              [&]() noexcept { return release_first; });
                             }
                             condition.notify_all();
                           }};
  pipeline.start();

  ASSERT_TRUE(pipeline.requestStaticWork(false, false));
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(
        condition.wait_for(lock, 1s, [&]() noexcept { return invocations == 1U; }));
  }
  EXPECT_FALSE(pipeline.requestStaticWork(false, false));
  EXPECT_TRUE(pipeline.requestStaticWork(true, false));
  EXPECT_FALSE(pipeline.requestStaticWork(false, false));
  {
    const std::scoped_lock lock{mutex};
    release_first = true;
  }
  condition.notify_all();
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(
        condition.wait_for(lock, 1s, [&]() noexcept { return invocations == 2U; }));
  }
  EXPECT_FALSE(pipeline.requestStaticWork(false, true));
  pipeline.stop();
}

TEST(WorldPipeline3DTest, StopCannotDeadlockAProcessorReenteringLifecycleApi) {
  std::mutex mutex;
  std::condition_variable condition;
  bool processor_entered{false};
  bool release_processor{false};
  bool processor_completed{false};
  bool resubmission_accepted{true};
  WorldPipeline3D* pipeline_address{nullptr};
  WorldPipeline3D pipeline{
      false,
      [&](const ProductionMppiRawWorld3D&) {
        {
          std::unique_lock lock{mutex};
          processor_entered = true;
          condition.notify_all();
          condition.wait(lock, [&]() noexcept { return release_processor; });
        }
        const bool accepted = pipeline_address->scheduleLatestRawWorldUrgently();
        {
          const std::scoped_lock lock{mutex};
          resubmission_accepted = accepted;
          processor_completed = true;
        }
        condition.notify_all();
        return std::optional<WorldPipeline3D::TimePoint>{};
      },
      {}};
  pipeline_address = std::addressof(pipeline);
  pipeline.start();
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(ingestAndCommit(pipeline, grid, 42U, 1U).committed());
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(
        condition.wait_for(lock, 1s, [&]() noexcept { return processor_entered; }));
  }

  std::jthread stopper{[&pipeline]() { pipeline.stop(); }};
  const auto stop_deadline = std::chrono::steady_clock::now() + 1s;
  while (pipeline.accepting() && std::chrono::steady_clock::now() < stop_deadline) {
    std::this_thread::yield();
  }
  ASSERT_FALSE(pipeline.accepting());
  {
    const std::scoped_lock lock{mutex};
    release_processor = true;
  }
  condition.notify_all();
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(
        condition.wait_for(lock, 1s, [&]() noexcept { return processor_completed; }));
  }
  stopper.join();
  EXPECT_FALSE(resubmission_accepted);
  EXPECT_EQ(pipeline.statistics().rejected_after_stop, 1U);
}

TEST(WorldPipeline3DTest, ProcessingFailureIsContainedAndStopRejectsWork) {
  std::mutex mutex;
  std::condition_variable condition;
  bool processed{false};
  WorldPipeline3D pipeline{
      false,
      [&](const ProductionMppiRawWorld3D&)
          -> std::optional<WorldPipeline3D::TimePoint> {
        {
          const std::scoped_lock lock{mutex};
          processed = true;
        }
        condition.notify_all();
        throw std::runtime_error{"expected world processing failure"};
      },
      {},
      [](const std::exception_ptr&) {
        throw std::runtime_error{"expected world failure-handler failure"};
      }};
  pipeline.start();
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(ingestAndCommit(pipeline, grid, 42U, 1U).committed());
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(condition.wait_for(lock, 1s, [&]() noexcept { return processed; }));
  }
  pipeline.stop();
  const WorldPipelineStatistics3D statistics = pipeline.statistics();
  EXPECT_EQ(statistics.processing_failures, 1U);
  EXPECT_EQ(statistics.failure_handler_failures, 1U);

  const RawWorldCommitResult3D rejected = ingestAndCommit(pipeline, grid, 42U, 2U);
  EXPECT_EQ(rejected.status, RawWorldCommitStatus3D::kStopped);
  EXPECT_EQ(pipeline.statistics().rejected_after_stop, 1U);
}

} // namespace
} // namespace drone_city_nav
