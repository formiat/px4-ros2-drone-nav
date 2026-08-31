#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
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

using RawWorldProbe3D =
    std::function<void(std::shared_ptr<const ProductionMppiRawWorld3D>)>;

[[nodiscard]] ObservedWorldRuntime3D observedRuntime(RawWorldProbe3D probe = {}) {
  return ObservedWorldRuntime3D{
      .builder_config = ObservedWorldBuilderConfig3D{},
      .request_provider = [probe = std::move(probe)](
                              std::shared_ptr<const ProductionMppiRawWorld3D> raw_world)
          -> std::optional<ObservedWorldBuildRequest3D> {
        if (probe) {
          probe(std::move(raw_world));
        }
        return std::nullopt;
      },
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

[[nodiscard]] std::shared_ptr<const ProductionMppiRawWorld3D>
rawWorld(const ObservedOccupancyGrid3D& grid, const std::uint64_t revision,
         std::vector<OccupancyChunkIndex3D> dirty_chunks = {},
         const bool full_reset = false) {
  constexpr std::uint64_t kProducer{91U};
  constexpr std::uint64_t kBaseRevision{1U};
  const auto occupancy = std::make_shared<const ObservedOccupancyGrid3D>(grid);
  const RawMapVersion version{
      .producer_instance_id = kProducer,
      .base_snapshot_revision = kBaseRevision,
      .revision = revision,
  };
  return std::make_shared<const ProductionMppiRawWorld3D>(ProductionMppiRawWorld3D{
      .version = version,
      .source_stamp_ns = static_cast<std::int64_t>(revision) * 1'000'000'000LL,
      .receive_stamp_ns =
          static_cast<std::int64_t>(revision) * 1'000'000'000LL + 10'000'000LL,
      .ready_stamp_ns =
          static_cast<std::int64_t>(revision) * 1'000'000'000LL + 20'000'000LL,
      .reconstruction_ms = 1.25,
      .occupancy = occupancy,
      .execution_owner = VersionedObservedRawWorld3D::captureOwned(
          version, occupancy, std::nullopt, std::nullopt),
      .dirty_chunks = std::move(dirty_chunks),
      .full_reset = full_reset,
  });
}

[[nodiscard]] ObservedWorldBuildRequest3D observedRequest(
    std::shared_ptr<const ProductionMppiRawWorld3D> raw_world,
    const WorldPipeline3D::TimePoint build_started_at,
    std::optional<ProprioceptiveFreeSpaceSeed3D> free_space_seed = std::nullopt,
    const bool launch_support_resolution_pending = false) {
  return ObservedWorldBuildRequest3D{
      .raw_world = std::move(raw_world),
      .position = Point3{0.0, 0.0, 0.0},
      .pose_revision = 17U,
      .ready_stamp_ns = 9'000'000'000LL,
      .free_space_seed = free_space_seed,
      .launch_support_contact = std::nullopt,
      .launch_support_resolution_pending = launch_support_resolution_pending,
      .build_started_at = build_started_at,
  };
}

[[nodiscard]] std::shared_ptr<const OccupancyGrid3D> staticOccupancy() {
  OccupancyGrid3D occupancy{GridBounds3D{0.0, 0.0, 0.0, 1.0, 64, 32, 8}, 73U};
  occupancy.setOccupied(GridIndex3D{4, 4, 2});
  occupancy.setOccupied(GridIndex3D{45, 20, 3});
  return std::make_shared<const OccupancyGrid3D>(std::move(occupancy));
}

[[nodiscard]] std::shared_ptr<const FreeSpaceTopology3D>
staticTopology(const OccupancyGrid3D& occupancy) {
  const FreeSpaceRegionId region_id{"route-region"};
  const PassagePortalId entry_id{"route-entry"};
  const PassagePortalId exit_id{"route-exit"};
  const Point3 entry{2.0, 2.0, 2.0};
  const Point3 exit{18.0, 2.0, 2.0};
  const auto portal = [&](const PassagePortalId& id, const Point3& center,
                          const Vec3& normal) {
    return PassagePortal{
        .id = id,
        .region_id = region_id,
        .center = center,
        .outward_normal = normal,
        .opening_polygon = {{center.x, center.y - 1.0, center.z - 1.0},
                            {center.x, center.y + 1.0, center.z - 1.0},
                            {center.x, center.y, center.z + 1.0}},
        .surface_voxels = {},
        .traversable_anchors = {},
        .local_u_axis = {},
        .local_v_axis = {},
        .minimum_clearance_m = 0.0,
        .mean_clearance_m = 0.0,
        .maximum_clearance_m = 0.0,
    };
  };
  std::vector<RouteSample3D> centerline =
      sampleRoute3D(std::vector<Point3>{entry, exit}, 0.5, 3.0);
  std::vector<PassageTraversalEdge> traversals{
      PassageTraversalEdge{
          .id = PassageTraversalId{"route-traversal"},
          .region_id = region_id,
          .entry_portal_id = entry_id,
          .exit_portal_id = exit_id,
          .centerline = centerline,
          .entry = entry,
          .exit = exit,
          .min_z_m = 0.0,
          .max_z_m = 6.0,
          .width_m = 4.0,
          .height_m = 6.0,
          .minimum_clearance_m = 2.0,
          .speed_limit_mps = 3.0,
          .segment_spans = {},
      },
  };
  return std::make_shared<const FreeSpaceTopology3D>(
      occupancy.fingerprint(), occupancy.bounds(),
      std::vector<FreeSpaceRegion>{FreeSpaceRegion{
          .id = region_id,
          .representative = Point3{10.0, 2.0, 2.0},
          .maximum_clearance_m = 2.0,
          .portal_ids = {entry_id, exit_id},
      }},
      std::vector<PassagePortal>{portal(entry_id, entry, Vec3{-1.0, 0.0, 0.0}),
                                 portal(exit_id, exit, Vec3{1.0, 0.0, 0.0})},
      std::move(traversals));
}

[[nodiscard]] StaticWorldBuildRequest3D
staticRequest(const StaticWorldRefreshRequest3D refresh = {},
              const std::uint64_t resident_route_generation = 0U,
              const Point3 position = Point3{2.0, 2.0, 2.0},
              const Point3 goal = Point3{18.0, 2.0, 2.0}) noexcept {
  return StaticWorldBuildRequest3D{
      .refresh = refresh,
      .objective =
          StaticWorldObjective3D{
              .goal = goal,
              .mission_epoch = 3U,
              .sample_sequence = 5U,
              .assignment_generation = 7U,
              .available = true,
          },
      .position = position,
      .resident_route_generation = resident_route_generation,
      .source_stamp_ns = 4'000'000'000LL,
      .world_state_authoritative = true,
  };
}

[[nodiscard]] StaticWorldCommitContext3D
staticCommit(const std::uint64_t resident_route_generation = 0U) noexcept {
  return StaticWorldCommitContext3D{
      .position = Point3{2.25, 2.0, 2.0},
      .pose_revision = 19U,
      .resident_route_generation = resident_route_generation,
      .ready_stamp_ns = 4'100'000'000LL,
      .navigation_valid = true,
  };
}

using StaticRequestProvider3D =
    std::function<StaticWorldBuildRequest3D(const StaticWorldRefreshRequest3D&)>;
using StaticCommitProvider3D = std::function<StaticWorldCommitContext3D()>;
using StaticUploader3D =
    std::function<WorldEsdfUploadResult3D(const WorldEsdfUploadRequest3D&)>;

[[nodiscard]] StaticWorldRuntime3D
staticRuntime(const std::shared_ptr<const OccupancyGrid3D>& occupancy,
              StaticRequestProvider3D request_provider = {},
              StaticCommitProvider3D commit_provider = {},
              StaticUploader3D uploader = {},
              std::function<void(const StaticWorldUpdate3D&)> update_handler = {},
              std::shared_ptr<const FreeSpaceTopology3D> topology = nullptr) {
  if (!request_provider) {
    request_provider = [](const StaticWorldRefreshRequest3D& refresh) {
      return staticRequest(refresh, refresh.base_route_generation);
    };
  }
  if (!commit_provider) {
    commit_provider = []() { return staticCommit(); };
  }
  if (!uploader) {
    uploader = [](const WorldEsdfUploadRequest3D& request) {
      return WorldEsdfUploadResult3D{
          .accepted = true,
          .revision = request.revision,
      };
    };
  }
  if (topology == nullptr) {
    topology = std::make_shared<const FreeSpaceTopology3D>(
        occupancy->fingerprint(), occupancy->bounds(), std::vector<FreeSpaceRegion>{},
        std::vector<PassagePortal>{}, std::vector<PassageTraversalEdge>{});
  }
  return StaticWorldRuntime3D{
      .builder_config =
          StaticWorldBuilderConfig3D{
              .resources =
                  StaticWorldResources3D{
                      .occupancy = occupancy,
                      .topology = topology,
                      .esdf_cache = std::nullopt,
                  },
              .route_lookahead_m = 12.0,
              .roi_halo_m = 0.0,
              .maximum_distance_m = 8.0,
          },
      .request_provider = std::move(request_provider),
      .commit_context_provider = std::move(commit_provider),
      .uploader = std::move(uploader),
      .update_handler = std::move(update_handler),
  };
}

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

TEST(WorldPipeline3DTest, LatestWinsWorkerOwnsOverloadAndDirtyLineage) {
  std::mutex mutex;
  std::condition_variable condition;
  bool first_entered{false};
  bool release_first{false};
  std::vector<std::uint64_t> processed;
  WorldPipeline3D pipeline{observedRuntime(
      [&](const std::shared_ptr<const ProductionMppiRawWorld3D>& world) {
        std::unique_lock lock{mutex};
        processed.push_back(world->version.revision);
        if (world->version.revision == 1U) {
          first_entered = true;
          condition.notify_all();
          condition.wait(lock, [&]() noexcept { return release_first; });
        }
        condition.notify_all();
      })};
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
  WorldPipeline3D pipeline{observedRuntime()};
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

TEST(WorldPipeline3DTest, ObservedWorldServicePublishesExactOwnedFullArtifact) {
  std::size_t uploads{0U};
  ObservedWorldRuntime3D runtime = observedRuntime();
  runtime.uploader = [&](const WorldEsdfUploadRequest3D& request) {
    ++uploads;
    EXPECT_FALSE(request.distances_m.empty());
    EXPECT_NE(request.revision, 0U);
    return WorldEsdfUploadResult3D{
        .accepted = true,
        .upload_ms = 2.5,
        .revision = request.revision,
    };
  };
  WorldPipeline3D pipeline{std::move(runtime)};
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(grid.setState({3, 3, 1}, ObservedVoxelState::kOccupied));
  const std::shared_ptr<const ProductionMppiRawWorld3D> raw = rawWorld(grid, 1U);
  ASSERT_NE(raw->execution_owner, nullptr);

  const ObservedWorldUpdate3D update = pipeline.updateObservedWorld(
      observedRequest(raw, std::chrono::steady_clock::now()));

  ASSERT_TRUE(update.published());
  ASSERT_NE(update.world->observed_raw_world_owner, nullptr);
  EXPECT_EQ(uploads, 1U);
  EXPECT_EQ(update.stats.mode, ObservedEsdf3DBuildMode::kFull);
  EXPECT_DOUBLE_EQ(update.telemetry.upload_ms, 2.5);
  EXPECT_TRUE(productionWorldGenerationCoherent(*update.world));
  EXPECT_EQ(update.world, pipeline.residentSnapshot().world);
  EXPECT_EQ(update.world->observed_occupancy, raw->occupancy);
  EXPECT_EQ(&update.world->observed_raw_world_owner->occupancy(), raw->occupancy.get());
  EXPECT_TRUE(raw->execution_owner->sharesObservationOwner(
      *update.world->observed_raw_world_owner));
  EXPECT_EQ(update.world->observed_raw_world_owner->occupiedSnapshot(),
            raw->execution_owner->occupiedSnapshot());
  EXPECT_EQ(update.world->local_world_generation.raw_map.revision,
            raw->version.revision);
  EXPECT_EQ(pipeline.statistics().observed_full_builds, 1U);
}

TEST(WorldPipeline3DTest, ResidentLeaseLinearizesReadersAgainstPublication) {
  WorldPipeline3D pipeline{observedRuntime()};
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(grid.setState({3, 3, 1}, ObservedVoxelState::kOccupied));
  const auto raw = rawWorld(grid, 1U);
  const auto started_at = std::chrono::steady_clock::now();
  const ObservedWorldUpdate3D initial =
      pipeline.updateObservedWorld(observedRequest(raw, started_at));
  ASSERT_TRUE(initial.published());

  std::optional<WorldPipeline3D::ResidentLease> reader{std::in_place,
                                                       pipeline.lockResident()};
  std::mutex mutex;
  std::condition_variable condition;
  bool publication_attempted{false};
  bool publication_completed{false};
  ObservedWorldUpdate3D publication;
  std::jthread publisher{[&]() {
    {
      const std::scoped_lock lock{mutex};
      publication_attempted = true;
    }
    condition.notify_all();
    ObservedWorldUpdate3D update = pipeline.updateObservedWorld(
        observedRequest(raw, started_at + 100ms, std::nullopt, true));
    {
      const std::scoped_lock lock{mutex};
      publication = std::move(update);
      publication_completed = true;
    }
    condition.notify_all();
  }};
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(
        condition.wait_for(lock, 1s, [&]() noexcept { return publication_attempted; }));
    EXPECT_FALSE(condition.wait_for(lock, 50ms,
                                    [&]() noexcept { return publication_completed; }));
  }
  reader.reset();
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(
        condition.wait_for(lock, 1s, [&]() noexcept { return publication_completed; }));
  }
  ASSERT_TRUE(publication.published());
  EXPECT_GT(publication.world->local_world_generation.generation,
            initial.world->local_world_generation.generation);
}

TEST(WorldPipeline3DTest, MismatchedGpuRevisionFailsClosedThroughServiceApi) {
  ObservedWorldRuntime3D runtime = observedRuntime();
  runtime.uploader = [](const WorldEsdfUploadRequest3D& request) {
    return WorldEsdfUploadResult3D{
        .accepted = true,
        .revision = request.revision + 1U,
    };
  };
  WorldPipeline3D pipeline{std::move(runtime)};
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(grid.setState({3, 3, 1}, ObservedVoxelState::kOccupied));

  const ObservedWorldUpdate3D update = pipeline.updateObservedWorld(
      observedRequest(rawWorld(grid, 1U), std::chrono::steady_clock::now()));

  EXPECT_EQ(update.status, ObservedWorldUpdateStatus3D::kMixedLocalWorldGeneration);
  EXPECT_EQ(pipeline.residentSnapshot().world, nullptr);
  EXPECT_EQ(pipeline.statistics().rejected_world_publications, 1U);
}

TEST(WorldPipeline3DTest, ObservedWorldServiceRefreshesOnlyTransientEvidence) {
  std::size_t uploads{0U};
  std::size_t evidence_events{0U};
  ObservedWorldRuntime3D runtime = observedRuntime();
  runtime.uploader = [&](const WorldEsdfUploadRequest3D& request) {
    ++uploads;
    return WorldEsdfUploadResult3D{
        .accepted = true,
        .revision = request.revision,
    };
  };
  runtime.evidence_handler = [&](const ObservedWorldEvidenceChange3D& change) {
    ++evidence_events;
    EXPECT_TRUE(change.transient_changed);
    EXPECT_FALSE(change.persistent_changed);
  };
  WorldPipeline3D pipeline{std::move(runtime)};
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(grid.setState({3, 3, 1}, ObservedVoxelState::kOccupied));
  const std::shared_ptr<const ProductionMppiRawWorld3D> raw = rawWorld(grid, 1U);
  const auto started_at = std::chrono::steady_clock::now();
  const ObservedWorldUpdate3D initial =
      pipeline.updateObservedWorld(observedRequest(raw, started_at));
  ASSERT_TRUE(initial.published());
  const ProprioceptiveFreeSpaceSeed3D seed{
      .position = Point3{0.0, 0.0, 0.0},
      .body_axis = FootprintBodyAxis{0.0, 0.0, 1.0},
      .footprint = SweptFootprintConfig{},
  };

  const ObservedWorldUpdate3D refreshed =
      pipeline.updateObservedWorld(observedRequest(raw, started_at + 100ms, seed));

  EXPECT_EQ(refreshed.status, ObservedWorldUpdateStatus3D::kAlreadyCurrent);
  EXPECT_TRUE(refreshed.transient_evidence_refreshed);
  ASSERT_NE(refreshed.world, nullptr);
  EXPECT_EQ(uploads, 1U);
  EXPECT_EQ(evidence_events, 1U);
  EXPECT_NE(refreshed.world, initial.world);
  EXPECT_TRUE(refreshed.world->local_world_generation.sameSnapshot(
      initial.world->local_world_generation));
  if (!refreshed.world->proprioceptive_free_space_seed.has_value()) {
    FAIL() << "refreshed world did not retain the free-space seed";
    return;
  }
  const ProprioceptiveFreeSpaceSeed3D refreshed_seed =
      refreshed.world->proprioceptive_free_space_seed.value_or(
          ProprioceptiveFreeSpaceSeed3D{});
  EXPECT_TRUE(sameProprioceptiveFreeSpaceSeed3D(refreshed_seed, seed));
  EXPECT_EQ(pipeline.statistics().observedBuilds(), 1U);
}

TEST(WorldPipeline3DTest, EvidenceEventSupersedesExactParentBeforePublication) {
  std::size_t uploads{0U};
  bool parent_replaced{false};
  bool replacing_parent{false};
  WorldPipeline3D* pipeline_address{nullptr};
  std::shared_ptr<const ProductionMppiRawWorld3D> callback_raw;
  auto callback_started_at = std::chrono::steady_clock::now();
  ObservedWorldRuntime3D runtime = observedRuntime();
  runtime.uploader = [&](const WorldEsdfUploadRequest3D& request) {
    ++uploads;
    return WorldEsdfUploadResult3D{
        .accepted = true,
        .revision = request.revision,
    };
  };
  runtime.evidence_handler = [&](const ObservedWorldEvidenceChange3D& change) {
    if (replacing_parent || callback_raw == nullptr || !change.persistent_changed) {
      return;
    }
    replacing_parent = true;
    const ProprioceptiveFreeSpaceSeed3D seed{
        .position = Point3{0.0, 0.0, 0.0},
        .body_axis = FootprintBodyAxis{0.0, 0.0, 1.0},
        .footprint = SweptFootprintConfig{},
    };
    const ObservedWorldUpdate3D nested = pipeline_address->updateObservedWorld(
        observedRequest(callback_raw, callback_started_at + 100ms, seed));
    parent_replaced = nested.status == ObservedWorldUpdateStatus3D::kAlreadyCurrent &&
                      nested.transient_evidence_refreshed;
    replacing_parent = false;
  };
  WorldPipeline3D pipeline{std::move(runtime)};
  pipeline_address = std::addressof(pipeline);
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(grid.setState({3, 3, 1}, ObservedVoxelState::kOccupied));
  const auto raw = rawWorld(grid, 1U);
  const auto started_at = std::chrono::steady_clock::now();
  callback_started_at = started_at;
  const ObservedWorldUpdate3D initial =
      pipeline.updateObservedWorld(observedRequest(raw, started_at));
  ASSERT_TRUE(initial.published());
  callback_raw = raw;

  const ObservedWorldUpdate3D superseded = pipeline.updateObservedWorld(
      observedRequest(raw, started_at + 2s, std::nullopt, true));

  EXPECT_TRUE(parent_replaced);
  EXPECT_EQ(superseded.status, ObservedWorldUpdateStatus3D::kSupersededEsdfParent);
  EXPECT_TRUE(superseded.retry_not_before.has_value());
  EXPECT_EQ(superseded.world, nullptr);
  EXPECT_EQ(uploads, 1U);
  EXPECT_NE(pipeline.residentSnapshot().world, initial.world);
  EXPECT_EQ(pipeline.statistics().observedBuilds(), 1U);
}

TEST(WorldPipeline3DTest, PersistentEvidencePublishesAReuseGenerationWithoutUpload) {
  std::size_t uploads{0U};
  ObservedWorldRuntime3D runtime = observedRuntime();
  runtime.uploader = [&](const WorldEsdfUploadRequest3D& request) {
    ++uploads;
    return WorldEsdfUploadResult3D{
        .accepted = true,
        .revision = request.revision,
    };
  };
  WorldPipeline3D pipeline{std::move(runtime)};
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(grid.setState({3, 3, 1}, ObservedVoxelState::kOccupied));
  const auto raw = rawWorld(grid, 1U);
  const auto started_at = std::chrono::steady_clock::now();
  const ObservedWorldUpdate3D initial =
      pipeline.updateObservedWorld(observedRequest(raw, started_at));
  ASSERT_TRUE(initial.published());

  const ObservedWorldUpdate3D evidence_only = pipeline.updateObservedWorld(
      observedRequest(raw, started_at + 100ms, std::nullopt, true));

  ASSERT_TRUE(evidence_only.published());
  EXPECT_EQ(evidence_only.stats.mode, ObservedEsdf3DBuildMode::kReused);
  EXPECT_EQ(uploads, 1U);
  EXPECT_TRUE(evidence_only.world->launch_support_resolution_pending);
  EXPECT_TRUE(evidence_only.world->planner_full_reset);
  EXPECT_EQ(evidence_only.world->distances_m, initial.world->distances_m);
  EXPECT_EQ(evidence_only.world->observed_esdf_resource.known_obstacle_distance,
            initial.world->observed_esdf_resource.known_obstacle_distance);
  EXPECT_EQ(
      evidence_only.world->observed_esdf_resource.coverage.source_raw_version.revision,
      initial.world->observed_esdf_resource.coverage.source_raw_version.revision);
  EXPECT_GT(evidence_only.world->local_world_generation.generation,
            initial.world->local_world_generation.generation);
  EXPECT_TRUE(productionWorldGenerationCoherent(*evidence_only.world));
  EXPECT_EQ(pipeline.statistics().observedBuilds(), 1U);
  EXPECT_EQ(pipeline.statistics().observed_reused_builds, 1U);
}

TEST(WorldPipeline3DTest, ObservedWorldServiceRateLimitsThenPublishesIncrementalChild) {
  std::size_t uploads{0U};
  ObservedWorldRuntime3D runtime = observedRuntime();
  runtime.builder_config.incremental_maximum_rebuild_ratio = 1.0;
  runtime.uploader = [&](const WorldEsdfUploadRequest3D& request) {
    ++uploads;
    return WorldEsdfUploadResult3D{
        .accepted = true,
        .revision = request.revision,
    };
  };
  WorldPipeline3D pipeline{std::move(runtime)};
  const GridBounds3D incremental_bounds{-32.0, -32.0, -8.0, 1.0, 64, 64, 16};
  ObservedOccupancyGrid3D first_grid{incremental_bounds};
  ASSERT_TRUE(first_grid.setState({29, 29, 8}, ObservedVoxelState::kOccupied));
  const auto first_raw = rawWorld(first_grid, 1U);
  const auto started_at = std::chrono::steady_clock::now();
  const ObservedWorldUpdate3D first =
      pipeline.updateObservedWorld(observedRequest(first_raw, started_at));
  ASSERT_TRUE(first.published());

  ObservedOccupancyGrid3D second_grid = first_grid;
  const GridIndex3D inserted_cell{30, 29, 8};
  ASSERT_TRUE(second_grid.setState(inserted_cell, ObservedVoxelState::kOccupied));
  const auto second_raw =
      rawWorld(second_grid, 2U,
               std::vector<OccupancyChunkIndex3D>{
                   ObservedOccupancyGrid3D::chunkIndex(inserted_cell)});
  const ObservedWorldUpdate3D throttled =
      pipeline.updateObservedWorld(observedRequest(second_raw, started_at + 100ms));
  EXPECT_EQ(throttled.status, ObservedWorldUpdateStatus3D::kRateLimited);
  EXPECT_TRUE(throttled.retry_not_before.has_value());
  EXPECT_EQ(uploads, 1U);
  EXPECT_EQ(pipeline.statistics().throttled_observed_builds, 1U);
  EXPECT_EQ(pipeline.residentSnapshot().world, first.world);

  const ObservedWorldUpdate3D incremental =
      pipeline.updateObservedWorld(observedRequest(second_raw, started_at + 2s));
  ASSERT_TRUE(incremental.published());
  EXPECT_FALSE(incremental.stats.incremental_fallback);
  EXPECT_EQ(incremental.stats.mode, ObservedEsdf3DBuildMode::kIncremental);
  EXPECT_EQ(uploads, 2U);
  EXPECT_EQ(
      incremental.world->observed_esdf_resource.coverage.parent_raw_version.revision,
      first_raw->version.revision);
  EXPECT_EQ(incremental.world->planner_parent_raw_revision,
            first.world->source_raw_revision);
  EXPECT_GT(incremental.world->local_world_generation.generation,
            first.world->local_world_generation.generation);
  EXPECT_EQ(pipeline.statistics().observed_incremental_builds, 1U);
}

TEST(WorldPipeline3DTest, ObservedWorldServiceRejectsInvalidOwnerAndFailedUpload) {
  std::size_t uploads{0U};
  ObservedWorldRuntime3D runtime = observedRuntime();
  runtime.uploader = [&](const WorldEsdfUploadRequest3D&) {
    ++uploads;
    return WorldEsdfUploadResult3D{};
  };
  WorldPipeline3D pipeline{std::move(runtime)};
  ObservedOccupancyGrid3D grid{kRawBounds};
  ASSERT_TRUE(grid.setState({3, 3, 1}, ObservedVoxelState::kOccupied));
  const auto valid_raw = rawWorld(grid, 1U);
  auto invalid_raw = std::make_shared<ProductionMppiRawWorld3D>(*valid_raw);
  invalid_raw->execution_owner.reset();

  const ObservedWorldUpdate3D invalid = pipeline.updateObservedWorld(
      observedRequest(invalid_raw, std::chrono::steady_clock::now()));
  EXPECT_EQ(invalid.status, ObservedWorldUpdateStatus3D::kRawExecutionOwnerMismatch);
  EXPECT_EQ(uploads, 0U);
  EXPECT_EQ(pipeline.residentSnapshot().world, nullptr);

  const ObservedWorldUpdate3D upload_rejected = pipeline.updateObservedWorld(
      observedRequest(valid_raw, std::chrono::steady_clock::now()));
  EXPECT_EQ(upload_rejected.status, ObservedWorldUpdateStatus3D::kUploadRejected);
  EXPECT_EQ(uploads, 1U);
  EXPECT_EQ(pipeline.residentSnapshot().world, nullptr);
  EXPECT_EQ(pipeline.statistics().observedBuilds(), 0U);
}

TEST(WorldPipeline3DTest, ThrowingUploadInvalidatesPreviouslyResidentWorld) {
  std::size_t uploads{0U};
  ObservedWorldRuntime3D runtime = observedRuntime();
  runtime.builder_config.incremental_maximum_rebuild_ratio = 1.0;
  runtime.uploader = [&](const WorldEsdfUploadRequest3D& request) {
    ++uploads;
    if (uploads == 2U) {
      throw std::runtime_error{"upload may have changed GPU state"};
    }
    return WorldEsdfUploadResult3D{
        .accepted = true,
        .revision = request.revision,
    };
  };
  WorldPipeline3D pipeline{std::move(runtime)};
  const GridBounds3D bounds{-32.0, -32.0, -8.0, 1.0, 64, 64, 16};
  ObservedOccupancyGrid3D first_grid{bounds};
  ASSERT_TRUE(first_grid.setState({29, 29, 8}, ObservedVoxelState::kOccupied));
  const auto first_raw = rawWorld(first_grid, 1U);
  const auto started_at = std::chrono::steady_clock::now();
  ASSERT_TRUE(
      pipeline.updateObservedWorld(observedRequest(first_raw, started_at)).published());
  ASSERT_NE(pipeline.residentSnapshot().world, nullptr);

  ObservedOccupancyGrid3D second_grid = first_grid;
  const GridIndex3D inserted_cell{30, 29, 8};
  ASSERT_TRUE(second_grid.setState(inserted_cell, ObservedVoxelState::kOccupied));
  const auto second_raw =
      rawWorld(second_grid, 2U,
               std::vector<OccupancyChunkIndex3D>{
                   ObservedOccupancyGrid3D::chunkIndex(inserted_cell)});

  EXPECT_THROW(static_cast<void>(pipeline.updateObservedWorld(
                   observedRequest(second_raw, started_at + 2s))),
               std::runtime_error);
  EXPECT_EQ(uploads, 2U);
  EXPECT_EQ(pipeline.residentSnapshot().world, nullptr);
  EXPECT_EQ(pipeline.statistics().observedBuilds(), 1U);
  EXPECT_EQ(pipeline.statistics().rejected_world_publications, 1U);
}

TEST(WorldPipeline3DTest, StaticWorldServicePublishesImmutableOwnedArtifact) {
  const std::shared_ptr<const OccupancyGrid3D> occupancy = staticOccupancy();
  const std::shared_ptr<const FreeSpaceTopology3D> topology =
      staticTopology(*occupancy);
  std::size_t uploads{0U};
  WorldPipeline3D pipeline{staticRuntime(
      occupancy, {}, {},
      [&](const WorldEsdfUploadRequest3D& request) {
        ++uploads;
        EXPECT_FALSE(request.distances_m.empty());
        return WorldEsdfUploadResult3D{
            .accepted = true,
            .upload_ms = 1.75,
            .revision = request.revision,
        };
      },
      {}, topology)};

  const StaticWorldUpdate3D published = pipeline.updateStaticWorld(staticRequest());

  ASSERT_TRUE(published.published());
  ASSERT_NE(published.world->topology_passage_traversals, nullptr);
  ASSERT_EQ(published.world->topology_passage_traversals->size(), 1U);
  EXPECT_EQ(published.world->topology_passage_traversals->front().id,
            PassageTraversalId{"route-traversal"});
  EXPECT_EQ(published.world->static_occupancy, occupancy);
  EXPECT_EQ(pipeline.staticOccupancy(), occupancy);
  EXPECT_TRUE(productionWorldGenerationCoherent(*published.world));
  EXPECT_EQ(published.world, pipeline.residentSnapshot().world);
  EXPECT_DOUBLE_EQ(published.telemetry.upload_ms, 1.75);
  EXPECT_EQ(uploads, 1U);
  EXPECT_EQ(pipeline.statistics().static_builds, 1U);

  const StaticWorldUpdate3D already_current =
      pipeline.updateStaticWorld(staticRequest());
  EXPECT_EQ(already_current.status, StaticWorldUpdateStatus3D::kAlreadyCurrent);
  EXPECT_EQ(already_current.world, published.world);
  EXPECT_EQ(uploads, 1U);
}

TEST(WorldPipeline3DTest, StaticRefreshReusesResourcesAndPublishesNewGeneration) {
  const std::shared_ptr<const OccupancyGrid3D> occupancy = staticOccupancy();
  std::size_t uploads{0U};
  WorldPipeline3D pipeline{staticRuntime(
      occupancy, {}, []() { return staticCommit(11U); },
      [&](const WorldEsdfUploadRequest3D& request) {
        ++uploads;
        return WorldEsdfUploadResult3D{
            .accepted = true,
            .revision = request.revision,
        };
      })};
  const StaticWorldUpdate3D initial =
      pipeline.updateStaticWorld(staticRequest({}, 11U));
  ASSERT_TRUE(initial.published());
  const StaticWorldRefreshRequest3D refresh{
      .sequence = 1U,
      .base_route_generation = 11U,
      .purpose = StaticWorldRefreshPurpose3D::kRouteExtension,
  };

  const StaticWorldUpdate3D refreshed =
      pipeline.updateStaticWorld(staticRequest(refresh, 11U));

  ASSERT_TRUE(refreshed.published());
  EXPECT_TRUE(refreshed.proactive_refresh);
  EXPECT_TRUE(refreshed.diagnostics.cpu_resource_reused);
  EXPECT_TRUE(refreshed.diagnostics.gpu_resource_reused);
  EXPECT_EQ(refreshed.world->distances_m, initial.world->distances_m);
  EXPECT_GT(refreshed.world->local_world_generation.generation,
            initial.world->local_world_generation.generation);
  EXPECT_EQ(uploads, 1U);
  const WorldPipelineStatistics3D statistics = pipeline.statistics();
  EXPECT_EQ(statistics.static_builds, 1U);
  EXPECT_EQ(statistics.static_cpu_reuses, 1U);
  EXPECT_EQ(statistics.static_gpu_reuses, 1U);
}

TEST(WorldPipeline3DTest, StaticRefreshRejectsEarlyAndLateRouteSupersession) {
  const std::shared_ptr<const OccupancyGrid3D> occupancy = staticOccupancy();
  std::atomic<std::uint64_t> commit_route_generation{11U};
  std::size_t uploads{0U};
  WorldPipeline3D pipeline{staticRuntime(
      occupancy, {},
      [&]() {
        return staticCommit(commit_route_generation.load(std::memory_order_acquire));
      },
      [&](const WorldEsdfUploadRequest3D& request) {
        ++uploads;
        return WorldEsdfUploadResult3D{
            .accepted = true,
            .revision = request.revision,
        };
      })};
  const StaticWorldRefreshRequest3D refresh{
      .sequence = 1U,
      .base_route_generation = 11U,
      .purpose = StaticWorldRefreshPurpose3D::kTrackingObjective,
  };

  const StaticWorldUpdate3D early =
      pipeline.updateStaticWorld(staticRequest(refresh, 12U));
  EXPECT_EQ(early.status, StaticWorldUpdateStatus3D::kRefreshSuperseded);

  commit_route_generation.store(12U, std::memory_order_release);
  const StaticWorldUpdate3D late =
      pipeline.updateStaticWorld(staticRequest(refresh, 11U));
  EXPECT_EQ(late.status, StaticWorldUpdateStatus3D::kRefreshSuperseded);
  EXPECT_EQ(uploads, 0U);
  EXPECT_EQ(pipeline.residentSnapshot().world, nullptr);
}

TEST(WorldPipeline3DTest, StaticUploadExceptionAndRevisionMismatchFailClosed) {
  const std::shared_ptr<const OccupancyGrid3D> occupancy = staticOccupancy();
  WorldPipeline3D throwing_pipeline{
      staticRuntime(occupancy, {}, {},
                    [](const WorldEsdfUploadRequest3D&) -> WorldEsdfUploadResult3D {
                      throw std::runtime_error{"upload may have changed GPU state"};
                    })};
  const StaticWorldUpdate3D failed =
      throwing_pipeline.updateStaticWorld(staticRequest());
  EXPECT_EQ(failed.status, StaticWorldUpdateStatus3D::kUploadFailed);
  EXPECT_NE(failed.failure, nullptr);
  EXPECT_EQ(throwing_pipeline.residentSnapshot().world, nullptr);
  EXPECT_EQ(throwing_pipeline.statistics().rejected_world_publications, 1U);

  WorldPipeline3D mismatched_pipeline{
      staticRuntime(occupancy, {}, {}, [](const WorldEsdfUploadRequest3D& request) {
        return WorldEsdfUploadResult3D{
            .accepted = true,
            .revision = request.revision + 1U,
        };
      })};
  const StaticWorldUpdate3D mismatched =
      mismatched_pipeline.updateStaticWorld(staticRequest());
  EXPECT_EQ(mismatched.status, StaticWorldUpdateStatus3D::kMixedLocalWorldGeneration);
  EXPECT_EQ(mismatched_pipeline.residentSnapshot().world, nullptr);
  EXPECT_EQ(mismatched_pipeline.statistics().rejected_world_publications, 1U);
}

TEST(WorldPipeline3DTest, StaticWorkerCoalescesLatestRefreshDuringBuild) {
  std::mutex mutex;
  std::condition_variable condition;
  std::size_t invocations{0U};
  bool release_first{false};
  std::vector<StaticWorldUpdate3D> updates;
  const std::shared_ptr<const OccupancyGrid3D> occupancy = staticOccupancy();
  WorldPipeline3D pipeline{staticRuntime(
      occupancy,
      [&](const StaticWorldRefreshRequest3D& refresh) {
        {
          std::unique_lock lock{mutex};
          ++invocations;
          condition.notify_all();
          if (invocations == 1U) {
            condition.wait(lock, [&]() noexcept { return release_first; });
          }
        }
        return staticRequest(refresh, refresh.valid() ? 17U : 0U);
      },
      []() { return staticCommit(17U); }, {},
      [&](const StaticWorldUpdate3D& update) {
        const std::scoped_lock lock{mutex};
        updates.push_back(update);
        condition.notify_all();
      })};
  pipeline.start();

  ASSERT_TRUE(pipeline.requestStaticWork());
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(
        condition.wait_for(lock, 1s, [&]() noexcept { return invocations == 1U; }));
  }
  EXPECT_FALSE(pipeline.requestStaticWork());
  const StaticWorldRefreshRequest3D first_refresh =
      pipeline.requestStaticRefresh(17U, StaticWorldRefreshPurpose3D::kRouteExtension);
  const StaticWorldRefreshRequest3D latest_refresh = pipeline.requestStaticRefresh(
      17U, StaticWorldRefreshPurpose3D::kTrackingObjective);
  ASSERT_TRUE(first_refresh.valid());
  ASSERT_TRUE(latest_refresh.valid());
  EXPECT_GT(latest_refresh.sequence, first_refresh.sequence);
  EXPECT_FALSE(pipeline.requestStaticWork());
  {
    const std::scoped_lock lock{mutex};
    release_first = true;
  }
  condition.notify_all();
  {
    std::unique_lock lock{mutex};
    ASSERT_TRUE(condition.wait_for(lock, 2s, [&]() noexcept {
      return invocations == 2U && updates.size() == 2U;
    }));
    ASSERT_TRUE(updates.front().published());
    ASSERT_TRUE(updates.back().published());
    EXPECT_FALSE(updates.front().request.refresh.valid());
    EXPECT_EQ(updates.back().request.refresh.sequence, latest_refresh.sequence);
    EXPECT_EQ(updates.back().request.refresh.purpose,
              StaticWorldRefreshPurpose3D::kTrackingObjective);
  }
  EXPECT_FALSE(pipeline.requestStaticWork());
  pipeline.stop();
  EXPECT_EQ(pipeline.statistics().static_refreshes, 2U);
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
      observedRuntime([&](const std::shared_ptr<const ProductionMppiRawWorld3D>&) {
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
      })};
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
      observedRuntime([&](const std::shared_ptr<const ProductionMppiRawWorld3D>&) {
        {
          const std::scoped_lock lock{mutex};
          processed = true;
        }
        condition.notify_all();
        throw std::runtime_error{"expected world processing failure"};
      }),
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
