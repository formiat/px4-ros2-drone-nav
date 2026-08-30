#pragma once

#include "drone_city_nav/observed_esdf_3d.hpp"
#include "drone_city_nav/raw_obstacle_3d_ros.hpp"
#include "drone_city_nav/world_generation.hpp"
#include "drone_city_nav/world_snapshot_3d.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <exception>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string_view>
#include <thread>
#include <variant>

#include "observed_world_builder_3d.hpp"
#include "production_mppi_raw_world.hpp"
#include "production_route_pipeline_artifacts_3d.hpp"

namespace drone_city_nav {

enum class RawWorldCommitStatus3D : std::uint8_t {
  kCommitted,
  kStopped,
  kInvalidUpdate,
  kInvalidExecutionOwner,
  kSupersededEvidence,
};

[[nodiscard]] std::string_view
rawWorldCommitStatus3DName(RawWorldCommitStatus3D status) noexcept;

struct RawWorldIngestionResult3D {
  RawObstacleGridUpdate3D update{};
  bool execution_revocation_required{false};
};

struct MemoryStatusIngestionResult3D {
  std::optional<RawObstacleGridUpdate3D> synchronized_update;
  bool execution_revocation_required{false};
};

struct RawWorldCommitResult3D {
  RawWorldCommitStatus3D status{RawWorldCommitStatus3D::kInvalidUpdate};
  std::shared_ptr<const ProductionMppiRawWorld3D> world;
  bool replaced_pending{false};

  [[nodiscard]] bool committed() const noexcept {
    return status == RawWorldCommitStatus3D::kCommitted && world != nullptr;
  }
};

struct WorldPipelineInputSnapshot3D {
  LatestObservation latest_observation{};
  std::shared_ptr<const ProductionMppiRawWorld3D> latest_raw_world;
  bool raw_world_identity_conflicted{false};
};

struct WorldPipelineResidentSnapshot3D {
  std::shared_ptr<const WorldSnapshot3D> world;
  ProductionWorldBuildTelemetry3D telemetry{};
};

struct WorldEsdfUploadRequest3D {
  mppi::EsdfGrid grid{};
  std::span<const float> distances_m;
  std::uint64_t revision{0U};
  std::span<const ObservedEsdfDirtyRegion3D> dirty_regions;
};

struct WorldEsdfUploadResult3D {
  bool accepted{false};
  double upload_ms{0.0};
  std::uint64_t revision{0U};
};

struct ObservedWorldUpdate3D {
  ObservedWorldUpdateStatus3D status{
      ObservedWorldUpdateStatus3D::kUnavailableObservedGrid};
  std::shared_ptr<const ProductionMppiRawWorld3D> raw_world;
  std::shared_ptr<const WorldSnapshot3D> world;
  ProductionWorldBuildTelemetry3D telemetry{};
  ObservedEsdf3DBuildStats stats{};
  ObservedWorldEvidenceChange3D evidence_change{};
  std::optional<std::chrono::steady_clock::time_point> retry_not_before;
  double maximum_distance_m{0.0};
  bool periodic_full_audit{false};
  bool recentered{false};
  bool transient_evidence_refreshed{false};

  [[nodiscard]] bool published() const noexcept {
    return status == ObservedWorldUpdateStatus3D::kPublished && world != nullptr;
  }
};

struct ObservedWorldRuntime3D {
  ObservedWorldBuilderConfig3D builder_config{};
  std::function<std::optional<ObservedWorldBuildRequest3D>(
      std::shared_ptr<const ProductionMppiRawWorld3D>)>
      request_provider;
  std::function<WorldEsdfUploadResult3D(const WorldEsdfUploadRequest3D&)> uploader;
  std::function<void(const ObservedWorldEvidenceChange3D&)> evidence_handler;
  std::function<void(const ObservedWorldUpdate3D&)> update_handler;
};

struct StaticWorldRuntime3D {
  std::function<void()> processor;
};

struct WorldPipelineStatistics3D {
  std::uint64_t raw_updates{0U};
  std::uint64_t dropped_raw_worlds{0U};
  std::uint64_t throttled_observed_builds{0U};
  std::uint64_t observed_full_builds{0U};
  std::uint64_t observed_incremental_builds{0U};
  std::uint64_t observed_reused_builds{0U};
  std::uint64_t observed_recomputed_voxels{0U};
  std::uint64_t observed_reused_voxels{0U};
  std::uint64_t superseded_planning_generations{0U};
  std::uint64_t rejected_world_publications{0U};
  std::uint64_t processing_failures{0U};
  std::uint64_t failure_handler_failures{0U};
  std::uint64_t rejected_after_stop{0U};

  [[nodiscard]] std::uint64_t observedBuilds() const noexcept {
    return observed_full_builds + observed_incremental_builds;
  }
};

class WorldPipeline3D final {
public:
  using TimePoint = std::chrono::steady_clock::time_point;
  using ProcessingFailureHandler = std::function<void(const std::exception_ptr&)>;

  class ResidentLease final {
  public:
    ResidentLease(ResidentLease&&) noexcept = default;
    ResidentLease& operator=(ResidentLease&&) noexcept = default;
    ResidentLease(const ResidentLease&) = delete;
    ResidentLease& operator=(const ResidentLease&) = delete;

    [[nodiscard]] const std::shared_ptr<const WorldSnapshot3D>& world() const noexcept;
    [[nodiscard]] const ProductionWorldBuildTelemetry3D& telemetry() const noexcept;

  private:
    friend class WorldPipeline3D;
    explicit ResidentLease(const WorldPipeline3D& owner);

    const WorldPipeline3D* owner_;
    std::unique_lock<std::mutex> lock_;
  };

  class PublicationLease final {
  public:
    PublicationLease(PublicationLease&&) noexcept = default;
    PublicationLease& operator=(PublicationLease&&) noexcept = default;
    PublicationLease(const PublicationLease&) = delete;
    PublicationLease& operator=(const PublicationLease&) = delete;

    [[nodiscard]] const std::shared_ptr<const WorldSnapshot3D>& world() const noexcept;
    [[nodiscard]] const ProductionWorldBuildTelemetry3D& telemetry() const noexcept;
    [[nodiscard]] std::optional<LocalWorldGeneration>
    issueGeneration(const RawMapVersion& raw_map, std::uint64_t pose_revision,
                    std::uint64_t esdf_revision,
                    std::uint64_t gpu_esdf_revision) noexcept;
    [[nodiscard]] bool
    publish(std::shared_ptr<const WorldSnapshot3D> world,
            const ProductionWorldBuildTelemetry3D& telemetry) noexcept;
    void recordRejection() noexcept;
    void invalidateAndRecordRejection() noexcept;

  private:
    friend class WorldPipeline3D;
    explicit PublicationLease(WorldPipeline3D& owner);

    WorldPipeline3D* owner_;
    std::unique_lock<std::mutex> lock_;
  };

  explicit WorldPipeline3D(ObservedWorldRuntime3D runtime,
                           ProcessingFailureHandler failure_handler = {});
  explicit WorldPipeline3D(StaticWorldRuntime3D runtime,
                           ProcessingFailureHandler failure_handler = {});
  ~WorldPipeline3D();

  WorldPipeline3D(const WorldPipeline3D&) = delete;
  WorldPipeline3D& operator=(const WorldPipeline3D&) = delete;
  WorldPipeline3D(WorldPipeline3D&&) = delete;
  WorldPipeline3D& operator=(WorldPipeline3D&&) = delete;

  void start();
  void stop() noexcept;
  [[nodiscard]] bool accepting() const noexcept;

  [[nodiscard]] RawWorldIngestionResult3D
  ingestRawSnapshot(const msg::RawObstacleSnapshot3D& message,
                    std::int64_t receive_stamp_ns,
                    const ProducerEpochAdmissionConfig& config, bool frame_matches);
  [[nodiscard]] RawWorldIngestionResult3D
  ingestRawDelta(const msg::RawObstacleDelta3D& message, std::int64_t receive_stamp_ns,
                 const ProducerEpochAdmissionConfig& config, bool frame_matches);
  [[nodiscard]] MemoryStatusIngestionResult3D
  ingestMemoryStatus(const ProducerEpochObservation& observation,
                     bool announces_raw_update, std::int64_t now_ns,
                     const ProducerEpochAdmissionConfig& config, bool frame_matches);
  [[nodiscard]] RawWorldCommitResult3D
  commitRawUpdate(const RawObstacleGridUpdate3D& update, double reconstruction_ms,
                  std::int64_t ready_stamp_ns,
                  const ProducerEpochAdmissionConfig& config);

  [[nodiscard]] WorldPipelineInputSnapshot3D inputSnapshot() const;
  [[nodiscard]] std::shared_ptr<const ProductionMppiRawWorld3D>
  latestRawWorld() const noexcept;
  [[nodiscard]] bool scheduleLatestRawWorldUrgently();
  [[nodiscard]] bool observedWorldNeedsRefresh(const Point3& position) const;
  [[nodiscard]] ObservedWorldUpdate3D
  updateObservedWorld(ObservedWorldBuildRequest3D request);

  [[nodiscard]] bool requestStaticWork(bool force_refresh, bool world_ready);

  [[nodiscard]] WorldPipelineResidentSnapshot3D residentSnapshot() const;
  [[nodiscard]] ResidentLease lockResident() const;
  [[nodiscard]] PublicationLease lockPublication();
  [[nodiscard]] bool refreshTransientEvidence(
      const std::shared_ptr<const WorldSnapshot3D>& expected_world,
      std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world_owner,
      std::optional<ProprioceptiveFreeSpaceSeed3D> free_space_seed);

  void recordSupersededPlanningGeneration() noexcept;
  [[nodiscard]] WorldPipelineStatistics3D statistics() const noexcept;

private:
  [[nodiscard]] RawWorldIngestionResult3D
  finishRawIngestionLocked(RawObstacleGridUpdate3D update);
  void run(std::stop_token stop_token) noexcept;
  void runObserved(std::stop_token stop_token) noexcept;
  void runStatic(std::stop_token stop_token) noexcept;
  [[nodiscard]] ObservedWorldUpdate3D
  finishObservedWorldUpdate(ObservedWorldBuildAssessment3D assessment,
                            const ObservedWorldRuntime3D& runtime);
  [[nodiscard]] bool residentParentMatches(
      const PreparedObservedWorldBuild3D& build,
      const std::shared_ptr<const WorldSnapshot3D>& resident_world) const noexcept;
  [[nodiscard]] ObservedWorldBuildHistory3D observedBuildHistory() const;
  void recordObservedBuild(TimePoint build_time, ObservedEsdf3DBuildMode mode,
                           std::size_t recomputed_voxels,
                           std::size_t reused_voxels) noexcept;
  void recordObservedBuildThrottled() noexcept;
  void completeStaticWork() noexcept;
  void handleProcessingFailure(std::exception_ptr failure) noexcept;

  std::unique_ptr<ObservedWorldBuilder3D> observed_builder_;
  std::variant<ObservedWorldRuntime3D, StaticWorldRuntime3D> runtime_;
  ProcessingFailureHandler failure_handler_;

  mutable std::mutex lifecycle_mutex_;
  std::jthread worker_;
  bool stopping_{false};
  std::atomic_bool accepting_{false};

  mutable std::mutex ingestion_mutex_;
  LatestObservationTracker latest_observation_tracker_{};
  RawObstacleDeltaAccumulator3D raw_delta_accumulator_{};
  ProductionMppiPendingRawWorldUpdate pending_raw_world_update_{};
  bool raw_world_identity_conflicted_{false};
  std::atomic<std::shared_ptr<const ProductionMppiRawWorld3D>> latest_raw_world_;

  mutable std::mutex queue_mutex_;
  std::condition_variable_any queue_condition_;
  LatestWinsDeferredScheduler<std::shared_ptr<const ProductionMppiRawWorld3D>>
      raw_world_scheduler_{};
  bool pending_static_work_{false};
  bool static_work_in_progress_{false};

  mutable std::mutex publication_mutex_;
  std::shared_ptr<const WorldSnapshot3D> resident_world_;
  ProductionWorldBuildTelemetry3D resident_telemetry_{};
  LocalWorldGenerationCounter local_world_generation_counter_{};

  mutable std::mutex build_state_mutex_;
  TimePoint last_observed_build_time_{};

  std::atomic<std::uint64_t> raw_updates_{0U};
  std::atomic<std::uint64_t> dropped_raw_worlds_{0U};
  std::atomic<std::uint64_t> throttled_observed_builds_{0U};
  std::atomic<std::uint64_t> observed_full_builds_{0U};
  std::atomic<std::uint64_t> observed_incremental_builds_{0U};
  std::atomic<std::uint64_t> observed_reused_builds_{0U};
  std::atomic<std::uint64_t> observed_recomputed_voxels_{0U};
  std::atomic<std::uint64_t> observed_reused_voxels_{0U};
  std::atomic<std::uint64_t> superseded_planning_generations_{0U};
  std::atomic<std::uint64_t> rejected_world_publications_{0U};
  std::atomic<std::uint64_t> processing_failures_{0U};
  std::atomic<std::uint64_t> failure_handler_failures_{0U};
  std::atomic<std::uint64_t> rejected_after_stop_{0U};
};

} // namespace drone_city_nav
