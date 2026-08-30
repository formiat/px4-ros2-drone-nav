#include "world_pipeline_3d.hpp"

#include <algorithm>
#include <ranges>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

void mergeDirtyChunks(std::vector<OccupancyChunkIndex3D>& destination,
                      const std::span<const OccupancyChunkIndex3D> source) {
  destination.insert(destination.end(), source.begin(), source.end());
  std::ranges::sort(destination, [](const OccupancyChunkIndex3D first,
                                    const OccupancyChunkIndex3D second) {
    return std::tie(first.z, first.y, first.x) < std::tie(second.z, second.y, second.x);
  });
  const auto duplicates = std::ranges::unique(destination);
  destination.erase(duplicates.begin(), duplicates.end());
}

[[nodiscard]] bool
evidenceMatches(const ProducerEvidenceAdmissionState& evidence,
                const ProducerEpochAuthority& authority,
                const ProducerEpochObservation& observation) noexcept {
  return authority.valid() && !evidence.current_identity_conflicted &&
         evidence.authority_generation == authority.generation &&
         evidence.producer_instance_id == authority.producer_instance_id &&
         evidence.producer_instance_id == observation.producer_instance_id &&
         evidence.sequence == observation.sequence &&
         evidence.source_stamp_ns == observation.source_stamp_ns &&
         evidence.content_fingerprint == observation.content_fingerprint;
}

} // namespace

std::string_view
rawWorldCommitStatus3DName(const RawWorldCommitStatus3D status) noexcept {
  switch (status) {
    case RawWorldCommitStatus3D::kCommitted:
      return "committed";
    case RawWorldCommitStatus3D::kStopped:
      return "stopped";
    case RawWorldCommitStatus3D::kInvalidUpdate:
      return "invalid_update";
    case RawWorldCommitStatus3D::kInvalidExecutionOwner:
      return "invalid_execution_owner";
    case RawWorldCommitStatus3D::kSupersededEvidence:
      return "superseded_evidence";
  }
  return "unknown";
}

WorldPipeline3D::ResidentLease::ResidentLease(const WorldPipeline3D& owner)
    : owner_{std::addressof(owner)},
      lock_{owner.publication_mutex_} {
}

const std::shared_ptr<const WorldSnapshot3D>&
WorldPipeline3D::ResidentLease::world() const noexcept {
  return owner_->resident_world_;
}

const ProductionWorldBuildTelemetry3D&
WorldPipeline3D::ResidentLease::telemetry() const noexcept {
  return owner_->resident_telemetry_;
}

WorldPipeline3D::PublicationLease::PublicationLease(WorldPipeline3D& owner)
    : owner_{std::addressof(owner)},
      lock_{owner.publication_mutex_} {
}

const std::shared_ptr<const WorldSnapshot3D>&
WorldPipeline3D::PublicationLease::world() const noexcept {
  return owner_->resident_world_;
}

const ProductionWorldBuildTelemetry3D&
WorldPipeline3D::PublicationLease::telemetry() const noexcept {
  return owner_->resident_telemetry_;
}

std::optional<LocalWorldGeneration> WorldPipeline3D::PublicationLease::issueGeneration(
    const RawMapVersion& raw_map, const std::uint64_t pose_revision,
    const std::uint64_t esdf_revision, const std::uint64_t gpu_esdf_revision) noexcept {
  return owner_->local_world_generation_counter_.issue(
      raw_map, pose_revision, esdf_revision, gpu_esdf_revision);
}

bool WorldPipeline3D::PublicationLease::publish(
    std::shared_ptr<const WorldSnapshot3D> world,
    const ProductionWorldBuildTelemetry3D& telemetry) noexcept {
  if (world == nullptr || !productionWorldGenerationCoherent(*world)) {
    owner_->resident_world_.reset();
    owner_->resident_telemetry_ = {};
    owner_->rejected_world_publications_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }
  owner_->resident_world_ = std::move(world);
  owner_->resident_telemetry_ = telemetry;
  return true;
}

void WorldPipeline3D::PublicationLease::invalidateAndRecordRejection() noexcept {
  owner_->resident_world_.reset();
  owner_->resident_telemetry_ = {};
  recordRejection();
}

void WorldPipeline3D::PublicationLease::recordRejection() noexcept {
  owner_->rejected_world_publications_.fetch_add(1U, std::memory_order_relaxed);
}

WorldPipeline3D::WorldPipeline3D(const bool static_world,
                                 ObservedWorldProcessor observed_processor,
                                 StaticWorldProcessor static_processor,
                                 ProcessingFailureHandler failure_handler)
    : static_world_{static_world},
      observed_processor_{std::move(observed_processor)},
      static_processor_{std::move(static_processor)},
      failure_handler_{std::move(failure_handler)} {
  if ((static_world_ && !static_processor_) ||
      (!static_world_ && !observed_processor_)) {
    throw std::invalid_argument{"world pipeline processor does not match mode"};
  }
}

WorldPipeline3D::~WorldPipeline3D() {
  stop();
}

void WorldPipeline3D::start() {
  const std::scoped_lock lock{lifecycle_mutex_};
  if (worker_.joinable() || stopping_) {
    return;
  }
  accepting_.store(true, std::memory_order_release);
  worker_ = std::jthread{[this](const std::stop_token stop_token) { run(stop_token); }};
}

void WorldPipeline3D::stop() noexcept {
  std::jthread stopping_worker;
  {
    const std::scoped_lock lock{lifecycle_mutex_};
    accepting_.store(false, std::memory_order_release);
    if (!worker_.joinable()) {
      return;
    }
    stopping_ = true;
    worker_.request_stop();
    queue_condition_.notify_all();
    stopping_worker = std::move(worker_);
  }
  stopping_worker.join();
  {
    const std::scoped_lock lock{lifecycle_mutex_};
    stopping_ = false;
  }
}

bool WorldPipeline3D::accepting() const noexcept {
  return accepting_.load(std::memory_order_acquire);
}

RawWorldIngestionResult3D WorldPipeline3D::ingestRawSnapshot(
    const msg::RawObstacleSnapshot3D& message, const std::int64_t receive_stamp_ns,
    const ProducerEpochAdmissionConfig& config, const bool frame_matches) {
  const std::scoped_lock lock{ingestion_mutex_};
  return finishRawIngestionLocked(raw_delta_accumulator_.apply(
      message, latest_observation_tracker_.admissionState(), receive_stamp_ns,
      receive_stamp_ns, config, frame_matches));
}

RawWorldIngestionResult3D WorldPipeline3D::ingestRawDelta(
    const msg::RawObstacleDelta3D& message, const std::int64_t receive_stamp_ns,
    const ProducerEpochAdmissionConfig& config, const bool frame_matches) {
  const std::scoped_lock lock{ingestion_mutex_};
  return finishRawIngestionLocked(raw_delta_accumulator_.apply(
      message, latest_observation_tracker_.admissionState(), receive_stamp_ns,
      receive_stamp_ns, config, frame_matches));
}

RawWorldIngestionResult3D
WorldPipeline3D::finishRawIngestionLocked(RawObstacleGridUpdate3D update) {
  const bool conflict_started =
      update.current_identity_conflict && !raw_world_identity_conflicted_;
  if (update.current_identity_conflict) {
    raw_world_identity_conflicted_ = true;
    latest_raw_world_.store(nullptr, std::memory_order_release);
  }
  return RawWorldIngestionResult3D{
      .update = std::move(update),
      .execution_revocation_required = conflict_started,
  };
}

MemoryStatusIngestionResult3D WorldPipeline3D::ingestMemoryStatus(
    const ProducerEpochObservation& observation, const bool announces_raw_update,
    const std::int64_t now_ns, const ProducerEpochAdmissionConfig& config,
    const bool frame_matches) {
  const std::scoped_lock lock{ingestion_mutex_};
  const ProducerEpochAdmissionResult admission =
      latest_observation_tracker_.observe(config, observation, now_ns, frame_matches);
  const bool authority_boundary =
      admission.status == ProducerEpochAdmissionStatus::kAcceptedInitial ||
      admission.producer_handoff;
  bool execution_revocation_required = false;
  if (authority_boundary) {
    raw_world_identity_conflicted_ = false;
    pending_raw_world_update_ = {};
  }
  if (admission.current_identity_conflict) {
    execution_revocation_required = !raw_world_identity_conflicted_;
    raw_world_identity_conflicted_ = true;
    latest_raw_world_.store(nullptr, std::memory_order_release);
  }
  if (admission.install_observation && announces_raw_update) {
    const ProducerEpochAuthority authority = latest_observation_tracker_.authority();
    const ProducerEvidenceAdmissionState& evidence =
        raw_delta_accumulator_.evidenceAdmissionState();
    const std::shared_ptr<const ProductionMppiRawWorld3D> raw_world =
        latest_raw_world_.load(std::memory_order_acquire);
    const bool raw_pointer_current =
        raw_world != nullptr &&
        raw_world->version.producer_instance_id == authority.producer_instance_id &&
        raw_world->version.revision == evidence.sequence;
    const bool installed_through_status =
        authority.valid() && !evidence.current_identity_conflicted &&
        evidence.authority_generation == authority.generation &&
        evidence.producer_instance_id == authority.producer_instance_id &&
        evidence.source_stamp_ns >= observation.source_stamp_ns && raw_pointer_current;
    if (!installed_through_status) {
      pending_raw_world_update_ = ProductionMppiPendingRawWorldUpdate{
          .authority_generation = authority.generation,
          .producer_instance_id = authority.producer_instance_id,
          .announced_sequence = observation.sequence,
          .minimum_source_stamp_ns = observation.source_stamp_ns,
      };
    } else if (raw_world != nullptr &&
               pending_raw_world_update_.satisfiedBy(evidence, raw_world->version)) {
      pending_raw_world_update_ = {};
    }
  }
  std::optional<RawObstacleGridUpdate3D> synchronized =
      raw_delta_accumulator_.synchronizeProducerEpoch(
          latest_observation_tracker_.admissionState(), now_ns, config);
  if (synchronized.has_value() && synchronized->current_identity_conflict) {
    if (!raw_world_identity_conflicted_) {
      execution_revocation_required = true;
    }
    raw_world_identity_conflicted_ = true;
    latest_raw_world_.store(nullptr, std::memory_order_release);
  }
  return MemoryStatusIngestionResult3D{
      .synchronized_update = std::move(synchronized),
      .execution_revocation_required = execution_revocation_required,
  };
}

RawWorldCommitResult3D WorldPipeline3D::commitRawUpdate(
    const RawObstacleGridUpdate3D& update, const double reconstruction_ms,
    const std::int64_t ready_stamp_ns, const ProducerEpochAdmissionConfig& config) {
  if (!update.accepted()) {
    return {
        .status = RawWorldCommitStatus3D::kInvalidUpdate,
        .world = nullptr,
        .replaced_pending = false,
    };
  }
  const RawMapVersion version{
      .producer_instance_id = update.state.producer_instance_id,
      .base_snapshot_revision = update.state.base_snapshot_revision,
      .revision = update.state.obstacle_snapshot_revision,
  };
  const std::shared_ptr<const VersionedObservedRawWorld3D> execution_owner =
      VersionedObservedRawWorld3D::captureOwned(version, update.state.occupancy,
                                                std::nullopt, std::nullopt);
  if (execution_owner == nullptr) {
    return {
        .status = RawWorldCommitStatus3D::kInvalidExecutionOwner,
        .world = nullptr,
        .replaced_pending = false,
    };
  }
  auto mutable_world =
      std::make_shared<ProductionMppiRawWorld3D>(ProductionMppiRawWorld3D{
          .version = version,
          .source_stamp_ns = update.evidence_observation.source_stamp_ns,
          .receive_stamp_ns = update.evidence_observation.receive_stamp_ns,
          .ready_stamp_ns = ready_stamp_ns,
          .reconstruction_ms = reconstruction_ms,
          .occupancy = update.state.occupancy,
          .execution_owner = execution_owner,
          .dirty_chunks = update.dirty_chunks,
          .full_reset = update.full_reset,
      });

  const std::scoped_lock lifecycle_lock{lifecycle_mutex_};
  if (!accepting_.load(std::memory_order_acquire)) {
    rejected_after_stop_.fetch_add(1U, std::memory_order_relaxed);
    return {
        .status = RawWorldCommitStatus3D::kStopped,
        .world = nullptr,
        .replaced_pending = false,
    };
  }
  bool replaced_pending{false};
  std::shared_ptr<const ProductionMppiRawWorld3D> immutable_world;
  {
    const std::scoped_lock lock{ingestion_mutex_, queue_mutex_};
    const ProducerEpochAuthority authority = latest_observation_tracker_.authority();
    const LatestObservation& status = latest_observation_tracker_.latest();
    const ProducerEvidenceAdmissionState& evidence =
        raw_delta_accumulator_.evidenceAdmissionState();
    if (!status.available() ||
        status.producer_instance_id != authority.producer_instance_id ||
        status.producer_epoch_generation != authority.generation ||
        update.authority_generation != authority.generation ||
        !evidenceMatches(evidence, authority, update.evidence_observation) ||
        !producerEpochObservationFresh(config, update.evidence_observation,
                                       ready_stamp_ns)) {
      return {
          .status = RawWorldCommitStatus3D::kSupersededEvidence,
          .world = nullptr,
          .replaced_pending = false,
      };
    }
    const auto& pending = raw_world_scheduler_.pending();
    if (pending.has_value() && *pending != nullptr) {
      replaced_pending = true;
      mutable_world->full_reset = mutable_world->full_reset || (*pending)->full_reset;
      mergeDirtyChunks(mutable_world->dirty_chunks, (*pending)->dirty_chunks);
    }
    immutable_world = mutable_world;
    static_cast<void>(raw_world_scheduler_.submit(immutable_world));
    latest_raw_world_.store(immutable_world, std::memory_order_release);
    raw_world_identity_conflicted_ = false;
    if (pending_raw_world_update_.satisfiedBy(evidence, immutable_world->version)) {
      pending_raw_world_update_ = {};
    }
  }
  raw_updates_.fetch_add(1U, std::memory_order_relaxed);
  if (replaced_pending) {
    dropped_raw_worlds_.fetch_add(1U, std::memory_order_relaxed);
  }
  queue_condition_.notify_all();
  return RawWorldCommitResult3D{
      .status = RawWorldCommitStatus3D::kCommitted,
      .world = std::move(immutable_world),
      .replaced_pending = replaced_pending,
  };
}

WorldPipelineInputSnapshot3D WorldPipeline3D::inputSnapshot() const {
  const std::scoped_lock lock{ingestion_mutex_};
  return WorldPipelineInputSnapshot3D{
      .latest_observation = latest_observation_tracker_.latest(),
      .latest_raw_world = latest_raw_world_.load(std::memory_order_acquire),
      .raw_world_identity_conflicted = raw_world_identity_conflicted_,
  };
}

std::shared_ptr<const ProductionMppiRawWorld3D>
WorldPipeline3D::latestRawWorld() const noexcept {
  return latest_raw_world_.load(std::memory_order_acquire);
}

bool WorldPipeline3D::scheduleLatestRawWorldUrgently() {
  const std::shared_ptr<const ProductionMppiRawWorld3D> raw_world = latestRawWorld();
  if (raw_world == nullptr) {
    return false;
  }
  const std::scoped_lock lifecycle_lock{lifecycle_mutex_};
  if (!accepting_.load(std::memory_order_acquire)) {
    rejected_after_stop_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }
  {
    const std::scoped_lock lock{queue_mutex_};
    static_cast<void>(raw_world_scheduler_.submit(raw_world, true));
  }
  queue_condition_.notify_all();
  return true;
}

bool WorldPipeline3D::requestStaticWork(const bool force_refresh,
                                        const bool world_ready) {
  const std::scoped_lock lifecycle_lock{lifecycle_mutex_};
  if (!accepting_.load(std::memory_order_acquire)) {
    rejected_after_stop_.fetch_add(1U, std::memory_order_relaxed);
    return false;
  }
  {
    const std::scoped_lock lock{queue_mutex_};
    if (!force_refresh &&
        (world_ready || static_work_in_progress_ || pending_static_work_)) {
      return false;
    }
    pending_static_work_ = true;
  }
  queue_condition_.notify_all();
  return true;
}

void WorldPipeline3D::completeStaticWork() noexcept {
  const std::scoped_lock lock{queue_mutex_};
  static_work_in_progress_ = false;
}

WorldPipelineResidentSnapshot3D WorldPipeline3D::residentSnapshot() const {
  const std::scoped_lock lock{publication_mutex_};
  return WorldPipelineResidentSnapshot3D{
      .world = resident_world_,
      .telemetry = resident_telemetry_,
  };
}

WorldPipeline3D::ResidentLease WorldPipeline3D::lockResident() const {
  return ResidentLease{*this};
}

WorldPipeline3D::PublicationLease WorldPipeline3D::lockPublication() {
  return PublicationLease{*this};
}

bool WorldPipeline3D::refreshTransientEvidence(
    const std::shared_ptr<const WorldSnapshot3D>& expected_world,
    std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world_owner,
    std::optional<ProprioceptiveFreeSpaceSeed3D> free_space_seed) {
  if (expected_world == nullptr || observed_raw_world_owner == nullptr) {
    return false;
  }
  const std::scoped_lock lock{publication_mutex_};
  if (resident_world_ != expected_world ||
      !productionWorldGenerationCoherent(*resident_world_)) {
    return false;
  }
  WorldSnapshot3D refreshed_world = *resident_world_;
  refreshed_world.observed_raw_world_owner = std::move(observed_raw_world_owner);
  refreshed_world.proprioceptive_free_space_seed = free_space_seed;
  auto publication =
      std::make_shared<const WorldSnapshot3D>(std::move(refreshed_world));
  if (!productionWorldGenerationCoherent(*publication) ||
      !publication->local_world_generation.sameSnapshot(
          resident_world_->local_world_generation) ||
      publication->revision != resident_world_->revision ||
      publication->distances_m != resident_world_->distances_m ||
      publication->observed_occupancy != resident_world_->observed_occupancy) {
    return false;
  }
  resident_world_ = std::move(publication);
  return true;
}

WorldPipelineObservedBuildState3D WorldPipeline3D::observedBuildState() const {
  const std::scoped_lock lock{build_state_mutex_};
  return WorldPipelineObservedBuildState3D{
      .last_build_time = last_observed_build_time_,
      .completed_builds = observed_full_builds_.load(std::memory_order_relaxed) +
                          observed_incremental_builds_.load(std::memory_order_relaxed),
  };
}

void WorldPipeline3D::recordObservedBuild(const TimePoint build_time,
                                          const ObservedEsdf3DBuildMode mode,
                                          const std::size_t recomputed_voxels,
                                          const std::size_t reused_voxels) noexcept {
  if (mode != ObservedEsdf3DBuildMode::kReused) {
    const std::scoped_lock lock{build_state_mutex_};
    last_observed_build_time_ = build_time;
  }
  switch (mode) {
    case ObservedEsdf3DBuildMode::kFull:
      observed_full_builds_.fetch_add(1U, std::memory_order_relaxed);
      break;
    case ObservedEsdf3DBuildMode::kIncremental:
      observed_incremental_builds_.fetch_add(1U, std::memory_order_relaxed);
      break;
    case ObservedEsdf3DBuildMode::kReused:
      observed_reused_builds_.fetch_add(1U, std::memory_order_relaxed);
      break;
  }
  observed_recomputed_voxels_.fetch_add(recomputed_voxels, std::memory_order_relaxed);
  observed_reused_voxels_.fetch_add(reused_voxels, std::memory_order_relaxed);
}

void WorldPipeline3D::recordObservedBuildThrottled() noexcept {
  throttled_observed_builds_.fetch_add(1U, std::memory_order_relaxed);
}

void WorldPipeline3D::recordSupersededPlanningGeneration() noexcept {
  superseded_planning_generations_.fetch_add(1U, std::memory_order_relaxed);
}

WorldPipelineStatistics3D WorldPipeline3D::statistics() const noexcept {
  return WorldPipelineStatistics3D{
      .raw_updates = raw_updates_.load(std::memory_order_relaxed),
      .dropped_raw_worlds = dropped_raw_worlds_.load(std::memory_order_relaxed),
      .throttled_observed_builds =
          throttled_observed_builds_.load(std::memory_order_relaxed),
      .observed_full_builds = observed_full_builds_.load(std::memory_order_relaxed),
      .observed_incremental_builds =
          observed_incremental_builds_.load(std::memory_order_relaxed),
      .observed_reused_builds = observed_reused_builds_.load(std::memory_order_relaxed),
      .observed_recomputed_voxels =
          observed_recomputed_voxels_.load(std::memory_order_relaxed),
      .observed_reused_voxels = observed_reused_voxels_.load(std::memory_order_relaxed),
      .superseded_planning_generations =
          superseded_planning_generations_.load(std::memory_order_relaxed),
      .rejected_world_publications =
          rejected_world_publications_.load(std::memory_order_relaxed),
      .processing_failures = processing_failures_.load(std::memory_order_relaxed),
      .failure_handler_failures =
          failure_handler_failures_.load(std::memory_order_relaxed),
      .rejected_after_stop = rejected_after_stop_.load(std::memory_order_relaxed),
  };
}

void WorldPipeline3D::run(const std::stop_token stop_token) noexcept {
  if (static_world_) {
    runStatic(stop_token);
  } else {
    runObserved(stop_token);
  }
}

void WorldPipeline3D::runObserved(const std::stop_token stop_token) noexcept {
  while (!stop_token.stop_requested()) {
    std::shared_ptr<const ProductionMppiRawWorld3D> raw_world;
    {
      std::unique_lock lock{queue_mutex_};
      while (!stop_token.stop_requested()) {
        const TimePoint now = std::chrono::steady_clock::now();
        if (std::optional scheduled = raw_world_scheduler_.takeReady(now)) {
          raw_world = std::move(*scheduled);
          break;
        }
        if (const std::optional<TimePoint> not_before =
                raw_world_scheduler_.notBefore();
            not_before.has_value()) {
          static_cast<void>(queue_condition_.wait_until(
              lock, stop_token, not_before.value(), [this]() {
                return raw_world_scheduler_.ready(std::chrono::steady_clock::now());
              }));
        } else {
          queue_condition_.wait(lock, stop_token,
                                [this]() { return raw_world_scheduler_.hasPending(); });
        }
      }
    }
    if (stop_token.stop_requested()) {
      return;
    }
    if (raw_world == nullptr) {
      continue;
    }
    try {
      const std::optional<TimePoint> retry_not_before = observed_processor_(*raw_world);
      if (retry_not_before.has_value()) {
        {
          const std::scoped_lock lock{queue_mutex_};
          raw_world_scheduler_.defer(std::move(raw_world), *retry_not_before);
        }
        queue_condition_.notify_all();
      }
    } catch (...) {
      handleProcessingFailure(std::current_exception());
    }
  }
}

void WorldPipeline3D::runStatic(const std::stop_token stop_token) noexcept {
  while (!stop_token.stop_requested()) {
    bool process{false};
    {
      std::unique_lock lock{queue_mutex_};
      queue_condition_.wait(lock, stop_token,
                            [this]() { return pending_static_work_; });
      if (stop_token.stop_requested()) {
        return;
      }
      process = std::exchange(pending_static_work_, false);
      static_work_in_progress_ = process;
    }
    if (!process) {
      continue;
    }
    try {
      static_processor_();
    } catch (...) {
      handleProcessingFailure(std::current_exception());
    }
    completeStaticWork();
  }
}

void WorldPipeline3D::handleProcessingFailure(std::exception_ptr failure) noexcept {
  processing_failures_.fetch_add(1U, std::memory_order_relaxed);
  if (!failure_handler_) {
    return;
  }
  try {
    failure_handler_(failure);
  } catch (...) {
    failure_handler_failures_.fetch_add(1U, std::memory_order_relaxed);
  }
}

} // namespace drone_city_nav
