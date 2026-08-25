#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <ranges>
#include <span>
#include <string>
#include <tuple>
#include <vector>

#include "production_mppi_node.hpp"
#include "production_mppi_raw_input_internal.hpp"

namespace drone_city_nav {
namespace {

constexpr std::uint64_t kFnvOffset{1469598103934665603ULL};
constexpr std::uint64_t kFnvPrime{1099511628211ULL};
constexpr std::uint64_t kMemoryStatusFingerprintDomain{0x4d454d5354415455ULL};

void hashValue(std::uint64_t& hash, const std::uint64_t value) noexcept {
  for (std::size_t shift = 0U; shift < 64U; shift += 8U) {
    hash ^= (value >> shift) & 0xffU;
    hash *= kFnvPrime;
  }
}

void hashString(std::uint64_t& hash, const std::string& value) noexcept {
  hashValue(hash, value.size());
  for (const char character : value) {
    hashValue(hash, static_cast<unsigned char>(character));
  }
}

[[nodiscard]] std::uint64_t
memoryStatusFingerprint(const msg::ObstacleMemoryStatus& message) noexcept {
  std::uint64_t hash{kFnvOffset};
  hashValue(hash, kMemoryStatusFingerprintDomain);
  hashString(hash, message.header.frame_id);
  hashValue(hash, message.occupied_cell_count);
  hashValue(hash, message.raw_snapshot_published ? 1U : 0U);
  hashValue(hash, message.raw_delta_published ? 1U : 0U);
  hashValue(hash, message.full_snapshot_published ? 1U : 0U);
  return hash == 0U ? 1U : hash;
}

[[nodiscard]] std::int64_t
timeNanoseconds(const builtin_interfaces::msg::Time& stamp) noexcept {
  if (stamp.sec < 0 || stamp.nanosec >= 1'000'000'000U) {
    return 0;
  }
  return static_cast<std::int64_t>(stamp.sec) * 1'000'000'000LL +
         static_cast<std::int64_t>(stamp.nanosec);
}

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

[[nodiscard]] bool
statusAnnouncesRawUpdate(const msg::ObstacleMemoryStatus& message) noexcept {
  return message.raw_snapshot_published || message.raw_delta_published;
}

} // namespace

void ProductionMppiNode::onRawObstacleSnapshot(
    msg::RawObstacleSnapshot::ConstSharedPtr message) {
  if (use_static_map_) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  const ProducerEpochAdmissionConfig config =
      production_mppi_raw_input_detail::producerEpochConfig(
          maximum_esdf_age_ms_, stale_esdf_execution_window_ms_);
  RawObstacleGridUpdate update;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_,
                                raw_reconstruction_mutex_};
    update = raw_delta_accumulator_.apply(
        *message, latest_observation_tracker_.admissionState(), receive_stamp_ns,
        receive_stamp_ns, config, message->grid.header.frame_id == frame_id_);
    if (update.current_identity_conflict && !raw_world_identity_conflicted_) {
      raw_world_identity_conflicted_ = true;
      latest_raw_world_.store(nullptr, std::memory_order_release);
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
  }
  if (!update.accepted()) {
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "RAW_OBSTACLE_FULL rejected status=%s producer=%" PRIu64 " revision=%" PRIu64,
        rawObstacleGridUpdateStatusName(update.status), message->producer_instance_id,
        message->obstacle_snapshot_revision);
    return;
  }
  const double reconstruction_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - started)
                                       .count();
  queueRawWorld(update, reconstruction_ms);
}

void ProductionMppiNode::onRawObstacleDelta(
    msg::RawObstacleDelta::ConstSharedPtr message) {
  if (use_static_map_) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  const ProducerEpochAdmissionConfig config =
      production_mppi_raw_input_detail::producerEpochConfig(
          maximum_esdf_age_ms_, stale_esdf_execution_window_ms_);
  RawObstacleGridUpdate update;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_,
                                raw_reconstruction_mutex_};
    update = raw_delta_accumulator_.apply(
        *message, latest_observation_tracker_.admissionState(), receive_stamp_ns,
        receive_stamp_ns, config, message->header.frame_id == frame_id_);
    if (update.current_identity_conflict && !raw_world_identity_conflicted_) {
      raw_world_identity_conflicted_ = true;
      latest_raw_world_.store(nullptr, std::memory_order_release);
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
  }
  if (!update.accepted()) {
    if (update.status == RawObstacleGridUpdateStatus::kInvalidMessage ||
        update.status == RawObstacleGridUpdateStatus::kIdentityConflict) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "RAW_OBSTACLE_DELTA rejected status=%s producer=%" PRIu64 " base=%" PRIu64
          " revision=%" PRIu64,
          rawObstacleGridUpdateStatusName(update.status), message->producer_instance_id,
          message->base_snapshot_revision, message->obstacle_snapshot_revision);
    }
    return;
  }
  const double reconstruction_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - started)
                                       .count();
  queueRawWorld(update, reconstruction_ms);
}

void ProductionMppiNode::queueRawWorld(const RawObstacleGridUpdate& update,
                                       const double reconstruction_ms) {
  const std::int64_t ready_stamp_ns = get_clock()->now().nanoseconds();
  const ProducerEpochAdmissionConfig config =
      production_mppi_raw_input_detail::producerEpochConfig(
          maximum_esdf_age_ms_, stale_esdf_execution_window_ms_);
  auto world =
      std::make_shared<const ProductionMppiRawWorld2D>(ProductionMppiRawWorld2D{
          .version =
              RawMapVersion{
                  .producer_instance_id = update.state.producer_instance_id,
                  .base_snapshot_revision = update.state.base_snapshot_revision,
                  .revision = update.state.obstacle_snapshot_revision,
              },
          .source_stamp_ns = update.evidence_observation.source_stamp_ns,
          .receive_stamp_ns = update.evidence_observation.receive_stamp_ns,
          .ready_stamp_ns = ready_stamp_ns,
          .reconstruction_ms = reconstruction_ms,
          .occupancy = update.state.occupancy,
      });
  bool installed{false};
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_,
                                raw_reconstruction_mutex_, raw_queue_mutex_};
    const ProducerEpochAuthority authority = latest_observation_tracker_.authority();
    const LatestObservation& status = latest_observation_tracker_.latest();
    const ProducerEvidenceAdmissionState& evidence =
        raw_delta_accumulator_.evidenceAdmissionState();
    if (update.accepted() && status.available() &&
        status.producer_instance_id == authority.producer_instance_id &&
        status.producer_epoch_generation == authority.generation &&
        update.authority_generation == authority.generation &&
        evidenceMatches(evidence, authority, update.evidence_observation) &&
        producerEpochObservationFresh(config, update.evidence_observation,
                                      ready_stamp_ns)) {
      latest_raw_world_.store(world, std::memory_order_release);
      raw_world_identity_conflicted_ = false;
      if (pending_raw_world_update_.satisfiedBy(evidence, world->version)) {
        pending_raw_world_update_ = {};
      }
      const auto submission = raw_world_scheduler_.submit(world);
      if (submission.replaced_pending) {
        ++dropped_raw_snapshots_;
      }
      installed = true;
    }
  }
  if (!installed) {
    return;
  }
  no_static_raw_updates_.fetch_add(1U, std::memory_order_relaxed);
  raw_queue_condition_.notify_all();
}

void ProductionMppiNode::onRawObstacleSnapshot3D(
    msg::RawObstacleSnapshot3D::ConstSharedPtr message) {
  if (use_static_map_ ||
      no_static_world_model_ != ProductionNoStaticWorldModel::kObservedOccupancy3D) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  const ProducerEpochAdmissionConfig config =
      production_mppi_raw_input_detail::producerEpochConfig(
          maximum_esdf_age_ms_, stale_esdf_execution_window_ms_);
  RawObstacleGridUpdate3D update;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_,
                                raw_reconstruction_mutex_};
    update = raw_delta_accumulator_3d_.apply(
        *message, latest_observation_tracker_.admissionState(), receive_stamp_ns,
        receive_stamp_ns, config, message->header.frame_id == frame_id_);
    if (update.current_identity_conflict && !raw_world_identity_conflicted_) {
      raw_world_identity_conflicted_ = true;
      latest_raw_world_3d_.store(nullptr, std::memory_order_release);
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
  }
  if (!update.accepted()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                         "RAW_OBSTACLE_3D_FULL rejected status=%s producer=%" PRIu64
                         " revision=%" PRIu64,
                         rawObstacleGridUpdateStatus3DName(update.status),
                         message->producer_instance_id,
                         message->obstacle_snapshot_revision);
    return;
  }
  const double reconstruction_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - started)
                                       .count();
  queueRawWorld3D(update, reconstruction_ms);
}

void ProductionMppiNode::onRawObstacleDelta3D(
    msg::RawObstacleDelta3D::ConstSharedPtr message) {
  if (use_static_map_ ||
      no_static_world_model_ != ProductionNoStaticWorldModel::kObservedOccupancy3D) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  const ProducerEpochAdmissionConfig config =
      production_mppi_raw_input_detail::producerEpochConfig(
          maximum_esdf_age_ms_, stale_esdf_execution_window_ms_);
  RawObstacleGridUpdate3D update;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_,
                                raw_reconstruction_mutex_};
    update = raw_delta_accumulator_3d_.apply(
        *message, latest_observation_tracker_.admissionState(), receive_stamp_ns,
        receive_stamp_ns, config, message->header.frame_id == frame_id_);
    if (update.current_identity_conflict && !raw_world_identity_conflicted_) {
      raw_world_identity_conflicted_ = true;
      latest_raw_world_3d_.store(nullptr, std::memory_order_release);
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
  }
  if (!update.accepted()) {
    if (update.status == RawObstacleGridUpdateStatus3D::kInvalidMessage ||
        update.status == RawObstacleGridUpdateStatus3D::kIdentityConflict) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                           "RAW_OBSTACLE_3D_DELTA rejected status=%s producer=%" PRIu64
                           " base=%" PRIu64 " revision=%" PRIu64,
                           rawObstacleGridUpdateStatus3DName(update.status),
                           message->producer_instance_id,
                           message->base_snapshot_revision,
                           message->obstacle_snapshot_revision);
    }
    return;
  }
  const double reconstruction_ms = std::chrono::duration<double, std::milli>(
                                       std::chrono::steady_clock::now() - started)
                                       .count();
  queueRawWorld3D(update, reconstruction_ms);
}

void ProductionMppiNode::queueRawWorld3D(const RawObstacleGridUpdate3D& update,
                                         const double reconstruction_ms) {
  const std::int64_t ready_stamp_ns = get_clock()->now().nanoseconds();
  const ProducerEpochAdmissionConfig config =
      production_mppi_raw_input_detail::producerEpochConfig(
          maximum_esdf_age_ms_, stale_esdf_execution_window_ms_);
  const RawMapVersion version{
      .producer_instance_id = update.state.producer_instance_id,
      .base_snapshot_revision = update.state.base_snapshot_revision,
      .revision = update.state.obstacle_snapshot_revision,
  };
  const std::shared_ptr<const VersionedObservedRawWorld3D> execution_owner =
      VersionedObservedRawWorld3D::captureOwned(version, update.state.occupancy,
                                                std::nullopt, std::nullopt);
  if (!execution_owner) {
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_RAW_WORLD3D rejected producer=%" PRIu64
                 " revision=%" PRIu64 " reason=invalid_execution_owner",
                 version.producer_instance_id, version.revision);
    return;
  }
  auto world = std::make_shared<ProductionMppiRawWorld3D>(ProductionMppiRawWorld3D{
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
  std::shared_ptr<const ProductionMppiRawWorld3D> immutable_world;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_,
                                raw_reconstruction_mutex_, raw_queue_mutex_};
    const ProducerEpochAuthority authority = latest_observation_tracker_.authority();
    const LatestObservation& status = latest_observation_tracker_.latest();
    const ProducerEvidenceAdmissionState& evidence =
        raw_delta_accumulator_3d_.evidenceAdmissionState();
    if (!update.accepted() || !status.available() ||
        status.producer_instance_id != authority.producer_instance_id ||
        status.producer_epoch_generation != authority.generation ||
        update.authority_generation != authority.generation ||
        !evidenceMatches(evidence, authority, update.evidence_observation) ||
        !producerEpochObservationFresh(config, update.evidence_observation,
                                       ready_stamp_ns)) {
      return;
    }
    const auto& pending = raw_world_scheduler_3d_.pending();
    if (pending.has_value() && *pending) {
      dropped_raw_snapshots_.fetch_add(1U, std::memory_order_relaxed);
      world->full_reset = world->full_reset || (*pending)->full_reset;
      mergeDirtyChunks(world->dirty_chunks, (*pending)->dirty_chunks);
    }
    immutable_world = world;
    static_cast<void>(raw_world_scheduler_3d_.submit(immutable_world));
    latest_raw_world_3d_.store(immutable_world, std::memory_order_release);
    raw_world_identity_conflicted_ = false;
    if (pending_raw_world_update_.satisfiedBy(evidence, immutable_world->version)) {
      pending_raw_world_update_ = {};
    }
  }
  no_static_raw_updates_.fetch_add(1U, std::memory_order_relaxed);
  std::shared_ptr<const ProductionMppiRawWorld3D> topology_world = immutable_world;
  {
    const std::scoped_lock lock{topology_queue_mutex_};
    if (pending_topology_world_3d_) {
      auto merged = std::make_shared<ProductionMppiRawWorld3D>(*topology_world);
      merged->full_reset = merged->full_reset || pending_topology_world_3d_->full_reset;
      mergeDirtyChunks(merged->dirty_chunks, pending_topology_world_3d_->dirty_chunks);
      topology_world = std::move(merged);
    }
    pending_topology_world_3d_ = std::move(topology_world);
  }
  raw_queue_condition_.notify_all();
  topology_queue_condition_.notify_all();
}

void ProductionMppiNode::onMemoryStatus(const msg::ObstacleMemoryStatus& message) {
  if (use_static_map_) {
    return;
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  const std::int64_t source_stamp_ns = timeNanoseconds(message.header.stamp);
  const ProducerEpochAdmissionConfig config =
      production_mppi_raw_input_detail::producerEpochConfig(
          maximum_esdf_age_ms_, stale_esdf_execution_window_ms_);
  const ProducerEpochObservation observation{
      .producer_instance_id = message.producer_instance_id,
      .sequence = message.sequence,
      .source_stamp_ns = source_stamp_ns,
      .receive_stamp_ns = now_ns,
      .content_fingerprint = memoryStatusFingerprint(message),
  };
  std::optional<RawObstacleGridUpdate> synchronized_2d;
  std::optional<RawObstacleGridUpdate3D> synchronized_3d;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_,
                                raw_reconstruction_mutex_};
    const ProducerEpochAdmissionResult admission = latest_observation_tracker_.observe(
        config, observation, now_ns, message.header.frame_id == frame_id_);
    const bool authority_boundary =
        admission.status == ProducerEpochAdmissionStatus::kAcceptedInitial ||
        admission.producer_handoff;
    bool request_revocation = admission.current_identity_conflict;
    const auto clear_current_raw = [this]() noexcept {
      if (no_static_world_model_ ==
          ProductionNoStaticWorldModel::kObservedOccupancy3D) {
        latest_raw_world_3d_.store(nullptr, std::memory_order_release);
      } else {
        latest_raw_world_.store(nullptr, std::memory_order_release);
      }
    };
    if (authority_boundary) {
      raw_world_identity_conflicted_ = false;
      pending_raw_world_update_ = {};
    }
    if (admission.current_identity_conflict) {
      raw_world_identity_conflicted_ = true;
      clear_current_raw();
    }
    if (admission.install_observation && statusAnnouncesRawUpdate(message)) {
      const ProducerEpochAuthority authority = latest_observation_tracker_.authority();
      const ProducerEvidenceAdmissionState& evidence =
          no_static_world_model_ == ProductionNoStaticWorldModel::kObservedOccupancy3D
              ? raw_delta_accumulator_3d_.evidenceAdmissionState()
              : raw_delta_accumulator_.evidenceAdmissionState();
      const std::shared_ptr<const ProductionMppiRawWorld2D> raw_world_2d =
          latest_raw_world_.load(std::memory_order_acquire);
      const std::shared_ptr<const ProductionMppiRawWorld3D> raw_world_3d =
          latest_raw_world_3d_.load(std::memory_order_acquire);
      const bool raw_pointer_current =
          no_static_world_model_ == ProductionNoStaticWorldModel::kObservedOccupancy3D
              ? (raw_world_3d != nullptr &&
                 raw_world_3d->version.producer_instance_id ==
                     authority.producer_instance_id &&
                 raw_world_3d->version.revision == evidence.sequence)
              : (raw_world_2d != nullptr &&
                 raw_world_2d->version.producer_instance_id ==
                     authority.producer_instance_id &&
                 raw_world_2d->version.revision == evidence.sequence);
      const bool installed_through_status =
          authority.valid() && !evidence.current_identity_conflicted &&
          evidence.authority_generation == authority.generation &&
          evidence.producer_instance_id == authority.producer_instance_id &&
          evidence.source_stamp_ns >= source_stamp_ns && raw_pointer_current;
      if (!installed_through_status) {
        pending_raw_world_update_ = ProductionMppiPendingRawWorldUpdate{
            .authority_generation = authority.generation,
            .producer_instance_id = authority.producer_instance_id,
            .announced_sequence = message.sequence,
            .minimum_source_stamp_ns = source_stamp_ns,
        };
      } else {
        const RawMapVersion* committed_version = nullptr;
        if (no_static_world_model_ ==
                ProductionNoStaticWorldModel::kObservedOccupancy3D &&
            raw_world_3d != nullptr) {
          committed_version = std::addressof(raw_world_3d->version);
        } else if (no_static_world_model_ ==
                       ProductionNoStaticWorldModel::kOccupancy2D &&
                   raw_world_2d != nullptr) {
          committed_version = std::addressof(raw_world_2d->version);
        }
        if (committed_version != nullptr &&
            pending_raw_world_update_.satisfiedBy(evidence, *committed_version)) {
          pending_raw_world_update_ = {};
        }
      }
    }
    if (no_static_world_model_ == ProductionNoStaticWorldModel::kObservedOccupancy3D) {
      synchronized_3d = raw_delta_accumulator_3d_.synchronizeProducerEpoch(
          latest_observation_tracker_.admissionState(), now_ns, config);
      if (synchronized_3d->current_identity_conflict &&
          !raw_world_identity_conflicted_) {
        raw_world_identity_conflicted_ = true;
        clear_current_raw();
        request_revocation = true;
      }
    } else {
      synchronized_2d = raw_delta_accumulator_.synchronizeProducerEpoch(
          latest_observation_tracker_.admissionState(), now_ns, config);
      if (synchronized_2d->current_identity_conflict &&
          !raw_world_identity_conflicted_) {
        raw_world_identity_conflicted_ = true;
        clear_current_raw();
        request_revocation = true;
      }
    }
    if (request_revocation) {
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
  }
  if (synchronized_2d.has_value() && synchronized_2d->accepted()) {
    queueRawWorld(*synchronized_2d, 0.0);
  }
  if (synchronized_3d.has_value() && synchronized_3d->accepted()) {
    queueRawWorld3D(*synchronized_3d, 0.0);
  }
}

} // namespace drone_city_nav
