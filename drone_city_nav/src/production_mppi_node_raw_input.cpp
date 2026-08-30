#include <algorithm>
#include <atomic>
#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "production_mppi_node.hpp"
#include "production_mppi_raw_input_internal.hpp"
#include "world_pipeline_3d.hpp"

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

[[nodiscard]] bool
statusAnnouncesRawUpdate(const msg::ObstacleMemoryStatus& message) noexcept {
  return message.raw_snapshot_published || message.raw_delta_published;
}

} // namespace

void ProductionMppiNode::onRawObstacleSnapshot3D(
    msg::RawObstacleSnapshot3D::ConstSharedPtr message) {
  if (use_static_map_) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  const ProducerEpochAdmissionConfig config =
      production_mppi_raw_input_detail::producerEpochConfig(
          maximum_esdf_age_ms_, stale_esdf_execution_window_ms_);
  RawWorldIngestionResult3D ingestion;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_};
    ingestion = world_pipeline_->ingestRawSnapshot(
        *message, receive_stamp_ns, config, message->header.frame_id == frame_id_);
    if (ingestion.execution_revocation_required) {
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
  }
  const RawObstacleGridUpdate3D& update = ingestion.update;
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
  if (use_static_map_) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  const ProducerEpochAdmissionConfig config =
      production_mppi_raw_input_detail::producerEpochConfig(
          maximum_esdf_age_ms_, stale_esdf_execution_window_ms_);
  RawWorldIngestionResult3D ingestion;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_};
    ingestion = world_pipeline_->ingestRawDelta(*message, receive_stamp_ns, config,
                                                message->header.frame_id == frame_id_);
    if (ingestion.execution_revocation_required) {
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
  }
  const RawObstacleGridUpdate3D& update = ingestion.update;
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
  RawWorldCommitResult3D result;
  {
    // Keep raw-world replacement in the same outer transaction as execution
    // evidence publication until that integration gate moves into the execution
    // service facade.
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_};
    result = world_pipeline_->commitRawUpdate(update, reconstruction_ms, ready_stamp_ns,
                                              config);
  }
  if (result.status == RawWorldCommitStatus3D::kInvalidExecutionOwner) {
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_RAW_WORLD3D rejected producer=%" PRIu64
                 " revision=%" PRIu64 " reason=invalid_execution_owner",
                 update.state.producer_instance_id,
                 update.state.obstacle_snapshot_revision);
  }
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
  MemoryStatusIngestionResult3D ingestion;
  {
    const std::scoped_lock lock{execution_evidence_commit_mutex_, input_mutex_};
    ingestion = world_pipeline_->ingestMemoryStatus(
        observation, statusAnnouncesRawUpdate(message), now_ns, config,
        message.header.frame_id == frame_id_);
    if (ingestion.execution_revocation_required) {
      invalidateAppliedControlWitnessLocked();
      requestExecutionRevocation(ProductionMppiExecutionReason::kUnavailableWorld);
    }
  }
  if (ingestion.synchronized_update.has_value() &&
      ingestion.synchronized_update->accepted()) {
    queueRawWorld3D(*ingestion.synchronized_update, 0.0);
  }
}

} // namespace drone_city_nav
