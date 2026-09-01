#include <chrono>
#include <cinttypes>
#include <cstdint>
#include <memory>
#include <mutex>

#include "production_mppi_node.hpp"
#include "raw_world_ingress_ros_3d.hpp"

namespace drone_city_nav {

void ProductionMppiNode::onRawObstacleSnapshot3D(
    msg::RawObstacleSnapshot3D::ConstSharedPtr message) {
  if (config_.world.use_static_map) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  RawWorldIngestionResult3D ingestion;
  {
    // Raw reconstruction is serialized by RawWorldIngressRos3D and belongs to
    // the execution-evidence domain.  Keep PX4 navigation and applied-control
    // callbacks live while the large occupancy snapshot is reconstructed.
    const auto evidence_lock = evidence_boundary_.evidence();
    ingestion = raw_world_ingress_->ingestRawSnapshot(*message, receive_stamp_ns);
    if (ingestion.execution_revocation_required) {
      const auto input_lock = evidence_boundary_.input();
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
  if (config_.world.use_static_map) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  const std::int64_t receive_stamp_ns = get_clock()->now().nanoseconds();
  RawWorldIngestionResult3D ingestion;
  {
    const auto evidence_lock = evidence_boundary_.evidence();
    ingestion = raw_world_ingress_->ingestRawDelta(*message, receive_stamp_ns);
    if (ingestion.execution_revocation_required) {
      const auto input_lock = evidence_boundary_.input();
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
  RawWorldCommitResult3D result;
  {
    // Keep raw-world replacement in the same outer transaction as execution
    // evidence publication until that integration gate moves into the execution
    // service facade. Navigation and control feedback are independent inputs and
    // must remain live while the immutable raw-world owner is constructed.
    const auto evidence_lock = evidence_boundary_.evidence();
    result =
        raw_world_ingress_->commitRawUpdate(update, reconstruction_ms, ready_stamp_ns);
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
  if (config_.world.use_static_map) {
    return;
  }
  const std::int64_t now_ns = get_clock()->now().nanoseconds();
  MemoryStatusIngestionResult3D ingestion;
  {
    const auto evidence_lock = evidence_boundary_.evidence();
    ingestion = raw_world_ingress_->ingestMemoryStatus(message, now_ns);
    if (ingestion.execution_revocation_required) {
      const auto input_lock = evidence_boundary_.input();
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
