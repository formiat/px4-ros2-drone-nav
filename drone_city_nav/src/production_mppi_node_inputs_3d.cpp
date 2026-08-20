#include <chrono>
#include <cinttypes>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {

void ProductionMppiNode::onRawObstacleSnapshot3D(
    msg::RawObstacleSnapshot3D::ConstSharedPtr message) {
  if (use_static_map_ ||
      no_static_world_model_ != ProductionNoStaticWorldModel::kObservedOccupancy3D) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  RawObstacleGridUpdate3D update;
  {
    const std::scoped_lock lock{raw_reconstruction_mutex_};
    update = raw_delta_accumulator_3d_.apply(*message);
    if (update.accepted()) {
      msg::RawObstacleDelta3D::ConstSharedPtr pending =
          std::exchange(pending_raw_delta_3d_, nullptr);
      if (pending &&
          pending->producer_instance_id == update.state.producer_instance_id &&
          pending->base_snapshot_revision == update.state.base_snapshot_revision &&
          pending->obstacle_snapshot_revision >
              update.state.obstacle_snapshot_revision) {
        const RawObstacleGridUpdate3D pending_update =
            raw_delta_accumulator_3d_.apply(*pending);
        if (pending_update.accepted()) {
          update = pending_update;
        }
      }
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
  queueRawWorld3D(update.state, reconstruction_ms);
}

void ProductionMppiNode::onRawObstacleDelta3D(
    msg::RawObstacleDelta3D::ConstSharedPtr message) {
  if (use_static_map_ ||
      no_static_world_model_ != ProductionNoStaticWorldModel::kObservedOccupancy3D) {
    return;
  }
  const auto started = std::chrono::steady_clock::now();
  RawObstacleGridUpdate3D update;
  {
    const std::scoped_lock lock{raw_reconstruction_mutex_};
    update = raw_delta_accumulator_3d_.apply(*message);
    if (update.status == RawObstacleGridUpdateStatus3D::kBaseUnavailable &&
        (!pending_raw_delta_3d_ ||
         message->obstacle_snapshot_revision >
             pending_raw_delta_3d_->obstacle_snapshot_revision)) {
      pending_raw_delta_3d_ = message;
    }
  }
  if (!update.accepted()) {
    if (update.status == RawObstacleGridUpdateStatus3D::kInvalidMessage) {
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
  queueRawWorld3D(update.state, reconstruction_ms);
}

void ProductionMppiNode::queueRawWorld3D(const RawObstacleGridState3D& state,
                                         const double reconstruction_ms) {
  auto world =
      std::make_shared<const ProductionMppiRawWorld3D>(ProductionMppiRawWorld3D{
          .producer_instance_id = state.producer_instance_id,
          .base_snapshot_revision = state.base_snapshot_revision,
          .revision = state.obstacle_snapshot_revision,
          .ready_stamp_ns = get_clock()->now().nanoseconds(),
          .reconstruction_ms = reconstruction_ms,
          .occupancy = state.occupancy,
      });
  latest_raw_world_3d_.store(world, std::memory_order_release);
  no_static_raw_updates_.fetch_add(1U, std::memory_order_relaxed);
  {
    const std::scoped_lock lock{raw_queue_mutex_};
    if (pending_raw_world_3d_) {
      dropped_raw_snapshots_.fetch_add(1U, std::memory_order_relaxed);
    }
    pending_raw_world_3d_ = std::move(world);
  }
  raw_queue_condition_.notify_all();
}

} // namespace drone_city_nav
