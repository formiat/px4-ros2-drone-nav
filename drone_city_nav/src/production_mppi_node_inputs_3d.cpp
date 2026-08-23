#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <ranges>
#include <tuple>
#include <utility>

#include "production_mppi_node.hpp"

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

void mergeUpdateProvenance(RawObstacleGridUpdate3D& destination,
                           const RawObstacleGridUpdate3D& source) {
  destination.full_reset = destination.full_reset || source.full_reset;
  mergeDirtyChunks(destination.dirty_chunks, source.dirty_chunks);
}

} // namespace

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
          const RawObstacleGridUpdate3D snapshot_update = std::move(update);
          update = pending_update;
          mergeUpdateProvenance(update, snapshot_update);
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
  queueRawWorld3D(update, reconstruction_ms);
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
  queueRawWorld3D(update, reconstruction_ms);
}

void ProductionMppiNode::queueRawWorld3D(const RawObstacleGridUpdate3D& update,
                                         const double reconstruction_ms) {
  auto world = std::make_shared<ProductionMppiRawWorld3D>(ProductionMppiRawWorld3D{
      .version =
          RawMapVersion{
              .producer_instance_id = update.state.producer_instance_id,
              .base_snapshot_revision = update.state.base_snapshot_revision,
              .revision = update.state.obstacle_snapshot_revision,
          },
      .ready_stamp_ns = get_clock()->now().nanoseconds(),
      .reconstruction_ms = reconstruction_ms,
      .occupancy = update.state.occupancy,
      .dirty_chunks = update.dirty_chunks,
      .full_reset = update.full_reset,
  });
  no_static_raw_updates_.fetch_add(1U, std::memory_order_relaxed);
  std::shared_ptr<const ProductionMppiRawWorld3D> immutable_world;
  {
    const std::scoped_lock lock{raw_queue_mutex_};
    const auto& pending = raw_world_scheduler_3d_.pending();
    if (pending.has_value() && *pending) {
      dropped_raw_snapshots_.fetch_add(1U, std::memory_order_relaxed);
      world->full_reset = world->full_reset || (*pending)->full_reset;
      mergeDirtyChunks(world->dirty_chunks, (*pending)->dirty_chunks);
    }
    immutable_world = world;
    static_cast<void>(raw_world_scheduler_3d_.submit(immutable_world));
    latest_raw_world_3d_.store(immutable_world, std::memory_order_release);
  }
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

void ProductionMppiNode::queueLatestObservedWorldForPose(
    const ProductionMppiNavigation& navigation) {
  if (use_static_map_ || !navigation.valid ||
      no_static_world_model_ != ProductionNoStaticWorldModel::kObservedOccupancy3D) {
    return;
  }
  const std::shared_ptr<const ProductionMppiRawWorld3D> raw_world =
      latest_raw_world_3d_.load(std::memory_order_acquire);
  if (!raw_world || !raw_world->occupancy) {
    return;
  }

  bool local_world_required{false};
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    if (!prepared_esdf_ || !prepared_esdf_->distances_m ||
        prepared_esdf_->producer_instance_id !=
            raw_world->version.producer_instance_id ||
        prepared_esdf_->grid.depth <= 1 || !prepared_esdf_->grid.outside_is_unknown) {
      local_world_required = true;
    } else {
      const GridBounds3D local_bounds{
          .origin_x = prepared_esdf_->grid.origin_x_m,
          .origin_y = prepared_esdf_->grid.origin_y_m,
          .origin_z = prepared_esdf_->grid.origin_z_m,
          .resolution_m = prepared_esdf_->grid.resolution_m,
          .width_cells = prepared_esdf_->grid.width,
          .height_cells = prepared_esdf_->grid.height,
          .depth_cells = prepared_esdf_->grid.depth,
      };
      const Point3 position{navigation.state.x, navigation.state.y, navigation.state.z};
      local_world_required =
          localObservedEsdfNeedsRecenter(local_bounds, raw_world->occupancy->bounds(),
                                         position, no_static_3d_esdf_window_);
    }
  }
  if (!local_world_required) {
    return;
  }

  {
    const std::scoped_lock lock{raw_queue_mutex_};
    static_cast<void>(raw_world_scheduler_3d_.submit(raw_world, true));
  }
  raw_queue_condition_.notify_all();
}

} // namespace drone_city_nav
