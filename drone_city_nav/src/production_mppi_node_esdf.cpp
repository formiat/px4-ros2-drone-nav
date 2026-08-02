#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/ros_conversions.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] GridBounds3D localStaticEsdfBounds(const OccupancyGrid3D& occupancy,
                                                 const Point3& start,
                                                 const Point3& goal,
                                                 const double planning_distance_m,
                                                 const double halo_m) {
  const GridBounds3D& world = occupancy.bounds();
  const double horizontal_distance = std::hypot(goal.x - start.x, goal.y - start.y);
  const double ratio = horizontal_distance > planning_distance_m
                           ? planning_distance_m / horizontal_distance
                           : 1.0;
  const Point2 endpoint{std::lerp(start.x, goal.x, ratio),
                        std::lerp(start.y, goal.y, ratio)};
  const auto clamp_cell = [](const int value, const int maximum) {
    return std::clamp(value, 0, maximum - 1);
  };
  const int min_x =
      clamp_cell(static_cast<int>(std::floor(
                     (std::min(start.x, endpoint.x) - halo_m - world.origin_x) /
                     world.resolution_m)),
                 world.width_cells);
  const int max_x =
      clamp_cell(static_cast<int>(std::floor(
                     (std::max(start.x, endpoint.x) + halo_m - world.origin_x) /
                     world.resolution_m)),
                 world.width_cells);
  const int min_y =
      clamp_cell(static_cast<int>(std::floor(
                     (std::min(start.y, endpoint.y) - halo_m - world.origin_y) /
                     world.resolution_m)),
                 world.height_cells);
  const int max_y =
      clamp_cell(static_cast<int>(std::floor(
                     (std::max(start.y, endpoint.y) + halo_m - world.origin_y) /
                     world.resolution_m)),
                 world.height_cells);
  return GridBounds3D{
      .origin_x = world.origin_x + static_cast<double>(min_x) * world.resolution_m,
      .origin_y = world.origin_y + static_cast<double>(min_y) * world.resolution_m,
      .origin_z = world.origin_z,
      .resolution_m = world.resolution_m,
      .width_cells = max_x - min_x + 1,
      .height_cells = max_y - min_y + 1,
      .depth_cells = world.depth_cells,
  };
}

} // namespace

void ProductionMppiNode::esdfWorker(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    msg::RawObstacleSnapshot::ConstSharedPtr snapshot;
    {
      std::unique_lock lock{raw_queue_mutex_};
      raw_queue_condition_.wait(lock, stop_token,
                                [this]() { return pending_raw_snapshot_ != nullptr; });
      if (stop_token.stop_requested()) {
        return;
      }
      snapshot = std::exchange(pending_raw_snapshot_, nullptr);
    }
    if (!snapshot) {
      continue;
    }
    const std::int64_t source_stamp_ns = get_clock()->now().nanoseconds();
    if (use_static_map_ && static_occupancy_3d_) {
      const std::uint64_t release_generation =
          guide_release_generation_.load(std::memory_order_acquire);
      const std::uint64_t roi_refresh_generation =
          static_roi_refresh_request_generation_.load(std::memory_order_acquire);
      std::optional<ProductionMppiPreparedEsdf> active_prepared;
      {
        const std::scoped_lock lock{esdf_state_mutex_};
        active_prepared = prepared_esdf_;
      }
      bool proactive_roi_refresh =
          roi_refresh_generation > static_roi_refresh_completed_generation_ &&
          release_generation == static_guide_release_generation_ && active_prepared &&
          active_prepared->global_guide_generation == roi_refresh_generation;
      double static_build_ms = active_prepared ? active_prepared->build_ms : 0.0;
      if (roi_refresh_generation > static_roi_refresh_completed_generation_ &&
          !proactive_roi_refresh) {
        static_roi_refresh_completed_generation_ = roi_refresh_generation;
        finishStaticRouteExtension(roi_refresh_generation);
      }
      if (static_esdf_3d_) {
        if (release_generation == static_guide_release_generation_ &&
            !proactive_roi_refresh) {
          const std::scoped_lock lock{esdf_state_mutex_};
          if (prepared_esdf_) {
            prepared_esdf_->ready_stamp_ns = source_stamp_ns;
            prepared_esdf_->source_stamp_ns = source_stamp_ns;
            prepared_esdf_->producer_instance_id = snapshot->producer_instance_id;
          }
          continue;
        }
        if (!proactive_roi_refresh) {
          static_esdf_3d_.reset();
          active_prepared.reset();
        }
      }
      if (!static_esdf_3d_ || proactive_roi_refresh) {
        ProductionMppiNavigation navigation;
        {
          const std::scoped_lock lock{input_mutex_};
          navigation = navigation_;
        }
        if (!navigation.valid) {
          continue;
        }
        const GridBounds3D local_bounds = localStaticEsdfBounds(
            *static_occupancy_3d_,
            Point3{navigation.state.x, navigation.state.y, navigation.state.z},
            mission_goal_, lattice_3d_config_.planning_goal_distance_m, 40.0);
        const DistanceField3D field = DistanceField3D::buildLocal(
            *static_occupancy_3d_, local_bounds,
            static_cast<double>(mppi_config_.risk.preferred_distance_m) + 20.0);
        const GridBounds3D& bounds = field.bounds();
        static_esdf_grid_ = mppi::EsdfGrid{bounds.width_cells,
                                           bounds.height_cells,
                                           static_cast<float>(bounds.resolution_m),
                                           static_cast<float>(bounds.origin_x),
                                           static_cast<float>(bounds.origin_y),
                                           bounds.depth_cells,
                                           static_cast<float>(bounds.origin_z)};
        static_esdf_3d_ = std::make_shared<const std::vector<float>>(
            field.distancesM().begin(), field.distancesM().end());
        static_guide_release_generation_ = release_generation;
        static_build_ms = field.stats().duration_ms;
        RCLCPP_INFO(get_logger(),
                    "STATIC_ESDF3D_READY build_ms=%.2f voxels=%zu "
                    "dimensions=%dx%dx%d proactive_extension=%s "
                    "base_generation=%" PRIu64,
                    field.stats().duration_ms, field.stats().voxel_count,
                    local_bounds.width_cells, local_bounds.height_cells,
                    local_bounds.depth_cells, proactive_roi_refresh ? "true" : "false",
                    proactive_roi_refresh ? roi_refresh_generation : 0U);
      }
      const mppi::EsdfUploadResult upload = engine_->updateEsdf(mppi::EsdfSnapshot{
          static_esdf_grid_, *static_esdf_3d_, static_occupancy_3d_->fingerprint()});
      if (!upload.accepted) {
        continue;
      }
      if (proactive_roi_refresh) {
        static_roi_refresh_completed_generation_ = roi_refresh_generation;
      }
      ProductionMppiPreparedEsdf prepared;
      if (proactive_roi_refresh && active_prepared) {
        prepared = *active_prepared;
      }
      prepared.producer_instance_id = snapshot->producer_instance_id;
      prepared.revision = static_occupancy_3d_->fingerprint();
      prepared.source_stamp_ns = source_stamp_ns;
      prepared.ready_stamp_ns = get_clock()->now().nanoseconds();
      prepared.build_ms = static_build_ms;
      prepared.upload_ms = upload.upload_ms;
      prepared.grid = static_esdf_grid_;
      prepared.distances_m = static_esdf_3d_;
      prepared.channel_edges = static_channel_edges_;
      prepared.static_route_extension_request = proactive_roi_refresh;
      prepared.static_route_extension_base_generation =
          proactive_roi_refresh ? roi_refresh_generation : 0U;
      {
        const std::scoped_lock lock{esdf_state_mutex_};
        prepared_esdf_ = prepared;
      }
      {
        const std::scoped_lock lock{guide_queue_mutex_};
        if (pending_guide_world_) {
          dropped_guide_worlds_.fetch_add(1U, std::memory_order_relaxed);
        }
        pending_guide_world_ =
            std::make_shared<const ProductionMppiPreparedEsdf>(prepared);
      }
      guide_queue_condition_.notify_all();
      RCLCPP_INFO(get_logger(),
                  "PRODUCTION_MPPI_ESDF3D revision=%" PRIu64
                  " upload_ms=%.2f dimensions=%dx%dx%d",
                  prepared.revision, prepared.upload_ms, prepared.grid.width,
                  prepared.grid.height, prepared.grid.depth);
      continue;
    }
    RawOccupancyGridFromRosResult conversion =
        rawOccupancyGridFromRos(snapshot->grid, RawOccupancyGridFromRosConfig{100, 0});
    if (!conversion.grid.has_value()) {
      RCLCPP_WARN(get_logger(),
                  "PRODUCTION_MPPI_ESDF rejected revision=%" PRIu64
                  " reason=invalid_raw_grid",
                  snapshot->obstacle_snapshot_revision);
      continue;
    }
    const auto build_started = std::chrono::steady_clock::now();
    const DistanceField2D field = DistanceField2D::build(
        *conversion.grid,
        static_cast<double>(mppi_config_.risk.preferred_distance_m) + 20.0,
        DistanceFieldSource::kOccupied);
    const double build_ms = std::chrono::duration<double, std::milli>(
                                std::chrono::steady_clock::now() - build_started)
                                .count();
    const auto conversion_started = std::chrono::steady_clock::now();
    std::vector<float> distances;
    distances.reserve(field.distancesM().size());
    for (const double distance_m : field.distancesM()) {
      distances.push_back(static_cast<float>(distance_m));
    }
    const double conversion_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  conversion_started)
            .count();
    const GridBounds& bounds = field.bounds();
    const mppi::EsdfGrid grid{bounds.width_cells, bounds.height_cells,
                              static_cast<float>(bounds.resolution_m),
                              static_cast<float>(bounds.origin_x),
                              static_cast<float>(bounds.origin_y)};
    const mppi::EsdfUploadResult upload = engine_->updateEsdf(
        mppi::EsdfSnapshot{grid, distances, snapshot->obstacle_snapshot_revision});
    if (!upload.accepted) {
      continue;
    }

    ProductionMppiPreparedEsdf prepared;
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      if (prepared_esdf_.has_value()) {
        prepared = *prepared_esdf_;
      }
    }
    prepared.producer_instance_id = snapshot->producer_instance_id;
    prepared.revision = snapshot->obstacle_snapshot_revision;
    prepared.source_stamp_ns = source_stamp_ns;
    prepared.ready_stamp_ns = get_clock()->now().nanoseconds();
    prepared.build_ms = build_ms;
    prepared.conversion_ms = conversion_ms;
    prepared.upload_ms = upload.upload_ms;
    prepared.grid = grid;
    prepared.distances_m =
        std::make_shared<const std::vector<float>>(std::move(distances));
    prepared.lattice_search_performed = false;
    prepared.lattice_continuation_attempt = 0U;

    {
      const std::scoped_lock lock{esdf_state_mutex_};
      prepared_esdf_ = prepared;
    }
    auto guide_world = std::make_shared<const ProductionMppiPreparedEsdf>(prepared);
    {
      const std::scoped_lock lock{guide_queue_mutex_};
      if (pending_guide_world_) {
        dropped_guide_worlds_.fetch_add(1U, std::memory_order_relaxed);
      }
      pending_guide_world_ = std::move(guide_world);
    }
    guide_queue_condition_.notify_all();

    RCLCPP_INFO(
        get_logger(),
        "PRODUCTION_MPPI_ESDF revision=%" PRIu64
        " build_ms=%.2f conversion_ms=%.2f upload_ms=%.2f "
        "raw_to_ready_ms=%.2f dropped_raw=%" PRIu64 " dropped_guide_worlds=%" PRIu64,
        prepared.revision, prepared.build_ms, prepared.conversion_ms,
        prepared.upload_ms,
        static_cast<double>(prepared.ready_stamp_ns - prepared.source_stamp_ns) / 1.0e6,
        dropped_raw_snapshots_.load(std::memory_order_relaxed),
        dropped_guide_worlds_.load(std::memory_order_relaxed));
  }
}

} // namespace drone_city_nav
