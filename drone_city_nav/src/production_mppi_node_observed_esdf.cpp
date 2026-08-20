#include "drone_city_nav/observed_esdf_3d.hpp"

#include <chrono>
#include <cinttypes>
#include <memory>
#include <utility>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
void ProductionMppiNode::processObservedEsdf3D(
    const ProductionMppiRawWorld3D& raw_world) {
  const std::shared_ptr<const ObservedOccupancyGrid3D> occupancy = raw_world.occupancy;
  if (!occupancy) {
    RCLCPP_WARN(get_logger(),
                "PRODUCTION_MPPI_ESDF3D_ONLINE rejected revision=%" PRIu64
                " reason=unavailable_observed_grid",
                raw_world.revision);
    return;
  }

  ProductionMppiNavigation navigation;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
  }
  if (!navigation.valid) {
    return;
  }

  std::optional<ProductionMppiPreparedEsdf> active_prepared;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    active_prepared = prepared_esdf_;
  }
  const GridBounds3D& world_bounds = occupancy->bounds();
  const Point3 position{navigation.state.x, navigation.state.y, navigation.state.z};
  GridBounds3D local_bounds;
  bool recenter = true;
  if (active_prepared && active_prepared->distances_m &&
      active_prepared->grid.depth > 1 && active_prepared->grid.outside_is_unknown) {
    local_bounds = GridBounds3D{
        .origin_x = active_prepared->grid.origin_x_m,
        .origin_y = active_prepared->grid.origin_y_m,
        .origin_z = active_prepared->grid.origin_z_m,
        .resolution_m = active_prepared->grid.resolution_m,
        .width_cells = active_prepared->grid.width,
        .height_cells = active_prepared->grid.height,
        .depth_cells = active_prepared->grid.depth,
    };
    recenter = localObservedEsdfNeedsRecenter(local_bounds, world_bounds, position,
                                              no_static_3d_esdf_recenter_margin_m_);
  }
  if (recenter) {
    local_bounds = selectLocalObservedEsdfBounds(world_bounds, position,
                                                 no_static_3d_esdf_half_extent_m_);
  }

  const std::uint64_t local_fingerprint =
      observedOccupancyFingerprint(*occupancy, local_bounds);
  const bool local_occupancy_unchanged =
      active_prepared && active_prepared->distances_m && !recenter &&
      active_prepared->source_occupied_fingerprint == local_fingerprint;
  const auto build_started_at = std::chrono::steady_clock::now();
  const bool first_build =
      no_static_esdf_last_build_time_ == std::chrono::steady_clock::time_point{};
  const bool build_rate_due =
      first_build ||
      std::chrono::duration<double>(build_started_at - no_static_esdf_last_build_time_)
              .count() >= 1.0 / no_static_3d_esdf_update_rate_hz_;
  if (local_occupancy_unchanged || (active_prepared && !recenter && !build_rate_due)) {
    if (!local_occupancy_unchanged) {
      no_static_esdf_throttled_updates_.fetch_add(1U, std::memory_order_relaxed);
    } else {
      const std::scoped_lock lock{esdf_state_mutex_};
      if (prepared_esdf_ && prepared_esdf_->revision == active_prepared->revision) {
        prepared_esdf_->source_stamp_ns = raw_world.ready_stamp_ns;
        prepared_esdf_->ready_stamp_ns = raw_world.ready_stamp_ns;
      }
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NO_STATIC_ESDF3D_DEFERRED raw_revision=%" PRIu64
        " reason=%s reconstruction_ms=%.2f raw_updates=%" PRIu64 " builds=%" PRIu64
        " throttled=%" PRIu64,
        raw_world.revision,
        local_occupancy_unchanged ? "observed_unchanged" : "rate_limited",
        raw_world.reconstruction_ms,
        no_static_raw_updates_.load(std::memory_order_relaxed),
        no_static_esdf_builds_.load(std::memory_order_relaxed),
        no_static_esdf_throttled_updates_.load(std::memory_order_relaxed));
    return;
  }

  ObservedEsdf3D field = buildObservedEsdf3D(
      *occupancy, local_bounds,
      static_cast<double>(mppi_config_.risk.preferred_distance_m) + 20.0,
      planning_worker_pool_.get());
  const mppi::EsdfUploadResult upload = engine_->updateEsdf(
      mppi::EsdfSnapshot{field.grid, field.distances_m, field.occupancy_fingerprint});
  if (!upload.accepted) {
    return;
  }
  auto host_distances =
      std::make_shared<const std::vector<float>>(std::move(field.distances_m));
  no_static_esdf_last_build_time_ = build_started_at;
  no_static_esdf_builds_.fetch_add(1U, std::memory_order_relaxed);

  ProductionMppiPreparedEsdf prepared;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    if (prepared_esdf_) {
      prepared = *prepared_esdf_;
    }
  }
  prepared.producer_instance_id = raw_world.producer_instance_id;
  prepared.revision = field.occupancy_fingerprint;
  prepared.source_occupied_fingerprint = field.occupancy_fingerprint;
  prepared.source_stamp_ns = raw_world.ready_stamp_ns;
  prepared.ready_stamp_ns = get_clock()->now().nanoseconds();
  prepared.build_ms =
      field.stats.distance_field.duration_ms + field.stats.classification_ms;
  prepared.esdf_x_pass_ms = field.stats.distance_field.x_pass_ms;
  prepared.esdf_y_pass_ms = field.stats.distance_field.y_pass_ms;
  prepared.esdf_z_pass_ms = field.stats.distance_field.z_pass_ms;
  prepared.esdf_finalize_ms = field.stats.distance_field.finalize_ms;
  prepared.conversion_ms = raw_world.reconstruction_ms + field.stats.classification_ms;
  prepared.upload_ms = upload.upload_ms;
  prepared.grid = field.grid;
  prepared.distances_m = host_distances;
  prepared.raw_occupancy.reset();
  prepared.observed_occupancy = field.local_occupancy;
  if (const std::shared_ptr<const ProductionNavigationObjective> objective =
          navigationObjective()) {
    prepared.search_objective = makeStaticRouteObjective(*objective);
  }
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
  if (!world_ready_.exchange(true, std::memory_order_acq_rel)) {
    publishWorldReadiness(true);
  }

  RCLCPP_INFO(get_logger(),
              "PRODUCTION_MPPI_ESDF3D_ONLINE revision=%" PRIu64 " raw_revision=%" PRIu64
              " build_ms=%.2f classify_ms=%.2f upload_ms=%.2f dimensions=%dx%dx%d "
              "known=%zu free=%zu occupied=%zu unknown=%zu recenter=%s builds=%" PRIu64
              " throttled=%" PRIu64 " dropped_raw=%" PRIu64,
              prepared.revision, raw_world.revision, prepared.build_ms,
              field.stats.classification_ms, prepared.upload_ms, prepared.grid.width,
              prepared.grid.height, prepared.grid.depth, field.stats.known_voxels,
              field.stats.free_voxels, field.stats.occupied_voxels,
              field.stats.unknown_voxels, recenter ? "true" : "false",
              no_static_esdf_builds_.load(std::memory_order_relaxed),
              no_static_esdf_throttled_updates_.load(std::memory_order_relaxed),
              dropped_raw_snapshots_.load(std::memory_order_relaxed));
}

} // namespace drone_city_nav
