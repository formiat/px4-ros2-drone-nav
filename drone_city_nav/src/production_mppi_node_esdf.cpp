#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/local_esdf_2d.hpp"

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

[[nodiscard]] bool sameStaticEsdfGrid(const mppi::EsdfGrid& grid,
                                      const GridBounds3D& bounds) noexcept {
  constexpr double tolerance{1.0e-6};
  return grid.width == bounds.width_cells && grid.height == bounds.height_cells &&
         grid.depth == bounds.depth_cells &&
         std::abs(static_cast<double>(grid.resolution_m) - bounds.resolution_m) <=
             tolerance &&
         std::abs(static_cast<double>(grid.origin_x_m) - bounds.origin_x) <=
             tolerance &&
         std::abs(static_cast<double>(grid.origin_y_m) - bounds.origin_y) <=
             tolerance &&
         std::abs(static_cast<double>(grid.origin_z_m) - bounds.origin_z) <= tolerance;
}

} // namespace

void ProductionMppiNode::esdfWorker(const std::stop_token stop_token) {
  while (!stop_token.stop_requested()) {
    std::shared_ptr<const ProductionMppiRawWorld2D> raw_world;
    bool static_work{false};
    {
      std::unique_lock lock{raw_queue_mutex_};
      raw_queue_condition_.wait(lock, stop_token, [this]() {
        return pending_raw_world_ != nullptr || pending_static_esdf_work_;
      });
      if (stop_token.stop_requested()) {
        return;
      }
      if (use_static_map_) {
        static_work = std::exchange(pending_static_esdf_work_, false);
        static_esdf_work_in_progress_ = static_work;
      } else {
        raw_world = std::exchange(pending_raw_world_, nullptr);
      }
    }
    if ((!use_static_map_ && !raw_world) || (use_static_map_ && !static_work)) {
      continue;
    }
    const std::int64_t source_stamp_ns =
        use_static_map_ ? get_clock()->now().nanoseconds() : raw_world->ready_stamp_ns;
    if (use_static_map_ && !static_occupancy_3d_) {
      RCLCPP_ERROR(get_logger(),
                   "STATIC_ESDF3D rejected reason=resident_occupancy_unavailable");
      completeStaticEsdfWork(false);
      continue;
    }
    if (use_static_map_) {
      const std::shared_ptr<const ProductionNavigationObjective> objective =
          navigationObjective();
      const StaticRouteRoiRefreshRequest roi_refresh =
          static_roi_refresh_lifecycle_.latest();
      std::optional<ProductionMppiPreparedEsdf> active_prepared;
      {
        const std::scoped_lock lock{esdf_state_mutex_};
        active_prepared = prepared_esdf_;
      }
      const bool roi_refresh_pending =
          static_roi_refresh_lifecycle_.pending(roi_refresh);
      bool proactive_roi_refresh =
          roi_refresh_pending && active_prepared &&
          active_prepared->global_guide_generation == roi_refresh.base_route_generation;
      double static_build_ms = active_prepared ? active_prepared->build_ms : 0.0;
      double static_x_pass_ms = active_prepared ? active_prepared->esdf_x_pass_ms : 0.0;
      double static_y_pass_ms = active_prepared ? active_prepared->esdf_y_pass_ms : 0.0;
      double static_z_pass_ms = active_prepared ? active_prepared->esdf_z_pass_ms : 0.0;
      double static_finalize_ms =
          active_prepared ? active_prepared->esdf_finalize_ms : 0.0;
      if (roi_refresh_pending && !proactive_roi_refresh) {
        static_roi_refresh_lifecycle_.complete(roi_refresh.sequence);
        if (roi_refresh.purpose ==
            StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective) {
          finishStaticRouteReplan(roi_refresh.base_route_generation);
        } else {
          finishStaticRouteExtension(roi_refresh.base_route_generation);
        }
      }
      if (static_esdf_3d_ && static_esdf_uploaded_ && !proactive_roi_refresh &&
          active_prepared) {
        const bool readiness_transition = !world_ready_.load(std::memory_order_acquire);
        completeStaticEsdfWork(true);
        if (readiness_transition) {
          publishWorldReadiness(true);
        }
        continue;
      }
      bool reused_uploaded_roi{false};
      if (!static_esdf_3d_ || proactive_roi_refresh || !static_esdf_uploaded_) {
        ProductionMppiNavigation navigation;
        {
          const std::scoped_lock lock{input_mutex_};
          navigation = navigation_;
        }
        if (!navigation.valid) {
          if (proactive_roi_refresh) {
            static_roi_refresh_lifecycle_.complete(roi_refresh.sequence);
            if (roi_refresh.purpose ==
                StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective) {
              finishStaticRouteReplan(roi_refresh.base_route_generation);
            } else {
              finishStaticRouteExtension(roi_refresh.base_route_generation);
            }
          }
          completeStaticEsdfWork(false);
          continue;
        }
        const Point3 mission_goal = objective ? objective->goal : mission_goal_;
        const GridBounds3D requested_bounds = localStaticEsdfBounds(
            *static_occupancy_3d_,
            Point3{navigation.state.x, navigation.state.y, navigation.state.z},
            mission_goal, lattice_3d_config_.planning_goal_distance_m, 40.0);
        const GridBounds3D local_bounds = StaticEsdfCache::alignRegionToChunks(
            static_occupancy_3d_->bounds(), requested_bounds);
        const double maximum_distance_m =
            static_cast<double>(mppi_config_.risk.preferred_distance_m) + 20.0;
        reused_uploaded_roi = static_esdf_uploaded_ && static_esdf_3d_ &&
                              sameStaticEsdfGrid(static_esdf_grid_, local_bounds);
        if (!reused_uploaded_roi) {
          bool precomputed_cache_used{false};
          StaticEsdfCacheExtractionStats cache_stats;
          std::optional<DistanceField3D> cached_field;
          if (static_esdf_cache_ && static_esdf_cache_->compatibleWith(
                                        *static_occupancy_3d_, maximum_distance_m)) {
            try {
              StaticEsdfCacheExtraction extraction =
                  static_esdf_cache_->extract(local_bounds, maximum_distance_m);
              cache_stats = extraction.stats;
              cached_field.emplace(std::move(extraction.field));
              precomputed_cache_used = true;
            } catch (const std::exception& error) {
              RCLCPP_ERROR(get_logger(),
                           "STATIC_ESDF_CACHE_FALLBACK reason=extract_failed error=%s",
                           error.what());
              static_esdf_cache_.reset();
            }
          }
          const DistanceField3D field =
              cached_field
                  ? std::move(*cached_field)
                  : DistanceField3D::buildLocal(*static_occupancy_3d_, local_bounds,
                                                maximum_distance_m,
                                                planning_worker_pool_.get());
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
          static_esdf_uploaded_ = false;
          static_build_ms = field.stats().duration_ms;
          static_x_pass_ms = field.stats().x_pass_ms;
          static_y_pass_ms = field.stats().y_pass_ms;
          static_z_pass_ms = field.stats().z_pass_ms;
          static_finalize_ms = field.stats().finalize_ms;
          RCLCPP_INFO(
              get_logger(),
              "STATIC_ESDF3D_READY source=%s build_ms=%.2f voxels=%zu "
              "dimensions=%dx%dx%d proactive_refresh=%s refresh_purpose=%s "
              "base_generation=%" PRIu64
              " cache_requested_chunks=%zu cache_decoded_chunks=%zu "
              "cache_hits=%zu cache_resident_chunks=%zu cache_decode_ms=%.2f "
              "cache_copy_ms=%.2f cache_finite_voxels=%zu",
              precomputed_cache_used ? "precomputed_cache" : "runtime_edt",
              field.stats().duration_ms, field.stats().voxel_count,
              local_bounds.width_cells, local_bounds.height_cells,
              local_bounds.depth_cells, proactive_roi_refresh ? "true" : "false",
              proactive_roi_refresh &&
                      roi_refresh.purpose ==
                          StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective
                  ? "tracking_objective"
                  : "route_extension",
              proactive_roi_refresh ? roi_refresh.base_route_generation : 0U,
              cache_stats.requested_chunks, cache_stats.decoded_chunks,
              cache_stats.decoded_chunk_cache_hits, cache_stats.resident_decoded_chunks,
              cache_stats.decode_ms, cache_stats.copy_ms, cache_stats.finite_voxels);
        } else {
          RCLCPP_INFO(get_logger(),
                      "STATIC_ESDF3D_REUSED reason=stable_chunk_roi "
                      "dimensions=%dx%dx%d base_generation=%" PRIu64,
                      static_esdf_grid_.width, static_esdf_grid_.height,
                      static_esdf_grid_.depth,
                      proactive_roi_refresh ? roi_refresh.base_route_generation : 0U);
        }
      }
      mppi::EsdfUploadResult upload{true, 0.0, static_occupancy_3d_->fingerprint()};
      if (!reused_uploaded_roi) {
        upload = engine_->updateEsdf(mppi::EsdfSnapshot{
            static_esdf_grid_, *static_esdf_3d_, static_occupancy_3d_->fingerprint()});
      }
      if (!upload.accepted) {
        if (proactive_roi_refresh) {
          static_roi_refresh_lifecycle_.complete(roi_refresh.sequence);
          if (roi_refresh.purpose ==
              StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective) {
            finishStaticRouteReplan(roi_refresh.base_route_generation);
          } else {
            finishStaticRouteExtension(roi_refresh.base_route_generation);
          }
        }
        completeStaticEsdfWork(false);
        continue;
      }
      static_esdf_uploaded_ = true;
      if (proactive_roi_refresh) {
        static_roi_refresh_lifecycle_.complete(roi_refresh.sequence);
      }
      ProductionMppiPreparedEsdf prepared;
      if (proactive_roi_refresh && active_prepared) {
        prepared = *active_prepared;
      }
      prepared.producer_instance_id =
          active_prepared ? active_prepared->producer_instance_id : 0U;
      prepared.revision = static_occupancy_3d_->fingerprint();
      prepared.source_stamp_ns = source_stamp_ns;
      prepared.ready_stamp_ns = get_clock()->now().nanoseconds();
      prepared.build_ms = static_build_ms;
      prepared.esdf_x_pass_ms = static_x_pass_ms;
      prepared.esdf_y_pass_ms = static_y_pass_ms;
      prepared.esdf_z_pass_ms = static_z_pass_ms;
      prepared.esdf_finalize_ms = static_finalize_ms;
      prepared.upload_ms = upload.upload_ms;
      prepared.grid = static_esdf_grid_;
      prepared.distances_m = static_esdf_3d_;
      prepared.channel_edges = static_channel_edges_;
      prepared.channel_lane_sets = static_channel_lane_sets_;
      if (objective) {
        prepared.search_objective = makeStaticRouteObjective(*objective);
      }
      const bool tracking_roi_refresh =
          proactive_roi_refresh &&
          roi_refresh.purpose ==
              StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective;
      prepared.static_route_extension_request =
          proactive_roi_refresh && !tracking_roi_refresh;
      prepared.static_route_extension_base_generation =
          prepared.static_route_extension_request ? roi_refresh.base_route_generation
                                                  : 0U;
      prepared.static_route_replan_request = tracking_roi_refresh;
      prepared.static_route_replan_base_generation =
          tracking_roi_refresh ? roi_refresh.base_route_generation : 0U;
      prepared.static_route_replan_reason =
          tracking_roi_refresh ? GlobalGuideReleaseReason::kObjectiveChanged
                               : GlobalGuideReleaseReason::kNone;
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
      const bool readiness_transition = !world_ready_.load(std::memory_order_acquire);
      completeStaticEsdfWork(true);
      if (readiness_transition) {
        publishWorldReadiness(true);
      }
      RCLCPP_INFO(get_logger(),
                  "PRODUCTION_MPPI_ESDF3D revision=%" PRIu64
                  " upload_ms=%.2f dimensions=%dx%dx%d",
                  prepared.revision, prepared.upload_ms, prepared.grid.width,
                  prepared.grid.height, prepared.grid.depth);
      continue;
    }
    const std::shared_ptr<const OccupancyGrid2D> raw_occupancy = raw_world->occupancy;
    if (!raw_occupancy) {
      RCLCPP_WARN(get_logger(),
                  "PRODUCTION_MPPI_ESDF rejected revision=%" PRIu64
                  " reason=unavailable_raw_grid",
                  raw_world->revision);
      continue;
    }
    const double raw_conversion_ms = raw_world->reconstruction_ms;

    ProductionMppiNavigation navigation;
    {
      const std::scoped_lock lock{input_mutex_};
      navigation = navigation_;
    }
    if (!navigation.valid) {
      continue;
    }
    std::optional<ProductionMppiPreparedEsdf> active_prepared;
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      active_prepared = prepared_esdf_;
    }
    const GridBounds& world_bounds = raw_occupancy->bounds();
    const Point2 position{navigation.state.x, navigation.state.y};
    GridBounds local_bounds;
    bool recenter = true;
    if (active_prepared && active_prepared->distances_m) {
      local_bounds = GridBounds{
          .origin_x = active_prepared->grid.origin_x_m,
          .origin_y = active_prepared->grid.origin_y_m,
          .resolution_m = active_prepared->grid.resolution_m,
          .width_cells = active_prepared->grid.width,
          .height_cells = active_prepared->grid.height,
      };
      recenter = localEsdfNeedsRecenter(local_bounds, world_bounds, position,
                                        no_static_esdf_recenter_margin_m_);
    }
    if (recenter) {
      local_bounds =
          selectLocalEsdfBounds(world_bounds, position, no_static_esdf_half_extent_m_);
    }
    OccupancyGrid2D local_occupancy = cropOccupancyGrid(*raw_occupancy, local_bounds);
    const std::uint64_t local_occupied_fingerprint =
        local_occupancy.occupiedFingerprint();
    const bool local_occupancy_unchanged =
        active_prepared && active_prepared->distances_m && !recenter &&
        active_prepared->source_occupied_fingerprint == local_occupied_fingerprint;
    const auto build_started_at = std::chrono::steady_clock::now();
    const bool first_build =
        no_static_esdf_last_build_time_ == std::chrono::steady_clock::time_point{};
    const bool build_rate_due =
        first_build || std::chrono::duration<double>(build_started_at -
                                                     no_static_esdf_last_build_time_)
                               .count() >= 1.0 / no_static_esdf_update_rate_hz_;
    if (local_occupancy_unchanged ||
        (active_prepared && !recenter && !build_rate_due)) {
      if (!local_occupancy_unchanged) {
        no_static_esdf_throttled_updates_.fetch_add(1U, std::memory_order_relaxed);
      } else {
        const std::scoped_lock lock{esdf_state_mutex_};
        if (prepared_esdf_ && prepared_esdf_->revision == active_prepared->revision) {
          prepared_esdf_->source_stamp_ns = source_stamp_ns;
          prepared_esdf_->ready_stamp_ns = raw_world->ready_stamp_ns;
          prepared_esdf_->raw_occupancy = raw_occupancy;
        }
      }
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "NO_STATIC_ESDF_DEFERRED raw_revision=%" PRIu64
          " reason=%s raw_conversion_ms=%.2f raw_updates=%" PRIu64 " builds=%" PRIu64
          " throttled=%" PRIu64,
          raw_world->revision,
          local_occupancy_unchanged ? "occupied_unchanged" : "rate_limited",
          raw_conversion_ms, no_static_raw_updates_.load(std::memory_order_relaxed),
          no_static_esdf_builds_.load(std::memory_order_relaxed),
          no_static_esdf_throttled_updates_.load(std::memory_order_relaxed));
      continue;
    }
    const DistanceField2D field = DistanceField2D::build(
        local_occupancy,
        static_cast<double>(mppi_config_.risk.preferred_distance_m) + 20.0,
        DistanceFieldSource::kOccupied, planning_worker_pool_.get());
    const double build_ms = field.stats().duration_ms;
    const auto float_conversion_started = std::chrono::steady_clock::now();
    std::vector<float> distances;
    distances.reserve(field.distancesM().size());
    for (const double distance_m : field.distancesM()) {
      distances.push_back(static_cast<float>(distance_m));
    }
    const double conversion_ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() -
                                                  float_conversion_started)
            .count() +
        raw_conversion_ms;
    const GridBounds& bounds = field.bounds();
    const mppi::EsdfGrid grid{bounds.width_cells, bounds.height_cells,
                              static_cast<float>(bounds.resolution_m),
                              static_cast<float>(bounds.origin_x),
                              static_cast<float>(bounds.origin_y)};
    const mppi::EsdfUploadResult upload = engine_->updateEsdf(
        mppi::EsdfSnapshot{grid, distances, local_occupied_fingerprint});
    if (!upload.accepted) {
      continue;
    }
    no_static_esdf_last_build_time_ = build_started_at;
    no_static_esdf_builds_.fetch_add(1U, std::memory_order_relaxed);

    ProductionMppiPreparedEsdf prepared;
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      if (prepared_esdf_.has_value()) {
        prepared = *prepared_esdf_;
      }
    }
    prepared.producer_instance_id = raw_world->producer_instance_id;
    prepared.revision = local_occupied_fingerprint;
    prepared.source_occupied_fingerprint = local_occupied_fingerprint;
    prepared.source_stamp_ns = source_stamp_ns;
    prepared.ready_stamp_ns = get_clock()->now().nanoseconds();
    prepared.build_ms = build_ms;
    prepared.esdf_x_pass_ms = field.stats().x_pass_ms;
    prepared.esdf_y_pass_ms = field.stats().y_pass_ms;
    prepared.esdf_z_pass_ms = 0.0;
    prepared.esdf_finalize_ms = field.stats().finalize_ms;
    prepared.conversion_ms = conversion_ms;
    prepared.upload_ms = upload.upload_ms;
    prepared.grid = grid;
    prepared.distances_m =
        std::make_shared<const std::vector<float>>(std::move(distances));
    prepared.raw_occupancy = raw_occupancy;
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

    RCLCPP_INFO(
        get_logger(),
        "PRODUCTION_MPPI_ESDF revision=%" PRIu64 " raw_revision=%" PRIu64
        " build_ms=%.2f conversion_ms=%.2f upload_ms=%.2f "
        "raw_to_ready_ms=%.2f full_cells=%zu local_cells=%zu dimensions=%dx%d "
        "recenter=%s builds=%" PRIu64 " throttled=%" PRIu64 " dropped_raw=%" PRIu64
        " dropped_guide_worlds=%" PRIu64,
        prepared.revision, raw_world->revision, prepared.build_ms,
        prepared.conversion_ms, prepared.upload_ms,
        static_cast<double>(prepared.ready_stamp_ns - prepared.source_stamp_ns) / 1.0e6,
        raw_occupancy->cellCount(), local_occupancy.cellCount(), prepared.grid.width,
        prepared.grid.height, recenter ? "true" : "false",
        no_static_esdf_builds_.load(std::memory_order_relaxed),
        no_static_esdf_throttled_updates_.load(std::memory_order_relaxed),
        dropped_raw_snapshots_.load(std::memory_order_relaxed),
        dropped_guide_worlds_.load(std::memory_order_relaxed));
  }
}

} // namespace drone_city_nav
