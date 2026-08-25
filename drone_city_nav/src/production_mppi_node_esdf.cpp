#include "drone_city_nav/distance_field.hpp"
#include "drone_city_nav/local_esdf_2d.hpp"

#include <algorithm>
#include <chrono>
#include <cinttypes>
#include <cmath>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

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
    std::shared_ptr<const ProductionMppiRawWorld3D> raw_world_3d;
    bool static_work{false};
    {
      std::unique_lock lock{raw_queue_mutex_};
      while (!stop_token.stop_requested()) {
        if (use_static_map_) {
          raw_queue_condition_.wait(lock, stop_token,
                                    [this]() { return pending_static_esdf_work_; });
          static_work = std::exchange(pending_static_esdf_work_, false);
          static_esdf_work_in_progress_ = static_work;
          break;
        }
        const auto now = std::chrono::steady_clock::now();
        if (no_static_world_model_ ==
            ProductionNoStaticWorldModel::kObservedOccupancy3D) {
          if (std::optional scheduled = raw_world_scheduler_3d_.takeReady(now)) {
            raw_world_3d = std::move(*scheduled);
            break;
          }
          if (raw_world_scheduler_3d_.notBefore().has_value()) {
            static_cast<void>(raw_queue_condition_.wait_until(
                lock, stop_token, *raw_world_scheduler_3d_.notBefore(), [this]() {
                  return raw_world_scheduler_3d_.ready(
                      std::chrono::steady_clock::now());
                }));
          } else {
            raw_queue_condition_.wait(lock, stop_token, [this]() {
              return raw_world_scheduler_3d_.hasPending();
            });
          }
          continue;
        }
        if (std::optional scheduled = raw_world_scheduler_.takeReady(now)) {
          raw_world = std::move(*scheduled);
          break;
        }
        if (raw_world_scheduler_.notBefore().has_value()) {
          static_cast<void>(raw_queue_condition_.wait_until(
              lock, stop_token, *raw_world_scheduler_.notBefore(), [this]() {
                return raw_world_scheduler_.ready(std::chrono::steady_clock::now());
              }));
        } else {
          raw_queue_condition_.wait(
              lock, stop_token, [this]() { return raw_world_scheduler_.hasPending(); });
        }
      }
    }
    if (stop_token.stop_requested()) {
      return;
    }
    if (use_static_map_ && !static_work) {
      continue;
    }
    if (!use_static_map_ &&
        no_static_world_model_ == ProductionNoStaticWorldModel::kObservedOccupancy3D) {
      if (raw_world_3d) {
        const std::optional<std::chrono::steady_clock::time_point> retry_not_before =
            processObservedEsdf3D(*raw_world_3d);
        if (retry_not_before.has_value()) {
          {
            const std::scoped_lock lock{raw_queue_mutex_};
            raw_world_scheduler_3d_.defer(std::move(raw_world_3d), *retry_not_before);
          }
          raw_queue_condition_.notify_all();
        }
      }
      continue;
    }
    if (!use_static_map_ && !raw_world) {
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
      const auto finish_roi_refresh = [this, &roi_refresh]() {
        static_roi_refresh_lifecycle_.complete(roi_refresh.sequence);
        if (roi_refresh.purpose ==
            StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective) {
          finishStaticRouteReplan(roi_refresh.base_route_generation, false);
        } else {
          finishStaticRouteExtension(roi_refresh.base_route_generation);
        }
      };
      std::optional<ProductionMppiPreparedEsdf> active_prepared;
      {
        const std::scoped_lock lock{esdf_state_mutex_};
        active_prepared = prepared_esdf_;
      }
      const bool roi_refresh_pending =
          static_roi_refresh_lifecycle_.pending(roi_refresh);
      const std::shared_ptr<const ExecutionRouteSnapshot3D> refresh_execution =
          roi_refresh_pending ? execution_route_store_.snapshot() : nullptr;
      const bool proactive_roi_refresh =
          roi_refresh_pending && refresh_execution != nullptr &&
          refresh_execution->route.has_value() &&
          refresh_execution->route->identity.generation ==
              roi_refresh.base_route_generation;
      double static_build_ms = active_prepared ? active_prepared->build_ms : 0.0;
      double static_x_pass_ms = active_prepared ? active_prepared->esdf_x_pass_ms : 0.0;
      double static_y_pass_ms = active_prepared ? active_prepared->esdf_y_pass_ms : 0.0;
      double static_z_pass_ms = active_prepared ? active_prepared->esdf_z_pass_ms : 0.0;
      double static_finalize_ms =
          active_prepared ? active_prepared->esdf_finalize_ms : 0.0;
      if (roi_refresh_pending && !proactive_roi_refresh) {
        finish_roi_refresh();
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
        if (!navigation.world_state_authoritative) {
          if (proactive_roi_refresh) {
            finish_roi_refresh();
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
      ProductionMppiNavigation activation_navigation;
      {
        const std::scoped_lock lock{input_mutex_};
        activation_navigation = navigation_;
      }
      if (!activation_navigation.valid) {
        if (proactive_roi_refresh) {
          finish_roi_refresh();
        }
        completeStaticEsdfWork(false);
        continue;
      }
      std::unique_lock generation_lock{world_generation_publication_mutex_};
      mppi::EsdfUploadResult upload{true, 0.0, static_occupancy_3d_->fingerprint()};
      if (!reused_uploaded_roi) {
        upload = engine_->updateEsdf(mppi::EsdfSnapshot{
            static_esdf_grid_, *static_esdf_3d_, static_occupancy_3d_->fingerprint()});
      }
      if (!upload.accepted) {
        if (proactive_roi_refresh) {
          finish_roi_refresh();
        }
        completeStaticEsdfWork(false);
        continue;
      }
      static_esdf_uploaded_ = true;
      const std::shared_ptr<const ExecutionRouteSnapshot3D> binding_execution =
          proactive_roi_refresh ? execution_route_store_.snapshot() : nullptr;
      const bool refresh_base_current = proactive_roi_refresh &&
                                        binding_execution != nullptr &&
                                        binding_execution->route.has_value() &&
                                        binding_execution->route->identity.generation ==
                                            roi_refresh.base_route_generation;
      const bool refresh_superseded = proactive_roi_refresh && !refresh_base_current;
      if (refresh_superseded) {
        RCLCPP_INFO(
            get_logger(),
            "STATIC_ESDF3D refresh_superseded=true requested_generation=%" PRIu64
            " resident_generation=%" PRIu64
            " action=publish_world_without_stale_route_search",
            roi_refresh.base_route_generation,
            binding_execution != nullptr ? binding_execution->routeGenerationHighWater()
                                         : 0U);
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
      prepared.passage_traversals = static_portal_edges_;
      prepared.topological_graph = topological_navigation_3d_->snapshot();
      if (prepared.topological_graph) {
        prepared.topological_graph_update.revision =
            prepared.topological_graph->revision();
        prepared.topological_graph_update.node_count =
            prepared.topological_graph->nodes().size();
        prepared.topological_graph_update.edge_count =
            prepared.topological_graph->edges().size();
        prepared.topology_source_raw_revision = prepared.topological_graph->revision();
      }
      const RawMapVersion static_world_version{
          .base_snapshot_revision = prepared.revision,
          .revision = prepared.revision,
      };
      const std::optional<LocalWorldGeneration> local_world_generation =
          local_world_generation_counter_.issue(
              static_world_version, activation_navigation.revision, prepared.revision,
              upload.revision, prepared.topology_source_raw_revision);
      if (!local_world_generation.has_value()) {
        {
          const std::scoped_lock lock{esdf_state_mutex_};
          prepared_esdf_.reset();
        }
        generation_lock.unlock();
        rejected_world_generation_publications_.fetch_add(1U,
                                                          std::memory_order_relaxed);
        if (world_ready_.exchange(false, std::memory_order_acq_rel)) {
          publishWorldReadiness(false);
        }
        RCLCPP_ERROR(get_logger(),
                     "STATIC_ESDF3D rejected reason=invalid_local_world_generation");
        if (proactive_roi_refresh) {
          finish_roi_refresh();
        }
        completeStaticEsdfWork(false);
        continue;
      }
      prepared.local_world_generation = *local_world_generation;
      if (objective) {
        prepared.search_objective = makeStaticRouteObjective(*objective);
      }
      if (refresh_base_current) {
        const CertifiedRouteSuffix3D& active_route = *binding_execution->route;
        const RouteProjection3D projection = projectOntoRoute3DWithinStationWindow(
            *active_route.geometry->route,
            Point3{activation_navigation.state.x, activation_navigation.state.y,
                   activation_navigation.state.z},
            active_route.progress.station_m, active_route.endStationM());
        bindStaticRouteRequestToExecution(
            prepared, active_route,
            GlobalGuideProjection{
                .valid = projection.valid,
                .station_m = projection.station_m,
                .total_length_m = active_route.endStationM(),
                .remaining_m = projection.remaining_m,
                .cross_track_m = projection.distance_m,
                .point = {projection.point.x, projection.point.y},
            });
      }
      const bool tracking_roi_refresh =
          refresh_base_current &&
          roi_refresh.purpose ==
              StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective;
      prepared.static_route_extension_request =
          refresh_base_current && !tracking_roi_refresh;
      prepared.static_route_extension_base_generation =
          prepared.static_route_extension_request ? roi_refresh.base_route_generation
                                                  : 0U;
      prepared.static_route_replan_request = tracking_roi_refresh;
      prepared.static_route_replan_base_generation =
          tracking_roi_refresh ? roi_refresh.base_route_generation : 0U;
      prepared.static_route_replan_reason =
          tracking_roi_refresh ? GlobalGuideReleaseReason::kObjectiveChanged
                               : GlobalGuideReleaseReason::kNone;
      const bool coherent_generation = productionWorldGenerationCoherent(prepared);
      {
        const std::scoped_lock lock{esdf_state_mutex_};
        if (coherent_generation) {
          prepared_esdf_ = prepared;
        } else {
          prepared_esdf_.reset();
        }
      }
      generation_lock.unlock();
      if (!coherent_generation) {
        rejected_world_generation_publications_.fetch_add(1U,
                                                          std::memory_order_relaxed);
        if (world_ready_.exchange(false, std::memory_order_acq_rel)) {
          publishWorldReadiness(false);
        }
        RCLCPP_ERROR(get_logger(),
                     "STATIC_ESDF3D rejected reason=mixed_local_world_generation");
        if (proactive_roi_refresh) {
          finish_roi_refresh();
        }
        completeStaticEsdfWork(false);
        continue;
      }
      if (proactive_roi_refresh) {
        static_roi_refresh_lifecycle_.complete(roi_refresh.sequence);
        if (refresh_superseded) {
          if (roi_refresh.purpose ==
              StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective) {
            finishStaticRouteReplan(roi_refresh.base_route_generation, false);
          } else {
            finishStaticRouteExtension(roi_refresh.base_route_generation);
          }
        }
      }
      const bool readiness_transition = !world_ready_.load(std::memory_order_acquire);
      completeStaticEsdfWork(true);
      if (readiness_transition) {
        publishWorldReadiness(true);
      }
      const bool route_search_required =
          !refresh_superseded &&
          (prepared.static_route_extension_request ||
           prepared.static_route_replan_request ||
           vehicle_navigation_ready_.load(std::memory_order_acquire));
      if (route_search_required) {
        bool queued = false;
        {
          const std::scoped_lock lock{guide_queue_mutex_};
          if (pending_guide_world_ && (prepared.static_route_extension_request ||
                                       prepared.static_route_replan_request)) {
            dropped_guide_worlds_.fetch_add(1U, std::memory_order_relaxed);
            pending_guide_world_.reset();
          }
          if (!pending_guide_world_) {
            pending_guide_world_ =
                std::make_shared<const ProductionMppiPreparedEsdf>(prepared);
            queued = true;
          }
        }
        if (queued) {
          guide_queue_condition_.notify_all();
        }
      } else {
        RCLCPP_INFO(get_logger(), "STATIC_ESDF3D_PREWARMED route_search_deferred=true "
                                  "reason=navigation_not_ready");
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
                  raw_world->version.revision);
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
    if (local_occupancy_unchanged) {
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "NO_STATIC_ESDF_DEFERRED raw_revision=%" PRIu64
          " reason=occupied_unchanged raw_conversion_ms=%.2f raw_updates=%" PRIu64
          " builds=%" PRIu64 " throttled=%" PRIu64,
          raw_world->version.revision, raw_conversion_ms,
          no_static_raw_updates_.load(std::memory_order_relaxed),
          no_static_esdf_builds_.load(std::memory_order_relaxed),
          no_static_esdf_throttled_updates_.load(std::memory_order_relaxed));
      continue;
    }
    if (active_prepared && !recenter && !build_rate_due) {
      no_static_esdf_throttled_updates_.fetch_add(1U, std::memory_order_relaxed);
      const std::uint64_t deferred_raw_revision = raw_world->version.revision;
      const auto update_period =
          std::chrono::duration<double>{1.0 / no_static_esdf_update_rate_hz_};
      const auto retry_not_before =
          no_static_esdf_last_build_time_ +
          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
              update_period);
      {
        const std::scoped_lock lock{raw_queue_mutex_};
        raw_world_scheduler_.defer(std::move(raw_world), retry_not_before);
      }
      raw_queue_condition_.notify_all();
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "NO_STATIC_ESDF_DEFERRED raw_revision=%" PRIu64
          " reason=rate_limited raw_conversion_ms=%.2f raw_updates=%" PRIu64
          " builds=%" PRIu64 " throttled=%" PRIu64,
          deferred_raw_revision, raw_conversion_ms,
          no_static_raw_updates_.load(std::memory_order_relaxed),
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
    std::unique_lock generation_lock{world_generation_publication_mutex_};
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
    prepared.producer_instance_id = raw_world->version.producer_instance_id;
    prepared.revision = local_occupied_fingerprint;
    prepared.source_raw_revision = raw_world->version.revision;
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
    const std::optional<LocalWorldGeneration> local_world_generation =
        local_world_generation_counter_.issue(raw_world->version, navigation.revision,
                                              prepared.revision, upload.revision);
    if (!local_world_generation.has_value()) {
      {
        const std::scoped_lock lock{esdf_state_mutex_};
        prepared_esdf_.reset();
      }
      generation_lock.unlock();
      rejected_world_generation_publications_.fetch_add(1U, std::memory_order_relaxed);
      if (world_ready_.exchange(false, std::memory_order_acq_rel)) {
        publishWorldReadiness(false);
      }
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ESDF rejected revision=%" PRIu64
                   " reason=invalid_local_world_generation",
                   raw_world->version.revision);
      continue;
    }
    prepared.local_world_generation = *local_world_generation;
    if (const std::shared_ptr<const ProductionNavigationObjective> objective =
            navigationObjective()) {
      prepared.search_objective = makeStaticRouteObjective(*objective);
    }
    prepared.lattice_search_performed = false;
    prepared.lattice_continuation_attempt = 0U;

    const bool coherent_generation = productionWorldGenerationCoherent(prepared);
    {
      const std::scoped_lock lock{esdf_state_mutex_};
      if (coherent_generation) {
        prepared_esdf_ = prepared;
      } else {
        prepared_esdf_.reset();
      }
    }
    generation_lock.unlock();
    if (!coherent_generation) {
      rejected_world_generation_publications_.fetch_add(1U, std::memory_order_relaxed);
      if (world_ready_.exchange(false, std::memory_order_acq_rel)) {
        publishWorldReadiness(false);
      }
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ESDF rejected revision=%" PRIu64
                   " reason=mixed_local_world_generation",
                   raw_world->version.revision);
      continue;
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
        prepared.revision, raw_world->version.revision, prepared.build_ms,
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
