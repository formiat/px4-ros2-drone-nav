
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
    std::shared_ptr<const ProductionMppiRawWorld3D> raw_world;
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
        if (std::optional scheduled = raw_world_scheduler_3d_.takeReady(now)) {
          raw_world = std::move(*scheduled);
          break;
        }
        if (raw_world_scheduler_3d_.notBefore().has_value()) {
          static_cast<void>(raw_queue_condition_.wait_until(
              lock, stop_token, *raw_world_scheduler_3d_.notBefore(), [this]() {
                return raw_world_scheduler_3d_.ready(std::chrono::steady_clock::now());
              }));
        } else {
          raw_queue_condition_.wait(lock, stop_token, [this]() {
            return raw_world_scheduler_3d_.hasPending();
          });
        }
      }
    }
    if (stop_token.stop_requested()) {
      return;
    }
    if (!use_static_map_) {
      if (raw_world) {
        const std::optional<std::chrono::steady_clock::time_point> retry_not_before =
            processObservedEsdf3D(*raw_world);
        if (retry_not_before.has_value()) {
          {
            const std::scoped_lock lock{raw_queue_mutex_};
            raw_world_scheduler_3d_.defer(std::move(raw_world), *retry_not_before);
          }
          raw_queue_condition_.notify_all();
        }
      }
      continue;
    }
    if (!static_work) {
      continue;
    }
    const std::int64_t source_stamp_ns = get_clock()->now().nanoseconds();
    if (!static_occupancy_3d_) {
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
      std::shared_ptr<const WorldSnapshot3D> active_world;
      ProductionWorldBuildTelemetry3D active_world_build;
      {
        const std::scoped_lock lock{esdf_state_mutex_};
        active_world = resident_world_;
        active_world_build = resident_world_build_telemetry_;
      }
      const bool roi_refresh_pending =
          static_roi_refresh_lifecycle_.pending(roi_refresh);
      const std::shared_ptr<const ExecutionPlan3D> refresh_execution =
          roi_refresh_pending ? execution_route_store_.snapshot() : nullptr;
      const bool proactive_roi_refresh =
          roi_refresh_pending && refresh_execution != nullptr &&
          refresh_execution->route() != nullptr &&
          refresh_execution->route()->identity.generation ==
              roi_refresh.base_route_generation;
      double static_build_ms = active_world_build.build_ms;
      double static_x_pass_ms = active_world_build.esdf_x_pass_ms;
      double static_y_pass_ms = active_world_build.esdf_y_pass_ms;
      double static_z_pass_ms = active_world_build.esdf_z_pass_ms;
      double static_finalize_ms = active_world_build.esdf_finalize_ms;
      if (roi_refresh_pending && !proactive_roi_refresh) {
        finish_roi_refresh();
      }
      if (static_esdf_3d_ && static_esdf_uploaded_ && !proactive_roi_refresh &&
          active_world) {
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
            mission_goal, static_esdf_route_lookahead_m_, 40.0);
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
      const std::shared_ptr<const ExecutionPlan3D> binding_execution =
          proactive_roi_refresh ? execution_route_store_.snapshot() : nullptr;
      const bool refresh_base_current =
          proactive_roi_refresh && binding_execution != nullptr &&
          binding_execution->route() != nullptr &&
          binding_execution->route()->identity.generation ==
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
      ProductionWorldBuildTelemetry3D world_build =
          proactive_roi_refresh ? active_world_build
                                : ProductionWorldBuildTelemetry3D{};
      WorldSnapshot3D world;
      world.producer_instance_id =
          active_world ? active_world->producer_instance_id : 0U;
      world.revision = static_occupancy_3d_->fingerprint();
      world.source_occupied_fingerprint = static_occupancy_3d_->contentFingerprint();
      world.raw_occupied_fingerprint = static_occupancy_3d_->contentFingerprint();
      world.source_stamp_ns = source_stamp_ns;
      world.ready_stamp_ns = get_clock()->now().nanoseconds();
      world_build.build_ms = static_build_ms;
      world_build.esdf_x_pass_ms = static_x_pass_ms;
      world_build.esdf_y_pass_ms = static_y_pass_ms;
      world_build.esdf_z_pass_ms = static_z_pass_ms;
      world_build.esdf_finalize_ms = static_finalize_ms;
      world_build.upload_ms = upload.upload_ms;
      world.grid = static_esdf_grid_;
      world.distances_m = static_esdf_3d_;
      world.static_occupancy = static_occupancy_3d_;
      world.topology_passage_traversals = static_portal_edges_;
      const RawMapVersion static_world_version{
          .base_snapshot_revision = world.revision,
          .revision = world.revision,
      };
      const std::optional<LocalWorldGeneration> local_world_generation =
          local_world_generation_counter_.issue(static_world_version,
                                                activation_navigation.revision,
                                                world.revision, upload.revision);
      if (!local_world_generation.has_value()) {
        {
          const std::scoped_lock lock{esdf_state_mutex_};
          resident_world_.reset();
          resident_world_build_telemetry_ = {};
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
      world.local_world_generation = *local_world_generation;
      const std::shared_ptr<const WorldSnapshot3D> world_snapshot =
          std::make_shared<const WorldSnapshot3D>(std::move(world));
      const bool tracking_roi_refresh =
          refresh_base_current &&
          roi_refresh.purpose ==
              StaticRouteRoiRefreshRequest::Purpose::kTrackingObjective;
      const bool extension_search = refresh_base_current && !tracking_roi_refresh;
      std::optional<PlannerSearchContinuityBase3D> continuity_base;
      if (refresh_base_current) {
        const CertifiedRouteSuffix3D& active_route = *binding_execution->route();
        const RouteProjection3D projection = projectOntoRoute3DWithinStationWindow(
            *active_route.geometry->route,
            Point3{activation_navigation.state.x, activation_navigation.state.y,
                   activation_navigation.state.z},
            active_route.progress.station_m, active_route.endStationM());
        const RouteProgressProjection3D request_projection{
            .valid = projection.valid,
            .station_m = projection.station_m,
            .total_length_m = active_route.endStationM(),
            .remaining_m = projection.remaining_m,
            .cross_track_m = projection.distance_m,
            .point = {projection.point.x, projection.point.y},
        };
        if (extension_search) {
          continuity_base = PlannerSearchContinuityBase3D{
              .route = std::make_shared<const CertifiedRouteSuffix3D>(active_route),
              .request_projection = request_projection,
          };
        }
      }
      const bool coherent_generation =
          productionWorldGenerationCoherent(*world_snapshot);
      {
        const std::scoped_lock lock{esdf_state_mutex_};
        if (coherent_generation) {
          resident_world_ = world_snapshot;
          resident_world_build_telemetry_ = world_build;
        } else {
          resident_world_.reset();
          resident_world_build_telemetry_ = {};
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
          (extension_search || tracking_roi_refresh ||
           vehicle_navigation_ready_.load(std::memory_order_acquire));
      if (route_search_required) {
        const std::shared_ptr<const ExecutionPlan3D> resident_execution =
            binding_execution != nullptr ? binding_execution
                                         : execution_route_store_.snapshot();
        const std::uint64_t resident_route_generation =
            resident_execution != nullptr
                ? resident_execution->routeGenerationHighWater()
                : 0U;
        StaticRouteSearchRequestIdentity request_identity;
        if (extension_search) {
          request_identity = StaticRouteSearchRequestIdentity{
              .kind = StaticRouteSearchRequestKind::kExtension,
              .base_route_generation = roi_refresh.base_route_generation,
          };
        } else if (tracking_roi_refresh) {
          request_identity = StaticRouteSearchRequestIdentity{
              .kind = StaticRouteSearchRequestKind::kReplan,
              .base_route_generation = roi_refresh.base_route_generation,
          };
        } else {
          request_identity = StaticRouteSearchRequestIdentity{
              .kind = resident_route_generation == 0U
                          ? StaticRouteSearchRequestKind::kInitial
                          : StaticRouteSearchRequestKind::kResidentRefresh,
              .base_route_generation = resident_route_generation,
          };
        }
        const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
            objective
                ? makePlannerSearchTransaction3D(
                      world_snapshot, captureResidentPlannerWorld3D(*world_snapshot),
                      makeStaticRouteObjective(*objective), request_identity,
                      std::move(continuity_base),
                      tracking_roi_refresh ? RouteReleaseReason3D::kObjectiveChanged
                                           : RouteReleaseReason3D::kNone)
                : nullptr;
        bool queued = false;
        std::shared_ptr<const PlannerSearchTransaction3D> superseded_transaction;
        {
          const std::scoped_lock lock{route_planning_queue_mutex_};
          if (pending_route_planning_work_ &&
              (extension_search || tracking_roi_refresh) && transaction != nullptr) {
            dropped_route_planning_worlds_.fetch_add(1U, std::memory_order_relaxed);
            superseded_transaction = pending_route_planning_work_->transaction;
            pending_route_planning_work_.reset();
          }
          if (!pending_route_planning_work_ && transaction != nullptr) {
            pending_route_planning_work_ = ProductionRoutePlanningWork3D{
                .transaction = transaction,
                .world_telemetry = world_build,
                .continuation_session = nullptr,
            };
            queued = true;
          }
        }
        const bool lifecycle_transferred =
            superseded_transaction != nullptr && transaction != nullptr &&
            superseded_transaction->request.kind == transaction->request.kind &&
            superseded_transaction->request.base_route_generation ==
                transaction->request.base_route_generation;
        if (superseded_transaction != nullptr && !lifecycle_transferred) {
          finishStaticRouteSearch(*superseded_transaction);
        }
        if (queued) {
          route_planning_queue_condition_.notify_all();
        } else if (transaction == nullptr) {
          RCLCPP_ERROR(get_logger(),
                       "STATIC_ROUTE_SEARCH_REQUEST "
                       "status=rejected_invalid_transaction generation=%" PRIu64,
                       request_identity.base_route_generation);
          if (extension_search) {
            finishStaticRouteExtension(request_identity.base_route_generation);
          } else if (tracking_roi_refresh) {
            finishStaticRouteReplan(request_identity.base_route_generation, false);
          }
        }
      } else {
        RCLCPP_INFO(get_logger(), "STATIC_ESDF3D_PREWARMED route_search_deferred=true "
                                  "reason=navigation_not_ready");
      }
      RCLCPP_INFO(get_logger(),
                  "PRODUCTION_MPPI_ESDF3D revision=%" PRIu64
                  " upload_ms=%.2f dimensions=%dx%dx%d",
                  world_snapshot->revision, world_build.upload_ms,
                  world_snapshot->grid.width, world_snapshot->grid.height,
                  world_snapshot->grid.depth);
      continue;
    }
  }
}

} // namespace drone_city_nav
