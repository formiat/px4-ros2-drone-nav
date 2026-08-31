#include <cinttypes>
#include <memory>
#include <optional>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] StaticWorldObjective3D staticWorldObjective3D(
    const std::shared_ptr<const ProductionNavigationObjective>& objective) noexcept {
  if (objective == nullptr) {
    return {};
  }
  return StaticWorldObjective3D{
      .goal = objective->goal,
      .mission_epoch = objective->mission_epoch,
      .sample_sequence = objective->sample_sequence,
      .assignment_generation = objective->assignment_generation,
      .target_detection_id = objective->target_detection_id,
      .target_track_id = objective->target_track_id,
      .continuous_tracking = objective->continuous_tracking,
      .available = true,
  };
}

[[nodiscard]] StaticRouteObjective
staticRouteObjective(const StaticWorldObjective3D& objective) noexcept {
  return StaticRouteObjective{
      .goal = objective.goal,
      .mission_epoch = objective.mission_epoch,
      .sample_sequence = objective.sample_sequence,
      .assignment_generation = objective.assignment_generation,
      .target_detection_id = objective.target_detection_id,
      .target_track_id = objective.target_track_id,
      .continuous_tracking = objective.continuous_tracking,
      .available = objective.available,
  };
}

[[nodiscard]] const char*
staticRefreshPurposeName(const StaticWorldRefreshRequest3D& refresh) noexcept {
  if (!refresh.valid()) {
    return "none";
  }
  return refresh.purpose == StaticWorldRefreshPurpose3D::kTrackingObjective
             ? "tracking_objective"
             : "route_extension";
}

} // namespace

StaticWorldBuildRequest3D ProductionMppiNode::makeStaticWorldBuildRequest3D(
    const StaticWorldRefreshRequest3D& refresh) {
  ProductionMppiNavigation navigation;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
  }
  const std::shared_ptr<const ExecutionPlan3D> execution = execution_supervisor_.plan();
  return StaticWorldBuildRequest3D{
      .refresh = refresh,
      .objective = staticWorldObjective3D(navigationObjective()),
      .position = {navigation.state.x, navigation.state.y, navigation.state.z},
      .resident_route_generation =
          execution != nullptr ? execution->routeGenerationHighWater() : 0U,
      .source_stamp_ns = get_clock()->now().nanoseconds(),
      .world_state_authoritative = navigation.world_state_authoritative,
  };
}

StaticWorldCommitContext3D ProductionMppiNode::makeStaticWorldCommitContext3D() {
  ProductionMppiNavigation navigation;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
  }
  const std::shared_ptr<const ExecutionPlan3D> execution = execution_supervisor_.plan();
  return StaticWorldCommitContext3D{
      .position = {navigation.state.x, navigation.state.y, navigation.state.z},
      .pose_revision = navigation.revision,
      .resident_route_generation =
          execution != nullptr ? execution->routeGenerationHighWater() : 0U,
      .ready_stamp_ns = get_clock()->now().nanoseconds(),
      .navigation_valid = navigation.valid,
  };
}

void ProductionMppiNode::handleStaticWorldUpdate3D(const StaticWorldUpdate3D& update) {
  const StaticWorldRefreshRequest3D& refresh = update.request.refresh;
  const auto finish_refresh = [this, &refresh]() {
    if (!refresh.valid()) {
      return;
    }
    if (refresh.purpose == StaticWorldRefreshPurpose3D::kTrackingObjective) {
      finishStaticRouteReplan(refresh.base_route_generation, false);
    } else {
      finishStaticRouteExtension(refresh.base_route_generation);
    }
  };

  if (update.status == StaticWorldUpdateStatus3D::kAlreadyCurrent) {
    const bool readiness_transition = !world_ready_.load(std::memory_order_acquire);
    markStaticWorldReady();
    if (readiness_transition) {
      publishWorldReadiness(true);
    }
    return;
  }

  if (!update.published()) {
    finish_refresh();
    const std::string_view status_name = staticWorldUpdateStatus3DName(update.status);
    if (update.status == StaticWorldUpdateStatus3D::kUploadFailed ||
        update.status == StaticWorldUpdateStatus3D::kMixedLocalWorldGeneration) {
      if (world_ready_.exchange(false, std::memory_order_acq_rel)) {
        publishWorldReadiness(false);
      }
    }
    if (update.status == StaticWorldUpdateStatus3D::kRefreshSuperseded) {
      RCLCPP_INFO(get_logger(),
                  "STATIC_ESDF3D status=%.*s requested_generation=%" PRIu64,
                  static_cast<int>(status_name.size()), status_name.data(),
                  refresh.base_route_generation);
    } else if (update.status == StaticWorldUpdateStatus3D::kUnavailableNavigation ||
               update.status == StaticWorldUpdateStatus3D::kUnavailableObjective) {
      RCLCPP_DEBUG(get_logger(), "STATIC_ESDF3D status=%.*s",
                   static_cast<int>(status_name.size()), status_name.data());
    } else {
      RCLCPP_ERROR(get_logger(),
                   "STATIC_ESDF3D status=%.*s refresh_purpose=%s "
                   "base_generation=%" PRIu64,
                   static_cast<int>(status_name.size()), status_name.data(),
                   staticRefreshPurposeName(refresh), refresh.base_route_generation);
    }
    return;
  }

  const std::shared_ptr<const WorldSnapshot3D>& world_snapshot = update.world;
  if (!update.diagnostics.cache_fallback_error.empty()) {
    RCLCPP_ERROR(get_logger(),
                 "STATIC_ESDF_CACHE_FALLBACK reason=extract_failed error=%s",
                 update.diagnostics.cache_fallback_error.c_str());
  }
  if (update.diagnostics.cpu_resource_reused) {
    RCLCPP_INFO(get_logger(),
                "STATIC_ESDF3D_REUSED reason=stable_chunk_roi "
                "dimensions=%dx%dx%d gpu_resource_reused=%s "
                "base_generation=%" PRIu64,
                world_snapshot->grid.width, world_snapshot->grid.height,
                world_snapshot->grid.depth,
                update.diagnostics.gpu_resource_reused ? "true" : "false",
                refresh.valid() ? refresh.base_route_generation : 0U);
  } else {
    const std::string_view source_name =
        staticWorldEsdfSource3DName(update.diagnostics.source);
    RCLCPP_INFO(get_logger(),
                "STATIC_ESDF3D_READY source=%.*s build_ms=%.2f voxels=%zu "
                "dimensions=%dx%dx%d proactive_refresh=%s refresh_purpose=%s "
                "base_generation=%" PRIu64
                " cache_requested_chunks=%zu cache_decoded_chunks=%zu "
                "cache_hits=%zu cache_resident_chunks=%zu cache_decode_ms=%.2f "
                "cache_copy_ms=%.2f cache_finite_voxels=%zu",
                static_cast<int>(source_name.size()), source_name.data(),
                update.diagnostics.field.duration_ms,
                update.diagnostics.field.voxel_count, world_snapshot->grid.width,
                world_snapshot->grid.height, world_snapshot->grid.depth,
                update.proactive_refresh ? "true" : "false",
                staticRefreshPurposeName(refresh),
                refresh.valid() ? refresh.base_route_generation : 0U,
                update.diagnostics.cache.requested_chunks,
                update.diagnostics.cache.decoded_chunks,
                update.diagnostics.cache.decoded_chunk_cache_hits,
                update.diagnostics.cache.resident_decoded_chunks,
                update.diagnostics.cache.decode_ms, update.diagnostics.cache.copy_ms,
                update.diagnostics.cache.finite_voxels);
  }

  const std::shared_ptr<const ExecutionPlan3D> binding_execution =
      refresh.valid() ? execution_supervisor_.plan() : nullptr;
  const CertifiedRouteSuffix3D* const binding_route =
      binding_execution != nullptr ? binding_execution->route() : nullptr;
  const bool refresh_base_current =
      refresh.valid() && binding_route != nullptr &&
      binding_route->identity.generation == refresh.base_route_generation;
  if (refresh.valid() && !refresh_base_current) {
    RCLCPP_INFO(get_logger(),
                "STATIC_ESDF3D refresh_superseded=true "
                "requested_generation=%" PRIu64 " resident_generation=%" PRIu64
                " action=publish_world_without_stale_route_search",
                refresh.base_route_generation,
                binding_execution != nullptr
                    ? binding_execution->routeGenerationHighWater()
                    : 0U);
    finish_refresh();
  }

  const bool tracking_refresh =
      refresh_base_current &&
      refresh.purpose == StaticWorldRefreshPurpose3D::kTrackingObjective;
  const bool extension_search = refresh_base_current && !tracking_refresh;
  std::optional<PlannerSearchContinuityBase3D> continuity_base;
  if (extension_search && binding_route != nullptr && binding_route->valid()) {
    const RouteProjection3D projection = projectOntoRoute3DWithinStationWindow(
        *binding_route->geometry->route, update.commit_context.position,
        binding_route->progress.station_m, binding_route->endStationM());
    continuity_base = PlannerSearchContinuityBase3D{
        .route = std::make_shared<const CertifiedRouteSuffix3D>(*binding_route),
        .request_projection =
            RouteProgressProjection3D{
                .valid = projection.valid,
                .station_m = projection.station_m,
                .total_length_m = binding_route->endStationM(),
                .remaining_m = projection.remaining_m,
                .cross_track_m = projection.distance_m,
                .point = {projection.point.x, projection.point.y},
            },
    };
  }

  const bool readiness_transition = !world_ready_.load(std::memory_order_acquire);
  markStaticWorldReady();
  if (readiness_transition) {
    publishWorldReadiness(true);
  }

  const bool route_search_required =
      !refresh.valid() ? vehicle_navigation_ready_.load(std::memory_order_acquire)
                       : refresh_base_current;
  if (route_search_required) {
    const std::shared_ptr<const ExecutionPlan3D> resident_execution =
        binding_execution != nullptr ? binding_execution : execution_supervisor_.plan();
    const std::uint64_t resident_route_generation =
        resident_execution != nullptr ? resident_execution->routeGenerationHighWater()
                                      : 0U;
    StaticRouteSearchRequestIdentity request_identity;
    if (extension_search) {
      request_identity = StaticRouteSearchRequestIdentity{
          .kind = StaticRouteSearchRequestKind::kExtension,
          .base_route_generation = refresh.base_route_generation,
      };
    } else if (tracking_refresh) {
      request_identity = StaticRouteSearchRequestIdentity{
          .kind = StaticRouteSearchRequestKind::kReplan,
          .base_route_generation = refresh.base_route_generation,
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
        makePlannerSearchTransaction3D(
            world_snapshot, captureResidentPlannerWorld3D(*world_snapshot),
            staticRouteObjective(update.request.objective), request_identity,
            std::move(continuity_base),
            tracking_refresh ? RouteReleaseReason3D::kObjectiveChanged
                             : RouteReleaseReason3D::kNone);
    RoutePlanningEnqueueResult3D enqueue;
    if (transaction != nullptr) {
      enqueue = route_planning_coordinator_->enqueue(
          RoutePlanningRequest3D{
              .transaction = transaction,
              .world_telemetry = update.telemetry,
              .continuation_session = nullptr,
          },
          RoutePlanningQueuePolicy3D::kReplacePending);
    }
    if (transaction != nullptr && enqueue.displaced.has_value() &&
        enqueue.displaced->transaction != nullptr &&
        !enqueue.lifecycleTransferredTo(*transaction)) {
      finishStaticRouteSearch(*enqueue.displaced->transaction);
    }
    if (transaction == nullptr) {
      RCLCPP_ERROR(get_logger(),
                   "STATIC_ROUTE_SEARCH_REQUEST "
                   "status=rejected_invalid_transaction generation=%" PRIu64,
                   request_identity.base_route_generation);
      finish_refresh();
    }
  } else if (!refresh.valid()) {
    RCLCPP_INFO(get_logger(), "STATIC_ESDF3D_PREWARMED route_search_deferred=true "
                              "reason=navigation_not_ready");
  }

  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_ESDF3D revision=%" PRIu64 " upload_ms=%.2f dimensions=%dx%dx%d",
      world_snapshot->revision, update.telemetry.upload_ms, world_snapshot->grid.width,
      world_snapshot->grid.height, world_snapshot->grid.depth);
}

} // namespace drone_city_nav
