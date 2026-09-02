#include <chrono>
#include <cinttypes>
#include <memory>
#include <optional>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"
#include "world_pipeline_3d.hpp"

namespace drone_city_nav {

std::optional<ObservedWorldBuildRequest3D>
ProductionMppiNode::makeObservedWorldBuildRequest3D(
    std::shared_ptr<const ProductionMppiRawWorld3D> raw_world) {
  if (raw_world == nullptr) {
    return std::nullopt;
  }
  ProductionMppiNavigation navigation;
  std::shared_ptr<const CommittedExecutionAuthority3D> execution_authority;
  {
    const auto lock = evidence_boundary_.input();
    navigation = navigation_;
    execution_authority = execution_supervisor_.authority();
  }
  if (!navigation.world_state_authoritative) {
    return std::nullopt;
  }
  const std::optional<ProprioceptiveFreeSpaceSeed3D> free_space_seed =
      prepareObservedExecutionEvidence3D(*raw_world, navigation, execution_authority);
  return ObservedWorldBuildRequest3D{
      .raw_world = std::move(raw_world),
      .position = Point3{navigation.state.x, navigation.state.y, navigation.state.z},
      .pose_revision = navigation.revision,
      .ready_stamp_ns = get_clock()->now().nanoseconds(),
      .free_space_seed = free_space_seed,
      .launch_support_contact = launch_support_contact_,
      .launch_support_resolution_pending = !launch_support_evaluated_,
  };
}

void ProductionMppiNode::handleObservedWorldEvidenceChange3D(
    const ObservedWorldEvidenceChange3D& change) {
  if (!change.changed() || change.raw_world == nullptr) {
    return;
  }
  const std::uint64_t raw_revision = change.raw_world->version().revision;
  if (change.persistent_changed) {
    static_cast<void>(route_lifecycle_coordinator_->cancelPending());
    RCLCPP_INFO(get_logger(),
                "EXECUTION_EVIDENCE_WORLD_CHANGED raw_revision=%" PRIu64
                " free_space_seed=%s support_active=%s resolution_pending=%s"
                " incremental_refresh=true",
                raw_revision, change.free_space_seed.has_value() ? "true" : "false",
                change.launch_support_contact.has_value() ? "true" : "false",
                change.launch_support_resolution_pending ? "true" : "false");
    return;
  }
  RCLCPP_INFO(get_logger(),
              "TRANSIENT_EXECUTION_EVIDENCE_CHANGED raw_revision=%" PRIu64
              " free_space_seed=%s persistent_world_change=false",
              raw_revision, change.free_space_seed.has_value() ? "true" : "false");
}

void ProductionMppiNode::handleObservedWorldUpdate3D(
    const ObservedWorldUpdate3D& update) {
  const std::uint64_t raw_revision =
      update.raw_world != nullptr ? update.raw_world->version().revision : 0U;
  const double reconstruction_ms =
      update.raw_world != nullptr ? update.raw_world->reconstructionMs() : 0.0;
  switch (update.status) {
    case ObservedWorldUpdateStatus3D::kUnavailableObservedGrid:
      RCLCPP_WARN(get_logger(),
                  "PRODUCTION_MPPI_ESDF3D_ONLINE rejected revision=%" PRIu64
                  " reason=unavailable_observed_grid",
                  raw_revision);
      return;
    case ObservedWorldUpdateStatus3D::kRawExecutionOwnerMismatch:
    case ObservedWorldUpdateStatus3D::kRouteEvidenceDerivationFailed:
    case ObservedWorldUpdateStatus3D::kInvalidCoverage: {
      const std::string_view reason = observedWorldUpdateStatus3DName(update.status);
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ESDF3D_ONLINE rejected revision=%" PRIu64
                   " reason=%.*s",
                   raw_revision, static_cast<int>(reason.size()), reason.data());
      return;
    }
    case ObservedWorldUpdateStatus3D::kSupersededTransientEvidenceParent:
    case ObservedWorldUpdateStatus3D::kSupersededEsdfParent: {
      const std::string_view reason = observedWorldUpdateStatus3DName(update.status);
      RCLCPP_INFO(get_logger(),
                  "PRODUCTION_MPPI_ESDF3D_ONLINE deferred raw_revision=%" PRIu64
                  " reason=%.*s",
                  raw_revision, static_cast<int>(reason.size()), reason.data());
      return;
    }
    case ObservedWorldUpdateStatus3D::kUploadRejected:
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ESDF3D_ONLINE rejected raw_revision=%" PRIu64
                   " reason=upload_rejected",
                   raw_revision);
      return;
    case ObservedWorldUpdateStatus3D::kMixedLocalWorldGeneration:
      if (world_ready_.exchange(false, std::memory_order_acq_rel)) {
        publishWorldReadiness(false);
      }
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ESDF3D_ONLINE rejected raw_revision=%" PRIu64
                   " reason=mixed_local_world_generation",
                   raw_revision);
      return;
    case ObservedWorldUpdateStatus3D::kAlreadyCurrent: {
      if (update.transient_evidence_refreshed && update.world != nullptr) {
        RCLCPP_INFO(get_logger(),
                    "TRANSIENT_EXECUTION_EVIDENCE_REFRESHED raw_revision=%" PRIu64
                    " esdf_revision=%" PRIu64 " gpu_upload=false",
                    raw_revision, update.world->revision);
      }
      const WorldPipelineStatistics3D statistics = world_pipeline_->statistics();
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "NO_STATIC_ESDF3D_DEFERRED raw_revision=%" PRIu64
          " reason=already_current reconstruction_ms=%.2f raw_updates=%" PRIu64
          " builds=%" PRIu64 " throttled=%" PRIu64,
          raw_revision, reconstruction_ms, statistics.raw_updates,
          statistics.observedBuilds(), statistics.throttled_observed_builds);
      return;
    }
    case ObservedWorldUpdateStatus3D::kRateLimited: {
      const WorldPipelineStatistics3D statistics = world_pipeline_->statistics();
      RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "NO_STATIC_ESDF3D_DEFERRED raw_revision=%" PRIu64
          " reason=rate_limited reconstruction_ms=%.2f raw_updates=%" PRIu64
          " builds=%" PRIu64 " throttled=%" PRIu64,
          raw_revision, reconstruction_ms, statistics.raw_updates,
          statistics.observedBuilds(), statistics.throttled_observed_builds);
      return;
    }
    case ObservedWorldUpdateStatus3D::kPrepared:
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ESDF3D_ONLINE rejected raw_revision=%" PRIu64
                   " reason=unfinished_world_transaction",
                   raw_revision);
      return;
    case ObservedWorldUpdateStatus3D::kPublished:
      break;
  }

  if (!update.published() || update.raw_world == nullptr) {
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_ESDF3D_ONLINE rejected raw_revision=%" PRIu64
                 " reason=missing_published_world",
                 raw_revision);
    return;
  }
  const std::shared_ptr<const WorldSnapshot3D>& published_world = update.world;
  const std::uint64_t blocked_raw_revision =
      observed_route_blocked_raw_revision_.load(std::memory_order_acquire);
  const std::uint64_t dispatched_raw_revision =
      observed_route_replan_dispatched_raw_revision_.load(std::memory_order_acquire);
  const std::shared_ptr<const ExecutionPlan3D> resident_execution =
      execution_supervisor_.plan();
  const std::uint64_t resident_route_generation =
      resident_execution != nullptr ? resident_execution->routeGenerationHighWater()
                                    : 0U;
  if (blocked_raw_revision != 0U && blocked_raw_revision <= raw_revision &&
      dispatched_raw_revision < blocked_raw_revision &&
      resident_route_generation != 0U) {
    observed_route_replan_dispatched_raw_revision_.store(blocked_raw_revision,
                                                         std::memory_order_release);
    RCLCPP_INFO(get_logger(),
                "OBSERVED_ROUTE_REPLAN status=esdf_caught_up raw_revision=%" PRIu64
                " esdf_revision=%" PRIu64 " generation=%" PRIu64,
                raw_revision, published_world->revision, resident_route_generation);
    requestStaticRouteReplan(RouteReleaseReason3D::kBlocked, resident_route_generation);
  }

  const bool initial_route_search_required = resident_route_generation == 0U;
  bool initial_route_search_queued = false;
  bool initial_route_search_replaced_pending = false;
  if (initial_route_search_required) {
    initial_route_search_queued = requestInitialRouteSearch3D(
        published_world, update.telemetry, initial_route_search_replaced_pending);
  }
  if (!world_ready_.exchange(true, std::memory_order_acq_rel)) {
    publishWorldReadiness(true);
  }

  const char* route_search_status = "active_route_preserved";
  if (initial_route_search_required) {
    if (initial_route_search_queued) {
      route_search_status = initial_route_search_replaced_pending
                                ? "initial_replaced_pending"
                                : "initial_queued";
    } else {
      route_search_status = "initial_not_queued";
    }
  }
  const ObservedEsdf3DBuildStats& stats = update.stats;
  const WorldPipelineStatistics3D pipeline_statistics = world_pipeline_->statistics();
  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_ESDF3D_ONLINE revision=%" PRIu64 " raw_revision=%" PRIu64
      " build_ms=%.2f classify_ms=%.2f source_collection_ms=%.2f transform_ms=%.2f "
      "upload_ms=%.2f dimensions=%dx%dx%d known=%zu free=%zu occupied=%zu "
      "unknown=%zu proprioceptive_free=%zu launch_support=%zu mode=%s "
      "recomputed=%zu reused=%zu sources=%zu transform_voxels=%zu "
      "finite_distance_voxels=%zu maximum_distance_m=%.2f recenter=%s "
      "local_world_generation=%" PRIu64 " route_generation=%" PRIu64
      " route_search=%s builds=%" PRIu64 " throttled=%" PRIu64 " dropped_raw=%" PRIu64
      " mode_totals=(full=%" PRIu64 ",reused=%" PRIu64 ")",
      published_world->revision, raw_revision, update.telemetry.build_ms,
      stats.classification_ms, stats.distance_field.source_collection_ms,
      stats.distance_field.transform_ms, update.telemetry.upload_ms,
      published_world->grid.width, published_world->grid.height,
      published_world->grid.depth, stats.known_voxels, stats.free_voxels,
      stats.occupied_voxels, stats.unknown_voxels, stats.proprioceptive_free_voxels,
      stats.launch_support_voxels, observedEsdf3DBuildModeName(stats.mode),
      stats.recomputed_voxels, stats.reused_voxels, stats.distance_field.source_voxels,
      stats.distance_field.transform_voxels,
      stats.distance_field.finite_distance_voxels, update.maximum_distance_m,
      update.recentered ? "true" : "false",
      published_world->local_world_generation.generation, resident_route_generation,
      route_search_status, pipeline_statistics.observedBuilds(),
      pipeline_statistics.throttled_observed_builds,
      pipeline_statistics.dropped_raw_worlds, pipeline_statistics.observed_full_builds,
      pipeline_statistics.observed_reused_builds);
}

} // namespace drone_city_nav
