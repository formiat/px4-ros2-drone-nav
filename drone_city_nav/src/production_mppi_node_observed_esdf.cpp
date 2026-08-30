#include "drone_city_nav/observed_esdf_3d.hpp"

#include <chrono>
#include <cinttypes>
#include <memory>
#include <optional>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  if (!value.has_value()) {
    return nullptr;
  }
  return std::addressof(value.value());
}

} // namespace

std::optional<std::chrono::steady_clock::time_point>
ProductionMppiNode::processObservedEsdf3D(const ProductionMppiRawWorld3D& raw_world) {
  const std::shared_ptr<const ObservedOccupancyGrid3D> occupancy = raw_world.occupancy;
  if (!occupancy) {
    RCLCPP_WARN(get_logger(),
                "PRODUCTION_MPPI_ESDF3D_ONLINE rejected revision=%" PRIu64
                " reason=unavailable_observed_grid",
                raw_world.version.revision);
    return std::nullopt;
  }
  const std::shared_ptr<const VersionedObservedRawWorld3D> execution_owner =
      raw_world.execution_owner;
  if (!execution_owner || !execution_owner->valid() ||
      std::addressof(execution_owner->occupancy()) != occupancy.get() ||
      execution_owner->version().producer_instance_id !=
          raw_world.version.producer_instance_id ||
      execution_owner->version().base_snapshot_revision !=
          raw_world.version.base_snapshot_revision ||
      execution_owner->version().revision != raw_world.version.revision ||
      execution_owner->proprioceptiveFreeSpaceSeed().has_value() ||
      execution_owner->launchSupportContact().has_value()) {
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_ESDF3D_ONLINE rejected revision=%" PRIu64
                 " reason=raw_execution_owner_mismatch",
                 raw_world.version.revision);
    return std::nullopt;
  }

  ProductionMppiNavigation navigation;
  ProductionMppiAppliedControl applied_control;
  ProductionMppiExecutionHorizonOwner execution_horizon_owner;
  {
    const std::scoped_lock lock{input_mutex_};
    navigation = navigation_;
    applied_control = applied_control_;
    execution_horizon_owner = execution_horizon_owner_;
  }
  if (!navigation.world_state_authoritative) {
    return std::nullopt;
  }

  std::shared_ptr<const WorldSnapshot3D> active_world;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    active_world = resident_world_;
  }
  const GridBounds3D& world_bounds = occupancy->bounds();
  const Point3 position{navigation.state.x, navigation.state.y, navigation.state.z};
  const std::optional<ProprioceptiveFreeSpaceSeed3D> free_space_seed =
      prepareObservedExecutionEvidence3D(raw_world, navigation, applied_control,
                                         execution_horizon_owner);
  const LaunchSupportContact3D* const launch_support_contact =
      optionalAddress(launch_support_contact_);
  GridBounds3D local_bounds;
  bool recenter = true;
  if (active_world && active_world->distances_m && active_world->grid.depth > 1 &&
      active_world->grid.outside_is_unknown) {
    local_bounds = active_world->observed_esdf_resource.local_occupancy
                       ? active_world->observed_esdf_resource.local_occupancy->bounds()
                       : GridBounds3D{
                             .origin_x = active_world->grid.origin_x_m,
                             .origin_y = active_world->grid.origin_y_m,
                             .origin_z = active_world->grid.origin_z_m,
                             .resolution_m = active_world->grid.resolution_m,
                             .width_cells = active_world->grid.width,
                             .height_cells = active_world->grid.height,
                             .depth_cells = active_world->grid.depth,
                         };
    recenter = localObservedEsdfNeedsRecenter(local_bounds, world_bounds, position,
                                              no_static_3d_esdf_window_);
  }
  if (recenter) {
    local_bounds = selectLocalObservedEsdfBounds(world_bounds, position,
                                                 no_static_3d_esdf_window_);
  }

  const double maximum_distance_m = requiredObservedEsdfMaximumDistanceM(
      static_cast<double>(mppi_config_.risk.preferred_distance_m),
      physical_footprint_config_, local_bounds.resolution_m);
  const GridBounds3D source_bounds = knownObstacleDistanceSourceBounds3D(
      world_bounds, local_bounds, maximum_distance_m);
  const std::uint64_t local_fingerprint =
      knownObstacleFingerprint3D(*occupancy, source_bounds);
  const bool launch_support_resolution_pending = !launch_support_evaluated_;
  const ProprioceptiveFreeSpaceSeed3D* const resident_free_space_seed =
      active_world ? optionalAddress(active_world->proprioceptive_free_space_seed)
                   : nullptr;
  const ProprioceptiveFreeSpaceSeed3D* const current_free_space_seed =
      optionalAddress(free_space_seed);
  const bool transient_seed_unchanged =
      active_world &&
      ((resident_free_space_seed == nullptr && current_free_space_seed == nullptr) ||
       (resident_free_space_seed != nullptr && current_free_space_seed != nullptr &&
        sameProprioceptiveFreeSpaceSeed3D(*resident_free_space_seed,
                                          *current_free_space_seed)));
  const LaunchSupportContact3D* const resident_launch_support =
      active_world ? optionalAddress(active_world->launch_support_contact) : nullptr;
  const bool launch_support_unchanged =
      active_world &&
      ((resident_launch_support == nullptr && launch_support_contact == nullptr) ||
       (resident_launch_support != nullptr && launch_support_contact != nullptr &&
        sameLaunchSupportContact3D(*resident_launch_support,
                                   *launch_support_contact))) &&
      active_world->launch_support_resolution_pending ==
          launch_support_resolution_pending;
  if (active_world && !launch_support_unchanged) {
    std::shared_ptr<const PlannerSearchTransaction3D> superseded_transaction;
    {
      const std::scoped_lock lock{route_planning_queue_mutex_};
      if (pending_route_planning_work_) {
        superseded_transaction = pending_route_planning_work_->transaction;
        pending_route_planning_work_.reset();
      }
    }
    if (superseded_transaction != nullptr) {
      finishStaticRouteSearch(*superseded_transaction);
    }
    RCLCPP_INFO(get_logger(),
                "EXECUTION_EVIDENCE_WORLD_CHANGED raw_revision=%" PRIu64
                " free_space_seed=%s support_active=%s resolution_pending=%s"
                " incremental_refresh=true",
                raw_world.version.revision,
                free_space_seed.has_value() ? "true" : "false",
                launch_support_contact != nullptr ? "true" : "false",
                launch_support_resolution_pending ? "true" : "false");
  } else if (active_world && !transient_seed_unchanged) {
    RCLCPP_INFO(get_logger(),
                "TRANSIENT_EXECUTION_EVIDENCE_CHANGED raw_revision=%" PRIu64
                " free_space_seed=%s persistent_world_change=false",
                raw_world.version.revision,
                free_space_seed.has_value() ? "true" : "false");
  }
  const bool active_incremental_parent_available =
      active_world && active_world->distances_m && !recenter &&
      active_world->observed_occupancy &&
      productionWorldGenerationCoherent(*active_world) &&
      active_world->observed_esdf_resource.local_occupancy &&
      active_world->observed_esdf_resource.known_obstacle_distance &&
      active_world->observed_esdf_resource.classification_override_cells &&
      active_world->observed_esdf_resource.coverage.coherent() &&
      active_world->observed_esdf_resource.coverage.source_raw_version.revision ==
          active_world->source_raw_revision &&
      active_world->observed_esdf_resource.coverage.maximum_distance_m ==
          maximum_distance_m;
  const bool same_raw_lineage =
      active_incremental_parent_available && !raw_world.full_reset &&
      active_world->observed_esdf_resource.coverage.source_raw_version
              .producer_instance_id == raw_world.version.producer_instance_id &&
      active_world->observed_esdf_resource.coverage.source_raw_version
              .base_snapshot_revision == raw_world.version.base_snapshot_revision &&
      active_world->observed_esdf_resource.coverage.source_raw_version.revision <=
          raw_world.version.revision;
  const bool known_obstacle_sources_unchanged =
      same_raw_lineage && launch_support_unchanged &&
      active_world->source_occupied_fingerprint == local_fingerprint;
  const std::uint64_t completed_esdf_builds =
      no_static_esdf_builds_.load(std::memory_order_relaxed);
  const bool periodic_full_audit = observedEsdfFullAuditDue(
      completed_esdf_builds, no_static_3d_esdf_full_audit_interval_builds_);
  const auto build_started_at = std::chrono::steady_clock::now();
  const bool first_build =
      no_static_3d_esdf_last_build_time_ == std::chrono::steady_clock::time_point{};
  const bool build_rate_due =
      first_build || std::chrono::duration<double>(build_started_at -
                                                   no_static_3d_esdf_last_build_time_)
                             .count() >= 1.0 / no_static_3d_esdf_update_rate_hz_;
  const std::shared_ptr<const VersionedObservedRawWorld3D> observed_raw_world_owner =
      execution_owner->deriveRouteEvidence(free_space_seed, launch_support_contact_);
  if (!observed_raw_world_owner ||
      !execution_owner->sharesObservationOwner(*observed_raw_world_owner) ||
      std::addressof(observed_raw_world_owner->occupancy()) != occupancy.get() ||
      observed_raw_world_owner->occupiedSnapshot() !=
          execution_owner->occupiedSnapshot()) {
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_ESDF3D_ONLINE rejected revision=%" PRIu64
                 " reason=route_evidence_derivation_failed",
                 raw_world.version.revision);
    return std::nullopt;
  }
  const bool already_current =
      known_obstacle_sources_unchanged && !periodic_full_audit &&
      active_world->source_raw_revision == raw_world.version.revision;
  if (already_current) {
    if (!transient_seed_unchanged) {
      bool refreshed{false};
      {
        const std::scoped_lock lock{world_generation_publication_mutex_,
                                    esdf_state_mutex_};
        if (resident_world_ && productionWorldGenerationCoherent(*resident_world_) &&
            resident_world_->local_world_generation.sameSnapshot(
                active_world->local_world_generation) &&
            resident_world_->revision == active_world->revision &&
            resident_world_->observed_occupancy == active_world->observed_occupancy) {
          WorldSnapshot3D refreshed_world = *resident_world_;
          refreshed_world.observed_raw_world_owner = observed_raw_world_owner;
          refreshed_world.proprioceptive_free_space_seed = free_space_seed;
          resident_world_ =
              std::make_shared<const WorldSnapshot3D>(std::move(refreshed_world));
          refreshed = true;
        }
      }
      if (!refreshed) {
        RCLCPP_INFO(get_logger(),
                    "PRODUCTION_MPPI_ESDF3D_ONLINE deferred raw_revision=%" PRIu64
                    " reason=superseded_transient_evidence_parent",
                    raw_world.version.revision);
        return std::chrono::steady_clock::now();
      }
      RCLCPP_INFO(get_logger(),
                  "TRANSIENT_EXECUTION_EVIDENCE_REFRESHED raw_revision=%" PRIu64
                  " esdf_revision=%" PRIu64 " gpu_upload=false",
                  raw_world.version.revision, active_world->revision);
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NO_STATIC_ESDF3D_DEFERRED raw_revision=%" PRIu64
        " reason=already_current reconstruction_ms=%.2f raw_updates=%" PRIu64
        " builds=%" PRIu64 " throttled=%" PRIu64,
        raw_world.version.revision, raw_world.reconstruction_ms,
        no_static_raw_updates_.load(std::memory_order_relaxed),
        no_static_esdf_builds_.load(std::memory_order_relaxed),
        no_static_esdf_throttled_updates_.load(std::memory_order_relaxed));
    return std::nullopt;
  }
  if ((!known_obstacle_sources_unchanged || periodic_full_audit) && active_world &&
      !recenter && !build_rate_due) {
    no_static_esdf_throttled_updates_.fetch_add(1U, std::memory_order_relaxed);
    const auto update_period =
        std::chrono::duration<double>{1.0 / no_static_3d_esdf_update_rate_hz_};
    const auto retry_not_before =
        no_static_3d_esdf_last_build_time_ +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(update_period);
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "NO_STATIC_ESDF3D_DEFERRED raw_revision=%" PRIu64
        " reason=rate_limited reconstruction_ms=%.2f raw_updates=%" PRIu64
        " builds=%" PRIu64 " throttled=%" PRIu64,
        raw_world.version.revision, raw_world.reconstruction_ms,
        no_static_raw_updates_.load(std::memory_order_relaxed),
        no_static_esdf_builds_.load(std::memory_order_relaxed),
        no_static_esdf_throttled_updates_.load(std::memory_order_relaxed));
    return retry_not_before;
  }

  std::optional<PreviousObservedEsdf3D> previous;
  if (same_raw_lineage) {
    previous = PreviousObservedEsdf3D{
        .grid = active_world->grid,
        .distances_m = active_world->distances_m,
        .known_obstacle_distance =
            active_world->observed_esdf_resource.known_obstacle_distance,
        .source_occupancy = active_world->observed_occupancy,
        .local_occupancy = active_world->observed_esdf_resource.local_occupancy,
        .classification_override_cells =
            *active_world->observed_esdf_resource.classification_override_cells,
        .occupancy_fingerprint = active_world->revision,
        .maximum_distance_m =
            active_world->observed_esdf_resource.coverage.maximum_distance_m,
    };
  }
  ObservedEsdf3D field = updateObservedEsdf3D(
      *occupancy, local_bounds, maximum_distance_m,
      previous.has_value() ? std::addressof(*previous) : nullptr,
      raw_world.dirty_chunks, raw_world.full_reset || recenter || periodic_full_audit,
      no_static_3d_esdf_incremental_maximum_rebuild_ratio_, planning_worker_pool_.get(),
      launch_support_contact);
  const std::shared_ptr<const std::vector<float>> host_distances = field.distances_m;
  std::shared_ptr<const std::vector<GridIndex3D>> classification_override_cells =
      std::make_shared<const std::vector<GridIndex3D>>(
          std::move(field.classification_override_cells));

  const RawMapVersion parent_raw_version =
      field.stats.mode == ObservedEsdf3DBuildMode::kFull
          ? RawMapVersion{}
          : active_world->observed_esdf_resource.coverage.source_raw_version;
  const std::uint64_t parent_esdf_fingerprint =
      field.stats.mode == ObservedEsdf3DBuildMode::kFull ? 0U : active_world->revision;
  const ObservedEsdfCoverage3D coverage{
      .source_raw_version = raw_world.version,
      .parent_raw_version = parent_raw_version,
      .raw_local_fingerprint = local_fingerprint,
      .esdf_fingerprint = field.occupancy_fingerprint,
      .parent_esdf_fingerprint = parent_esdf_fingerprint,
      .total_voxels = host_distances->size(),
      .recomputed_voxels = field.stats.recomputed_voxels,
      .reused_voxels = field.stats.reused_voxels,
      .maximum_distance_m = maximum_distance_m,
      .mode = field.stats.mode,
  };
  if (!coverage.coherent()) {
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_ESDF3D_ONLINE rejected revision=%" PRIu64
                 " reason=invalid_coverage mode=%s",
                 raw_world.version.revision,
                 observedEsdf3DBuildModeName(field.stats.mode));
    return std::nullopt;
  }

  ProductionWorldBuildTelemetry3D world_build;
  WorldSnapshot3D mutable_world;
  mutable_world.producer_instance_id = raw_world.version.producer_instance_id;
  mutable_world.revision = field.occupancy_fingerprint;
  mutable_world.source_raw_revision = raw_world.version.revision;
  mutable_world.source_occupied_fingerprint = local_fingerprint;
  mutable_world.raw_occupied_fingerprint =
      execution_owner->occupiedContentFingerprint();
  mutable_world.planner_parent_raw_revision =
      same_raw_lineage && active_world ? active_world->source_raw_revision : 0U;
  mutable_world.source_stamp_ns = raw_world.source_stamp_ns;
  mutable_world.ready_stamp_ns = get_clock()->now().nanoseconds();
  world_build.build_ms =
      field.stats.distance_cache.duration_ms + field.stats.classification_ms;
  world_build.conversion_ms =
      raw_world.reconstruction_ms + field.stats.classification_ms;
  mutable_world.grid = field.grid;
  mutable_world.distances_m = host_distances;
  mutable_world.observed_occupancy = occupancy;
  mutable_world.observed_raw_world_owner = observed_raw_world_owner;
  mutable_world.observed_esdf_resource = ObservedEsdfResource3D{
      .local_occupancy = field.local_occupancy,
      .known_obstacle_distance = field.known_obstacle_distance,
      .classification_override_cells = std::move(classification_override_cells),
      .coverage = coverage};
  mutable_world.planner_dirty_chunks = raw_world.dirty_chunks;
  mutable_world.proprioceptive_free_space_seed = free_space_seed;
  mutable_world.launch_support_contact = launch_support_contact_;
  mutable_world.launch_support_resolution_pending = launch_support_resolution_pending;
  mutable_world.planner_full_reset = raw_world.full_reset || !same_raw_lineage;

  const std::shared_ptr<const ProductionNavigationObjective> current_objective =
      navigationObjective();
  std::shared_ptr<const WorldSnapshot3D> published_world;
  bool local_world_generation_valid{false};
  bool resident_parent_valid{true};
  const bool parent_required = field.stats.mode != ObservedEsdf3DBuildMode::kFull;
  const bool upload_required = field.stats.mode != ObservedEsdf3DBuildMode::kReused;
  std::unique_lock generation_lock{world_generation_publication_mutex_};
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    if (parent_required) {
      const ObservedEsdfCoverage3D& resident_coverage =
          resident_world_ != nullptr ? resident_world_->observed_esdf_resource.coverage
                                     : ObservedEsdfCoverage3D{};
      resident_parent_valid =
          active_world != nullptr && resident_world_ != nullptr &&
          productionWorldGenerationCoherent(*resident_world_) &&
          resident_world_->revision == parent_esdf_fingerprint &&
          resident_world_->distances_m == active_world->distances_m &&
          resident_world_->observed_occupancy == active_world->observed_occupancy &&
          resident_world_->observed_esdf_resource.local_occupancy ==
              active_world->observed_esdf_resource.local_occupancy &&
          resident_world_->observed_esdf_resource.known_obstacle_distance ==
              active_world->observed_esdf_resource.known_obstacle_distance &&
          resident_coverage.source_raw_version.producer_instance_id ==
              parent_raw_version.producer_instance_id &&
          resident_coverage.source_raw_version.base_snapshot_revision ==
              parent_raw_version.base_snapshot_revision &&
          resident_coverage.source_raw_version.revision ==
              parent_raw_version.revision &&
          resident_world_->local_world_generation.gpu_esdf_revision ==
              parent_esdf_fingerprint;
    }
  }
  if (!resident_parent_valid) {
    generation_lock.unlock();
    RCLCPP_INFO(get_logger(),
                "PRODUCTION_MPPI_ESDF3D_ONLINE deferred raw_revision=%" PRIu64
                " reason=superseded_esdf_parent",
                raw_world.version.revision);
    return std::chrono::steady_clock::now();
  }
  mppi::EsdfUploadResult upload{
      .accepted = true, .upload_ms = 0.0, .revision = field.occupancy_fingerprint};
  if (upload_required) {
    upload = engine_->updateEsdf(mppi::EsdfSnapshot{
        field.grid, *host_distances, field.occupancy_fingerprint, field.dirty_regions});
    if (!upload.accepted) {
      return std::nullopt;
    }
  }
  world_build.upload_ms = upload.upload_ms;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    // World publication is independent of route admission. A completed build
    // can only replace the resident immutable world and its telemetry event.
    const std::optional<LocalWorldGeneration> local_world_generation =
        local_world_generation_counter_.issue(raw_world.version, navigation.revision,
                                              mutable_world.revision, upload.revision);
    if (local_world_generation.has_value()) {
      mutable_world.local_world_generation = *local_world_generation;
      published_world =
          std::make_shared<const WorldSnapshot3D>(std::move(mutable_world));
      local_world_generation_valid =
          productionWorldGenerationCoherent(*published_world);
      if (local_world_generation_valid) {
        resident_world_ = published_world;
        resident_world_build_telemetry_ = world_build;
      }
    }
    if (!local_world_generation_valid && upload_required) {
      resident_world_.reset();
      resident_world_build_telemetry_ = {};
    }
  }
  generation_lock.unlock();
  if (!local_world_generation_valid) {
    rejected_world_generation_publications_.fetch_add(1U, std::memory_order_relaxed);
    if (world_ready_.exchange(false, std::memory_order_acq_rel)) {
      publishWorldReadiness(false);
    }
    RCLCPP_ERROR(get_logger(),
                 "PRODUCTION_MPPI_ESDF3D_ONLINE rejected raw_revision=%" PRIu64
                 " reason=mixed_local_world_generation",
                 raw_world.version.revision);
    return std::nullopt;
  }
  if (field.stats.mode != ObservedEsdf3DBuildMode::kReused) {
    no_static_3d_esdf_last_build_time_ = build_started_at;
    no_static_esdf_builds_.fetch_add(1U, std::memory_order_relaxed);
  }
  std::atomic<std::uint64_t>* mode_counter = nullptr;
  switch (field.stats.mode) {
    case ObservedEsdf3DBuildMode::kFull:
      mode_counter = &observed_esdf_3d_counters_.full_builds;
      break;
    case ObservedEsdf3DBuildMode::kIncremental:
      mode_counter = &observed_esdf_3d_counters_.incremental_builds;
      break;
    case ObservedEsdf3DBuildMode::kReused:
      mode_counter = &observed_esdf_3d_counters_.reused_builds;
      break;
  }
  if (mode_counter != nullptr) {
    mode_counter->fetch_add(1U, std::memory_order_relaxed);
  }
  observed_esdf_3d_counters_.recomputed_voxels.fetch_add(field.stats.recomputed_voxels,
                                                         std::memory_order_relaxed);
  observed_esdf_3d_counters_.reused_voxels.fetch_add(field.stats.reused_voxels,
                                                     std::memory_order_relaxed);
  const std::uint64_t blocked_raw_revision =
      observed_route_blocked_raw_revision_.load(std::memory_order_acquire);
  const std::uint64_t dispatched_raw_revision =
      observed_route_replan_dispatched_raw_revision_.load(std::memory_order_acquire);
  const std::shared_ptr<const ExecutionRouteSnapshot3D> resident_execution =
      execution_route_store_.snapshot();
  const std::uint64_t resident_route_generation =
      resident_execution != nullptr ? resident_execution->routeGenerationHighWater()
                                    : 0U;
  if (blocked_raw_revision != 0U &&
      blocked_raw_revision <= raw_world.version.revision &&
      dispatched_raw_revision < blocked_raw_revision &&
      resident_route_generation != 0U) {
    observed_route_replan_dispatched_raw_revision_.store(blocked_raw_revision,
                                                         std::memory_order_release);
    RCLCPP_INFO(get_logger(),
                "OBSERVED_ROUTE_REPLAN status=esdf_caught_up raw_revision=%" PRIu64
                " esdf_revision=%" PRIu64 " generation=%" PRIu64,
                raw_world.version.revision, published_world->revision,
                resident_route_generation);
    requestStaticRouteReplan(RouteReleaseReason3D::kBlocked, resident_route_generation);
  }
  const bool initial_route_search_required = resident_route_generation == 0U;
  bool initial_route_search_queued = false;
  bool initial_route_search_already_pending = false;
  if (initial_route_search_required && current_objective) {
    const std::shared_ptr<const PlannerSearchTransaction3D> transaction =
        makePlannerSearchTransaction3D(
            published_world, captureResidentPlannerWorld3D(*published_world),
            makeStaticRouteObjective(*current_objective),
            StaticRouteSearchRequestIdentity{
                .kind = StaticRouteSearchRequestKind::kInitial,
            });
    if (transaction == nullptr) {
      RCLCPP_ERROR(get_logger(),
                   "PRODUCTION_MPPI_ROUTE3D status=invalid_initial_transaction "
                   "raw_revision=%" PRIu64,
                   published_world->source_raw_revision);
    }
    {
      const std::scoped_lock lock{route_planning_queue_mutex_};
      if (pending_route_planning_work_) {
        initial_route_search_already_pending = true;
      } else if (transaction != nullptr) {
        pending_route_planning_work_ = ProductionRoutePlanningWork3D{
            .transaction = transaction,
            .world_telemetry = world_build,
            .continuation_session = nullptr,
        };
        initial_route_search_queued = true;
      }
    }
    if (initial_route_search_queued) {
      route_planning_queue_condition_.notify_all();
    }
  }
  if (!world_ready_.exchange(true, std::memory_order_acq_rel)) {
    publishWorldReadiness(true);
  }

  RCLCPP_INFO(
      get_logger(),
      "PRODUCTION_MPPI_ESDF3D_ONLINE revision=%" PRIu64 " raw_revision=%" PRIu64
      " build_ms=%.2f classify_ms=%.2f upload_ms=%.2f dimensions=%dx%dx%d "
      "known=%zu free=%zu occupied=%zu unknown=%zu proprioceptive_free=%zu "
      "launch_support=%zu mode=%s fallback=%s changed=%zu recomputed=%zu reused=%zu "
      "classified=%zu classification_reused=%zu sources=%zu source_chunks=%zu "
      "distance_chunks=%zu "
      "finite_distance_voxels=%zu inserted_sources=%zu removed_sources=%zu "
      "recomputed_chunks=%zu reused_chunks=%zu changed_chunks=%zu queried_voxels=%zu "
      "source_index_ms=%.2f distance_query_ms=%.2f maximum_distance_m=%.2f audit=%s "
      "recenter=%s local_world_generation=%" PRIu64 " route_generation=%" PRIu64
      " route_search=%s builds=%" PRIu64 " throttled=%" PRIu64 " dropped_raw=%" PRIu64
      " mode_totals=(full=%" PRIu64 ",incremental=%" PRIu64 ",reused=%" PRIu64 ")",
      published_world->revision, raw_world.version.revision, world_build.build_ms,
      field.stats.classification_ms, world_build.upload_ms, published_world->grid.width,
      published_world->grid.height, published_world->grid.depth,
      field.stats.known_voxels, field.stats.free_voxels, field.stats.occupied_voxels,
      field.stats.unknown_voxels, field.stats.proprioceptive_free_voxels,
      field.stats.launch_support_voxels, observedEsdf3DBuildModeName(field.stats.mode),
      field.stats.incremental_fallback ? "true" : "false", field.stats.changed_voxels,
      field.stats.recomputed_voxels, field.stats.reused_voxels,
      field.stats.classified_voxels, field.stats.reused_classification_voxels,
      field.stats.distance_cache.source_voxels,
      field.stats.distance_cache.source_chunks,
      field.stats.distance_cache.stored_distance_chunks,
      field.stats.distance_cache.finite_distance_voxels,
      field.stats.distance_cache.inserted_sources,
      field.stats.distance_cache.removed_sources,
      field.stats.distance_cache.recomputed_chunks,
      field.stats.distance_cache.reused_chunks,
      field.stats.distance_cache.changed_chunks,
      field.stats.distance_cache.queried_voxels,
      field.stats.distance_cache.source_index_ms,
      field.stats.distance_cache.distance_query_ms, maximum_distance_m,
      periodic_full_audit ? "true" : "false", recenter ? "true" : "false",
      published_world->local_world_generation.generation, resident_route_generation,
      !initial_route_search_required
          ? "active_route_preserved"
          : (initial_route_search_queued
                 ? "initial_queued"
                 : (initial_route_search_already_pending ? "initial_already_pending"
                                                         : "initial_not_queued")),
      no_static_esdf_builds_.load(std::memory_order_relaxed),
      no_static_esdf_throttled_updates_.load(std::memory_order_relaxed),
      dropped_raw_snapshots_.load(std::memory_order_relaxed),
      observed_esdf_3d_counters_.full_builds.load(std::memory_order_relaxed),
      observed_esdf_3d_counters_.incremental_builds.load(std::memory_order_relaxed),
      observed_esdf_3d_counters_.reused_builds.load(std::memory_order_relaxed));
  return std::nullopt;
}

} // namespace drone_city_nav
