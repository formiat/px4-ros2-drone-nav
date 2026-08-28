#include "drone_city_nav/observed_esdf_3d.hpp"

#include <chrono>
#include <cinttypes>
#include <memory>
#include <optional>
#include <utility>

#include "production_mppi_node.hpp"
#include "production_mppi_route_world.hpp"

namespace drone_city_nav {

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

  std::optional<ProductionMppiPreparedEsdf> active_prepared;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    active_prepared = prepared_esdf_;
  }
  const GridBounds3D& world_bounds = occupancy->bounds();
  const Point3 position{navigation.state.x, navigation.state.y, navigation.state.z};
  const std::optional<ProprioceptiveFreeSpaceSeed3D> free_space_seed =
      prepareObservedExecutionEvidence3D(raw_world, navigation, applied_control,
                                         execution_horizon_owner);
  const LaunchSupportContact3D* const launch_support_contact =
      launch_support_contact_ ? &*launch_support_contact_ : nullptr;
  GridBounds3D local_bounds;
  bool recenter = true;
  if (active_prepared && active_prepared->distances_m &&
      active_prepared->grid.depth > 1 && active_prepared->grid.outside_is_unknown) {
    local_bounds =
        active_prepared->observed_esdf_resource.local_occupancy
            ? active_prepared->observed_esdf_resource.local_occupancy->bounds()
            : GridBounds3D{
                  .origin_x = active_prepared->grid.origin_x_m,
                  .origin_y = active_prepared->grid.origin_y_m,
                  .origin_z = active_prepared->grid.origin_z_m,
                  .resolution_m = active_prepared->grid.resolution_m,
                  .width_cells = active_prepared->grid.width,
                  .height_cells = active_prepared->grid.height,
                  .depth_cells = active_prepared->grid.depth,
              };
    recenter = localObservedEsdfNeedsRecenter(local_bounds, world_bounds, position,
                                              no_static_3d_esdf_window_);
  }
  if (recenter) {
    local_bounds = selectLocalObservedEsdfBounds(world_bounds, position,
                                                 no_static_3d_esdf_window_);
  }

  const std::uint64_t local_fingerprint =
      knownObstacleFingerprint3D(*occupancy, local_bounds);
  const bool launch_support_resolution_pending = !launch_support_evaluated_;
  const bool transient_seed_unchanged =
      active_prepared &&
      active_prepared->proprioceptive_free_space_seed.has_value() ==
          free_space_seed.has_value() &&
      (!free_space_seed.has_value() ||
       sameProprioceptiveFreeSpaceSeed3D(
           *active_prepared->proprioceptive_free_space_seed, *free_space_seed));
  const bool launch_support_unchanged =
      active_prepared &&
      active_prepared->launch_support_contact.has_value() ==
          launch_support_contact_.has_value() &&
      (!launch_support_contact_.has_value() ||
       sameLaunchSupportContact3D(*active_prepared->launch_support_contact,
                                  *launch_support_contact_)) &&
      active_prepared->launch_support_resolution_pending ==
          launch_support_resolution_pending;
  if (active_prepared && !launch_support_unchanged) {
    {
      const std::scoped_lock lock{guide_queue_mutex_};
      pending_guide_world_.reset();
    }
    RCLCPP_INFO(get_logger(),
                "EXECUTION_EVIDENCE_WORLD_CHANGED raw_revision=%" PRIu64
                " free_space_seed=%s support_active=%s resolution_pending=%s"
                " incremental_refresh=true",
                raw_world.version.revision,
                free_space_seed.has_value() ? "true" : "false",
                launch_support_contact != nullptr ? "true" : "false",
                launch_support_resolution_pending ? "true" : "false");
  } else if (active_prepared && !transient_seed_unchanged) {
    RCLCPP_INFO(get_logger(),
                "TRANSIENT_EXECUTION_EVIDENCE_CHANGED raw_revision=%" PRIu64
                " free_space_seed=%s persistent_world_change=false",
                raw_world.version.revision,
                free_space_seed.has_value() ? "true" : "false");
  }
  const double maximum_distance_m = requiredObservedEsdfMaximumDistanceM(
      static_cast<double>(mppi_config_.risk.preferred_distance_m),
      physical_footprint_config_, local_bounds.resolution_m);
  const bool active_incremental_parent_available =
      active_prepared && active_prepared->distances_m && !recenter &&
      active_prepared->observed_occupancy &&
      productionWorldGenerationCoherent(*active_prepared) &&
      active_prepared->observed_esdf_resource.local_occupancy &&
      active_prepared->observed_esdf_resource.nearest_obstacle_indices &&
      active_prepared->observed_esdf_resource.classification_override_cells &&
      active_prepared->observed_esdf_resource.coverage.coherent() &&
      active_prepared->observed_esdf_resource.coverage.source_raw_version.revision ==
          active_prepared->source_raw_revision &&
      active_prepared->observed_esdf_resource.coverage.maximum_distance_m ==
          maximum_distance_m;
  const bool same_raw_lineage =
      active_incremental_parent_available && !raw_world.full_reset &&
      active_prepared->observed_esdf_resource.coverage.source_raw_version
              .producer_instance_id == raw_world.version.producer_instance_id &&
      active_prepared->observed_esdf_resource.coverage.source_raw_version
              .base_snapshot_revision == raw_world.version.base_snapshot_revision &&
      active_prepared->observed_esdf_resource.coverage.source_raw_version.revision <=
          raw_world.version.revision;
  const bool local_occupancy_unchanged =
      same_raw_lineage && launch_support_unchanged &&
      active_prepared->source_occupied_fingerprint == local_fingerprint;
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
      local_occupancy_unchanged && !periodic_full_audit &&
      active_prepared->source_raw_revision == raw_world.version.revision;
  if (already_current) {
    if (!transient_seed_unchanged) {
      bool refreshed{false};
      {
        const std::scoped_lock lock{world_generation_publication_mutex_,
                                    esdf_state_mutex_};
        if (prepared_esdf_ && productionWorldGenerationCoherent(*prepared_esdf_) &&
            prepared_esdf_->local_world_generation.sameSnapshot(
                active_prepared->local_world_generation) &&
            prepared_esdf_->revision == active_prepared->revision &&
            prepared_esdf_->observed_occupancy == active_prepared->observed_occupancy) {
          prepared_esdf_->observed_raw_world_owner = observed_raw_world_owner;
          prepared_esdf_->proprioceptive_free_space_seed = free_space_seed;
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
                  raw_world.version.revision, active_prepared->revision);
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
  if ((!local_occupancy_unchanged || periodic_full_audit) && active_prepared &&
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
        .grid = active_prepared->grid,
        .distances_m = *active_prepared->distances_m,
        .nearest_obstacle_indices =
            *active_prepared->observed_esdf_resource.nearest_obstacle_indices,
        .source_occupancy = active_prepared->observed_occupancy,
        .local_occupancy = active_prepared->observed_esdf_resource.local_occupancy,
        .classification_override_cells =
            *active_prepared->observed_esdf_resource.classification_override_cells,
        .occupancy_fingerprint = active_prepared->revision,
        .maximum_distance_m =
            active_prepared->observed_esdf_resource.coverage.maximum_distance_m,
    };
  }
  ObservedEsdf3D field;
  std::shared_ptr<const std::vector<float>> host_distances;
  std::shared_ptr<const std::vector<std::size_t>> nearest_obstacle_indices;
  std::shared_ptr<const std::vector<GridIndex3D>> classification_override_cells;
  if (local_occupancy_unchanged && !periodic_full_audit) {
    field.grid = active_prepared->grid;
    field.local_occupancy = active_prepared->observed_esdf_resource.local_occupancy;
    field.occupancy_fingerprint = active_prepared->revision;
    field.maximum_distance_m = maximum_distance_m;
    field.stats.known_voxels = field.local_occupancy->knownVoxelCount();
    field.stats.free_voxels = field.local_occupancy->freeVoxelCount();
    field.stats.occupied_voxels = field.local_occupancy->occupiedVoxelCount();
    field.stats.unknown_voxels =
        active_prepared->distances_m->size() - field.stats.known_voxels;
    field.stats.reused_voxels = active_prepared->distances_m->size();
    field.stats.dirty_chunks = raw_world.dirty_chunks.size();
    field.stats.mode = ObservedEsdf3DBuildMode::kReused;
    host_distances = active_prepared->distances_m;
    nearest_obstacle_indices =
        active_prepared->observed_esdf_resource.nearest_obstacle_indices;
    classification_override_cells =
        active_prepared->observed_esdf_resource.classification_override_cells;
  } else {
    field = updateObservedEsdf3D(
        *occupancy, local_bounds, maximum_distance_m,
        previous.has_value() ? std::addressof(*previous) : nullptr,
        raw_world.dirty_chunks, raw_world.full_reset || recenter || periodic_full_audit,
        no_static_3d_esdf_incremental_maximum_rebuild_ratio_,
        planning_worker_pool_.get(), launch_support_contact);
    if (field.stats.mode == ObservedEsdf3DBuildMode::kReused && active_prepared &&
        active_prepared->revision == field.occupancy_fingerprint) {
      host_distances = active_prepared->distances_m;
    } else {
      host_distances =
          std::make_shared<const std::vector<float>>(std::move(field.distances_m));
    }
    nearest_obstacle_indices = std::make_shared<const std::vector<std::size_t>>(
        std::move(field.nearest_obstacle_indices));
    classification_override_cells = std::make_shared<const std::vector<GridIndex3D>>(
        std::move(field.classification_override_cells));
  }

  const RawMapVersion parent_raw_version =
      field.stats.mode == ObservedEsdf3DBuildMode::kFull
          ? RawMapVersion{}
          : active_prepared->observed_esdf_resource.coverage.source_raw_version;
  const std::uint64_t parent_esdf_fingerprint =
      field.stats.mode == ObservedEsdf3DBuildMode::kFull ? 0U
                                                         : active_prepared->revision;
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

  ProductionMppiPreparedEsdf world_update;
  world_update.producer_instance_id = raw_world.version.producer_instance_id;
  world_update.revision = field.occupancy_fingerprint;
  world_update.source_raw_revision = raw_world.version.revision;
  world_update.source_occupied_fingerprint = local_fingerprint;
  world_update.source_stamp_ns = raw_world.source_stamp_ns;
  world_update.ready_stamp_ns = get_clock()->now().nanoseconds();
  world_update.build_ms =
      field.stats.distance_field.duration_ms + field.stats.classification_ms;
  world_update.esdf_x_pass_ms = field.stats.distance_field.x_pass_ms;
  world_update.esdf_y_pass_ms = field.stats.distance_field.y_pass_ms;
  world_update.esdf_z_pass_ms = field.stats.distance_field.z_pass_ms;
  world_update.esdf_finalize_ms = field.stats.distance_field.finalize_ms;
  world_update.conversion_ms =
      raw_world.reconstruction_ms + field.stats.classification_ms;
  world_update.grid = field.grid;
  world_update.distances_m = host_distances;
  world_update.observed_occupancy = occupancy;
  world_update.observed_raw_world_owner = observed_raw_world_owner;
  world_update.observed_esdf_resource = ObservedEsdfResource3D{
      .local_occupancy = field.local_occupancy,
      .nearest_obstacle_indices = std::move(nearest_obstacle_indices),
      .classification_override_cells = std::move(classification_override_cells),
      .coverage = coverage};
  world_update.proprioceptive_free_space_seed = free_space_seed;
  world_update.launch_support_contact = launch_support_contact_;
  world_update.launch_support_resolution_pending = launch_support_resolution_pending;

  const std::shared_ptr<const ProductionNavigationObjective> current_objective =
      navigationObjective();
  ProductionMppiPreparedEsdf prepared;
  bool local_world_generation_valid{false};
  bool resident_parent_valid{true};
  const bool parent_required = field.stats.mode != ObservedEsdf3DBuildMode::kFull;
  const bool upload_required = field.stats.mode != ObservedEsdf3DBuildMode::kReused;
  std::unique_lock generation_lock{world_generation_publication_mutex_};
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    if (prepared_esdf_) {
      prepared = *prepared_esdf_;
    }
    if (parent_required) {
      const ObservedEsdfCoverage3D& resident_coverage =
          prepared.observed_esdf_resource.coverage;
      resident_parent_valid =
          active_prepared.has_value() && prepared_esdf_.has_value() &&
          productionWorldGenerationCoherent(prepared) &&
          prepared.revision == parent_esdf_fingerprint &&
          prepared.distances_m == active_prepared->distances_m &&
          prepared.observed_occupancy == active_prepared->observed_occupancy &&
          prepared.observed_esdf_resource.local_occupancy ==
              active_prepared->observed_esdf_resource.local_occupancy &&
          resident_coverage.source_raw_version.producer_instance_id ==
              parent_raw_version.producer_instance_id &&
          resident_coverage.source_raw_version.base_snapshot_revision ==
              parent_raw_version.base_snapshot_revision &&
          resident_coverage.source_raw_version.revision ==
              parent_raw_version.revision &&
          prepared.local_world_generation.gpu_esdf_revision == parent_esdf_fingerprint;
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
  world_update.upload_ms = upload.upload_ms;
  {
    const std::scoped_lock lock{esdf_state_mutex_};
    if (prepared_esdf_) {
      prepared = *prepared_esdf_;
    }

    // Route activation and coherent-world publication share this mutex. Merge the
    // completed world build into the latest resident route state so a build that
    // started before activation cannot restore an older route generation.
    const std::optional<LocalWorldGeneration> local_world_generation =
        local_world_generation_counter_.issue(raw_world.version, navigation.revision,
                                              world_update.revision, upload.revision);
    if (local_world_generation.has_value()) {
      world_update.local_world_generation = *local_world_generation;
      adoptWorldResources(prepared, world_update);
      if (current_objective) {
        prepared.search_objective = makeStaticRouteObjective(*current_objective);
      }
      prepared.lattice_search_performed = false;
      prepared.lattice_continuation_attempt = 0U;
      local_world_generation_valid = productionWorldGenerationCoherent(prepared);
      if (local_world_generation_valid) {
        prepared_esdf_ = prepared;
      }
    }
    if (!local_world_generation_valid && upload_required) {
      prepared_esdf_.reset();
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
  if (blocked_raw_revision != 0U &&
      blocked_raw_revision <= raw_world.version.revision &&
      dispatched_raw_revision < blocked_raw_revision &&
      prepared.global_guide_generation != 0U) {
    observed_route_replan_dispatched_raw_revision_.store(blocked_raw_revision,
                                                         std::memory_order_release);
    RCLCPP_INFO(get_logger(),
                "OBSERVED_ROUTE_REPLAN status=esdf_caught_up raw_revision=%" PRIu64
                " esdf_revision=%" PRIu64 " generation=%" PRIu64,
                raw_world.version.revision, prepared.revision,
                prepared.global_guide_generation);
    requestStaticRouteReplan(GlobalGuideReleaseReason::kBlocked,
                             prepared.global_guide_generation);
  }
  const bool initial_route_search_required = prepared.global_guide_generation == 0U;
  bool initial_route_search_queued = false;
  bool initial_route_search_already_pending = false;
  if (initial_route_search_required) {
    auto guide_world = std::make_shared<const ProductionMppiPreparedEsdf>(prepared);
    {
      const std::scoped_lock lock{guide_queue_mutex_};
      if (pending_guide_world_) {
        initial_route_search_already_pending = true;
      } else {
        pending_guide_world_ = std::move(guide_world);
        initial_route_search_queued = true;
      }
    }
    if (initial_route_search_queued) {
      guide_queue_condition_.notify_all();
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
      "classified=%zu classification_reused=%zu dependency_invalidated=%zu "
      "lowered=%zu distance_work=%zu maximum_distance_m=%.2f audit=%s "
      "recenter=%s local_world_generation=%" PRIu64 " route_generation=%" PRIu64
      " route_search=%s builds=%" PRIu64 " throttled=%" PRIu64 " dropped_raw=%" PRIu64
      " mode_totals=(full=%" PRIu64 ",incremental=%" PRIu64 ",reused=%" PRIu64 ")",
      prepared.revision, raw_world.version.revision, prepared.build_ms,
      field.stats.classification_ms, prepared.upload_ms, prepared.grid.width,
      prepared.grid.height, prepared.grid.depth, field.stats.known_voxels,
      field.stats.free_voxels, field.stats.occupied_voxels, field.stats.unknown_voxels,
      field.stats.proprioceptive_free_voxels, field.stats.launch_support_voxels,
      observedEsdf3DBuildModeName(field.stats.mode),
      field.stats.incremental_fallback ? "true" : "false", field.stats.changed_voxels,
      field.stats.recomputed_voxels, field.stats.reused_voxels,
      field.stats.classified_voxels, field.stats.reused_classification_voxels,
      field.stats.dependency_invalidated_voxels, field.stats.lowered_voxels,
      field.stats.distance_field.voxel_count, maximum_distance_m,
      periodic_full_audit ? "true" : "false", recenter ? "true" : "false",
      prepared.local_world_generation.generation, prepared.global_guide_generation,
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
