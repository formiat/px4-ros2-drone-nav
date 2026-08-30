#include "observed_world_builder_3d.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

#include "production_mppi_route_world.hpp"

namespace drone_city_nav {
namespace {

template<typename T>
[[nodiscard]] const T* optionalAddress(const std::optional<T>& value) noexcept {
  return value.has_value() ? std::addressof(*value) : nullptr;
}

[[nodiscard]] bool sameOptionalFreeSpaceSeed(
    const std::optional<ProprioceptiveFreeSpaceSeed3D>& first,
    const std::optional<ProprioceptiveFreeSpaceSeed3D>& second) noexcept {
  return first.has_value() == second.has_value() &&
         (!first.has_value() || sameProprioceptiveFreeSpaceSeed3D(*first, *second));
}

[[nodiscard]] bool sameOptionalLaunchSupport(
    const std::optional<LaunchSupportContact3D>& first,
    const std::optional<LaunchSupportContact3D>& second) noexcept {
  return first.has_value() == second.has_value() &&
         (!first.has_value() || sameLaunchSupportContact3D(*first, *second));
}

[[nodiscard]] bool configValid(const ObservedWorldBuilderConfig3D& config) noexcept {
  return localObservedEsdfWindow3DIsValid(config.local_window) &&
         std::isfinite(config.preferred_distance_m) &&
         config.preferred_distance_m >= 0.0 && std::isfinite(config.update_rate_hz) &&
         config.update_rate_hz > 0.0 &&
         std::isfinite(config.incremental_maximum_rebuild_ratio) &&
         config.incremental_maximum_rebuild_ratio > 0.0 &&
         config.incremental_maximum_rebuild_ratio <= 1.0 &&
         std::isfinite(config.footprint.radius_m) && config.footprint.radius_m >= 0.0 &&
         std::isfinite(config.footprint.lower_extent_m) &&
         config.footprint.lower_extent_m >= 0.0 &&
         std::isfinite(config.footprint.upper_extent_m) &&
         config.footprint.upper_extent_m >= 0.0 &&
         config.full_audit_interval_builds > 0U;
}

[[nodiscard]] GridBounds3D residentLocalBounds(const WorldSnapshot3D& world) noexcept {
  if (world.observed_esdf_resource.local_occupancy != nullptr) {
    return world.observed_esdf_resource.local_occupancy->bounds();
  }
  return GridBounds3D{
      .origin_x = world.grid.origin_x_m,
      .origin_y = world.grid.origin_y_m,
      .origin_z = world.grid.origin_z_m,
      .resolution_m = world.grid.resolution_m,
      .width_cells = world.grid.width,
      .height_cells = world.grid.height,
      .depth_cells = world.grid.depth,
  };
}

} // namespace

std::string_view
observedWorldUpdateStatus3DName(const ObservedWorldUpdateStatus3D status) noexcept {
  switch (status) {
    case ObservedWorldUpdateStatus3D::kPrepared:
      return "prepared";
    case ObservedWorldUpdateStatus3D::kPublished:
      return "published";
    case ObservedWorldUpdateStatus3D::kAlreadyCurrent:
      return "already_current";
    case ObservedWorldUpdateStatus3D::kRateLimited:
      return "rate_limited";
    case ObservedWorldUpdateStatus3D::kUnavailableObservedGrid:
      return "unavailable_observed_grid";
    case ObservedWorldUpdateStatus3D::kRawExecutionOwnerMismatch:
      return "raw_execution_owner_mismatch";
    case ObservedWorldUpdateStatus3D::kRouteEvidenceDerivationFailed:
      return "route_evidence_derivation_failed";
    case ObservedWorldUpdateStatus3D::kInvalidCoverage:
      return "invalid_coverage";
    case ObservedWorldUpdateStatus3D::kSupersededTransientEvidenceParent:
      return "superseded_transient_evidence_parent";
    case ObservedWorldUpdateStatus3D::kSupersededEsdfParent:
      return "superseded_esdf_parent";
    case ObservedWorldUpdateStatus3D::kUploadRejected:
      return "upload_rejected";
    case ObservedWorldUpdateStatus3D::kMixedLocalWorldGeneration:
      return "mixed_local_world_generation";
  }
  return "unknown";
}

ObservedWorldBuilder3D::ObservedWorldBuilder3D(
    const ObservedWorldBuilderConfig3D& config)
    : config_{config} {
  if (!configValid(config_)) {
    throw std::invalid_argument{"invalid observed world builder configuration"};
  }
}

ObservedWorldUpdateStatus3D ObservedWorldBuilder3D::assessRawWorld(
    const ProductionMppiRawWorld3D& raw_world) noexcept {
  const std::shared_ptr<const ObservedOccupancyGrid3D>& occupancy = raw_world.occupancy;
  if (occupancy == nullptr) {
    return ObservedWorldUpdateStatus3D::kUnavailableObservedGrid;
  }
  const std::shared_ptr<const VersionedObservedRawWorld3D>& owner =
      raw_world.execution_owner;
  if (owner == nullptr || !owner->valid() ||
      std::addressof(owner->occupancy()) != occupancy.get() ||
      owner->version().producer_instance_id != raw_world.version.producer_instance_id ||
      owner->version().base_snapshot_revision !=
          raw_world.version.base_snapshot_revision ||
      owner->version().revision != raw_world.version.revision ||
      owner->proprioceptiveFreeSpaceSeed().has_value() ||
      owner->launchSupportContact().has_value()) {
    return ObservedWorldUpdateStatus3D::kRawExecutionOwnerMismatch;
  }
  return ObservedWorldUpdateStatus3D::kPrepared;
}

bool ObservedWorldBuilder3D::needsRefresh(
    const ProductionMppiRawWorld3D& raw_world,
    const std::shared_ptr<const WorldSnapshot3D>& resident_world,
    const Point3& position) const noexcept {
  if (assessRawWorld(raw_world) != ObservedWorldUpdateStatus3D::kPrepared) {
    return false;
  }
  if (resident_world == nullptr ||
      !productionWorldGenerationCoherent(*resident_world) ||
      resident_world->producer_instance_id != raw_world.version.producer_instance_id ||
      resident_world->grid.depth <= 1 || !resident_world->grid.outside_is_unknown) {
    return true;
  }
  return localObservedEsdfNeedsRecenter(residentLocalBounds(*resident_world),
                                        raw_world.occupancy->bounds(), position,
                                        config_.local_window);
}

ObservedWorldBuildAssessment3D
ObservedWorldBuilder3D::assess(ObservedWorldBuildRequest3D request,
                               std::shared_ptr<const WorldSnapshot3D> active_world,
                               const ObservedWorldBuildHistory3D& history) const {
  ObservedWorldBuildAssessment3D result;
  result.request = std::move(request);
  result.active_world = std::move(active_world);
  if (result.request.raw_world == nullptr) {
    result.status = ObservedWorldUpdateStatus3D::kUnavailableObservedGrid;
    return result;
  }
  result.status = assessRawWorld(*result.request.raw_world);
  if (result.status != ObservedWorldUpdateStatus3D::kPrepared) {
    return result;
  }

  const ProductionMppiRawWorld3D& raw_world = *result.request.raw_world;
  const ObservedOccupancyGrid3D& occupancy = *raw_world.occupancy;
  const GridBounds3D& world_bounds = occupancy.bounds();
  result.recentered = true;
  if (result.active_world != nullptr && result.active_world->distances_m != nullptr &&
      result.active_world->grid.depth > 1 &&
      result.active_world->grid.outside_is_unknown) {
    result.local_bounds = residentLocalBounds(*result.active_world);
    result.recentered =
        localObservedEsdfNeedsRecenter(result.local_bounds, world_bounds,
                                       result.request.position, config_.local_window);
  }
  if (result.recentered) {
    result.local_bounds = selectLocalObservedEsdfBounds(
        world_bounds, result.request.position, config_.local_window);
  }
  result.maximum_distance_m = requiredObservedEsdfMaximumDistanceM(
      config_.preferred_distance_m, config_.footprint,
      result.local_bounds.resolution_m);
  const GridBounds3D source_bounds = knownObstacleDistanceSourceBounds3D(
      world_bounds, result.local_bounds, result.maximum_distance_m);
  result.local_occupied_fingerprint =
      knownObstacleFingerprint3D(occupancy, source_bounds);

  const bool transient_seed_unchanged =
      result.active_world != nullptr &&
      sameOptionalFreeSpaceSeed(result.active_world->proprioceptive_free_space_seed,
                                result.request.free_space_seed);
  const bool launch_support_contact_unchanged =
      result.active_world != nullptr &&
      sameOptionalLaunchSupport(result.active_world->launch_support_contact,
                                result.request.launch_support_contact);
  const bool persistent_evidence_unchanged =
      launch_support_contact_unchanged &&
      result.active_world->launch_support_resolution_pending ==
          result.request.launch_support_resolution_pending;
  result.evidence_change = ObservedWorldEvidenceChange3D{
      .raw_world = result.request.raw_world,
      .free_space_seed = result.request.free_space_seed,
      .launch_support_contact = result.request.launch_support_contact,
      .launch_support_resolution_pending =
          result.request.launch_support_resolution_pending,
      .persistent_changed =
          result.active_world != nullptr && !persistent_evidence_unchanged,
      .transient_changed = result.active_world != nullptr && !transient_seed_unchanged,
  };

  const bool active_incremental_parent_available =
      result.active_world != nullptr && result.active_world->distances_m != nullptr &&
      !result.recentered && result.active_world->observed_occupancy != nullptr &&
      productionWorldGenerationCoherent(*result.active_world) &&
      result.active_world->observed_esdf_resource.local_occupancy != nullptr &&
      result.active_world->observed_esdf_resource.known_obstacle_distance != nullptr &&
      result.active_world->observed_esdf_resource.classification_override_cells !=
          nullptr &&
      result.active_world->observed_esdf_resource.coverage.coherent() &&
      result.active_world->observed_esdf_resource.coverage.source_raw_version
              .revision == result.active_world->source_raw_revision &&
      result.active_world->observed_esdf_resource.coverage.maximum_distance_m ==
          result.maximum_distance_m;
  result.same_raw_lineage =
      active_incremental_parent_available && !raw_world.full_reset &&
      result.active_world->observed_esdf_resource.coverage.source_raw_version
              .producer_instance_id == raw_world.version.producer_instance_id &&
      result.active_world->observed_esdf_resource.coverage.source_raw_version
              .base_snapshot_revision == raw_world.version.base_snapshot_revision &&
      result.active_world->observed_esdf_resource.coverage.source_raw_version
              .revision <= raw_world.version.revision;
  const bool known_obstacle_sources_unchanged =
      result.same_raw_lineage && launch_support_contact_unchanged &&
      result.active_world->source_occupied_fingerprint ==
          result.local_occupied_fingerprint;
  result.periodic_full_audit = observedEsdfFullAuditDue(
      history.completed_builds, config_.full_audit_interval_builds);

  result.observed_raw_world_owner = raw_world.execution_owner->deriveRouteEvidence(
      result.request.free_space_seed, result.request.launch_support_contact);
  if (result.observed_raw_world_owner == nullptr ||
      !raw_world.execution_owner->sharesObservationOwner(
          *result.observed_raw_world_owner) ||
      std::addressof(result.observed_raw_world_owner->occupancy()) !=
          raw_world.occupancy.get() ||
      result.observed_raw_world_owner->occupiedSnapshot() !=
          raw_world.execution_owner->occupiedSnapshot()) {
    result.status = ObservedWorldUpdateStatus3D::kRouteEvidenceDerivationFailed;
    return result;
  }

  const bool already_current =
      known_obstacle_sources_unchanged && !result.periodic_full_audit &&
      result.active_world->source_raw_revision == raw_world.version.revision;
  if (already_current && !result.evidence_change.persistent_changed) {
    result.status = ObservedWorldUpdateStatus3D::kAlreadyCurrent;
    return result;
  }

  const bool first_build =
      history.last_build_time == std::chrono::steady_clock::time_point{};
  const double elapsed_s =
      std::chrono::duration<double>(result.request.build_started_at -
                                    history.last_build_time)
          .count();
  const bool build_rate_due = first_build || elapsed_s >= 1.0 / config_.update_rate_hz;
  if ((!known_obstacle_sources_unchanged || result.periodic_full_audit) &&
      result.active_world != nullptr && !result.recentered && !build_rate_due) {
    const std::chrono::duration<double> update_period{1.0 / config_.update_rate_hz};
    result.retry_not_before =
        history.last_build_time +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(update_period);
    result.status = ObservedWorldUpdateStatus3D::kRateLimited;
  }
  return result;
}

PreparedObservedWorldBuild3D
ObservedWorldBuilder3D::materialize(ObservedWorldBuildAssessment3D assessment) const {
  PreparedObservedWorldBuild3D result;
  result.status = assessment.status;
  result.request = assessment.request;
  result.expected_parent = assessment.active_world;
  result.evidence_change = assessment.evidence_change;
  result.maximum_distance_m = assessment.maximum_distance_m;
  result.periodic_full_audit = assessment.periodic_full_audit;
  result.recentered = assessment.recentered;
  if (!assessment.buildRequired() || assessment.request.raw_world == nullptr) {
    return result;
  }
  const ProductionMppiRawWorld3D& raw_world = *assessment.request.raw_world;
  std::optional<PreviousObservedEsdf3D> previous;
  if (assessment.same_raw_lineage) {
    previous = PreviousObservedEsdf3D{
        .grid = assessment.active_world->grid,
        .distances_m = assessment.active_world->distances_m,
        .known_obstacle_distance =
            assessment.active_world->observed_esdf_resource.known_obstacle_distance,
        .source_occupancy = assessment.active_world->observed_occupancy,
        .local_occupancy =
            assessment.active_world->observed_esdf_resource.local_occupancy,
        .classification_override_cells =
            *assessment.active_world->observed_esdf_resource
                 .classification_override_cells,
        .occupancy_fingerprint = assessment.active_world->revision,
        .maximum_distance_m =
            assessment.active_world->observed_esdf_resource.coverage.maximum_distance_m,
    };
  }
  ObservedEsdf3D field = updateObservedEsdf3D(
      *raw_world.occupancy, assessment.local_bounds, assessment.maximum_distance_m,
      optionalAddress(previous), raw_world.dirty_chunks,
      raw_world.full_reset || assessment.recentered || assessment.periodic_full_audit,
      config_.incremental_maximum_rebuild_ratio, config_.worker_pool,
      optionalAddress(assessment.request.launch_support_contact));

  const bool evidence_only_reuse =
      field.stats.mode == ObservedEsdf3DBuildMode::kReused &&
      assessment.active_world != nullptr &&
      assessment.active_world->source_raw_revision == raw_world.version.revision;
  const RawMapVersion parent_raw_version =
      field.stats.mode == ObservedEsdf3DBuildMode::kFull
          ? RawMapVersion{}
          : assessment.active_world->observed_esdf_resource.coverage.source_raw_version;
  const std::uint64_t parent_esdf_fingerprint =
      field.stats.mode == ObservedEsdf3DBuildMode::kFull
          ? 0U
          : assessment.active_world->revision;
  ObservedEsdfCoverage3D coverage;
  if (evidence_only_reuse) {
    if (field.distances_m != assessment.active_world->distances_m ||
        field.known_obstacle_distance !=
            assessment.active_world->observed_esdf_resource.known_obstacle_distance ||
        field.occupancy_fingerprint != assessment.active_world->revision) {
      result.status = ObservedWorldUpdateStatus3D::kInvalidCoverage;
      return result;
    }
    coverage = assessment.active_world->observed_esdf_resource.coverage;
  } else {
    coverage = ObservedEsdfCoverage3D{
        .source_raw_version = raw_world.version,
        .parent_raw_version = parent_raw_version,
        .raw_local_fingerprint = assessment.local_occupied_fingerprint,
        .esdf_fingerprint = field.occupancy_fingerprint,
        .parent_esdf_fingerprint = parent_esdf_fingerprint,
        .total_voxels = field.distances_m->size(),
        .recomputed_voxels = field.stats.recomputed_voxels,
        .reused_voxels = field.stats.reused_voxels,
        .maximum_distance_m = assessment.maximum_distance_m,
        .mode = field.stats.mode,
    };
  }
  if (!coverage.coherent()) {
    result.status = ObservedWorldUpdateStatus3D::kInvalidCoverage;
    return result;
  }

  result.world.producer_instance_id = raw_world.version.producer_instance_id;
  result.world.revision = field.occupancy_fingerprint;
  result.world.source_raw_revision = raw_world.version.revision;
  result.world.source_occupied_fingerprint = assessment.local_occupied_fingerprint;
  result.world.raw_occupied_fingerprint =
      raw_world.execution_owner->occupiedContentFingerprint();
  result.world.planner_parent_raw_revision =
      assessment.same_raw_lineage && assessment.active_world != nullptr
          ? assessment.active_world->source_raw_revision
          : 0U;
  result.world.source_stamp_ns = raw_world.source_stamp_ns;
  result.world.ready_stamp_ns = assessment.request.ready_stamp_ns;
  result.world.grid = field.grid;
  result.world.distances_m = field.distances_m;
  result.world.observed_occupancy = raw_world.occupancy;
  result.world.observed_raw_world_owner = assessment.observed_raw_world_owner;
  result.world.observed_esdf_resource = ObservedEsdfResource3D{
      .local_occupancy = field.local_occupancy,
      .known_obstacle_distance = field.known_obstacle_distance,
      .classification_override_cells = std::make_shared<const std::vector<GridIndex3D>>(
          std::move(field.classification_override_cells)),
      .coverage = coverage,
  };
  result.world.planner_dirty_chunks = raw_world.dirty_chunks;
  result.world.proprioceptive_free_space_seed = assessment.request.free_space_seed;
  result.world.launch_support_contact = assessment.request.launch_support_contact;
  result.world.launch_support_resolution_pending =
      assessment.request.launch_support_resolution_pending;
  result.world.planner_full_reset = raw_world.full_reset ||
                                    !assessment.same_raw_lineage ||
                                    assessment.evidence_change.persistent_changed;

  result.expected_parent_raw_version = parent_raw_version;
  result.expected_parent_esdf_fingerprint = parent_esdf_fingerprint;
  result.telemetry.build_ms =
      field.stats.distance_cache.duration_ms + field.stats.classification_ms;
  result.telemetry.conversion_ms =
      raw_world.reconstruction_ms + field.stats.classification_ms;
  result.stats = field.stats;
  result.dirty_regions = std::move(field.dirty_regions);
  result.parent_required = field.stats.mode != ObservedEsdf3DBuildMode::kFull;
  result.upload_required = field.stats.mode != ObservedEsdf3DBuildMode::kReused;
  result.status = ObservedWorldUpdateStatus3D::kPrepared;
  return result;
}

} // namespace drone_city_nav
