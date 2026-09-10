#include "production_mppi_route_world.hpp"

#include "drone_city_nav/persistent_dstar_lite_planner_3d.hpp"
#include "drone_city_nav/versioned_world_evidence_3d.hpp"

#include <cmath>
#include <utility>

#include "production_mppi_raw_world.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool observedEsdfCoverageMatches(const WorldSnapshot3D& world,
                                               const RawMapVersion& raw) noexcept {
  const ObservedEsdfResource3D& resource = world.observed_esdf_resource;
  if (!resource.local_occupancy || !resource.known_obstacle_distance ||
      !resource.classification_override_cells || !resource.coverage.coherent()) {
    return false;
  }
  const ObservedEsdfCoverage3D& coverage = resource.coverage;
  const GridBounds3D& bounds = resource.local_occupancy->bounds();
  const GridBounds3D& distance_bounds = resource.known_obstacle_distance->bounds();
  const std::size_t voxel_count = static_cast<std::size_t>(world.grid.width) *
                                  static_cast<std::size_t>(world.grid.height) *
                                  static_cast<std::size_t>(world.grid.depth);
  constexpr double kTolerance{1.0e-5};
  return coverage.source_raw_version.producer_instance_id == raw.producer_instance_id &&
         coverage.source_raw_version.base_snapshot_revision ==
             raw.base_snapshot_revision &&
         coverage.source_raw_version.revision == raw.revision &&
         coverage.raw_local_fingerprint == world.source_occupied_fingerprint &&
         coverage.esdf_fingerprint == world.revision &&
         coverage.total_voxels == world.distances_m->size() &&
         coverage.total_voxels == voxel_count && world.grid.outside_is_unknown &&
         resource.known_obstacle_distance->valid() &&
         resource.known_obstacle_distance->sourceFingerprint() == world.revision &&
         std::abs(resource.known_obstacle_distance->maximumDistanceM() -
                  coverage.maximum_distance_m) <= kTolerance &&
         distance_bounds.width_cells == bounds.width_cells &&
         distance_bounds.height_cells == bounds.height_cells &&
         distance_bounds.depth_cells == bounds.depth_cells &&
         std::abs(distance_bounds.resolution_m - bounds.resolution_m) <= kTolerance &&
         std::abs(distance_bounds.origin_x - bounds.origin_x) <= kTolerance &&
         std::abs(distance_bounds.origin_y - bounds.origin_y) <= kTolerance &&
         std::abs(distance_bounds.origin_z - bounds.origin_z) <= kTolerance &&
         bounds.width_cells == world.grid.width &&
         bounds.height_cells == world.grid.height &&
         bounds.depth_cells == world.grid.depth &&
         std::abs(bounds.resolution_m - world.grid.resolution_m) <= kTolerance &&
         std::abs(bounds.origin_x - world.grid.origin_x_m) <= kTolerance &&
         std::abs(bounds.origin_y - world.grid.origin_y_m) <= kTolerance &&
         std::abs(bounds.origin_z - world.grid.origin_z_m) <= kTolerance;
}

} // namespace

ProductionWorldGenerationStatus
assessProductionWorldGeneration(const WorldSnapshot3D& world) noexcept {
  const LocalWorldGeneration& generation = world.local_world_generation;
  if (!generation.coherent()) {
    return ProductionWorldGenerationStatus::kInvalidGeneration;
  }
  if (!world.distances_m || world.revision == 0U || world.grid.width <= 1 ||
      world.grid.height <= 1) {
    return ProductionWorldGenerationStatus::kMissingEsdfResources;
  }
  if (generation.esdf_revision != world.revision ||
      generation.gpu_esdf_revision != world.revision) {
    return ProductionWorldGenerationStatus::kEsdfRevisionMismatch;
  }
  const RawMapVersion& raw = generation.raw_map;
  if (world.observed_occupancy) {
    if (world.producer_instance_id != raw.producer_instance_id ||
        world.source_raw_revision != raw.revision) {
      return ProductionWorldGenerationStatus::kRawVersionMismatch;
    }
    const std::shared_ptr<const VersionedObservedRawWorld3D>& owner =
        world.observed_raw_world_owner;
    if (!owner || !owner->valid() ||
        owner->version().producer_instance_id != raw.producer_instance_id ||
        owner->version().base_snapshot_revision != raw.base_snapshot_revision ||
        owner->version().revision != raw.revision ||
        std::addressof(owner->occupancy()) != world.observed_occupancy.get()) {
      return ProductionWorldGenerationStatus::kObservedOwnerMismatch;
    }
    if (world.static_occupancy != nullptr || world.raw_occupied_fingerprint == 0U ||
        owner->occupiedContentFingerprint() != world.raw_occupied_fingerprint) {
      return ProductionWorldGenerationStatus::kObservedOwnerMismatch;
    }
    if (!observedEsdfCoverageMatches(world, raw)) {
      return ProductionWorldGenerationStatus::kObservedEsdfCoverageMismatch;
    }
  } else {
    if (world.static_occupancy == nullptr || world.producer_instance_id != 0U ||
        world.source_raw_revision != 0U || raw.producer_instance_id != 0U ||
        raw.base_snapshot_revision != world.revision ||
        raw.revision != world.revision) {
      return ProductionWorldGenerationStatus::kRawVersionMismatch;
    }
    if (world.static_occupancy->fingerprint() != world.revision ||
        world.static_occupancy->contentFingerprint() !=
            world.raw_occupied_fingerprint) {
      return ProductionWorldGenerationStatus::kRawVersionMismatch;
    }
  }
  return ProductionWorldGenerationStatus::kCoherent;
}

bool productionWorldGenerationCoherent(const WorldSnapshot3D& world) noexcept {
  return assessProductionWorldGeneration(world) ==
         ProductionWorldGenerationStatus::kCoherent;
}

std::string_view productionWorldGenerationStatusName(
    const ProductionWorldGenerationStatus status) noexcept {
  switch (status) {
    case ProductionWorldGenerationStatus::kCoherent:
      return "coherent";
    case ProductionWorldGenerationStatus::kInvalidGeneration:
      return "invalid_generation";
    case ProductionWorldGenerationStatus::kMissingEsdfResources:
      return "missing_esdf_resources";
    case ProductionWorldGenerationStatus::kEsdfRevisionMismatch:
      return "esdf_revision_mismatch";
    case ProductionWorldGenerationStatus::kRawVersionMismatch:
      return "raw_version_mismatch";
    case ProductionWorldGenerationStatus::kObservedOwnerMismatch:
      return "observed_owner_mismatch";
    case ProductionWorldGenerationStatus::kObservedEsdfCoverageMismatch:
      return "observed_esdf_coverage_mismatch";
  }
  return "unknown";
}

NavigationWorldCertificate3D
navigationWorldCertificate3D(const WorldSnapshot3D& world) noexcept {
  if (!productionWorldGenerationCoherent(world)) {
    return {};
  }
  return NavigationWorldCertificate3D{
      .producer_instance_id = world.producer_instance_id,
      .esdf_fingerprint = world.revision,
      .esdf_source_raw_revision = world.source_raw_revision,
      .esdf_source_occupied_fingerprint = world.source_occupied_fingerprint,
      .raw_validated_through_revision = world.source_raw_revision,
      .local_world_generation = world.local_world_generation.generation,
      .topology_revision = 0U,
  };
}

bool routeSearchRequiresLatestRawOverlay3D(const RouteReleaseReason3D reason) noexcept {
  return reason == RouteReleaseReason3D::kNoActiveRoute ||
         reason == RouteReleaseReason3D::kBlocked;
}

std::shared_ptr<const PersistentPlannerWorld3D> captureObservedRouteSearchWorld3D(
    const ProductionMppiRawWorld3D& raw_world,
    std::optional<ProprioceptiveFreeSpaceSeed3D> proprioceptive_free_space_seed,
    std::optional<LaunchSupportContact3D> launch_support_contact) {
  if (!raw_world.valid()) {
    return nullptr;
  }
  const std::shared_ptr<const VersionedObservedRawWorld3D>& owner =
      raw_world.authoritativeOwner();
  const std::uint64_t occupied_fingerprint = owner->occupiedContentFingerprint();
  if (occupied_fingerprint == 0U) {
    return nullptr;
  }
  return std::make_shared<const PersistentPlannerWorld3D>(PersistentPlannerWorld3D{
      .observed_occupancy = raw_world.occupancyOwner(),
      .static_occupancy = nullptr,
      .proprioceptive_free_space_seed = proprioceptive_free_space_seed,
      .launch_support_contact = std::move(launch_support_contact),
      .dirty_chunks = {},
      .producer_instance_id = raw_world.version().producer_instance_id,
      .revision = raw_world.version().revision,
      .incremental_parent_revision = 0U,
      .occupied_fingerprint = occupied_fingerprint,
      .full_reset = true,
  });
}

std::shared_ptr<const PersistentPlannerWorld3D>
captureResidentPlannerWorld3D(const WorldSnapshot3D& world) {
  if (!productionWorldGenerationCoherent(world)) {
    return nullptr;
  }
  if (world.observed_occupancy != nullptr) {
    return std::make_shared<const PersistentPlannerWorld3D>(PersistentPlannerWorld3D{
        .observed_occupancy = world.observed_occupancy,
        .static_occupancy = nullptr,
        .observed_clearance_field =
            world.observed_esdf_resource.known_obstacle_distance,
        .proprioceptive_free_space_seed = world.proprioceptive_free_space_seed,
        .launch_support_contact = world.launch_support_contact,
        .dirty_chunks = world.planner_dirty_chunks,
        .producer_instance_id = world.producer_instance_id,
        .revision = world.source_raw_revision,
        .incremental_parent_revision = world.planner_parent_raw_revision,
        .occupied_fingerprint = world.raw_occupied_fingerprint,
        .full_reset = world.planner_full_reset,
    });
  }
  return std::make_shared<const PersistentPlannerWorld3D>(PersistentPlannerWorld3D{
      .observed_occupancy = nullptr,
      .static_occupancy = world.static_occupancy,
      .proprioceptive_free_space_seed = world.proprioceptive_free_space_seed,
      .launch_support_contact = world.launch_support_contact,
      .dirty_chunks = {},
      .producer_instance_id = world.static_occupancy->fingerprint(),
      .revision = 1U,
      .incremental_parent_revision = 0U,
      .occupied_fingerprint = world.raw_occupied_fingerprint,
      .full_reset = false,
  });
}

} // namespace drone_city_nav
