#include "production_mppi_route_world.hpp"

#include <cmath>

#include "production_mppi_node.hpp"

namespace drone_city_nav {
namespace {

[[nodiscard]] bool observedEsdfCoverageMatches(const ProductionMppiPreparedEsdf& world,
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
assessProductionWorldGeneration(const ProductionMppiPreparedEsdf& world) noexcept {
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
    if (!observedEsdfCoverageMatches(world, raw)) {
      return ProductionWorldGenerationStatus::kObservedEsdfCoverageMismatch;
    }
  } else if (world.producer_instance_id != 0U || world.source_raw_revision != 0U ||
             raw.producer_instance_id != 0U ||
             raw.base_snapshot_revision != world.revision ||
             raw.revision != world.revision) {
    return ProductionWorldGenerationStatus::kRawVersionMismatch;
  }
  return ProductionWorldGenerationStatus::kCoherent;
}

bool productionWorldGenerationCoherent(
    const ProductionMppiPreparedEsdf& world) noexcept {
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
navigationWorldCertificate3D(const ProductionMppiPreparedEsdf& world) noexcept {
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

void adoptWorldResources(ProductionMppiPreparedEsdf& target,
                         const ProductionMppiPreparedEsdf& source) {
  target.local_world_generation = source.local_world_generation;
  target.producer_instance_id = source.producer_instance_id;
  target.revision = source.revision;
  target.source_raw_revision = source.source_raw_revision;
  target.source_occupied_fingerprint = source.source_occupied_fingerprint;
  target.source_stamp_ns = source.source_stamp_ns;
  target.ready_stamp_ns = source.ready_stamp_ns;
  target.build_ms = source.build_ms;
  target.esdf_x_pass_ms = source.esdf_x_pass_ms;
  target.esdf_y_pass_ms = source.esdf_y_pass_ms;
  target.esdf_z_pass_ms = source.esdf_z_pass_ms;
  target.esdf_finalize_ms = source.esdf_finalize_ms;
  target.conversion_ms = source.conversion_ms;
  target.upload_ms = source.upload_ms;
  target.grid = source.grid;
  target.distances_m = source.distances_m;
  target.observed_occupancy = source.observed_occupancy;
  target.observed_raw_world_owner = source.observed_raw_world_owner;
  target.observed_esdf_resource = source.observed_esdf_resource;
  target.proprioceptive_free_space_seed = source.proprioceptive_free_space_seed;
  target.launch_support_contact = source.launch_support_contact;
  target.launch_support_resolution_pending = source.launch_support_resolution_pending;
}

} // namespace drone_city_nav
