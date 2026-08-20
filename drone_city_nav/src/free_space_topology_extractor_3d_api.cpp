#include "drone_city_nav/distance_field_3d.hpp"
#include "drone_city_nav/free_space_topology_extractor_3d.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>

namespace drone_city_nav {
namespace {

[[nodiscard]] bool finitePositive(const double value) noexcept {
  return std::isfinite(value) && value > 0.0;
}

} // namespace

bool freeSpaceTopologyExtractorConfigIsValid(
    const FreeSpaceTopologyExtractorConfig& config) noexcept {
  const bool valid_minimum_z = !config.minimum_center_z_m.has_value() ||
                               std::isfinite(*config.minimum_center_z_m);
  const bool valid_maximum_z = !config.maximum_center_z_m.has_value() ||
                               std::isfinite(*config.maximum_center_z_m);
  const bool valid_z_interval = !config.minimum_center_z_m.has_value() ||
                                !config.maximum_center_z_m.has_value() ||
                                *config.minimum_center_z_m < *config.maximum_center_z_m;
  return finitePositive(config.maximum_clearance_m) &&
         finitePositive(config.open_space_clearance_m) &&
         config.maximum_clearance_m > config.open_space_clearance_m &&
         finitePositive(config.speed_limit_mps) &&
         std::isfinite(config.medial_clearance_weight) &&
         config.medial_clearance_weight >= 0.0 &&
         finitePositive(config.medial_ridge_prominence_m) &&
         config.medial_band_radius_cells <=
             static_cast<std::size_t>(std::numeric_limits<std::uint16_t>::max()) &&
         config.chunk_size_cells > 0U && config.minimum_open_region_voxels > 0U &&
         config.minimum_constrained_component_voxels > 0U &&
         config.minimum_portal_voxels > 0U &&
         config.maximum_portal_voxels >= config.minimum_portal_voxels &&
         config.maximum_portal_voxels <=
             FreeSpaceTopologyFormatLimits::maximum_geometry_point_count &&
         std::isfinite(config.footprint.radius_m) && config.footprint.radius_m >= 0.0 &&
         std::isfinite(config.footprint.lower_extent_m) &&
         config.footprint.lower_extent_m >= 0.0 &&
         std::isfinite(config.footprint.upper_extent_m) &&
         config.footprint.upper_extent_m >= 0.0 && valid_minimum_z && valid_maximum_z &&
         valid_z_interval;
}

ExtractedFreeSpaceTopology3D
extractFreeSpaceTopology3D(const OccupancyGrid3D& occupancy,
                           const FreeSpaceTopologyExtractorConfig& config,
                           const std::stop_token stop_token) {
  if (!freeSpaceTopologyExtractorConfigIsValid(config)) {
    throw std::invalid_argument{"invalid FreeSpaceTopologyExtractor3D config"};
  }
  const auto started = std::chrono::steady_clock::now();
  const DistanceField3D clearance_field =
      DistanceField3D::build(occupancy, config.maximum_clearance_m);
  ExtractedFreeSpaceTopology3D result =
      extractFreeSpaceTopology3D(occupancy, clearance_field, config, stop_token);
  result.stats.clearance_ms = clearance_field.stats().duration_ms;
  result.stats.duration_ms = std::chrono::duration<double, std::milli>(
                                 std::chrono::steady_clock::now() - started)
                                 .count();
  return result;
}

} // namespace drone_city_nav
