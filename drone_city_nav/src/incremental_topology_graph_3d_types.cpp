#include "drone_city_nav/incremental_topology_graph_3d.hpp"

#include <cmath>
#include <cstdint>
#include <functional>

#include "incremental_topology_graph_3d_internal.hpp"

namespace drone_city_nav {

std::size_t IncrementalTopologyNodeIdHash::operator()(
    const IncrementalTopologyNodeId id) const noexcept {
  return std::hash<std::uint64_t>{}(id.value);
}

std::size_t IncrementalTopologyEdgeIdHash::operator()(
    const IncrementalTopologyEdgeId id) const noexcept {
  return std::hash<std::uint64_t>{}(id.value);
}

std::size_t IncrementalTopologyTileIndex3DHash::operator()(
    const IncrementalTopologyTileIndex3D index) const noexcept {
  std::uint64_t hash{incremental_topology_detail::kFnvOffset};
  incremental_topology_detail::hashInteger(hash, index.x);
  incremental_topology_detail::hashInteger(hash, index.y);
  incremental_topology_detail::hashInteger(hash, index.z);
  return static_cast<std::size_t>(hash);
}

bool incrementalTopologyGraph3DConfigIsValid(
    const IncrementalTopologyGraph3DConfig& config) noexcept {
  return config.tile_size_cells > 0 && config.coarse_sample_stride_cells > 0 &&
         config.refined_sample_stride_cells > 0 &&
         config.refined_sample_stride_cells <= config.coarse_sample_stride_cells &&
         config.coarse_sample_stride_cells <= config.tile_size_cells &&
         config.coarse_sample_stride_cells % config.refined_sample_stride_cells == 0 &&
         config.maximum_observed_tiles_per_update > 0U &&
         config.maximum_frontier_evaluations_per_component > 0U &&
         std::isfinite(config.footprint.radius_m) &&
         std::isfinite(config.footprint.lower_extent_m) &&
         std::isfinite(config.footprint.upper_extent_m) &&
         std::isfinite(config.footprint.sweep_step_m) &&
         config.footprint.radius_m >= 0.0 && config.footprint.lower_extent_m >= 0.0 &&
         config.footprint.upper_extent_m >= 0.0 &&
         config.footprint.perimeter_samples > 0U &&
         config.footprint.radial_rings > 0U && config.footprint.axial_samples > 0U &&
         config.footprint.sweep_step_m > 0.0 &&
         config.observability.maximum_observation_range_m > 0.0 &&
         config.observability.minimum_known_free_ray_m >= 0.0 &&
         config.observability.minimum_supporting_rays > 0U &&
         config.observability.minimum_information_gain_voxels > 0U;
}

} // namespace drone_city_nav
